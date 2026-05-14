"""
calibrate_camera.py

Automatically determine the camera's heading, tilt, and horizontal field of view
by matching the observed skyline in an image against a pre-computed synthetic
horizon profile (from build_horizon.py).

No manual landmark clicking required. The mountain ridge shape itself is the
calibration target.

How it works
------------
1. Detect the sky/terrain boundary in the image (skyline) per pixel column using
   a vertical intensity gradient.
2. Convert detected pixel rows to elevation angles relative to the camera.
3. Do a coarse grid search over (heading, tilt, HFOV) to find the combination
   that best aligns the synthetic horizon with the observed skyline.
4. Refine with a local optimiser (Nelder-Mead).
5. Save the result to calibration.json.

The camera is fixed, so you only need to run this ONCE (or once per zoom level).
Use a clear frame with minimal smoke or cloud near the ridgeline.

Usage:
    python calibrate_camera.py ^
        --frame frames/frame4_frame_0006.png ^
        --horizon horizon_profile.csv ^
        --img-w 640 --img-h 480 ^
        --out calibration.json ^
        --show

Arguments
---------
--frame        A clear PNG frame to calibrate from.
--horizon      Horizon profile CSV produced by build_horizon.py.
--img-w/h      Image dimensions in pixels (default: 640 × 480).
--hfov-min/max Search range for HFOV (default: 20–90 °).
--tilt-min/max Search range for camera tilt (default: −20 to +20 °).
--show         Display the calibration result visually for verification.
--out          Output JSON file (default: calibration.json).
"""

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import scipy.interpolate
import scipy.optimize
import scipy.ndimage


# ---------------------------------------------------------------------------
# Horizon profile loader
# ---------------------------------------------------------------------------

def load_horizon(csv_path: str):
    """
    Load horizon_profile.csv and return a fast interpolation function.
    Handles 360°->0° wraparound by duplicating data at boundaries.
    """
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    az = data[:, 0]
    elev = data[:, 1]

    # Extend for periodic interpolation across the 0/360 boundary
    az_ext = np.concatenate([az - 360, az, az + 360])
    elev_ext = np.concatenate([elev, elev, elev])

    interp = scipy.interpolate.interp1d(
        az_ext, elev_ext, kind="linear", bounds_error=False, fill_value=np.nan
    )
    return interp


# ---------------------------------------------------------------------------
# Night-mode pre-processing
# ---------------------------------------------------------------------------

def preprocess_night(image: np.ndarray) -> np.ndarray:
    """
    Prepare a low-light frame for skyline detection.

    Two steps:
      1. Remove blue overlay boxes drawn by detection systems (e.g. LUMS bounding
         boxes).  These are not real scene content and create false edges.
         Detection criterion: B channel significantly larger than both R and G.
      2. Apply CLAHE to the luminance channel so the faint sky/terrain boundary
         gets enough contrast for the skyline detector to latch on to.
    """
    result = image.copy()

    # --- Step 1: erase blue overlay pixels ---
    B = result[:, :, 0].astype(np.int16)
    G = result[:, :, 1].astype(np.int16)
    R = result[:, :, 2].astype(np.int16)
    blue_mask = (B > 80) & ((B - G) > 40) & ((B - R) > 40)
    # Dilate 3 px to catch anti-aliased box edges
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    blue_mask_d = cv2.dilate(blue_mask.astype(np.uint8), kernel).astype(bool)
    result[blue_mask_d] = 0

    # --- Step 2: CLAHE on luminance ---
    lab = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    result = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

    return result


# ---------------------------------------------------------------------------
# Skyline detection
# ---------------------------------------------------------------------------

def detect_skyline(image: np.ndarray, search_frac: float = 0.5) -> np.ndarray:
    """
    Detect the sky/terrain boundary row for each pixel column.

    Strategy: build a sky-probability mask using two cues:
      1. Brightness — sky is brighter than terrain/vegetation
      2. Low saturation — distant sky and ridge are desaturated compared
         to vivid green foreground trees

    For each column, the skyline is the lowest row that still looks like sky.
    This correctly ignores bright-green foreground vegetation.

    Parameters
    ----------
    image       : BGR image (H × W × 3)
    search_frac : only search in the top `search_frac` fraction of rows

    Returns
    -------
    skyline_v : float array of shape (W,) — detected row per column
    """
    H, W = image.shape[:2]
    search_rows = int(H * search_frac)

    # Work in HSV — easy to separate sky from vivid green trees
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    val = hsv[:, :, 2]   # brightness  (0–255)
    sat = hsv[:, :, 1]   # saturation  (0–255)

    # Sky pixels: bright AND low saturation
    # Foreground trees: high saturation green — penalised heavily
    sky_score = val - 1.5 * sat   # high = sky-like, low = vegetated

    region = sky_score[:search_rows, :]   # (search_rows, W)

    # Smooth horizontally to reduce per-pixel noise
    region_smooth = scipy.ndimage.uniform_filter1d(region, size=5, axis=1)

    # For each column, find the LAST (lowest) row that is still sky-like.
    # We threshold at half the column's max sky score, then take the last True row.
    skyline_v = np.zeros(W, dtype=np.float64)
    col_max = region_smooth.max(axis=0)   # (W,)

    for u in range(W):
        threshold = col_max[u] * 0.5
        sky_rows = np.where(region_smooth[:, u] >= threshold)[0]
        if len(sky_rows) > 0:
            skyline_v[u] = float(sky_rows[-1])
        else:
            skyline_v[u] = search_rows * 0.3   # fallback: upper third

    # Smooth the detected line to suppress column-to-column jitter
    skyline_v = scipy.ndimage.uniform_filter1d(skyline_v, size=40)

    return skyline_v


# ---------------------------------------------------------------------------
# Cost function: how well does (heading, tilt, hfov) match the skyline?
# ---------------------------------------------------------------------------

def compute_cost(
    heading: float,
    tilt: float,
    hfov: float,
    skyline_v: np.ndarray,
    horizon_interp,
    W: int,
    H: int,
) -> float:
    """
    For a given camera orientation, compute the RMS pixel error between
    the expected skyline position and the detected skyline position.

    For each image column u:
      azimuth(u) = heading + arctan((u - cx) / fx)    [degrees]
      expected_world_elev = horizon_interp(azimuth)
      expected_v = cy - fy * tan(expected_world_elev - tilt)  [pixels]

    Cost = RMS(expected_v - observed_v)
    """
    hfov_rad = math.radians(hfov)
    fx = (W / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx
    cx = W / 2.0
    cy = H / 2.0

    u_arr = np.arange(W, dtype=np.float64)
    rel_angles_deg = np.degrees(np.arctan((u_arr - cx) / fx))
    az_arr = (heading + rel_angles_deg) % 360.0

    horizon_elev = horizon_interp(az_arr)          # expected world elevation angle (deg)
    diff_rad = np.radians(horizon_elev - tilt)

    # Avoid tan blowing up for near-vertical angles
    diff_rad = np.clip(diff_rad, -math.radians(80), math.radians(80))
    v_expected = cy - fy * np.tan(diff_rad)

    # Mask columns where the horizon lookup failed or expected position is out of frame
    valid = (
        np.isfinite(horizon_elev) &
        np.isfinite(v_expected) &
        (v_expected >= 0) &
        (v_expected < H)
    )
    if valid.sum() < W * 0.3:
        return 1e9

    return float(np.sqrt(np.mean((v_expected[valid] - skyline_v[valid]) ** 2)))


# ---------------------------------------------------------------------------
# Calibration optimisation
# ---------------------------------------------------------------------------

def calibrate(
    skyline_v: np.ndarray,
    horizon_interp,
    W: int,
    H: int,
    hfov_min: float,
    hfov_max: float,
    tilt_min: float,
    tilt_max: float,
    heading_min: float = 0.0,
    heading_max: float = 360.0,
) -> dict:
    """
    Two-stage calibration:
      1. Coarse grid search across (heading, tilt, HFOV)
      2. Nelder-Mead refinement from best coarse estimate

    Returns dict with heading_deg, tilt_deg, hfov_deg, cost_px.
    """
    # --- Coarse grid ---
    headings = np.arange(heading_min, heading_max, 3.0)
    tilts    = np.arange(tilt_min, tilt_max + 1, 3.0)
    hfovs    = np.arange(hfov_min, hfov_max + 1, 5.0)

    best_cost = 1e9
    best = (0.0, 0.0, 45.0)

    total = len(headings) * len(tilts) * len(hfovs)
    done  = 0

    for hfov in hfovs:
        for tilt in tilts:
            for heading in headings:
                cost = compute_cost(heading, tilt, hfov, skyline_v, horizon_interp, W, H)
                if cost < best_cost:
                    best_cost = cost
                    best = (heading, tilt, hfov)
                done += 1

            pct = done / total * 100
            if done % (len(headings) * len(tilts) // 4 + 1) == 0:
                print(f"  Coarse search: {pct:.0f}%  best so far: "
                      f"H={best[0]:.1f}° T={best[1]:.1f}° HFOV={best[2]:.1f}°  "
                      f"cost={best_cost:.2f} px", end="\r")

    print(f"\n  Coarse result -> heading={best[0]:.1f}° tilt={best[1]:.1f}° "
          f"HFOV={best[2]:.1f}°  cost={best_cost:.2f} px")

    # --- Fine optimisation (Nelder-Mead) ---
    def objective(params):
        h, t, f = params
        return compute_cost(h % 360, t, f, skyline_v, horizon_interp, W, H)

    result = scipy.optimize.minimize(
        objective,
        x0=list(best),
        method="Nelder-Mead",
        options={"xatol": 0.05, "fatol": 0.05, "maxiter": 5000},
    )

    h_opt, t_opt, f_opt = result.x
    h_opt = h_opt % 360.0
    cost_opt = compute_cost(h_opt, t_opt, f_opt, skyline_v, horizon_interp, W, H)

    print(f"  Fine result   -> heading={h_opt:.3f}° tilt={t_opt:.3f}° "
          f"HFOV={f_opt:.3f}°  cost={cost_opt:.2f} px")

    return {
        "heading_deg": round(h_opt, 4),
        "tilt_deg":    round(t_opt, 4),
        "hfov_deg":    round(f_opt, 4),
        "cost_px":     round(cost_opt, 4),
    }


# ---------------------------------------------------------------------------
# Visualisation
# ---------------------------------------------------------------------------

def show_result(image: np.ndarray, skyline_v: np.ndarray, calibration: dict,
                horizon_interp, W: int, H: int) -> None:
    """
    Draw detected skyline and expected skyline from calibration on the image.
    Green = detected, Red = expected from calibration.
    """
    vis = image.copy()
    heading = calibration["heading_deg"]
    tilt    = calibration["tilt_deg"]
    hfov    = calibration["hfov_deg"]

    hfov_rad = math.radians(hfov)
    fx = (W / 2.0) / math.tan(hfov_rad / 2.0)
    fy = fx
    cx = W / 2.0
    cy = H / 2.0

    for u in range(W):
        # Detected skyline (green)
        v_det = int(round(skyline_v[u]))
        if 0 <= v_det < H:
            cv2.circle(vis, (u, v_det), 1, (0, 255, 0), -1)

        # Expected skyline (red)
        rel_deg = math.degrees(math.atan((u - cx) / fx))
        az = (heading + rel_deg) % 360.0
        h_elev = float(horizon_interp(az))
        if not math.isfinite(h_elev):
            continue
        diff_rad = math.radians(h_elev - tilt)
        diff_rad = max(-math.radians(80), min(math.radians(80), diff_rad))
        v_exp = int(round(cy - fy * math.tan(diff_rad)))
        if 0 <= v_exp < H:
            cv2.circle(vis, (u, v_exp), 1, (0, 0, 255), -1)

    legend = [
        "Green = detected skyline",
        "Red   = expected (from calibration)",
        f"Heading {heading:.1f}  Tilt {tilt:.1f}  HFOV {hfov:.1f}",
        f"Cost {calibration['cost_px']:.2f} px",
        "Press any key to close",
    ]
    for i, txt in enumerate(legend):
        cv2.putText(vis, txt, (10, 20 + i * 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

    cv2.imshow("Calibration result", vis)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Calibrate camera heading/tilt/HFOV via skyline matching."
    )
    ap.add_argument("--frame",    required=True, nargs="+",
                    help="One or more PNG frames to calibrate from. "
                         "When multiple are given their detected skylines are averaged "
                         "before optimisation, reducing per-frame noise.")
    ap.add_argument("--horizon",  required=True, help="horizon_profile.csv from build_horizon.py.")
    ap.add_argument("--img-w",    type=int, default=640, help="Image width in pixels.")
    ap.add_argument("--img-h",    type=int, default=480, help="Image height in pixels.")
    ap.add_argument("--hfov-min",    type=float, default=20.0,  help="Min HFOV to search (default: 20°).")
    ap.add_argument("--hfov-max",    type=float, default=90.0,  help="Max HFOV to search (default: 90°).")
    ap.add_argument("--tilt-min",    type=float, default=-20.0, help="Min tilt to search (default: −20°).")
    ap.add_argument("--tilt-max",    type=float, default=20.0,  help="Max tilt to search (default: +20°).")
    ap.add_argument("--heading-min", type=float, default=0.0,   help="Min heading to search (default: 0°).")
    ap.add_argument("--heading-max", type=float, default=360.0, help="Max heading to search (default: 360°).")
    ap.add_argument("--sky-frac",  type=float, default=0.5,
                    help="Only search the top fraction of the image for the skyline (default: 0.5). "
                         "Lower this if foreground trees are pulling the detected line down.")
    ap.add_argument("--night",    action="store_true",
                    help="Night mode: erase blue overlay boxes and apply CLAHE contrast "
                         "enhancement before skyline detection. Use for low-light frames.")
    ap.add_argument("--show",     action="store_true",       help="Show calibration result visually.")
    ap.add_argument("--out",      default="calibration.json", help="Output JSON (default: calibration.json).")
    args = ap.parse_args()

    # Load horizon profile
    print(f"Loading horizon profile: {args.horizon}")
    horizon_interp = load_horizon(args.horizon)

    # Detect and average skylines across all provided frames
    skylines = []
    W, H = args.img_w, args.img_h
    last_image = None
    for fp in args.frame:
        frame_path = Path(fp)
        if not frame_path.exists():
            raise FileNotFoundError(f"Frame not found: {frame_path}")
        image = cv2.imread(str(frame_path))
        if image is None:
            raise ValueError(f"Could not read image: {frame_path}")
        H_img, W_img = image.shape[:2]
        if not args.img_w:
            W = W_img
        if not args.img_h:
            H = H_img
        if args.night:
            image = preprocess_night(image)
        skylines.append(detect_skyline(image, search_frac=args.sky_frac))
        last_image = image

    if len(skylines) == 1:
        print(f"Detecting skyline in image (searching top {args.sky_frac*100:.0f}% of rows) ...")
        skyline_v = skylines[0]
    else:
        print(f"Averaging skylines from {len(skylines)} frames "
              f"(searching top {args.sky_frac*100:.0f}% of rows each) ...")
        skyline_v = np.mean(np.stack(skylines, axis=0), axis=0)

    # Calibrate
    print("Running calibration search ...")
    cal = calibrate(
        skyline_v, horizon_interp, W, H,
        hfov_min=args.hfov_min,
        hfov_max=args.hfov_max,
        tilt_min=args.tilt_min,
        tilt_max=args.tilt_max,
        heading_min=args.heading_min,
        heading_max=args.heading_max,
    )
    cal["frame"] = ", ".join(Path(f).name for f in args.frame)

    # Save
    out_path = Path(args.out)
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(cal, f, indent=2)
    print(f"\nCalibration saved -> {out_path}")
    print(json.dumps(cal, indent=2))

    if args.show:
        show_result(last_image, skyline_v, cal, horizon_interp, W, H)


if __name__ == "__main__":
    main()
