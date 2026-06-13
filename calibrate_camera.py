"""
calibrate_camera.py

Automatically determine the camera's heading, tilt, and horizontal field of view
by matching the observed skyline in an image against a pre-computed synthetic
horizon profile (from build_horizon.py).

How it works
------------
1. Detect the sky/terrain boundary in the image (skyline) per pixel column.
2. Convert detected pixel rows to elevation angles relative to the camera.
3. Coarse 2-D grid search over (heading, HFOV) — tilt is solved analytically
   for each pair, eliminating it as a discrete search dimension and reducing
   search cost by ~14x while improving tilt accuracy.
4. Refine with Nelder-Mead.
5. Save the result to calibration.json.

Cost function uses a trimmed mean (drops worst 20 % of columns) so that
smoke-obscured or tree-covered columns cannot dominate the result.

Usage:
    python calibrate_camera.py ^
        --frame frames/frame4_frame_0006.png ^
        --horizon horizon_profile.csv ^
        --out calibration.json ^
        --show
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
    """Load horizon_profile.csv and return a fast interpolation function."""
    data = np.loadtxt(csv_path, delimiter=",", skiprows=1)
    az   = data[:, 0]
    elev = data[:, 1]

    az_ext   = np.concatenate([az - 360, az, az + 360])
    elev_ext = np.concatenate([elev, elev, elev])

    return scipy.interpolate.interp1d(
        az_ext, elev_ext, kind="linear",
        bounds_error=False, fill_value=np.nan
    )


# ---------------------------------------------------------------------------
# Night-mode pre-processing
# ---------------------------------------------------------------------------

def preprocess_night(image: np.ndarray) -> np.ndarray:
    """
    Prepare a low-light frame for skyline detection.
    1. Erase blue overlay boxes (detection-system artefacts).
    2. Apply CLAHE to amplify faint sky/terrain boundary.
    """
    result = image.copy()

    B = result[:, :, 0].astype(np.int16)
    G = result[:, :, 1].astype(np.int16)
    R = result[:, :, 2].astype(np.int16)
    blue_mask = (B > 80) & ((B - G) > 40) & ((B - R) > 40)
    kernel     = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    blue_mask_d = cv2.dilate(blue_mask.astype(np.uint8), kernel).astype(bool)
    result[blue_mask_d] = 0

    lab = cv2.cvtColor(result, cv2.COLOR_BGR2LAB)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    lab[:, :, 0] = clahe.apply(lab[:, :, 0])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


# ---------------------------------------------------------------------------
# Skyline detection
# ---------------------------------------------------------------------------

def detect_skyline(image: np.ndarray, search_frac: float = 0.5) -> np.ndarray:
    """
    Detect the sky/terrain boundary row for each pixel column.

    Uses sky-probability = brightness − 1.5 × saturation so that vivid green
    foreground trees score low and are ignored. The skyline is the lowest row
    still classified as sky-like.
    """
    H, W      = image.shape[:2]
    search_rows = int(H * search_frac)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    val = hsv[:, :, 2]
    sat = hsv[:, :, 1]
    sky_score = val - 1.5 * sat

    region        = sky_score[:search_rows, :]
    region_smooth = scipy.ndimage.uniform_filter1d(region, size=5, axis=1)

    skyline_v = np.zeros(W, dtype=np.float64)
    col_max   = region_smooth.max(axis=0)

    for u in range(W):
        threshold = col_max[u] * 0.5
        sky_rows  = np.where(region_smooth[:, u] >= threshold)[0]
        skyline_v[u] = float(sky_rows[-1]) if len(sky_rows) > 0 else search_rows * 0.3

    return scipy.ndimage.uniform_filter1d(skyline_v, size=40)


# ---------------------------------------------------------------------------
# Cost function
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
    Trimmed-mean pixel error between expected and detected skyline.

    Dropping the worst 20 % of columns makes the cost robust to smoke,
    foreground trees, and partial cloud cover that corrupt individual columns.
    """
    hfov_rad = math.radians(hfov)
    fx = (W / 2.0) / math.tan(hfov_rad / 2.0)
    cx = W / 2.0
    cy = H / 2.0

    u_arr          = np.arange(W, dtype=np.float64)
    rel_angles_deg = np.degrees(np.arctan((u_arr - cx) / fx))
    az_arr         = (heading + rel_angles_deg) % 360.0

    horizon_elev = horizon_interp(az_arr)
    diff_rad     = np.radians(horizon_elev - tilt)
    diff_rad     = np.clip(diff_rad, -math.radians(80), math.radians(80))
    v_expected   = cy - fx * np.tan(diff_rad)  # fx == fy (square pixels)

    valid = (
        np.isfinite(horizon_elev) &
        np.isfinite(v_expected) &
        (v_expected >= 0) &
        (v_expected < H)
    )
    if valid.sum() < W * 0.3:
        return 1e9

    residuals = np.abs(v_expected[valid] - skyline_v[valid])

    # Trimmed mean: drop worst 20 % (outlier columns: smoke, trees, artefacts)
    n       = len(residuals)
    n_trim  = max(1, int(n * 0.20))
    return float(np.mean(np.sort(residuals)[:-n_trim]))


# ---------------------------------------------------------------------------
# Analytical tilt computation
# ---------------------------------------------------------------------------

def compute_optimal_tilt(
    heading: float,
    hfov: float,
    skyline_v: np.ndarray,
    horizon_interp,
    W: int,
    H: int,
    tilt_min: float,
    tilt_max: float,
) -> float:
    """
    Analytically compute the tilt that zeroes the mean vertical offset between
    the expected and detected skylines for a given (heading, HFOV) pair.

    Derivation (small-angle approximation):
        v_exp  = cy - fy * tan(horizon_elev - tilt)
        Setting mean(v_exp) = mean(v_det):
        tilt   = mean(horizon_elev) - degrees(arctan((cy - mean(v_det)) / fy))

    This eliminates tilt as a discrete search axis — the coarse grid becomes
    2-D (heading × HFOV) rather than 3-D (heading × tilt × HFOV).
    """
    hfov_rad = math.radians(hfov)
    fx       = (W / 2.0) / math.tan(hfov_rad / 2.0)
    cx       = W / 2.0
    cy       = H / 2.0

    u_arr          = np.arange(W, dtype=np.float64)
    rel_angles_deg = np.degrees(np.arctan((u_arr - cx) / fx))
    az_arr         = (heading + rel_angles_deg) % 360.0
    horizon_elev   = horizon_interp(az_arr)

    valid = np.isfinite(horizon_elev)
    if valid.sum() < W * 0.3:
        return 0.0

    mean_h_elev = float(np.mean(horizon_elev[valid]))
    mean_v_det  = float(np.mean(skyline_v[valid]))

    tilt_deg = mean_h_elev - math.degrees(math.atan2(cy - mean_v_det, fx))
    return float(np.clip(tilt_deg, tilt_min, tilt_max))


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
      1. Coarse 2-D grid (heading × HFOV) with tilt solved analytically per pair.
         This is ~14x faster than the old 3-D grid and finds better tilts.
      2. Nelder-Mead joint refinement of all three parameters.
    """
    headings = np.arange(heading_min, heading_max, 2.0)
    hfovs    = np.arange(hfov_min, hfov_max + 1, 3.0)

    best_cost = 1e9
    best      = (0.0, 0.0, 45.0)
    total     = len(headings) * len(hfovs)
    done      = 0

    for hfov in hfovs:
        for heading in headings:
            tilt = compute_optimal_tilt(
                heading, hfov, skyline_v, horizon_interp, W, H,
                tilt_min, tilt_max
            )
            cost = compute_cost(heading, tilt, hfov, skyline_v,
                                horizon_interp, W, H)
            if cost < best_cost:
                best_cost = cost
                best      = (heading, tilt, hfov)
            done += 1

        pct = done / total * 100
        if done % max(1, len(headings) // 4) == 0:
            print(f"  Coarse search: {pct:.0f}%  best: "
                  f"H={best[0]:.1f}° T={best[1]:.1f}° HFOV={best[2]:.1f}°  "
                  f"cost={best_cost:.2f} px", end="\r")

    print(f"\n  Coarse result -> heading={best[0]:.1f}° tilt={best[1]:.1f}° "
          f"HFOV={best[2]:.1f}°  cost={best_cost:.2f} px")

    def objective(params):
        h, t, f = params
        t_clip  = float(np.clip(t, tilt_min - 5, tilt_max + 5))
        return compute_cost(h % 360, t_clip, f, skyline_v, horizon_interp, W, H)

    result = scipy.optimize.minimize(
        objective,
        x0=list(best),
        method="Nelder-Mead",
        options={"xatol": 0.02, "fatol": 0.02, "maxiter": 8000},
    )

    h_opt, t_opt, f_opt = result.x
    h_opt  = h_opt % 360.0
    t_opt  = float(np.clip(t_opt, tilt_min, tilt_max))
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

def show_result(image: np.ndarray, skyline_v: np.ndarray,
                calibration: dict, horizon_interp, W: int, H: int) -> None:
    """Draw detected (green) and expected (red) skylines on the image."""
    vis     = image.copy()
    heading = calibration["heading_deg"]
    tilt    = calibration["tilt_deg"]
    hfov    = calibration["hfov_deg"]

    hfov_rad = math.radians(hfov)
    fx       = (W / 2.0) / math.tan(hfov_rad / 2.0)
    cx       = W / 2.0
    cy       = H / 2.0

    for u in range(W):
        v_det = int(round(skyline_v[u]))
        if 0 <= v_det < H:
            cv2.circle(vis, (u, v_det), 1, (0, 255, 0), -1)

        rel_deg = math.degrees(math.atan((u - cx) / fx))
        az      = (heading + rel_deg) % 360.0
        h_elev  = float(horizon_interp(az))
        if not math.isfinite(h_elev):
            continue
        diff_rad = math.radians(h_elev - tilt)
        diff_rad = max(-math.radians(80), min(math.radians(80), diff_rad))
        v_exp    = int(round(cy - fx * math.tan(diff_rad)))
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
# Importable run() entry point
# ---------------------------------------------------------------------------

def run(
    frame: list,
    horizon: str,
    img_w: int = 640,
    img_h: int = 480,
    heading_min: float = 0.0,
    heading_max: float = 360.0,
    hfov_min: float = 20.0,
    hfov_max: float = 90.0,
    tilt_min: float = -20.0,
    tilt_max: float = 20.0,
    sky_frac: float = 0.5,
    night: bool = False,
    show: bool = False,
    out: str = "calibration.json",
) -> dict:
    """
    Run calibration and save JSON. Returns the calibration dict.
    Importable by pipeline.py.

    Parameters
    ----------
    frame : list of str — one or more frame paths to calibrate from.
    """
    print(f"Loading horizon profile: {horizon}")
    horizon_interp = load_horizon(horizon)

    W, H       = img_w, img_h
    skylines   = []
    last_image = None

    for fp in frame:
        fpath = Path(fp)
        if not fpath.exists():
            raise FileNotFoundError(f"Frame not found: {fpath}")
        image = cv2.imread(str(fpath))
        if image is None:
            raise ValueError(f"Could not read image: {fpath}")
        if night:
            image = preprocess_night(image)
        skylines.append(detect_skyline(image, search_frac=sky_frac))
        last_image = image

    if len(skylines) == 1:
        print(f"Detecting skyline (top {sky_frac*100:.0f}% of rows) ...")
        skyline_v = skylines[0]
    else:
        print(f"Averaging skylines from {len(skylines)} frames ...")
        skyline_v = np.mean(np.stack(skylines, axis=0), axis=0)

    print("Running calibration search ...")
    cal = calibrate(
        skyline_v, horizon_interp, W, H,
        hfov_min=hfov_min, hfov_max=hfov_max,
        tilt_min=tilt_min, tilt_max=tilt_max,
        heading_min=heading_min, heading_max=heading_max,
    )
    cal["frame"] = ", ".join(Path(f).name for f in frame)

    out_path = Path(out)
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(cal, f, indent=2)
    print(f"\nCalibration saved -> {out_path}")
    print(json.dumps(cal, indent=2))

    if show and last_image is not None:
        show_result(last_image, skyline_v, cal, horizon_interp, W, H)

    return cal


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Calibrate camera heading/tilt/HFOV via skyline matching."
    )
    ap.add_argument("--frame",        required=True, nargs="+")
    ap.add_argument("--horizon",      required=True)
    ap.add_argument("--img-w",        type=int,   default=640)
    ap.add_argument("--img-h",        type=int,   default=480)
    ap.add_argument("--hfov-min",     type=float, default=20.0)
    ap.add_argument("--hfov-max",     type=float, default=90.0)
    ap.add_argument("--tilt-min",     type=float, default=-20.0)
    ap.add_argument("--tilt-max",     type=float, default=20.0)
    ap.add_argument("--heading-min",  type=float, default=0.0)
    ap.add_argument("--heading-max",  type=float, default=360.0)
    ap.add_argument("--sky-frac",     type=float, default=0.5)
    ap.add_argument("--night",        action="store_true")
    ap.add_argument("--show",         action="store_true")
    ap.add_argument("--out",          default="calibration.json")
    args = ap.parse_args()

    run(
        frame=args.frame,
        horizon=args.horizon,
        img_w=args.img_w,
        img_h=args.img_h,
        heading_min=args.heading_min,
        heading_max=args.heading_max,
        hfov_min=args.hfov_min,
        hfov_max=args.hfov_max,
        tilt_min=args.tilt_min,
        tilt_max=args.tilt_max,
        sky_frac=args.sky_frac,
        night=args.night,
        show=args.show,
        out=args.out,
    )


if __name__ == "__main__":
    main()
