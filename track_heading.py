"""
track_heading.py

Estimate per-frame camera heading by measuring how much the skyline shifts
horizontally between a clear reference frame and each target frame.

Since the camera is fixed but panning, the ridge silhouette moves left/right
across frames. The horizontal pixel shift directly maps to a heading change:

    heading_change_deg = pixel_shift * (HFOV / image_width)
    per_frame_heading  = reference_heading + heading_change_deg

Smoke-obscured columns are automatically masked out before cross-correlation
so they don't corrupt the shift estimate.

Usage:
    python track_heading.py ^
        --reference frames/frame4_frame_0001.png ^
        --frames frames/frame4_frame_0009.png frames/frame4_frame_0010.png frames/frame4_frame_0011.png ^
        --calibration calibration.json ^
        --sky-frac 0.35 ^
        --out frame_headings.csv

Output:
    frame_headings.csv  —  columns: frame, heading_deg, pixel_shift
"""

import argparse
import csv
import json
import math
from pathlib import Path

import cv2
import numpy as np
import scipy.ndimage


# ---------------------------------------------------------------------------
# Skyline extraction (same colour-based approach as calibrate_camera.py)
# ---------------------------------------------------------------------------

def extract_skyline(image: np.ndarray, search_frac: float = 0.35) -> np.ndarray:
    """
    Return sky-score profile: one value per column indicating how sky-like
    the detected boundary is. Higher = more confident sky/terrain edge.
    Also returns the detected row per column.
    """
    H, W = image.shape[:2]
    search_rows = int(H * search_frac)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    val = hsv[:, :, 2]
    sat = hsv[:, :, 1]
    sky_score = val - 1.5 * sat

    region = sky_score[:search_rows, :]
    region_smooth = scipy.ndimage.uniform_filter1d(region, size=5, axis=1)

    skyline_v = np.zeros(W, dtype=np.float64)
    col_max = region_smooth.max(axis=0)

    for u in range(W):
        threshold = col_max[u] * 0.5
        sky_rows = np.where(region_smooth[:, u] >= threshold)[0]
        skyline_v[u] = float(sky_rows[-1]) if len(sky_rows) > 0 else search_rows * 0.3

    skyline_v = scipy.ndimage.uniform_filter1d(skyline_v, size=40)
    return skyline_v


# ---------------------------------------------------------------------------
# Smoke mask
# ---------------------------------------------------------------------------

def detect_smoke_columns(image: np.ndarray, search_frac: float = 0.5) -> np.ndarray:
    """
    Return a boolean mask of shape (W,) — True for columns likely containing smoke.

    Smoke is bright, low-saturation, and appears in the upper portion of the image.
    We flag columns where a large fraction of the upper rows look like smoke.
    """
    H, W = image.shape[:2]
    search_rows = int(H * search_frac)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV).astype(np.float32)
    val = hsv[:search_rows, :, 2]   # brightness
    sat = hsv[:search_rows, :, 1]   # saturation

    # Smoke: very bright (>200) AND very low saturation (<30)
    # Use tighter thresholds to avoid flagging hazy sky as smoke
    smoke_px = (val > 200) & (sat < 30)

    # Flag columns where >50% of the searched rows look like smoke
    # (was 20% — too aggressive, caused almost all columns to be masked)
    smoke_frac = smoke_px.mean(axis=0)
    return smoke_frac > 0.50


# ---------------------------------------------------------------------------
# Cross-correlation shift estimator
# ---------------------------------------------------------------------------

def estimate_shift(ref_skyline: np.ndarray,
                   tgt_skyline: np.ndarray,
                   mask: np.ndarray,
                   max_shift: int = 80) -> tuple[float, float]:
    """
    Estimate horizontal pixel shift between reference and target skyline
    using normalised cross-correlation on unmasked columns only.

    Parameters
    ----------
    ref_skyline : reference skyline row array (W,)
    tgt_skyline : target skyline row array (W,)
    mask        : boolean array (W,) — True = column is smoke-obscured, skip it
    max_shift   : maximum shift to search in either direction (pixels)

    Returns
    -------
    (shift_px, confidence)
        shift_px   : positive = target is shifted right (camera panned right = heading increased)
        confidence : peak NCC value (0–1), higher = more reliable
    """
    valid = ~mask
    if valid.sum() < 50:
        # Not enough clean columns — return zero shift
        return 0.0, 0.0

    ref = ref_skyline[valid] - ref_skyline[valid].mean()
    tgt = tgt_skyline[valid] - tgt_skyline[valid].mean()

    # Normalise
    ref_std = ref.std() + 1e-9
    tgt_std = tgt.std() + 1e-9
    ref = ref / ref_std
    tgt = tgt / tgt_std

    W = len(ref_skyline)
    shifts = range(-max_shift, max_shift + 1)
    scores = []

    for s in shifts:
        # Shift the target skyline by s pixels relative to reference
        # Positive s = target moved right
        if s >= 0:
            r = ref[s:]
            t = tgt[:len(tgt) - s] if s > 0 else tgt
        else:
            r = ref[:len(ref) + s]
            t = tgt[-s:]

        if len(r) < 20:
            scores.append(-1.0)
            continue

        scores.append(float(np.mean(r * t)))

    scores = np.array(scores)
    best_idx = int(np.argmax(scores))
    best_shift = shifts[best_idx]
    confidence = float(scores[best_idx])

    # Sub-pixel refinement via parabolic interpolation
    if 0 < best_idx < len(scores) - 1:
        y0, y1, y2 = scores[best_idx - 1], scores[best_idx], scores[best_idx + 1]
        denom = 2 * (2 * y1 - y0 - y2)
        if abs(denom) > 1e-9:
            best_shift = best_shift + (y0 - y2) / denom

    return float(best_shift), confidence


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Estimate per-frame heading from horizontal skyline shift."
    )
    ap.add_argument("--reference",   required=True,
                    help="Clear reference frame used for calibration (e.g. frame4_frame_0001.png).")
    ap.add_argument("--frames",      nargs="+", required=True,
                    help="Fire frames to estimate heading for.")
    ap.add_argument("--calibration", required=True,
                    help="calibration.json from calibrate_camera.py.")
    ap.add_argument("--sky-frac",    type=float, default=0.35,
                    help="Top fraction of image to search for skyline (default: 0.35).")
    ap.add_argument("--max-shift",   type=int, default=80,
                    help="Maximum pixel shift to search (default: 80).")
    ap.add_argument("--out",         default="frame_headings.csv",
                    help="Output CSV (default: frame_headings.csv).")
    args = ap.parse_args()

    # Load calibration
    with open(args.calibration, "r", encoding="utf-8") as f:
        cal = json.load(f)
    ref_heading = float(cal["heading_deg"])
    hfov        = float(cal["hfov_deg"])
    img_w       = 640   # standard frame width

    deg_per_pixel = hfov / img_w
    print(f"Reference heading : {ref_heading:.3f}°")
    print(f"HFOV              : {hfov:.3f}°")
    print(f"deg/pixel         : {deg_per_pixel:.4f}°/px")

    # Extract reference skyline
    ref_path = Path(args.reference)
    ref_img  = cv2.imread(str(ref_path))
    if ref_img is None:
        raise FileNotFoundError(f"Cannot read reference frame: {ref_path}")
    ref_skyline = extract_skyline(ref_img, search_frac=args.sky_frac)
    print(f"\nReference frame   : {ref_path.name}")

    # Process each target frame
    results = []

    # Also include the reference frame itself (shift = 0)
    results.append({
        "frame":       ref_path.name,
        "heading_deg": round(ref_heading, 4),
        "pixel_shift": 0.0,
        "confidence":  1.0,
    })

    for frame_path_str in args.frames:
        frame_path = Path(frame_path_str)
        if not frame_path.exists():
            print(f"  [WARN] Not found: {frame_path}")
            continue

        if frame_path.resolve() == ref_path.resolve():
            continue  # already added above

        tgt_img = cv2.imread(str(frame_path))
        if tgt_img is None:
            print(f"  [WARN] Cannot read: {frame_path}")
            continue

        tgt_skyline   = extract_skyline(tgt_img, search_frac=args.sky_frac)
        smoke_mask    = detect_smoke_columns(tgt_img, search_frac=0.5)
        clean_cols    = int((~smoke_mask).sum())
        smoke_cols    = int(smoke_mask.sum())

        shift_px, conf = estimate_shift(
            ref_skyline, tgt_skyline, smoke_mask,
            max_shift=args.max_shift
        )

        heading = (ref_heading + shift_px * deg_per_pixel) % 360.0

        print(f"  {frame_path.name}: shift={shift_px:+.1f} px  "
              f"-> heading={heading:.3f}°  confidence={conf:.3f}  "
              f"clean={clean_cols}  smoke={smoke_cols}")

        results.append({
            "frame":       frame_path.name,
            "heading_deg": round(heading, 4),
            "pixel_shift": round(shift_px, 2),
            "confidence":  round(conf, 4),
        })

    # Write output
    out_path = Path(args.out)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["frame", "heading_deg", "pixel_shift", "confidence"])
        w.writeheader()
        w.writerows(results)

    print(f"\nWrote {len(results)} heading(s) -> {out_path}")


if __name__ == "__main__":
    main()
