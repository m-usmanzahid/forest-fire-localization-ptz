"""
heading_from_fire.py

Back-calculate per-frame camera heading using the fire pixel position.

Since the fire is a fixed point in the world, and one frame's heading + fire
pixel gives us the bearing to the fire, every other frame's heading can be
derived directly from where the fire pixel appears in that frame.

    bearing_to_fire = heading + arctan((fire_x - cx) / fx)
    -> heading = bearing_to_fire - arctan((fire_x - cx) / fx)

This is exact and requires no cross-correlation or skyline matching. It is
the most accurate per-frame heading estimator available when smoke is present.

Requirement: one "anchor" frame whose heading is trusted (ideally the frame
with the lowest skyline calibration cost and clearest image).

Usage:
    python heading_from_fire.py ^
        --fire fire_pixels.csv ^
        --calibration calibration.json ^
        --anchor-frame frames/frame4_frame_0009.png ^
        --anchor-heading 66.207 ^
        --out frame_headings.csv

    # Or let it compute anchor heading from calibration + anchor fire pixel:
    python heading_from_fire.py ^
        --fire fire_pixels.csv ^
        --calibration calibration.json ^
        --anchor-frame frames/frame4_frame_0009.png ^
        --out frame_headings.csv

Output:
    frame_headings.csv  —  columns: frame, heading_deg, fire_x, offset_deg
"""

import argparse
import csv
import json
import math
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(
        description="Compute per-frame headings from fire pixel positions."
    )
    ap.add_argument("--fire",           required=True,
                    help="fire_pixels.csv from annotate_fire.py.")
    ap.add_argument("--calibration",    required=True,
                    help="calibration.json from calibrate_camera.py.")
    ap.add_argument("--anchor-frame",   required=True,
                    help="The frame name whose heading is used as the reference "
                         "(e.g. frames/frame4_frame_0009.png). "
                         "Should be the clearest frame with most reliable calibration.")
    ap.add_argument("--anchor-heading", type=float, default=None,
                    help="Known heading for the anchor frame (degrees). "
                         "If omitted, uses the calibration heading directly.")
    ap.add_argument("--img-w",          type=int, default=640)
    ap.add_argument("--out",            default="frame_headings.csv")
    args = ap.parse_args()

    # Load calibration
    with open(args.calibration, "r", encoding="utf-8") as f:
        cal = json.load(f)
    hfov    = float(cal["hfov_deg"])
    cal_heading = float(cal["heading_deg"])

    W  = args.img_w
    cx = W / 2.0
    fx = (W / 2.0) / math.tan(math.radians(hfov / 2.0))
    print(f"HFOV={hfov:.3f}°  fx={fx:.1f}  cx={cx:.1f}")

    # Load fire pixels
    fire_pixels = {}
    with open(args.fire, "r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            frame = (row.get("frame") or "").strip()
            try:
                x = float(row["x"])
            except (KeyError, ValueError):
                continue
            if frame:
                fire_pixels[frame] = x

    # Resolve anchor frame name (strip directory prefix if needed)
    anchor_name = Path(args.anchor_frame).name
    if anchor_name not in fire_pixels:
        raise ValueError(
            f"Anchor frame '{anchor_name}' not found in {args.fire}.\n"
            f"Available frames: {list(fire_pixels.keys())}"
        )

    # Determine anchor heading
    if args.anchor_heading is not None:
        anchor_heading = args.anchor_heading
        print(f"Anchor heading (provided): {anchor_heading:.3f}°")
    else:
        anchor_heading = cal_heading
        print(f"Anchor heading (from calibration): {anchor_heading:.3f}°")

    # Compute bearing to fire from anchor frame
    anchor_x      = fire_pixels[anchor_name]
    anchor_offset = math.degrees(math.atan((anchor_x - cx) / fx))
    bearing_fire  = anchor_heading + anchor_offset

    print(f"Anchor frame  : {anchor_name}  fire_x={anchor_x:.1f}  "
          f"offset={anchor_offset:+.3f}°  bearing_to_fire={bearing_fire:.3f}°\n")

    # Back-calculate heading for every frame
    results = []
    for frame, fire_x in sorted(fire_pixels.items()):
        offset_deg = math.degrees(math.atan((fire_x - cx) / fx))
        heading    = (bearing_fire - offset_deg) % 360.0
        results.append({
            "frame":      frame,
            "heading_deg": round(heading, 4),
            "fire_x":     round(fire_x, 1),
            "offset_deg": round(offset_deg, 4),
        })
        marker = " <- anchor" if frame == anchor_name else ""
        print(f"  {frame}: fire_x={fire_x:.1f}  offset={offset_deg:+.3f}°  "
              f"heading={heading:.3f}°{marker}")

    # Write output
    out_path = Path(args.out)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["frame", "heading_deg", "fire_x", "offset_deg"])
        w.writeheader()
        w.writerows(results)

    print(f"\nWrote {len(results)} heading(s) -> {out_path}")


# ---------------------------------------------------------------------------
# Importable run() entry point
# ---------------------------------------------------------------------------

def run(
    fire: str,
    calibration: str,
    anchor_frame: str,
    anchor_heading: float = None,
    img_w: int = 640,
    out: str = "frame_headings.csv",
) -> tuple:
    """
    Compute per-frame headings and save CSV.
    Returns (out_path, headings_dict) where headings_dict maps frame name -> heading_deg.
    Importable by pipeline.py.
    """
    import csv as _csv
    import json as _json

    with open(calibration, "r", encoding="utf-8") as f:
        cal = _json.load(f)
    hfov = float(cal["hfov_deg"])

    W  = img_w
    cx = W / 2.0
    fx = (W / 2.0) / math.tan(math.radians(hfov / 2.0))
    print(f"HFOV={hfov:.3f}°  fx={fx:.1f}  cx={cx:.1f}")

    fire_pixels = {}
    with open(fire, "r", encoding="utf-8") as f:
        for row in _csv.DictReader(f):
            frame = (row.get("frame") or "").strip()
            try:
                x = float(row["x"])
            except (KeyError, ValueError):
                continue
            if frame:
                fire_pixels[frame] = x

    anchor_name = Path(anchor_frame).name
    if anchor_name not in fire_pixels:
        raise ValueError(
            f"Anchor frame '{anchor_name}' not found in {fire}.\n"
            f"Available frames: {list(fire_pixels.keys())}"
        )

    resolved_anchor_heading = (anchor_heading if anchor_heading is not None
                                else float(cal["heading_deg"]))
    print(f"Anchor heading: {resolved_anchor_heading:.3f}°")

    anchor_x      = fire_pixels[anchor_name]
    anchor_offset = math.degrees(math.atan((anchor_x - cx) / fx))
    bearing_fire  = resolved_anchor_heading + anchor_offset

    print(f"Anchor frame: {anchor_name}  fire_x={anchor_x:.1f}  "
          f"offset={anchor_offset:+.3f}°  bearing_to_fire={bearing_fire:.3f}°\n")

    results = []
    headings_out = {}
    for frame, fire_x in sorted(fire_pixels.items()):
        offset_deg = math.degrees(math.atan((fire_x - cx) / fx))
        hdg        = (bearing_fire - offset_deg) % 360.0
        results.append({
            "frame":       frame,
            "heading_deg": round(hdg, 4),
            "fire_x":      round(fire_x, 1),
            "offset_deg":  round(offset_deg, 4),
        })
        headings_out[frame] = hdg
        marker = " <- anchor" if frame == anchor_name else ""
        print(f"  {frame}: fire_x={fire_x:.1f}  offset={offset_deg:+.3f}°  "
              f"heading={hdg:.3f}°{marker}")

    out_path = Path(out)
    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = _csv.DictWriter(f, fieldnames=["frame", "heading_deg", "fire_x", "offset_deg"])
        w.writeheader()
        w.writerows(results)

    print(f"\nWrote {len(results)} heading(s) -> {out_path}")
    return str(out_path), headings_out


if __name__ == "__main__":
    main()
