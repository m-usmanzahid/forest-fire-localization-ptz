# python frames\estimate_heading_and_hfov.py --camera-lat 34.534508 --camera-lon 73.003801 --image-width 640 --exclude-id gps_tetoli --csv frames\gcp_frame4_0006.csv frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv --out frames\frame_heading_hfov.csv

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import numpy as np


def bearing_deg(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlon)
    brng = math.degrees(math.atan2(y, x))
    return (brng + 360.0) % 360.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-lat", type=float, required=True)
    ap.add_argument("--camera-lon", type=float, required=True)
    ap.add_argument("--image-width", type=float, default=640.0)
    ap.add_argument("--exclude-id", nargs="*", default=[], help="IDs to ignore (e.g., gps_tetoli)")
    ap.add_argument("--csv", nargs="+", required=True)
    ap.add_argument("--out", default="frames/frame_heading_hfov.csv")
    args = ap.parse_args()

    cam_lat, cam_lon = args.camera_lat, args.camera_lon
    W = float(args.image_width)
    cx = W / 2.0

    rows = []
    for csv_path in args.csv:
        p = Path(csv_path)
        if not p.exists():
            raise FileNotFoundError(p)

        with p.open("r", encoding="utf-8") as f:
            r = csv.DictReader(f)
            for row in r:
                frame = row["frame"].strip()
                lm_id = row["id"].strip()
                if lm_id in set(args.exclude_id):
                    continue

                lat_s = (row.get("lat") or "").strip()
                lon_s = (row.get("lon") or "").strip()
                if lat_s == "" or lon_s == "":
                    continue

                x = float(row["x"])
                lat = float(lat_s)
                lon = float(lon_s)
                brng = bearing_deg(cam_lat, cam_lon, lat, lon)

                t = (x - cx) / W  # normalized horizontal offset

                rows.append((frame, lm_id, brng, t))

    if len(rows) < 4:
        raise SystemExit("Need more GCP rows overall (>=4) to estimate HFOV + headings robustly.")

    frames = sorted(set(r[0] for r in rows))
    frame_to_col = {fr: i for i, fr in enumerate(frames)}

    # Unknowns: one heading per frame + one global HFOV
    # bearing = heading_frame + t * HFOV
    n = len(rows)
    m = len(frames) + 1
    A = np.zeros((n, m), dtype=float)
    y = np.zeros((n,), dtype=float)

    for i, (fr, lm_id, brng, t) in enumerate(rows):
        A[i, frame_to_col[fr]] = 1.0
        A[i, -1] = t
        y[i] = brng

    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    hfov = sol[-1]
    headings = {fr: (sol[frame_to_col[fr]] % 360.0) for fr in frames}

    yhat = A @ sol
    resid = y - yhat

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "heading_deg"])
        for fr in frames:
            w.writerow([fr, f"{headings[fr]:.6f}"])
        w.writerow([])
        w.writerow(["global_hfov_deg", f"{hfov:.6f}"])
        w.writerow(["rms_residual_deg", f"{math.sqrt(float(np.mean(resid**2))):.6f}"])

    print(f"Estimated global HFOV: {hfov:.3f} deg")
    for fr in frames:
        print(f"{fr}: heading {headings[fr]:.3f} deg")
    print(f"RMS residual: {math.sqrt(float(np.mean(resid**2))):.3f} deg")
    print(f"Wrote: {out_path}")


if __name__ == "__main__":
    main()