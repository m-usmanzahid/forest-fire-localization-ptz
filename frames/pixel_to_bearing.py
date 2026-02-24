import argparse
import csv
from pathlib import Path

def load_headings(path):
    headings = {}
    hfov = None
    with open(path, "r", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    # First section: frame,heading_deg
    for r in rows:
        if not r:
            continue
        if r[0] == "frame":
            continue
        if r[0].startswith("global_hfov_deg"):
            hfov = float(r[1])
            break
        if r[0].endswith(".png"):
            headings[r[0]] = float(r[1])
    # Find global hfov if not parsed
    for r in rows:
        if len(r) >= 2 and r[0] == "global_hfov_deg":
            hfov = float(r[1])
    if hfov is None:
        raise ValueError("global_hfov_deg not found in heading file")
    return headings, hfov

def wrap360(a):
    a = a % 360.0
    return a if a >= 0 else a + 360.0

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--headings", default="frames/frame_heading_hfov.csv")
    ap.add_argument("--pixels", default="frames/fire_pixels.csv")
    ap.add_argument("--image-width", type=float, default=640.0)
    ap.add_argument("--id", default="fire_origin")
    ap.add_argument("--out", default="frames/fire_bearings.csv")
    args = ap.parse_args()

    headings, hfov = load_headings(args.headings)
    W = args.image_width
    cx = W / 2.0

    out_rows = []
    with open(args.pixels, "r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            if row["id"].strip() != args.id:
                continue
            frame = row["frame"].strip()
            x = float(row["x"])
            if frame not in headings:
                continue
            heading = headings[frame]
            alpha = ((x - cx) / W) * hfov
            bearing = wrap360(heading + alpha)
            out_rows.append([frame, f"{x:.2f}", f"{heading:.6f}", f"{hfov:.6f}", f"{alpha:.6f}", f"{bearing:.6f}"])

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "x", "heading_deg", "hfov_deg", "alpha_deg", "bearing_deg"])
        w.writerows(out_rows)

    print(f"Used HFOV = {hfov:.3f} deg")
    for r in out_rows:
        print(f"{r[0]}: bearing = {r[-1]} deg (alpha={r[-2]})")
    print(f"Wrote: {out_path}")

if __name__ == "__main__":
    main()