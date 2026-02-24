# python frames/gcp_annotator.py --frame frame4_frame_0009.png --out gcp_frame4_0009.csv
# python frames/gcp_annotator.py --frame frame4_frame_0009.png --out gcp_frame4_0009.csv --append
# python frames/gcp_annotator.py --frame frame4_frame_0009.png --out gcp_frame4_0009.csv --hfov 60

# click a point on the frame
# terminal asks for ID (required) and optional lat/lon/alt
# it saves everything in one row: frame,id,x,y,lat,lon,alt_m

from __future__ import annotations
import argparse
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Tuple
import matplotlib.image as mpimg
import matplotlib.pyplot as plt

def _parse_float(s: str) -> Optional[float]:
    s = s.strip()
    if s == "":
        return None
    try:
        return float(s)
    except ValueError:
        return None


def _resolve_frame_path(project_root: Path, frame_arg: str) -> Path:
    p = Path(frame_arg)
    if p.exists():
        return p

    frames_dir = project_root / "frames"
    candidate = frames_dir / frame_arg
    if candidate.exists():
        return candidate

    raise FileNotFoundError(
        f"Could not find frame '{frame_arg}'. Provide an absolute path or a filename under {frames_dir}."
    )


def _resolve_out_path(project_root: Path, out_arg: str) -> Path:
    p = Path(out_arg)
    if p.is_absolute():
        return p
    return (project_root / "frames") / p


def _backup_file(path: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    bak = path.with_suffix(path.suffix + f".bak_{ts}")
    bak.write_bytes(path.read_bytes())
    return bak


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Click GCPs on a frame and optionally attach lat/lon/alt right away. Saves to CSV."
    )
    parser.add_argument(
        "--frame",
        required=True,
        help="Frame image path OR filename under ./frames (e.g., frame4_frame_0009.png).",
    )
    parser.add_argument(
        "--out",
        default="gcp_annotations.csv",
        help="Output CSV filename (saved under ./frames unless absolute path). Default: gcp_annotations.csv",
    )
    parser.add_argument(
        "--append",
        action="store_true",
        help="Append to existing CSV (keeps header if file already exists).",
    )
    parser.add_argument(
        "--hfov",
        type=float,
        default=None,
        help="Optional: if provided, prints per-click horizontal angle offset from image center (degrees).",
    )
    args = parser.parse_args()

    project_root = Path(__file__).resolve().parents[1]
    frame_path = _resolve_frame_path(project_root, args.frame)
    out_csv = _resolve_out_path(project_root, args.out)

    img = mpimg.imread(frame_path)
    H, W = img.shape[0], img.shape[1]
    cx = W / 2.0

    # We'll collect ONLY data rows first, and decide whether to write at the end.
    data_rows: List[str] = []

    fig, ax = plt.subplots()
    ax.imshow(img)
    ax.set_axis_off()
    ax.set_title(
        f"{frame_path.name}\n"
        "Click a point, then enter its ID + optional lat/lon/alt in the terminal.\n"
        "Close the window or type 'done' as ID to finish."
    )

    print("\nInstructions:")
    print("  1) Click a landmark in the image window.")
    print("  2) In the terminal: enter an ID (required). lat/lon/alt are optional.")
    print("  3) Repeat. Type ID = 'done' to stop (after any click), or close the window.\n")

    while plt.fignum_exists(fig.number):
        try:
            pts = plt.ginput(1, timeout=-1)  # waits for one click
        except Exception:
            break

        if not pts:
            break

        x, y = pts[0]

        lm_id = input("Landmark ID (e.g., belian_left, ridge_notch_1) or 'done': ").strip()
        if lm_id.lower() == "done":
            break
        if lm_id == "":
            print("ID cannot be empty. Click again.")
            continue

        lat = _parse_float(input("  lat (blank if unknown): "))
        lon = _parse_float(input("  lon (blank if unknown): "))
        alt = _parse_float(input("  alt_m (blank if unknown): "))

        if args.hfov is not None:
            alpha = ((x - cx) / W) * args.hfov
            print(f"  -> pixel (x={x:.2f}, y={y:.2f}), approx alpha_x={alpha:.3f} deg from center")

        # Draw marker + label for visual confirmation
        ax.plot([x], [y], marker="x")
        ax.text(x + 5, y + 5, lm_id, fontsize=9)
        fig.canvas.draw_idle()

        def fmt(v: Optional[float]) -> str:
            return "" if v is None else f"{v:.8f}"

        row = ",".join(
            [
                frame_path.name,
                lm_id,
                f"{x:.2f}",
                f"{y:.2f}",
                fmt(lat),
                fmt(lon),
                fmt(alt),
            ]
        )
        data_rows.append(row)
        print(f"Saved (buffered): {row}\n")

    plt.close(fig)

    # ✅ SAFETY: If no points were clicked, DO NOT write anything (prevents overwriting existing CSV).
    if len(data_rows) == 0:
        print(f"\nNo points were recorded. Not writing anything. Existing file (if any) is untouched:\n  {out_csv}")
        return

    header = "frame,id,x,y,lat,lon,alt_m"

    # Decide whether to write header
    need_header = True
    if args.append and out_csv.exists():
        need_header = False

    # ✅ SAFETY: If overwriting (not append) and file exists, create a backup first.
    if (not args.append) and out_csv.exists():
        bak = _backup_file(out_csv)
        print(f"Backup created:\n  {bak}")

    # Write / append
    out_csv.parent.mkdir(parents=True, exist_ok=True)

    if args.append and out_csv.exists():
        with out_csv.open("a", encoding="utf-8", newline="") as f:
            for r in data_rows:
                f.write(r + "\n")
    else:
        with out_csv.open("w", encoding="utf-8", newline="") as f:
            if need_header:
                f.write(header + "\n")
            for r in data_rows:
                f.write(r + "\n")

    print(f"\nDone. Wrote {len(data_rows)} points to:\n  {out_csv}")


if __name__ == "__main__":
    main()