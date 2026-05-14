"""
extract_frames.py

Extract individual PNG frames from animated GIF files.

Usage:
    python extract_frames.py --input data/frame4.gif --out frames/
    python extract_frames.py --input data/frame1.gif data/frame2.gif data/frame3.gif data/frame4.gif --out frames/

Output:
    frames/frame4_frame_0001.png
    frames/frame4_frame_0002.png
    ...
"""

import argparse
from pathlib import Path
from PIL import Image


def extract_gif(gif_path: Path, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = gif_path.stem
    img = Image.open(gif_path)

    n = getattr(img, "n_frames", 1)
    for i in range(n):
        img.seek(i)
        frame = img.convert("RGB")
        out_path = out_dir / f"{stem}_frame_{i + 1:04d}.png"
        frame.save(out_path)
        print(f"  Saved {out_path.name}")

    return n


def main():
    ap = argparse.ArgumentParser(description="Extract PNG frames from GIF files.")
    ap.add_argument("--input", nargs="+", required=True, help="GIF file(s) to extract.")
    ap.add_argument("--out", default="frames", help="Output directory (default: frames/).")
    args = ap.parse_args()

    out_dir = Path(args.out)
    total = 0
    for gif in args.input:
        p = Path(gif)
        if not p.exists():
            print(f"[WARN] File not found: {p}")
            continue
        print(f"Extracting {p.name} ...")
        n = extract_gif(p, out_dir)
        total += n
        print(f"  -> {n} frame(s)")

    print(f"\nDone. {total} frame(s) written to {out_dir}/")


if __name__ == "__main__":
    main()
