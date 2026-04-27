"""
annotate_fire.py

Simple click tool for marking the fire / smoke origin pixel in a frame.

Click the BASE of the smoke plume — the lowest visible point where smoke
meets the ground. This is the only manual step in the new pipeline.

Controls
--------
  Left-click   : place / move the fire marker
  S            : save and move to next frame (or exit if last)
  D            : skip this frame without saving
  Q            : quit immediately

Usage:
    python annotate_fire.py --frames frames/frame4_frame_0009.png frames/frame4_frame_0011.png --out fire_pixels.csv

    # Append more frames to an existing file:
    python annotate_fire.py --frames frames/frame4_frame_0013.png --out fire_pixels.csv --append

Output:
    fire_pixels.csv  —  columns: frame, x, y
"""

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


WINDOW = "Annotate fire origin  |  Click base of smoke  |  S=save  D=skip  Q=quit"


def annotate_frame(image: np.ndarray, frame_name: str) -> tuple[float, float] | None:
    """
    Show the image and let the user click the fire origin pixel.
    Returns (x, y) or None if skipped / quit.
    """
    click_state = {"pt": None, "quit": False, "skip": False}
    vis = image.copy()

    def mouse_cb(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            click_state["pt"] = (x, y)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(WINDOW, mouse_cb)

    instructions = [
        "Click the BASE of the smoke plume.",
        "S = save and continue",
        "D = skip this frame",
        "Q = quit",
        f"Frame: {frame_name}",
    ]

    while True:
        display = vis.copy()

        # Draw existing click
        if click_state["pt"] is not None:
            x, y = click_state["pt"]
            cv2.drawMarker(display, (x, y), (0, 0, 255),
                           cv2.MARKER_CROSS, 20, 2)
            cv2.circle(display, (x, y), 8, (0, 0, 255), 2)
            cv2.putText(display, f"({x}, {y})", (x + 12, y - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # Draw instructions
        for i, txt in enumerate(instructions):
            cv2.putText(display, txt, (10, 20 + i * 22),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1,
                        cv2.LINE_AA)

        cv2.imshow(WINDOW, display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord("s") or key == ord("S"):
            if click_state["pt"] is not None:
                cv2.destroyAllWindows()
                return click_state["pt"]
            else:
                # Remind user to click first
                cv2.putText(display, "Click a point first!", (10, 160),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                cv2.imshow(WINDOW, display)

        elif key == ord("d") or key == ord("D"):
            cv2.destroyAllWindows()
            return None  # skip

        elif key == ord("q") or key == ord("Q"):
            click_state["quit"] = True
            cv2.destroyAllWindows()
            return None

    cv2.destroyAllWindows()
    return None


def load_existing_frames(csv_path: Path) -> set:
    """Return set of frame names already in the CSV."""
    if not csv_path.exists():
        return set()
    seen = set()
    with csv_path.open("r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            seen.add((row.get("frame") or "").strip())
    return seen


def main():
    ap = argparse.ArgumentParser(
        description="Click to annotate fire origin pixel(s) in frames."
    )
    ap.add_argument("--frames", nargs="+", required=True,
                    help="PNG frame(s) to annotate.")
    ap.add_argument("--out", default="fire_pixels.csv",
                    help="Output CSV (default: fire_pixels.csv).")
    ap.add_argument("--append", action="store_true",
                    help="Append to existing CSV instead of overwriting.")
    args = ap.parse_args()

    out_path = Path(args.out)
    already_done = load_existing_frames(out_path) if args.append else set()

    mode = "a" if (args.append and out_path.exists()) else "w"
    write_header = not (args.append and out_path.exists())

    saved = 0
    with out_path.open(mode, newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if write_header:
            w.writerow(["frame", "x", "y"])

        for frame_path_str in args.frames:
            frame_path = Path(frame_path_str)
            if not frame_path.exists():
                print(f"[WARN] File not found, skipping: {frame_path}")
                continue

            frame_name = frame_path.name
            if frame_name in already_done:
                print(f"[SKIP] Already annotated: {frame_name}")
                continue

            image = cv2.imread(str(frame_path))
            if image is None:
                print(f"[WARN] Cannot read image: {frame_path}")
                continue

            print(f"Annotating: {frame_name}")
            result = annotate_frame(image, frame_name)

            if result is None:
                print(f"  Skipped.")
                continue

            x, y = result
            w.writerow([frame_name, f"{x:.1f}", f"{y:.1f}"])
            f.flush()
            print(f"  Saved ({x:.1f}, {y:.1f})")
            saved += 1

    print(f"\nDone. {saved} annotation(s) written to {out_path}")


if __name__ == "__main__":
    main()
