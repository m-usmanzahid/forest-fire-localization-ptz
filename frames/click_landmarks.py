from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.image as mpimg


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    frames_dir = root / "frames"
    out_csv = frames_dir / "landmarks.csv"

    # First five frame4 images.
    frame_paths = [
        frames_dir / f"frame4_frame_{idx:04d}.png" for idx in range(1, 6)
    ]

    rows = ["frame,x,y"]
    for frame_path in frame_paths:
        if not frame_path.exists():
            raise SystemExit(f"Missing frame: {frame_path}")

        img = mpimg.imread(frame_path)
        fig, ax = plt.subplots()
        ax.imshow(img)
        ax.set_title(
            f"Click the same landmark in {frame_path.name} (close window after click)"
        )

        # Wait for a single click.
        clicked = plt.ginput(1, timeout=-1)
        plt.close(fig)
        if not clicked:
            raise SystemExit("No click captured; aborting.")

        x, y = clicked[0]
        rows.append(f"{frame_path.name},{x:.2f},{y:.2f}")

    out_csv.write_text("\n".join(rows), encoding="utf-8")
    print(f"Saved {len(frame_paths)} points to {out_csv}")


if __name__ == "__main__":
    main()
