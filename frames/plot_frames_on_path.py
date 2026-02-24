from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.offsetbox import AnnotationBbox, OffsetImage


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    frames_dir = root / "frames"
    path_csv = frames_dir / "yaw_path.csv"

    if not path_csv.exists():
        raise SystemExit(f"Missing {path_csv}")

    xs = []
    ys = []
    frames = []
    with path_csv.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            frames.append(int(row["frame"]))
            xs.append(float(row["x_km"]))
            ys.append(float(row["y_km"]))

    fig, ax = plt.subplots()
    ax.plot(xs, ys, "-o", linewidth=1, markersize=2, alpha=0.4)
    ax.scatter([0], [0], c="red", s=30, label="Tower (camera)")

    # Place each frame as a thumbnail near its path position.
    zoom = 0.18
    for frame_idx, x, y in zip(frames, xs, ys):
        img_path = frames_dir / f"frame4_frame_{frame_idx:04d}.png"
        if not img_path.exists():
            continue
        img = mpimg.imread(img_path)
        imagebox = OffsetImage(img, zoom=zoom)
        ab = AnnotationBbox(
            imagebox,
            (x, y),
            frameon=True,
            bboxprops={"edgecolor": "white", "linewidth": 1},
        )
        ax.add_artist(ab)

    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (km)")
    ax.set_ylabel("y (km)")
    ax.set_title("Frames Placed Along Estimated Path (Top-Down Schematic)")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend()

    plt.show()


if __name__ == "__main__":
    main()
