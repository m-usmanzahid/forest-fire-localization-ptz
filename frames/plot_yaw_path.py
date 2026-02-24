from __future__ import annotations

import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    csv_path = root / "frames" / "yaw_path.csv"
    if not csv_path.exists():
        raise SystemExit(f"Missing {csv_path}")

    xs = []
    ys = []
    frames = []
    yaws = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            frames.append(int(row["frame"]))
            xs.append(float(row["x_km"]))
            ys.append(float(row["y_km"]))
            yaws.append(math.radians(float(row["yaw_deg"])))

    fig, ax = plt.subplots()
    ax.plot(xs, ys, "-o", linewidth=2, markersize=4)
    for f, x, y in zip(frames, xs, ys):
        ax.text(x, y, str(f), fontsize=8, ha="left", va="bottom")

    ax.scatter([0], [0], c="red", s=40, label="Tower (camera)")
    # Draw short viewing rays from each camera position toward the scene center.
    ray_len_km = 2.5
    for x, y, yaw in zip(xs, ys, yaws):
        dx = -ray_len_km * math.cos(yaw)
        dy = -ray_len_km * math.sin(yaw)
        ax.plot([x, x + dx], [y, y + dy], color="gray", alpha=0.6, linewidth=1)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x (km)")
    ax.set_ylabel("y (km)")
    ax.set_title("Estimated Camera Path (Satellite View)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend()

    plt.show()


if __name__ == "__main__":
    main()
