from __future__ import annotations

import csv
import math
from pathlib import Path


def load_landmarks(csv_path: Path) -> list[tuple[int, float]]:
    rows = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            name = row["frame"].strip()
            x = float(row["x"])
            frame_idx = int(name.split("_")[-1].split(".")[0])
            rows.append((frame_idx, x))
    return sorted(rows, key=lambda item: item[0])


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    frames_dir = root / "frames"
    landmarks_csv = frames_dir / "landmarks.csv"

    if not landmarks_csv.exists():
        raise SystemExit(f"Missing {landmarks_csv}")

    # Assumed camera parameters.
    width_px = 640.0
    hfov_deg = 60.0
    radius_km = 15.0

    landmarks = load_landmarks(landmarks_csv)
    if not landmarks:
        raise SystemExit("No landmarks found in CSV.")

    cx = width_px / 2.0
    hfov_rad = math.radians(hfov_deg)

    # Convert x offsets to bearing offsets.
    frames = []
    for frame_idx, x in landmarks:
        offset_px = x - cx
        bearing_offset = (offset_px / width_px) * hfov_rad
        frames.append((frame_idx, bearing_offset))

    ref_frame, ref_offset = frames[0]
    rel = [(idx, -(offset - ref_offset)) for idx, offset in frames]

    # Fit a line yaw = a * frame + b using least squares.
    n = len(rel)
    sum_x = sum(idx for idx, _ in rel)
    sum_y = sum(yaw for _, yaw in rel)
    sum_xx = sum(idx * idx for idx, _ in rel)
    sum_xy = sum(idx * yaw for idx, yaw in rel)
    denom = n * sum_xx - sum_x * sum_x
    if denom == 0:
        raise SystemExit("Not enough distinct frames to fit yaw.")

    a = (n * sum_xy - sum_x * sum_y) / denom
    b = (sum_y - a * sum_x) / n

    out_csv = frames_dir / "yaw_path.csv"
    with out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["frame", "yaw_deg", "x_km", "y_km"])
        # Reverse mapping so frame1 uses the last yaw and frame15 uses the first.
        for frame_idx in range(1, 16):
            src_idx = 16 - frame_idx
            yaw = a * src_idx + b
            yaw_deg = math.degrees(yaw)
            x_km = radius_km * math.cos(yaw)
            y_km = radius_km * math.sin(yaw)
            writer.writerow([frame_idx, f"{yaw_deg:.3f}", f"{x_km:.3f}", f"{y_km:.3f}"])

    print(f"Saved path to {out_csv}")


if __name__ == "__main__":
    main()
