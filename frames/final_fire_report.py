"""
final_fire_report.py

Purpose
-------
Generate ONE final, human-readable report that:
  1) Reads per-frame fire intersections (frames/fire_intersections.csv)
  2) Reads per-frame pose quality and centerline direction (frames/frame_poses.csv)
  3) Automatically rejects bad frames (pose flips, unrealistic ranges, poor fit)
  4) Fuses remaining frames into ONE final fire-origin coordinate (lat/lon)
  5) Computes an empirical confidence radius (meters) and a conservative radius

Key behaviour
-------------
- Pose-flip detection:
    Uses pose centerline bearing/elevation clustering to identify frames whose pose
    is inconsistent with the main cluster (e.g., sudden 180°+ flips).
- Range filtering:
    Removes intersections that are unrealistically close (often caused by pose flip) or beyond max range.
- Fusion:
    Weighted fusion in local ENU coordinates, with weights based on inliers and reprojection RMS.

Outputs
-------
- frames/fire_final_report.txt   (single final file; easy to read)
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

WGS84_R = 6378137.0  # meters


# ----------------------------
# Basic geometry helpers
# ----------------------------

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = phi2 - phi1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(math.sqrt(a))


def wgs84_to_enu(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """Approximate WGS84 -> local ENU (East, North) meters relative to (lat0, lon0)."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    lat0r = math.radians(lat0)
    east = dlon * WGS84_R * math.cos(lat0r)
    north = dlat * WGS84_R
    return east, north


def enu_to_wgs84(e: float, n: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """Approximate local ENU (East, North) meters -> WGS84 lat/lon relative to (lat0, lon0)."""
    lat0r = math.radians(lat0)
    lat = lat0 + math.degrees(n / WGS84_R)
    lon = lon0 + math.degrees(e / (WGS84_R * math.cos(lat0r)))
    return lat, lon


def wrap360(a: float) -> float:
    return a % 360.0


def ang_diff_deg(a: float, b: float) -> float:
    """Smallest signed difference (a-b) in degrees in [-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


def circular_mean_deg(angles: List[float]) -> Optional[float]:
    if not angles:
        return None
    s = sum(math.sin(math.radians(a)) for a in angles)
    c = sum(math.cos(math.radians(a)) for a in angles)
    if abs(s) < 1e-12 and abs(c) < 1e-12:
        return None
    return wrap360(math.degrees(math.atan2(s, c)))


def parse_frame_index(frame_name: str) -> Optional[int]:
    m = re.search(r"_(\d{4})\.png$", frame_name)
    return int(m.group(1)) if m else None


# ----------------------------
# Data models
# ----------------------------

@dataclass
class PoseRow:
    frame: str
    n_gcps: int
    n_inliers: int
    reproj_rms_px: float
    bearing_center_deg: float
    elev_center_deg: float


@dataclass
class FireRow:
    frame: str
    fire_x: float
    fire_y: float
    bearing_deg: float
    elev_deg: float
    range_m: float
    lat: float
    lon: float
    terrain_alt_m: float
    reproj_rms_px: float
    n_inliers: int


# ----------------------------
# CSV reading
# ----------------------------

def read_pose_csv(path: Path) -> Dict[str, PoseRow]:
    out: Dict[str, PoseRow] = {}
    with path.open("r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            frame = (row.get("frame") or "").strip()
            if not frame:
                continue
            out[frame] = PoseRow(
                frame=frame,
                n_gcps=int(float(row["n_gcps"])),
                n_inliers=int(float(row["n_inliers"])),
                reproj_rms_px=float(row["reproj_rms_px"]),
                bearing_center_deg=float(row["bearing_center_deg"]),
                elev_center_deg=float(row["elev_center_deg"]),
            )
    return out


def read_fire_intersections_csv(path: Path) -> Dict[str, FireRow]:
    out: Dict[str, FireRow] = {}
    with path.open("r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            frame = (row.get("frame") or "").strip()
            if not frame:
                continue

            # lat/lon may be blank for DEM-less runs
            lat_s = (row.get("lat") or "").strip()
            lon_s = (row.get("lon") or "").strip()
            range_s = (row.get("range_m") or "").strip()
            terr_s = (row.get("terrain_alt_m") or "").strip()

            # If any are blank, skip (cannot fuse without coordinates)
            if lat_s == "" or lon_s == "" or range_s == "" or terr_s == "":
                continue

            out[frame] = FireRow(
                frame=frame,
                fire_x=float(row["fire_x"]),
                fire_y=float(row["fire_y"]),
                bearing_deg=float(row["bearing_deg"]),
                elev_deg=float(row["elev_deg"]),
                range_m=float(range_s),
                lat=float(lat_s),
                lon=float(lon_s),
                terrain_alt_m=float(terr_s),
                reproj_rms_px=float(row["reproj_rms_px"]),
                n_inliers=int(float(row["n_inliers"])),
            )
    return out


# ----------------------------
# Filtering logic
# ----------------------------

def robust_pose_reference(poses: List[PoseRow], bearing_keep_deg: float) -> Tuple[Optional[float], Optional[float]]:
    """
    Compute a robust reference (bearing_center, elev_center) from pose rows.
    - Circular mean for bearing (with one trimming pass)
    - Median for elevation on the trimmed set
    """
    if not poses:
        return None, None

    bearings = [p.bearing_center_deg for p in poses]
    mean0 = circular_mean_deg(bearings)
    if mean0 is None:
        return None, None

    trimmed = [p for p in poses if abs(ang_diff_deg(p.bearing_center_deg, mean0)) <= bearing_keep_deg]
    if len(trimmed) < 2:
        trimmed = poses

    mean1 = circular_mean_deg([p.bearing_center_deg for p in trimmed])
    elev_med = float(np.median([p.elev_center_deg for p in trimmed])) if trimmed else None
    return mean1, elev_med


def decide_frames(
    fire: Dict[str, FireRow],
    pose: Dict[str, PoseRow],
    min_range_m: float,
    max_range_m: float,
    max_reproj_px: float,
    pose_bearing_thresh_deg: float,
    pose_elev_thresh_deg: float,
) -> Tuple[List[FireRow], Dict[str, List[str]], Tuple[Optional[float], Optional[float]]]:
    """
    Returns:
      - accepted fire rows
      - decisions: frame -> list of rejection reasons (empty list means accepted)
      - (pose_bearing_ref, pose_elev_ref)
    """
    frames = sorted(fire.keys(), key=lambda fr: parse_frame_index(fr) if parse_frame_index(fr) is not None else 10**9)

    # Build pose list for reference
    pose_rows = [pose[fr] for fr in frames if fr in pose]
    pose_b_ref, pose_e_ref = robust_pose_reference(pose_rows, bearing_keep_deg=90.0)

    decisions: Dict[str, List[str]] = {}
    accepted: List[FireRow] = []

    for fr in frames:
        reasons: List[str] = []
        f = fire[fr]

        # Range checks (also catches many "pose-flip" intersections near the camera)
        if not (min_range_m <= f.range_m <= max_range_m):
            if f.range_m < min_range_m:
                reasons.append("range_too_small")
            else:
                reasons.append("range_too_large")

        # Pose availability / pose-flip checks
        if fr not in pose:
            reasons.append("missing_pose")
        else:
            p = pose[fr]
            if pose_b_ref is not None:
                if abs(ang_diff_deg(p.bearing_center_deg, pose_b_ref)) > pose_bearing_thresh_deg:
                    reasons.append("pose_flip_bearing")
            if pose_e_ref is not None:
                if abs(p.elev_center_deg - pose_e_ref) > pose_elev_thresh_deg:
                    reasons.append("pose_flip_elevation")

        # Fit quality check
        if f.reproj_rms_px > max_reproj_px:
            reasons.append("high_reproj_error")

        decisions[fr] = reasons
        if not reasons:
            accepted.append(f)

    return accepted, decisions, (pose_b_ref, pose_e_ref)


# ----------------------------
# Fusion logic
# ----------------------------

def fuse_points(
    rows: List[FireRow],
    camera_lat: float,
    camera_lon: float
) -> Tuple[float, float, Dict[str, float]]:
    """
    Weighted fusion in ENU, then convert back to lat/lon.
    weights = n_inliers / (reproj_rms_px^2)
    Returns (lat, lon, stats dict).
    """
    if len(rows) == 0:
        raise ValueError("No accepted frames to fuse.")

    weights = []
    EN = []
    for r in rows:
        w = r.n_inliers / (r.reproj_rms_px * r.reproj_rms_px + 1e-9)
        e, n = wgs84_to_enu(r.lat, r.lon, camera_lat, camera_lon)
        weights.append(w)
        EN.append((e, n))

    weights_np = np.array(weights, dtype=np.float64)
    EN_np = np.array(EN, dtype=np.float64)

    e_mean = float(np.sum(EN_np[:, 0] * weights_np) / np.sum(weights_np))
    n_mean = float(np.sum(EN_np[:, 1] * weights_np) / np.sum(weights_np))
    lat_f, lon_f = enu_to_wgs84(e_mean, n_mean, camera_lat, camera_lon)

    dists = [haversine_m(r.lat, r.lon, lat_f, lon_f) for r in rows]
    d_np = np.array(dists, dtype=np.float64)

    stats = {
        "n_used": float(len(rows)),
        "max_dev_m": float(np.max(d_np)) if len(d_np) else float("nan"),
        "rms_dev_m": float(np.sqrt(np.mean(d_np ** 2))) if len(d_np) else float("nan"),
        "p95_dev_m": float(np.percentile(d_np, 95)) if len(d_np) else float("nan"),
    }
    return lat_f, lon_f, stats


# ----------------------------
# Reporting
# ----------------------------

def write_report(
    out_path: Path,
    camera_lat: float,
    camera_lon: float,
    accepted: List[FireRow],
    decisions: Dict[str, List[str]],
    pose_ref: Tuple[Optional[float], Optional[float]],
    final_lat: Optional[float],
    final_lon: Optional[float],
    stats: Optional[Dict[str, float]],
    target_radius_m: float
) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)

    frames_sorted = sorted(decisions.keys(), key=lambda fr: parse_frame_index(fr) if parse_frame_index(fr) is not None else 10**9)
    used_frames = [r.frame for r in accepted]
    rejected_frames = [fr for fr in frames_sorted if fr not in used_frames]

    pose_b_ref, pose_e_ref = pose_ref

    with out_path.open("w", encoding="utf-8") as f:
        f.write("FINAL FIRE ORIGIN ESTIMATE REPORT\n")
        f.write("================================\n\n")

        f.write("Camera Reference\n")
        f.write("----------------\n")
        f.write(f"Camera lat/lon: {camera_lat:.6f}, {camera_lon:.6f}\n\n")

        f.write("Pose Reference (for flip detection)\n")
        f.write("----------------------------------\n")
        f.write(f"Reference centerline bearing (deg): {pose_b_ref:.3f}\n" if pose_b_ref is not None else "Reference bearing: n/a\n")
        f.write(f"Reference centerline elevation (deg): {pose_e_ref:.3f}\n\n" if pose_e_ref is not None else "Reference elevation: n/a\n\n")

        f.write("Frame Filtering Summary\n")
        f.write("-----------------------\n")
        f.write(f"Accepted frames: {len(used_frames)}\n")
        f.write(f"Rejected frames: {len(rejected_frames)}\n\n")

        f.write("Accepted Frames (used for fusion)\n")
        f.write("--------------------------------\n")
        if not accepted:
            f.write("None\n\n")
        else:
            f.write("frame, range_m, fire_bearing_deg, fire_elev_deg, lat, lon, reproj_rms_px, n_inliers\n")
            for r in accepted:
                f.write(
                    f"{r.frame}, {r.range_m:.2f}, {r.bearing_deg:.6f}, {r.elev_deg:.6f}, "
                    f"{r.lat:.8f}, {r.lon:.8f}, {r.reproj_rms_px:.3f}, {r.n_inliers}\n"
                )
            f.write("\n")

        f.write("Rejected Frames (with reasons)\n")
        f.write("------------------------------\n")
        if not rejected_frames:
            f.write("None\n\n")
        else:
            f.write("frame, reasons\n")
            for fr in rejected_frames:
                rs = decisions.get(fr, [])
                f.write(f"{fr}, {', '.join(rs) if rs else 'n/a'}\n")
            f.write("\n")

        f.write("Final Output\n")
        f.write("------------\n")
        if final_lat is None or final_lon is None or stats is None:
            f.write("Final coordinate could not be produced (no accepted frames).\n")
        else:
            f.write(f"Final fire origin (lat, lon): {final_lat:.8f}, {final_lon:.8f}\n")
            f.write(f"Frames used: {', '.join(used_frames)}\n")
            f.write(f"Empirical max deviation (m): {stats['max_dev_m']:.2f}\n")
            f.write(f"Empirical RMS deviation (m): {stats['rms_dev_m']:.2f}\n")
            f.write(f"Empirical 95th percentile deviation (m): {stats['p95_dev_m']:.2f}\n")
            f.write(f"Conservative radius (m): {max(stats['max_dev_m'], target_radius_m):.2f}\n")
        f.write("\n")

        f.write("Notes\n")
        f.write("-----\n")
        f.write("- 'pose_flip_bearing'/'pose_flip_elevation' indicate the estimated camera pose is inconsistent with the\n")
        f.write("  main pose cluster and is likely caused by tracking drift or incorrect correspondences.\n")
        f.write("- 'range_too_small' typically indicates a failure mode where the ray intersects terrain extremely near\n")
        f.write("  the camera due to a bad pose estimate.\n")
        f.write("- To improve stability, add an additional manual seed GCP file near late frames and re-run tracking.\n")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-lat", type=float, required=True)
    ap.add_argument("--camera-lon", type=float, required=True)

    ap.add_argument("--fire", type=str, default="frames/fire_intersections.csv")
    ap.add_argument("--poses", type=str, default="frames/frame_poses.csv")

    ap.add_argument("--out", type=str, default="frames/fire_final_report.txt")

    ap.add_argument("--min-range", type=float, default=200.0)     # meters
    ap.add_argument("--max-range", type=float, default=20000.0)   # meters (20 km requirement)
    ap.add_argument("--max-reproj", type=float, default=4.0)      # pixels

    ap.add_argument("--pose-bear-thresh", type=float, default=90.0)  # degrees
    ap.add_argument("--pose-elev-thresh", type=float, default=20.0)  # degrees

    ap.add_argument("--target-radius", type=float, default=500.0)    # meters
    args = ap.parse_args()

    fire_path = Path(args.fire)
    pose_path = Path(args.poses)
    out_path = Path(args.out)

    if not fire_path.exists():
        raise FileNotFoundError(f"Missing fire intersections CSV: {fire_path}")
    if not pose_path.exists():
        raise FileNotFoundError(f"Missing pose CSV: {pose_path}")

    fire = read_fire_intersections_csv(fire_path)
    pose = read_pose_csv(pose_path)

    accepted, decisions, pose_ref = decide_frames(
        fire=fire,
        pose=pose,
        min_range_m=args.min_range,
        max_range_m=args.max_range,
        max_reproj_px=args.max_reproj,
        pose_bearing_thresh_deg=args.pose_bear_thresh,
        pose_elev_thresh_deg=args.pose_elev_thresh,
    )

    final_lat = final_lon = None
    stats = None
    if len(accepted) >= 2:
        final_lat, final_lon, stats = fuse_points(accepted, args.camera_lat, args.camera_lon)
    elif len(accepted) == 1:
        # If only one frame is valid, use it directly (no fusion possible).
        final_lat, final_lon = accepted[0].lat, accepted[0].lon
        stats = {"n_used": 1.0, "max_dev_m": 0.0, "rms_dev_m": 0.0, "p95_dev_m": 0.0}

    write_report(
        out_path=out_path,
        camera_lat=args.camera_lat,
        camera_lon=args.camera_lon,
        accepted=accepted,
        decisions=decisions,
        pose_ref=pose_ref,
        final_lat=final_lat,
        final_lon=final_lon,
        stats=stats,
        target_radius_m=args.target_radius,
    )

    print(f"Wrote final report: {out_path}")
    if final_lat is not None and final_lon is not None:
        print(f"Final fire origin (lat, lon): {final_lat:.8f}, {final_lon:.8f}")


if __name__ == "__main__":
    main()