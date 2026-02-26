"""
localize_fire.py

Purpose
-------
Estimate smoke/fire origin coordinates (lat/lon) from a fixed PTZ camera using:
  1) Camera location + altitude
  2) Ground Control Points (GCPs) with known lat/lon/alt and clicked pixel locations
  3) Fire origin pixel clicks (e.g., bottom of smoke plume)
  4) Optional DEM GeoTIFF (for terrain intersection)

What this script does
---------------------
A) Per-frame camera pose estimation (rotation) using solvePnPRansac
   - Converts GCPs from (lat,lon,alt) into local ENU coordinates (meters) centered at the camera.
   - Builds camera intrinsics K from a calibrated HFOV + image size.
   - Fits camera pose for each frame that has enough GCPs.

B) Fire ray construction
   - Converts fire pixel (x,y) into a 3D viewing ray in camera coordinates.
   - Rotates it into world ENU coordinates using the estimated pose.
   - Computes bearing (azimuth) and elevation angle of the fire ray.

C) Optional DEM intersection (ray ∩ terrain)
   - If a DEM GeoTIFF is provided, marches along the ray and finds where it intersects terrain.
   - Outputs estimated fire origin latitude/longitude and range from camera.

Important design choices
------------------------
1) Fixed camera origin (prevents drift):
   - solvePnP estimates translation (tvec), but we already know the camera’s physical location.
   - For ray intersection we FORCE the camera origin to ENU=(0,0,0) to avoid translation drift due to annotation noise.
   - We still use solvePnP for rotation (yaw/pitch/roll), which is what determines ray direction.

2) Intrinsics improvement for vertical accuracy:
   - By default, many pipelines assume square pixels (fy = fx).
   - For range accuracy, vertical geometry (fy) matters a lot. Some video streams are vertically cropped/rescaled.
   - This script supports an OPTIONAL --vfov to compute fy from VFOV if you know/estimate it.
   - If --vfov is not provided, fy=fx is used (same behavior as before).

3) 20 km operational constraint:
   - Default max range is 20 km (can be overridden with --max-range).

Input CSV formats
-----------------
GCP CSV (one or more files):
  frame,id,x,y,lat,lon,alt_m

Fire pixel CSV:
  frame,id,x,y,lat,lon,alt_m
where id is typically "fire_origin" and lat/lon/alt can be blank.

Outputs
-------
1) frames/frame_poses.csv
   frame,n_gcps,n_inliers,reproj_rms_px,bearing_center_deg,elev_center_deg

2) frames/fire_intersections.csv
   frame,fire_x,fire_y,bearing_deg,elev_deg,range_m,lat,lon,terrain_alt_m,reproj_rms_px,n_inliers

Notes
-----
- If --dem is NOT provided, this script still outputs bearing/elevation angle, but lat/lon/range will be blank.
- Lens distortion is ignored in this version.
- ENU conversion uses a local approximation (sufficient for ~tens of km).

Example (PowerShell)
--------------------
Without DEM (angles only):
python frames\\localize_fire.py `
  --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 `
  --hfov 52.760104 --img-w 640 --img-h 480 `
  --gcp frames\\gcp_frame4_0009.csv frames\\gcp_frame4_0011.csv frames\\gcp_frame4_tracked.csv `
  --fire frames\\fire_pixels.csv

With DEM (full coordinates):
python frames\\localize_fire.py `
  --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 `
  --hfov 52.760104 --img-w 640 --img-h 480 `
  --gcp frames\\gcp_frame4_0009.csv frames\\gcp_frame4_0011.csv frames\\gcp_frame4_tracked.csv `
  --fire frames\\fire_pixels.csv `
  --dem dem\\oghi_dem.tif

With optional VFOV (if you want to tune vertical geometry):
python frames\\localize_fire.py `
  --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 `
  --hfov 52.760104 --vfov 42.0 --img-w 640 --img-h 480 `
  --gcp frames\\gcp_frame4_0009.csv frames\\gcp_frame4_0011.csv frames\\gcp_frame4_tracked.csv `
  --fire frames\\fire_pixels.csv `
  --dem dem\\oghi_dem.tif
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import cv2

# Optional dependency for DEM support. Script still runs without DEM if --dem is omitted.
try:
    import rasterio
    from rasterio.warp import transform as rio_transform
except Exception:
    rasterio = None
    rio_transform = None


# Spherical Earth radius approximation for local ENU conversion
WGS84_R = 6378137.0  # meters


# --------------------------------------------------------------------------------------
# Coordinate conversion helpers (WGS84 <-> local ENU)
# --------------------------------------------------------------------------------------

def wgs84_to_enu(lat: float, lon: float, alt: float,
                 lat0: float, lon0: float, alt0: float) -> np.ndarray:
    """
    Approximate conversion from WGS84 (lat, lon, alt) to local ENU coordinates (meters)
    relative to reference point (lat0, lon0, alt0).

    ENU axes:
      x = East
      y = North
      z = Up
    """
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    lat0r = math.radians(lat0)

    east = dlon * WGS84_R * math.cos(lat0r)
    north = dlat * WGS84_R
    up = alt - alt0
    return np.array([east, north, up], dtype=np.float64)


def enu_to_wgs84(e: float, n: float, u: float,
                 lat0: float, lon0: float, alt0: float) -> Tuple[float, float, float]:
    """
    Approximate conversion from local ENU (meters) back to WGS84 (lat, lon, alt)
    relative to reference point (lat0, lon0, alt0).
    """
    lat0r = math.radians(lat0)
    lat = lat0 + math.degrees(n / WGS84_R)
    lon = lon0 + math.degrees(e / (WGS84_R * math.cos(lat0r)))
    alt = alt0 + u
    return lat, lon, alt


# --------------------------------------------------------------------------------------
# Camera model helpers
# --------------------------------------------------------------------------------------

def build_K_from_fov(hfov_deg: float, w: int, h: int, vfov_deg: Optional[float] = None) -> np.ndarray:
    """
    Build camera intrinsics matrix K from FOV and image size.

    Inputs
    ------
    hfov_deg : Horizontal field of view (degrees).
    vfov_deg : Optional Vertical field of view (degrees).
               - If provided -> fy computed from vfov
               - If omitted  -> fy = fx (square pixel assumption)

    Returns
    -------
    K (3x3) camera intrinsics matrix.
    """
    hfov = math.radians(hfov_deg)
    fx = (w / 2.0) / math.tan(hfov / 2.0)

    if vfov_deg is None:
        fy = fx
    else:
        vfov = math.radians(vfov_deg)
        fy = (h / 2.0) / math.tan(vfov / 2.0)

    cx = w / 2.0
    cy = h / 2.0

    return np.array([
        [fx, 0.0, cx],
        [0.0, fy, cy],
        [0.0, 0.0, 1.0]
    ], dtype=np.float64)


def ray_from_pixel(u: float, v: float, K: np.ndarray) -> np.ndarray:
    """
    Convert image pixel (u,v) to a unit ray direction in camera coordinates.
    """
    invK = np.linalg.inv(K)
    d = invK @ np.array([u, v, 1.0], dtype=np.float64)
    d /= (np.linalg.norm(d) + 1e-12)
    return d


def bearing_elev_from_dir_enu(d_enu: np.ndarray) -> Tuple[float, float]:
    """
    Compute real-world bearing (deg clockwise from North) and elevation angle (deg)
    from an ENU direction vector.
    """
    e, n, u = float(d_enu[0]), float(d_enu[1]), float(d_enu[2])
    bearing = (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
    horiz = math.hypot(e, n)
    elev = math.degrees(math.atan2(u, horiz))
    return bearing, elev


# --------------------------------------------------------------------------------------
# PnP pose estimation
# --------------------------------------------------------------------------------------

@dataclass
class PoseResult:
    frame: str
    rvec: np.ndarray        # world->camera rotation vector (OpenCV)
    tvec: np.ndarray        # world->camera translation vector (OpenCV)
    n_gcps: int
    n_inliers: int
    reproj_rms_px: float
    bearing_center_deg: float
    elev_center_deg: float


def solve_pose_pnp(frame: str, obj_enu: np.ndarray, img_xy: np.ndarray, K: np.ndarray) -> PoseResult:
    """
    Estimate camera pose (rotation + translation) with solvePnPRansac.

    Notes
    -----
    - OpenCV pose maps world -> camera:
        X_cam = R * X_world + t
    - We mainly trust R (rotation) for ray direction.
    - We do NOT trust t for camera location in this project because camera position is already known.
    """
    objp = obj_enu.astype(np.float32)
    imgp = img_xy.astype(np.float32)

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=objp,
        imagePoints=imgp,
        cameraMatrix=K,
        distCoeffs=None,
        flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=6.0,
        iterationsCount=300,
        confidence=0.99
    )

    if not ok or rvec is None or tvec is None:
        raise RuntimeError(
            f"solvePnPRansac failed for frame '{frame}'. "
            f"Add more/better GCPs (6-12 stable points recommended)."
        )

    inlier_idx = inliers.flatten().tolist() if inliers is not None else list(range(len(objp)))
    obj_in = objp[inlier_idx]
    img_in = imgp[inlier_idx]

    # Compute reprojection RMS on inliers
    proj, _ = cv2.projectPoints(obj_in, rvec, tvec, K, None)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - img_in, axis=1)
    rms = float(np.sqrt(np.mean(err ** 2))) if len(err) > 0 else float("nan")

    # Camera centerline direction in world ENU for reporting:
    R_w2c, _ = cv2.Rodrigues(rvec)
    R_c2w = R_w2c.T
    forward_world = R_c2w @ np.array([0.0, 0.0, 1.0], dtype=np.float64)
    bearing_center, elev_center = bearing_elev_from_dir_enu(forward_world)

    return PoseResult(
        frame=frame,
        rvec=rvec.reshape(3, 1),
        tvec=tvec.reshape(3, 1),
        n_gcps=len(objp),
        n_inliers=len(inlier_idx),
        reproj_rms_px=rms,
        bearing_center_deg=bearing_center,
        elev_center_deg=elev_center,
    )


# --------------------------------------------------------------------------------------
# DEM helpers (optional)
# --------------------------------------------------------------------------------------

def dem_sample(ds, lat: float, lon: float) -> Optional[float]:
    """
    Sample DEM elevation at (lat, lon). Returns None if out-of-bounds or nodata.
    """
    if ds is None:
        return None

    # Convert WGS84 lat/lon to DEM CRS coordinates
    if ds.crs is None:
        # Best effort fallback: assume DEM is already EPSG:4326
        x, y = lon, lat
    else:
        epsg = ds.crs.to_epsg()
        if epsg == 4326:
            x, y = lon, lat
        else:
            if rio_transform is None:
                raise RuntimeError("rasterio.warp.transform is unavailable. Install rasterio properly.")
            xs, ys = rio_transform("EPSG:4326", ds.crs, [lon], [lat])
            x, y = xs[0], ys[0]

    try:
        vals = list(ds.sample([(x, y)]))
        if not vals:
            return None
        z = float(vals[0][0])

        if ds.nodata is not None and z == float(ds.nodata):
            return None
        if z < -1000:
            return None

        return z
    except Exception:
        return None


def intersect_ray_with_dem(
    ds,
    lat0: float, lon0: float, alt0: float,
    C_enu: np.ndarray, d_enu: np.ndarray,
    step_m: float = 25.0,
    max_range_m: float = 20000.0,
) -> Optional[Tuple[float, float, float, float]]:
    """
    March along the ray and find the first terrain intersection.

    Returns:
      (lat_hit, lon_hit, terrain_alt_m, range_m) or None
    """
    d = d_enu / (np.linalg.norm(d_enu) + 1e-12)

    prev_diff = None
    prev_s = None

    s = 0.0
    while s <= max_range_m:
        P = C_enu + s * d
        lat, lon, alt_ray = enu_to_wgs84(float(P[0]), float(P[1]), float(P[2]), lat0, lon0, alt0)

        z_terrain = dem_sample(ds, lat, lon)
        if z_terrain is None:
            s += step_m
            continue

        diff = alt_ray - z_terrain

        if prev_diff is not None and prev_diff > 0.0 and diff <= 0.0:
            t = prev_diff / (prev_diff - diff + 1e-12)
            s_hit = float(prev_s + t * (s - prev_s))
            P_hit = C_enu + s_hit * d
            lat_h, lon_h, alt_h = enu_to_wgs84(float(P_hit[0]), float(P_hit[1]), float(P_hit[2]), lat0, lon0, alt0)

            z_h = dem_sample(ds, lat_h, lon_h)
            if z_h is None:
                z_h = alt_h

            return lat_h, lon_h, float(z_h), s_hit

        prev_diff = diff
        prev_s = s
        s += step_m

    return None


# --------------------------------------------------------------------------------------
# CSV readers / data cleaning
# --------------------------------------------------------------------------------------

def _safe_float_field(row: dict, key: str) -> Optional[float]:
    s = (row.get(key) or "").strip()
    if s == "":
        return None
    try:
        return float(s)
    except ValueError:
        return None


def read_gcps(csv_path: Path) -> Dict[str, List[dict]]:
    """
    Read a GCP CSV and return:
      { frame_name: [ {id,x,y,lat,lon,alt_m}, ... ] }

    Rules
    -----
    - Requires lat/lon/alt_m to be present (3D GCPs only).
    - De-duplicates by (frame, id) inside each file (keeps first).
    """
    per_frame: Dict[str, List[dict]] = {}
    seen = set()

    with csv_path.open("r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            frame = (row.get("frame") or "").strip()
            gid = (row.get("id") or "").strip()
            if not frame or not gid:
                continue

            key = (frame, gid)
            if key in seen:
                continue
            seen.add(key)

            x = _safe_float_field(row, "x")
            y = _safe_float_field(row, "y")
            lat = _safe_float_field(row, "lat")
            lon = _safe_float_field(row, "lon")
            alt_m = _safe_float_field(row, "alt_m")

            if None in (x, y, lat, lon, alt_m):
                continue

            per_frame.setdefault(frame, []).append({
                "id": gid,
                "x": float(x),
                "y": float(y),
                "lat": float(lat),
                "lon": float(lon),
                "alt_m": float(alt_m),
            })

    return per_frame


def merge_gcp_sources(gcp_files: List[Path]) -> Dict[str, List[dict]]:
    """
    Merge multiple GCP CSV files into one per-frame mapping.

    De-duplication policy across files:
    - Key = (frame, id)
    - First occurrence wins (usually manual GCP file should come before tracked file)
    """
    merged: Dict[str, List[dict]] = {}
    seen = set()

    for p in gcp_files:
        per = read_gcps(p)
        for frame, rows in per.items():
            for g in rows:
                key = (frame, g["id"])
                if key in seen:
                    continue
                seen.add(key)
                merged.setdefault(frame, []).append(g)

    return merged


def read_fire_pixels(fire_csv: Path, fire_id: str = "fire_origin") -> Dict[str, Tuple[float, float]]:
    """
    Read fire pixel CSV and return:
      { frame_name: (x, y) }

    If multiple rows with same (frame, fire_id) exist, the last one is used.
    """
    out: Dict[str, Tuple[float, float]] = {}

    with fire_csv.open("r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            if (row.get("id") or "").strip() != fire_id:
                continue
            frame = (row.get("frame") or "").strip()
            x = _safe_float_field(row, "x")
            y = _safe_float_field(row, "y")
            if not frame or x is None or y is None:
                continue
            out[frame] = (float(x), float(y))

    return out


# --------------------------------------------------------------------------------------
# Main pipeline
# --------------------------------------------------------------------------------------

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-lat", type=float, required=True, help="Camera latitude (degrees).")
    ap.add_argument("--camera-lon", type=float, required=True, help="Camera longitude (degrees).")
    ap.add_argument("--camera-alt", type=float, required=True, help="Camera altitude above sea level (meters).")

    ap.add_argument("--hfov", type=float, required=True, help="Calibrated horizontal FOV (degrees).")
    ap.add_argument(
        "--vfov",
        type=float,
        default=None,
        help="Optional vertical FOV (degrees). If omitted, fy=fx is assumed."
    )

    ap.add_argument("--img-w", type=int, default=640, help="Image width in pixels.")
    ap.add_argument("--img-h", type=int, default=480, help="Image height in pixels.")

    ap.add_argument("--gcp", nargs="+", required=True, help="One or more GCP CSV files.")
    ap.add_argument("--fire", type=str, required=True, help="Fire pixel CSV (contains id=fire_origin rows).")
    ap.add_argument("--fire-id", type=str, default="fire_origin", help="Fire pixel ID to use (default: fire_origin).")

    ap.add_argument("--dem", type=str, default=None, help="Optional DEM GeoTIFF path for terrain intersection.")
    ap.add_argument("--dem-step", type=float, default=25.0, help="Ray marching step size in meters (default: 25).")

    # 20 km operational constraint by default (override if needed)
    ap.add_argument("--max-range", type=float, default=20000.0, help="Max ray range (meters) for DEM intersection (default: 20000).")

    ap.add_argument("--min-gcps", type=int, default=6, help="Minimum GCPs required per frame for PnP (default: 6).")
    ap.add_argument("--poses-out", type=str, default="frames/frame_poses.csv")
    ap.add_argument("--fires-out", type=str, default="frames/fire_intersections.csv")
    args = ap.parse_args()

    lat0 = args.camera_lat
    lon0 = args.camera_lon
    alt0 = args.camera_alt

    K = build_K_from_fov(args.hfov, args.img_w, args.img_h, vfov_deg=args.vfov)

    # Load and merge GCP sources (manual first, tracked last recommended)
    gcp_paths = [Path(p) for p in args.gcp]
    all_gcps = merge_gcp_sources(gcp_paths)

    # Load fire pixels
    fire_pixels = read_fire_pixels(Path(args.fire), fire_id=args.fire_id)

    # Open DEM if provided
    ds = None
    if args.dem:
        if rasterio is None:
            raise RuntimeError("rasterio is required for DEM support. Install with: pip install rasterio")
        ds = rasterio.open(args.dem)

    poses_out = Path(args.poses_out)
    fires_out = Path(args.fires_out)
    poses_out.parent.mkdir(parents=True, exist_ok=True)
    fires_out.parent.mkdir(parents=True, exist_ok=True)

    pose_rows: List[List[str]] = []
    fire_rows: List[List[str]] = []

    # Process each frame that has GCPs
    for frame in sorted(all_gcps.keys()):
        gcps = all_gcps[frame]

        if len(gcps) < args.min_gcps:
            continue

        # Build 3D ENU points + 2D image points
        obj_pts = []
        img_pts = []
        for g in gcps:
            obj_pts.append(wgs84_to_enu(g["lat"], g["lon"], g["alt_m"], lat0, lon0, alt0))
            img_pts.append([g["x"], g["y"]])

        obj_pts_np = np.vstack(obj_pts)
        img_pts_np = np.vstack(img_pts)

        # Estimate pose
        try:
            pose = solve_pose_pnp(frame, obj_pts_np, img_pts_np, K)
        except Exception as e:
            print(f"[WARN] {e}")
            continue

        pose_rows.append([
            pose.frame,
            str(pose.n_gcps),
            str(pose.n_inliers),
            f"{pose.reproj_rms_px:.3f}",
            f"{pose.bearing_center_deg:.3f}",
            f"{pose.elev_center_deg:.3f}",
        ])

        # If no fire pixel in this frame, skip localization for this frame
        if frame not in fire_pixels:
            continue

        fire_x, fire_y = fire_pixels[frame]

        # Pixel -> camera ray
        d_cam = ray_from_pixel(fire_x, fire_y, K)

        # Rotation world->camera (OpenCV), then invert to camera->world
        R_w2c, _ = cv2.Rodrigues(pose.rvec)
        R_c2w = R_w2c.T

        # Force camera origin to known physical location => ENU (0,0,0)
        C_world = np.zeros(3, dtype=np.float64)

        # Rotate ray into world ENU
        d_world = (R_c2w @ d_cam.reshape(3)).reshape(3)
        d_world /= (np.linalg.norm(d_world) + 1e-12)

        bearing_deg, elev_deg = bearing_elev_from_dir_enu(d_world)

        # If no DEM provided, output angles only
        if ds is None:
            fire_rows.append([
                frame,
                f"{fire_x:.2f}",
                f"{fire_y:.2f}",
                f"{bearing_deg:.6f}",
                f"{elev_deg:.6f}",
                "", "", "", "",
                f"{pose.reproj_rms_px:.3f}",
                str(pose.n_inliers),
            ])
            continue

        # DEM intersection
        hit = intersect_ray_with_dem(
            ds=ds,
            lat0=lat0, lon0=lon0, alt0=alt0,
            C_enu=C_world,
            d_enu=d_world,
            step_m=args.dem_step,
            max_range_m=args.max_range
        )

        if hit is None:
            fire_rows.append([
                frame,
                f"{fire_x:.2f}",
                f"{fire_y:.2f}",
                f"{bearing_deg:.6f}",
                f"{elev_deg:.6f}",
                "", "", "", "",
                f"{pose.reproj_rms_px:.3f}",
                str(pose.n_inliers),
            ])
        else:
            lat_h, lon_h, terrain_alt_m, range_m = hit
            fire_rows.append([
                frame,
                f"{fire_x:.2f}",
                f"{fire_y:.2f}",
                f"{bearing_deg:.6f}",
                f"{elev_deg:.6f}",
                f"{range_m:.2f}",
                f"{lat_h:.8f}",
                f"{lon_h:.8f}",
                f"{terrain_alt_m:.2f}",
                f"{pose.reproj_rms_px:.3f}",
                str(pose.n_inliers),
            ])

    if ds is not None:
        ds.close()

    # Write outputs
    with poses_out.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "n_gcps", "n_inliers", "reproj_rms_px", "bearing_center_deg", "elev_center_deg"])
        w.writerows(pose_rows)

    with fires_out.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "frame", "fire_x", "fire_y", "bearing_deg", "elev_deg",
            "range_m", "lat", "lon", "terrain_alt_m",
            "reproj_rms_px", "n_inliers"
        ])
        w.writerows(fire_rows)

    print(f"Wrote poses: {poses_out}")
    print(f"Wrote fire intersections: {fires_out}")

    if ds is None:
        print("DEM not provided: outputs include ray bearing/elevation only (lat/lon/range are blank).")
    else:
        print("DEM provided: lat/lon/range populated when ray-terrain intersection is found.")


if __name__ == "__main__":
    main()