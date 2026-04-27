"""
localize_fire.py

Estimate the GPS coordinates of a fire / smoke origin from a single fixed PTZ camera.

This script is the final step in the pipeline. It uses:
  - Camera GPS coordinates + tower height  (known)
  - Camera calibration (heading, tilt, HFOV)  from calibrate_camera.py
  - Fire origin pixel coordinates             from annotate_fire.py
  - DEM GeoTIFF                               for ray-terrain intersection

It does NOT require solvePnP or any Ground Control Points. The rotation matrix
is built directly from the calibrated heading and tilt.

Camera-to-world rotation
------------------------
Given heading H (clockwise from North, degrees) and tilt T (positive = looking up, degrees):

  Camera axes in world ENU (East-North-Up):
    X_cam (right)   = [ cos(H),         -sin(H),          0        ]
    Y_cam (down)    = [ sin(H)*sin(T),   cos(H)*sin(T),  -cos(T)   ]
    Z_cam (forward) = [ sin(H)*cos(T),   cos(H)*cos(T),   sin(T)   ]

  R_c2w = [X_cam | Y_cam | Z_cam]   (columns)

Usage:
    python localize_fire.py ^
        --camera-lat 34.534508 --camera-lon 73.003801 ^
        --tower-height 17.5 ^
        --calibration calibration.json ^
        --fire fire_pixels.csv ^
        --dem dem/oghi_dem.tif ^
        --out output/fire_locations.csv

Without DEM (bearing and elevation angle only):
    python localize_fire.py ^
        --camera-lat 34.534508 --camera-lon 73.003801 ^
        --tower-height 17.5 ^
        --calibration calibration.json ^
        --fire fire_pixels.csv

Output:
    output/fire_locations.csv
      frame, fire_x, fire_y, bearing_deg, elev_deg, range_m, lat, lon, terrain_alt_m
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

try:
    import rasterio
except ImportError:
    rasterio = None

WGS84_R = 6378137.0  # metres


# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def wgs84_to_enu(lat, lon, alt, lat0, lon0, alt0) -> np.ndarray:
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    lat0r = math.radians(lat0)
    east  = dlon * WGS84_R * math.cos(lat0r)
    north = dlat * WGS84_R
    up    = alt - alt0
    return np.array([east, north, up], dtype=np.float64)


def enu_to_wgs84(e, n, u, lat0, lon0, alt0) -> Tuple[float, float, float]:
    lat0r = math.radians(lat0)
    lat = lat0 + math.degrees(n / WGS84_R)
    lon = lon0 + math.degrees(e / (WGS84_R * math.cos(lat0r)))
    alt = alt0 + u
    return lat, lon, alt


# ---------------------------------------------------------------------------
# Camera model
# ---------------------------------------------------------------------------

def build_rotation_matrix(heading_deg: float, tilt_deg: float) -> np.ndarray:
    """
    Build the camera-to-world rotation matrix R_c2w from heading and tilt.

    R_c2w maps a vector in camera coordinates to world ENU coordinates:
        v_world = R_c2w @ v_camera

    Camera convention: X=right, Y=down, Z=forward (OpenCV).
    World convention:  X=East,  Y=North, Z=Up     (ENU).

    Verified:
        H=0, T=0  → Z_cam = [0,1,0] = North  ✓
                  → X_cam = [1,0,0] = East   ✓
                  → Y_cam = [0,0,-1]= Down   ✓
    """
    H = math.radians(heading_deg)
    T = math.radians(tilt_deg)

    cH, sH = math.cos(H), math.sin(H)
    cT, sT = math.cos(T), math.sin(T)

    # Columns = [X_cam_in_world | Y_cam_in_world | Z_cam_in_world]
    R = np.array([
        [ cH,  sH * sT,  sH * cT],
        [-sH,  cH * sT,  cH * cT],
        [ 0,  -cT,        sT     ],
    ], dtype=np.float64)

    return R


def build_K(hfov_deg: float, W: int, H: int) -> np.ndarray:
    hfov = math.radians(hfov_deg)
    fx = (W / 2.0) / math.tan(hfov / 2.0)
    return np.array([
        [fx,  0.0, W / 2.0],
        [0.0, fx,  H / 2.0],
        [0.0, 0.0, 1.0    ],
    ], dtype=np.float64)


def pixel_to_world_ray(u: float, v: float, K: np.ndarray, R_c2w: np.ndarray) -> np.ndarray:
    """
    Unproject pixel (u, v) through camera intrinsics K and rotate into world ENU.
    Returns a unit direction vector in ENU.
    """
    K_inv = np.linalg.inv(K)
    d_cam = K_inv @ np.array([u, v, 1.0])
    d_cam /= np.linalg.norm(d_cam)
    d_world = R_c2w @ d_cam
    d_world /= np.linalg.norm(d_world)
    return d_world


def bearing_elev_from_enu(d: np.ndarray) -> Tuple[float, float]:
    """Bearing (deg, clockwise from North) and elevation angle (deg) from ENU direction."""
    e, n, u = float(d[0]), float(d[1]), float(d[2])
    bearing = (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
    horiz   = math.hypot(e, n)
    elev    = math.degrees(math.atan2(u, horiz))
    return bearing, elev


# ---------------------------------------------------------------------------
# DEM intersection
# ---------------------------------------------------------------------------

def load_dem_to_memory(dem_path: str):
    """Load DEM into a numpy array for fast in-memory lookups."""
    ds = rasterio.open(dem_path)
    data = ds.read(1).astype(np.float64)
    transform = ds.transform
    nodata = ds.nodata
    ds.close()
    if nodata is not None:
        data[data == nodata] = np.nan
    data[data < -500] = np.nan
    return data, transform


def sample_dem_point(data: np.ndarray, transform, lat: float, lon: float) -> Optional[float]:
    col = (lon - transform.c) / transform.a
    row = (lat - transform.f) / transform.e
    ci, ri = int(round(col)), int(round(row))
    H, W = data.shape
    if ci < 0 or ri < 0 or ci >= W or ri >= H:
        return None
    v = data[ri, ci]
    return None if np.isnan(v) else float(v)


def get_camera_alt_from_dem(data, transform, cam_lat, cam_lon, tower_height) -> float:
    ground = sample_dem_point(data, transform, cam_lat, cam_lon)
    if ground is None:
        raise ValueError("Could not sample DEM at camera position. "
                         "Check DEM coverage or provide --camera-alt directly.")
    return ground + tower_height


def intersect_ray_dem(
    data, transform,
    lat0, lon0, alt0,
    d_enu: np.ndarray,
    step_m: float = 25.0,
    max_range_m: float = 40000.0,
) -> Optional[Tuple[float, float, float, float]]:
    """
    March along ray P(s) = camera_ENU + s * d_enu, find first terrain crossing.

    Returns (lat, lon, terrain_alt_m, range_m) or None.
    """
    d = d_enu / (np.linalg.norm(d_enu) + 1e-12)
    C = np.zeros(3)  # camera at ENU origin by design

    prev_diff = None
    prev_s    = None
    s = step_m

    while s <= max_range_m:
        P = C + s * d
        lat, lon, alt_ray = enu_to_wgs84(P[0], P[1], P[2], lat0, lon0, alt0)
        z_terrain = sample_dem_point(data, transform, lat, lon)

        if z_terrain is not None:
            diff = alt_ray - z_terrain  # positive = ray above terrain
            if prev_diff is not None and prev_diff > 0.0 and diff <= 0.0:
                # Crossed terrain — linear interpolation within this segment
                t_frac = prev_diff / (prev_diff - diff + 1e-12)
                s_hit  = prev_s + t_frac * (s - prev_s)
                P_hit  = C + s_hit * d
                lat_h, lon_h, _ = enu_to_wgs84(P_hit[0], P_hit[1], P_hit[2], lat0, lon0, alt0)
                z_h = sample_dem_point(data, transform, lat_h, lon_h) or (alt0 + P_hit[2])
                return lat_h, lon_h, z_h, s_hit
            prev_diff = diff
            prev_s    = s

        s += step_m

    return None


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def read_fire_pixels(csv_path: Path) -> dict[str, Tuple[float, float]]:
    """Returns {frame_name: (x, y)}."""
    out = {}
    with csv_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            frame = (row.get("frame") or "").strip()
            try:
                x = float(row["x"])
                y = float(row["y"])
            except (KeyError, ValueError):
                continue
            if frame:
                out[frame] = (x, y)
    return out


def read_frame_headings(csv_path: Path) -> dict[str, float]:
    """Returns {frame_name: heading_deg} from track_heading.py output."""
    out = {}
    with csv_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            frame = (row.get("frame") or "").strip()
            try:
                heading = float(row["heading_deg"])
            except (KeyError, ValueError):
                continue
            if frame:
                out[frame] = heading
    return out


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Localize fire origin from calibration + fire pixel + DEM."
    )
    ap.add_argument("--camera-lat",   type=float, required=True)
    ap.add_argument("--camera-lon",   type=float, required=True)
    ap.add_argument("--tower-height", type=float, default=17.5,
                    help="Tower height in metres (default: 17.5). Used to compute camera altitude from DEM.")
    ap.add_argument("--camera-alt",   type=float, default=None,
                    help="Camera altitude (m ASL). If provided, overrides --tower-height + DEM lookup.")
    ap.add_argument("--calibration",  required=True,
                    help="calibration.json from calibrate_camera.py.")
    ap.add_argument("--fire",         required=True,
                    help="fire_pixels.csv from annotate_fire.py.")
    ap.add_argument("--img-w",        type=int, default=640)
    ap.add_argument("--img-h",        type=int, default=480)
    ap.add_argument("--dem",          type=str, default=None,
                    help="DEM GeoTIFF path (optional). Without it, only bearing/elevation is output.")
    ap.add_argument("--dem-step",      type=float, default=25.0,
                    help="Ray-march step in metres (default: 25).")
    ap.add_argument("--max-range",     type=float, default=40000.0,
                    help="Max ray range in metres (default: 40000).")
    ap.add_argument("--tilt-override",    type=float, default=None,
                    help="Override the tilt from calibration.json (degrees). Useful for tuning.")
    ap.add_argument("--frame-headings",   type=str, default=None,
                    help="Per-frame headings CSV from track_heading.py. "
                         "If provided, overrides the single calibration heading per frame.")
    ap.add_argument("--out",              type=str, default="output/fire_locations.csv")
    args = ap.parse_args()

    # --- Load calibration ---
    with open(args.calibration, "r", encoding="utf-8") as f:
        cal = json.load(f)
    heading = float(cal["heading_deg"])
    tilt    = float(cal["tilt_deg"]) if args.tilt_override is None else args.tilt_override
    hfov    = float(cal["hfov_deg"])
    if args.tilt_override is not None:
        print(f"Tilt override: {tilt:.3f}° (calibration value was {cal['tilt_deg']:.3f}°)")
    print(f"Calibration: heading={heading:.3f}°  tilt={tilt:.3f}°  HFOV={hfov:.3f}°")

    # --- Load fire pixels ---
    fire_pixels = read_fire_pixels(Path(args.fire))
    print(f"Fire pixels loaded: {len(fire_pixels)} frame(s)")

    # --- Load per-frame headings (optional) ---
    frame_headings = {}
    if args.frame_headings:
        frame_headings = read_frame_headings(Path(args.frame_headings))
        print(f"Per-frame headings loaded: {len(frame_headings)} frame(s)")

    # --- Camera model (K is shared; R is built per-frame if headings provided) ---
    K    = build_K(hfov, args.img_w, args.img_h)

    lat0 = args.camera_lat
    lon0 = args.camera_lon

    # --- DEM ---
    dem_data = dem_transform = None
    if args.dem:
        if rasterio is None:
            raise RuntimeError("rasterio is required for DEM support. Run: pip install rasterio")
        print(f"Loading DEM: {args.dem}")
        dem_data, dem_transform = load_dem_to_memory(args.dem)

    # --- Camera altitude ---
    if args.camera_alt is not None:
        alt0 = args.camera_alt
        print(f"Camera altitude (provided): {alt0:.2f} m")
    elif dem_data is not None:
        alt0 = get_camera_alt_from_dem(dem_data, dem_transform, lat0, lon0, args.tower_height)
        print(f"Camera altitude (DEM + tower): {alt0:.2f} m")
    else:
        raise ValueError(
            "Cannot determine camera altitude: provide --camera-alt or --dem (to read elevation from DEM)."
        )

    # --- Process each fire pixel ---
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rows = []
    for frame, (fire_x, fire_y) in sorted(fire_pixels.items()):
        # Use per-frame heading if available, otherwise fall back to calibration heading
        if frame in frame_headings:
            frame_heading = frame_headings[frame]
        else:
            frame_heading = heading

        R_c2w = build_rotation_matrix(frame_heading, tilt)

        # Pixel → world ray
        d_world = pixel_to_world_ray(fire_x, fire_y, K, R_c2w)
        bearing, elev = bearing_elev_from_enu(d_world)

        row = {
            "frame":       frame,
            "fire_x":      f"{fire_x:.1f}",
            "fire_y":      f"{fire_y:.1f}",
            "bearing_deg": f"{bearing:.4f}",
            "elev_deg":    f"{elev:.4f}",
            "range_m":     "",
            "lat":         "",
            "lon":         "",
            "terrain_alt_m": "",
        }

        if dem_data is not None:
            hit = intersect_ray_dem(
                dem_data, dem_transform,
                lat0, lon0, alt0,
                d_world,
                step_m=args.dem_step,
                max_range_m=args.max_range,
            )
            if hit is not None:
                lat_h, lon_h, z_h, range_m = hit
                row["range_m"]       = f"{range_m:.1f}"
                row["lat"]           = f"{lat_h:.8f}"
                row["lon"]           = f"{lon_h:.8f}"
                row["terrain_alt_m"] = f"{z_h:.2f}"
                print(f"  {frame}: bearing={bearing:.2f}° elev={elev:.2f}° "
                      f"range={range_m/1000:.2f} km  → ({lat_h:.5f}, {lon_h:.5f})")
            else:
                print(f"  {frame}: bearing={bearing:.2f}° elev={elev:.2f}°  "
                      f"[no terrain intersection within {args.max_range/1000:.0f} km]")
        else:
            print(f"  {frame}: bearing={bearing:.2f}°  elev={elev:.2f}°  (no DEM)")

        rows.append(row)

    fields = ["frame", "fire_x", "fire_y", "bearing_deg", "elev_deg",
              "range_m", "lat", "lon", "terrain_alt_m"]

    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    print(f"\nWrote {len(rows)} result(s) → {out_path}")
    if dem_data is None:
        print("Note: DEM not provided — lat/lon/range are blank. "
              "Add --dem dem/oghi_dem.tif for full coordinates.")


if __name__ == "__main__":
    main()
