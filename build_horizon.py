"""
build_horizon.py

Build a synthetic horizon elevation profile from the camera's known GPS position and a DEM.

For every azimuth angle (0–360°) it casts a ray outward and records the maximum terrain
elevation angle visible from the camera. The result is saved as horizon_profile.csv and is
used by calibrate_camera.py to automatically determine which direction the camera is pointing.

This script only needs to be run ONCE per camera location. The result can be reused across
all frames from that camera.

Camera altitude is computed automatically as:
    camera_alt = DEM elevation at camera GPS + tower height

Usage:
    python build_horizon.py ^
        --camera-lat 34.534508 --camera-lon 73.003801 ^
        --tower-height 17.5 ^
        --dem dem/oghi_dem.tif ^
        --out horizon_profile.csv

Output:
    horizon_profile.csv  —  columns: azimuth_deg, horizon_elev_deg
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np
import rasterio

WGS84_R = 6378137.0  # metres


# ---------------------------------------------------------------------------
# DEM helpers
# ---------------------------------------------------------------------------

def load_dem(dem_path: str):
    """
    Load DEM into a numpy array. Returns (data, transform, nodata).
    Assumes DEM is in EPSG:4326 (geographic lat/lon), which is true for
    SRTM and ALOS DEMs.
    """
    ds = rasterio.open(dem_path)
    data = ds.read(1).astype(np.float64)
    transform = ds.transform
    nodata = ds.nodata
    ds.close()

    if nodata is not None:
        data[data == nodata] = np.nan
    data[data < -500] = np.nan  # guard against common sentinel values

    return data, transform


def sample_dem_batch(data: np.ndarray, transform, lats: np.ndarray, lons: np.ndarray) -> np.ndarray:
    """
    Fast batch DEM lookup by converting lat/lon to row/col indices.
    Returns elevation array (same shape as lats/lons), NaN for out-of-bounds.
    """
    # affine transform: col = (lon - west) / pixel_width
    #                   row = (lat - north) / pixel_height  (pixel_height is negative)
    cols = (lons - transform.c) / transform.a
    rows = (lats - transform.f) / transform.e

    H, W = data.shape
    ci = np.round(cols).astype(np.int64)
    ri = np.round(rows).astype(np.int64)

    valid = (ci >= 0) & (ci < W) & (ri >= 0) & (ri < H)
    result = np.full(lats.shape, np.nan)
    result[valid] = data[ri[valid], ci[valid]]
    return result


# ---------------------------------------------------------------------------
# Camera altitude from DEM
# ---------------------------------------------------------------------------

def get_camera_alt(data: np.ndarray, transform, cam_lat: float, cam_lon: float, tower_height: float) -> float:
    ground_elev = sample_dem_batch(data, transform,
                                   np.array([cam_lat]),
                                   np.array([cam_lon]))[0]
    if np.isnan(ground_elev):
        raise ValueError(
            "Could not sample DEM at camera coordinates. "
            "Check that the DEM covers the camera location."
        )
    camera_alt = float(ground_elev) + tower_height
    print(f"Ground elevation at camera: {ground_elev:.1f} m")
    print(f"Tower height: {tower_height:.1f} m")
    print(f"Camera optical centre altitude: {camera_alt:.2f} m")
    return camera_alt


# ---------------------------------------------------------------------------
# Horizon profile computation
# ---------------------------------------------------------------------------

def compute_horizon_profile(
    cam_lat: float,
    cam_lon: float,
    cam_alt: float,
    dem_data: np.ndarray,
    dem_transform,
    az_step_deg: float = 0.1,
    range_step_m: float = 50.0,
    max_range_m: float = 40000.0,
) -> tuple[np.ndarray, np.ndarray]:
    """
    For each azimuth compute the maximum terrain elevation angle visible from the camera.

    Uses fully vectorised numpy operations — all distances for each azimuth are
    evaluated simultaneously, so this typically completes in under 60 seconds.

    Returns
    -------
    azimuths  : 1-D array of azimuth values (degrees)
    max_elevs : 1-D array of horizon elevation angles (degrees, can be negative)
    """
    azimuths = np.arange(0.0, 360.0, az_step_deg)
    distances = np.arange(range_step_m, max_range_m + range_step_m, range_step_m)

    cos_lat0 = math.cos(math.radians(cam_lat))

    az_rad = np.radians(azimuths)
    de = np.sin(az_rad)   # East  component per unit distance, shape (A,)
    dn = np.cos(az_rad)   # North component per unit distance, shape (A,)

    # All (azimuth × distance) east / north offsets in metres — shapes (A, D)
    E = np.outer(de, distances)
    N = np.outer(dn, distances)

    # Convert ENU offsets to lat/lon
    lat_all = cam_lat + np.degrees(N / WGS84_R)                       # (A, D)
    lon_all = cam_lon + np.degrees(E / (WGS84_R * cos_lat0))          # (A, D)

    A, D = lat_all.shape

    # Batch sample DEM — flatten → sample → reshape
    alt_all = sample_dem_batch(
        dem_data, dem_transform,
        lat_all.ravel(), lon_all.ravel()
    ).reshape(A, D)

    # Elevation angles from camera to each terrain point
    elev_all = np.degrees(
        np.arctan2(alt_all - cam_alt, distances[np.newaxis, :])
    )

    # Maximum elevation angle per azimuth (the horizon)
    max_elevs = np.nanmax(elev_all, axis=1)

    return azimuths, max_elevs


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Build a synthetic horizon elevation profile from a DEM."
    )
    ap.add_argument("--camera-lat",   type=float, required=True, help="Camera latitude (degrees).")
    ap.add_argument("--camera-lon",   type=float, required=True, help="Camera longitude (degrees).")
    ap.add_argument("--tower-height", type=float, default=17.5,
                    help="Tower / mast height in metres (default: 17.5).")
    ap.add_argument("--dem",          type=str,   required=True, help="DEM GeoTIFF path.")
    ap.add_argument("--az-step",      type=float, default=0.1,
                    help="Azimuth resolution in degrees (default: 0.1).")
    ap.add_argument("--range-step",   type=float, default=50.0,
                    help="Ray-march step in metres (default: 50).")
    ap.add_argument("--max-range",    type=float, default=40000.0,
                    help="Maximum ray range in metres (default: 40000).")
    ap.add_argument("--out",          type=str,   default="horizon_profile.csv",
                    help="Output CSV path (default: horizon_profile.csv).")
    args = ap.parse_args()

    print(f"Loading DEM: {args.dem}")
    dem_data, dem_transform = load_dem(args.dem)

    cam_alt = get_camera_alt(dem_data, dem_transform,
                             args.camera_lat, args.camera_lon,
                             args.tower_height)

    print(f"\nComputing horizon profile ({360.0 / args.az_step:.0f} azimuths × "
          f"{args.max_range / args.range_step:.0f} distances) ...")

    azimuths, max_elevs = compute_horizon_profile(
        cam_lat=args.camera_lat,
        cam_lon=args.camera_lon,
        cam_alt=cam_alt,
        dem_data=dem_data,
        dem_transform=dem_transform,
        az_step_deg=args.az_step,
        range_step_m=args.range_step,
        max_range_m=args.max_range,
    )

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["azimuth_deg", "horizon_elev_deg"])
        for az, elev in zip(azimuths, max_elevs):
            w.writerow([f"{az:.2f}", f"{elev:.4f}"])

    print(f"Wrote horizon profile → {out_path}  ({len(azimuths)} rows)")


if __name__ == "__main__":
    main()
