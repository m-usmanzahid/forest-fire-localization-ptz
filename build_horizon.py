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
    """Load DEM into a numpy array. Returns (data, transform)."""
    ds = rasterio.open(dem_path)
    data = ds.read(1).astype(np.float64)
    transform = ds.transform
    nodata = ds.nodata
    ds.close()

    if nodata is not None:
        data[data == nodata] = np.nan
    data[data < -500] = np.nan
    return data, transform


def sample_dem_bilinear(data: np.ndarray, transform,
                        lats: np.ndarray, lons: np.ndarray) -> np.ndarray:
    """
    Bilinear interpolation of DEM values at given lat/lon arrays.

    Reduces quantisation noise from nearest-neighbour by ~10x on typical
    30 m DEMs. Falls back to nearest-neighbour where any of the four
    surrounding cells is NaN or out of bounds.
    """
    cols = (lons - transform.c) / transform.a
    rows = (lats - transform.f) / transform.e

    H_d, W_d = data.shape
    c0 = np.floor(cols).astype(np.int64)
    r0 = np.floor(rows).astype(np.int64)
    c1 = c0 + 1
    r1 = r0 + 1
    fc = (cols - c0).astype(np.float64)
    fr = (rows - r0).astype(np.float64)

    in_bounds = (c0 >= 0) & (c1 < W_d) & (r0 >= 0) & (r1 < H_d)
    result = np.full(lats.shape, np.nan, dtype=np.float64)

    # Clamp indices so array access is always safe; mask filters results later
    c0c = np.clip(c0, 0, W_d - 2)
    r0c = np.clip(r0, 0, H_d - 2)
    c1c = c0c + 1
    r1c = r0c + 1

    v00 = data[r0c, c0c]
    v01 = data[r0c, c1c]
    v10 = data[r1c, c0c]
    v11 = data[r1c, c1c]

    interp = (v00 * (1.0 - fc) * (1.0 - fr) +
              v01 * fc          * (1.0 - fr) +
              v10 * (1.0 - fc) * fr          +
              v11 * fc          * fr)

    # Where any bilinear neighbour is NaN, fall back to nearest-neighbour
    has_nan = ~(np.isfinite(v00) & np.isfinite(v01) &
                np.isfinite(v10) & np.isfinite(v11))
    nn_mask = in_bounds & has_nan
    if nn_mask.any():
        ci_nn = np.round(cols[nn_mask]).astype(np.int64).clip(0, W_d - 1)
        ri_nn = np.round(rows[nn_mask]).astype(np.int64).clip(0, H_d - 1)
        interp[nn_mask] = data[ri_nn, ci_nn]

    result[in_bounds] = interp[in_bounds]
    return result


# ---------------------------------------------------------------------------
# Camera altitude from DEM
# ---------------------------------------------------------------------------

def get_camera_alt(data: np.ndarray, transform,
                   cam_lat: float, cam_lon: float,
                   tower_height: float) -> float:
    ground_elev = sample_dem_bilinear(data, transform,
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
    For each azimuth compute the maximum terrain elevation angle visible from
    the camera. Uses bilinear DEM interpolation for smoother profiles.

    Returns (azimuths, max_elevs) in degrees.
    """
    azimuths  = np.arange(0.0, 360.0, az_step_deg)
    distances = np.arange(range_step_m, max_range_m + range_step_m, range_step_m)

    cos_lat0 = math.cos(math.radians(cam_lat))
    az_rad   = np.radians(azimuths)
    de = np.sin(az_rad)
    dn = np.cos(az_rad)

    E = np.outer(de, distances)   # (A, D)
    N = np.outer(dn, distances)   # (A, D)

    lat_all = cam_lat  + np.degrees(N / WGS84_R)
    lon_all = cam_lon  + np.degrees(E / (WGS84_R * cos_lat0))

    A, D = lat_all.shape
    alt_all = sample_dem_bilinear(
        dem_data, dem_transform,
        lat_all.ravel(), lon_all.ravel()
    ).reshape(A, D)

    # Elevation angles from camera to each terrain point
    elev_all = np.degrees(
        np.arctan2(alt_all - cam_alt, distances[np.newaxis, :])
    )

    max_elevs = np.nanmax(elev_all, axis=1)
    return azimuths, max_elevs


# ---------------------------------------------------------------------------
# Importable run() entry point
# ---------------------------------------------------------------------------

def run(
    camera_lat: float,
    camera_lon: float,
    tower_height: float,
    dem: str,
    out: str,
    az_step: float = 0.1,
    range_step: float = 50.0,
    max_range: float = 40000.0,
) -> str:
    """
    Build and save a horizon profile CSV. Returns the output path.
    Importable by pipeline.py.
    """
    print(f"Loading DEM: {dem}")
    dem_data, dem_transform = load_dem(dem)
    cam_alt = get_camera_alt(dem_data, dem_transform,
                             camera_lat, camera_lon, tower_height)

    n_az = int(360.0 / az_step)
    n_d  = int(max_range / range_step)
    print(f"\nComputing horizon profile ({n_az} azimuths × {n_d} distances) ...")

    azimuths, max_elevs = compute_horizon_profile(
        cam_lat=camera_lat,
        cam_lon=camera_lon,
        cam_alt=cam_alt,
        dem_data=dem_data,
        dem_transform=dem_transform,
        az_step_deg=az_step,
        range_step_m=range_step,
        max_range_m=max_range,
    )

    out_path = Path(out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["azimuth_deg", "horizon_elev_deg"])
        for az, elev in zip(azimuths, max_elevs):
            w.writerow([f"{az:.2f}", f"{elev:.4f}"])

    print(f"Wrote horizon profile -> {out_path}  ({len(azimuths)} rows)")
    return str(out_path)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Build a synthetic horizon elevation profile from a DEM."
    )
    ap.add_argument("--camera-lat",   type=float, required=True)
    ap.add_argument("--camera-lon",   type=float, required=True)
    ap.add_argument("--tower-height", type=float, default=17.5)
    ap.add_argument("--dem",          type=str,   required=True)
    ap.add_argument("--az-step",      type=float, default=0.1)
    ap.add_argument("--range-step",   type=float, default=50.0)
    ap.add_argument("--max-range",    type=float, default=40000.0)
    ap.add_argument("--out",          type=str,   default="horizon_profile.csv")
    args = ap.parse_args()

    run(
        camera_lat=args.camera_lat,
        camera_lon=args.camera_lon,
        tower_height=args.tower_height,
        dem=args.dem,
        out=args.out,
        az_step=args.az_step,
        range_step=args.range_step,
        max_range=args.max_range,
    )


if __name__ == "__main__":
    main()
