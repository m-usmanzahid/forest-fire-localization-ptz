"""
localize_fire.py

Estimate the GPS coordinates of a fire / smoke origin from a single fixed PTZ camera.

Improvements over v1:
  - Bilinear DEM interpolation  (replaces nearest-neighbour, reduces ±15 m noise)
  - Earth-curvature correction  (fixes systematic range overestimate at > 5 km)
  - Monte Carlo uncertainty      (500-sample propagation → confidence radius output)

Usage:
    python localize_fire.py ^
        --camera-lat 34.534508 --camera-lon 73.003801 ^
        --tower-height 17.5 ^
        --calibration calibration.json ^
        --fire fire_pixels.csv ^
        --dem dem/oghi_dem.tif ^
        --out output/fire_locations.csv

Output columns:
    frame, fire_x, fire_y, bearing_deg, elev_deg,
    range_m, lat, lon, terrain_alt_m, method,
    mc_lat_std_m, mc_lon_std_m, mc_range_std_m, mc_conf90_m, mc_n_valid
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
    dlat  = math.radians(lat - lat0)
    dlon  = math.radians(lon - lon0)
    lat0r = math.radians(lat0)
    east  = dlon * WGS84_R * math.cos(lat0r)
    north = dlat * WGS84_R
    up    = alt - alt0
    return np.array([east, north, up], dtype=np.float64)


def enu_to_wgs84(e, n, u, lat0, lon0, alt0) -> Tuple[float, float, float]:
    lat0r = math.radians(lat0)
    lat   = lat0 + math.degrees(n / WGS84_R)
    lon   = lon0 + math.degrees(e / (WGS84_R * math.cos(lat0r)))
    alt   = alt0 + u
    return lat, lon, alt


# ---------------------------------------------------------------------------
# Camera model
# ---------------------------------------------------------------------------

def build_rotation_matrix(heading_deg: float, tilt_deg: float) -> np.ndarray:
    """
    Camera-to-world rotation matrix (ENU convention).
    Camera: X=right, Y=down, Z=forward.  World: X=East, Y=North, Z=Up.
    """
    H  = math.radians(heading_deg)
    T  = math.radians(tilt_deg)
    cH, sH = math.cos(H), math.sin(H)
    cT, sT = math.cos(T), math.sin(T)
    return np.array([
        [ cH,  sH * sT,  sH * cT],
        [-sH,  cH * sT,  cH * cT],
        [ 0,  -cT,        sT     ],
    ], dtype=np.float64)


def build_K(hfov_deg: float, W: int, H: int) -> np.ndarray:
    hfov = math.radians(hfov_deg)
    fx   = (W / 2.0) / math.tan(hfov / 2.0)
    return np.array([
        [fx,  0.0, W / 2.0],
        [0.0, fx,  H / 2.0],
        [0.0, 0.0, 1.0    ],
    ], dtype=np.float64)


def pixel_to_world_ray(u: float, v: float,
                       K: np.ndarray, R_c2w: np.ndarray) -> np.ndarray:
    K_inv  = np.linalg.inv(K)
    d_cam  = K_inv @ np.array([u, v, 1.0])
    d_cam /= np.linalg.norm(d_cam)
    d_world = R_c2w @ d_cam
    d_world /= np.linalg.norm(d_world)
    return d_world


def bearing_elev_from_enu(d: np.ndarray) -> Tuple[float, float]:
    e, n, u = float(d[0]), float(d[1]), float(d[2])
    bearing = (math.degrees(math.atan2(e, n)) + 360.0) % 360.0
    horiz   = math.hypot(e, n)
    elev    = math.degrees(math.atan2(u, horiz))
    return bearing, elev


# ---------------------------------------------------------------------------
# DEM helpers — bilinear interpolation
# ---------------------------------------------------------------------------

def load_dem_to_memory(dem_path: str):
    ds        = rasterio.open(dem_path)
    data      = ds.read(1).astype(np.float64)
    transform = ds.transform
    nodata    = ds.nodata
    ds.close()
    if nodata is not None:
        data[data == nodata] = np.nan
    data[data < -500] = np.nan
    return data, transform


def sample_dem_point(data: np.ndarray, transform,
                     lat: float, lon: float) -> Optional[float]:
    """
    Bilinear interpolation of a single DEM point. Falls back to
    nearest-neighbour at the DEM boundary or where any neighbour is NaN.
    """
    col = (lon - transform.c) / transform.a
    row = (lat - transform.f) / transform.e
    H_d, W_d = data.shape

    c0 = int(math.floor(col))
    r0 = int(math.floor(row))
    c1, r1 = c0 + 1, r0 + 1

    if c0 < 0 or r0 < 0 or c1 >= W_d or r1 >= H_d:
        ci = int(round(col))
        ri = int(round(row))
        if ci < 0 or ri < 0 or ci >= W_d or ri >= H_d:
            return None
        v = data[ri, ci]
        return None if np.isnan(v) else float(v)

    fc = col - c0
    fr = row - r0
    v00, v01 = data[r0, c0], data[r0, c1]
    v10, v11 = data[r1, c0], data[r1, c1]

    if not (np.isfinite(v00) and np.isfinite(v01) and
            np.isfinite(v10) and np.isfinite(v11)):
        ci = int(round(col))
        ri = int(round(row))
        v  = data[ri, ci]
        return None if np.isnan(v) else float(v)

    v = (v00 * (1 - fc) * (1 - fr) + v01 * fc * (1 - fr) +
         v10 * (1 - fc) * fr        + v11 * fc * fr)
    return float(v) if np.isfinite(v) else None


def get_camera_alt_from_dem(data, transform,
                             cam_lat, cam_lon, tower_height) -> float:
    ground = sample_dem_point(data, transform, cam_lat, cam_lon)
    if ground is None:
        raise ValueError("Could not sample DEM at camera position.")
    return ground + tower_height


# ---------------------------------------------------------------------------
# DEM ray intersection — with earth-curvature correction
# ---------------------------------------------------------------------------

def intersect_ray_dem(
    data, transform,
    lat0, lon0, alt0,
    d_enu: np.ndarray,
    step_m: float = 25.0,
    max_range_m: float = 40000.0,
    ridge_gap_limit: float = 200.0,
) -> Optional[Tuple[float, float, float, float, str]]:
    """
    March along ray P(s) = s * d_enu from the camera origin and find the first
    terrain crossing.

    Earth-curvature correction
    --------------------------
    In flat ENU, the ray appears to be s²/(2R) higher above the ellipsoid than
    it really is (the tangent plane diverges from the curved surface). This
    causes range overestimation. The fix: subtract s_horiz²/(2R) from the ray
    height before comparing to the DEM terrain height.

    Returns (lat, lon, terrain_alt_m, range_m, method) or None.
    method: "intersection" for a clean hit, "ridge_fallback" for a near-graze.
    """
    d      = d_enu / (np.linalg.norm(d_enu) + 1e-12)
    C      = np.zeros(3)
    d_horiz_sq = float(d[0] * d[0] + d[1] * d[1])  # precompute for curvature

    prev_diff = None
    prev_s    = None
    s         = step_m

    MIN_FALLBACK_RANGE = 1000.0
    min_gap   = float("inf")
    min_gap_s = step_m

    while s <= max_range_m:
        P = C + s * d
        lat, lon, alt_ray = enu_to_wgs84(P[0], P[1], P[2], lat0, lon0, alt0)

        # Earth-curvature correction: ray is effectively lower by s_h²/(2R)
        s_horiz_sq  = s * s * d_horiz_sq
        curvature_m = s_horiz_sq / (2.0 * WGS84_R)
        alt_ray_corrected = alt_ray - curvature_m

        z_terrain = sample_dem_point(data, transform, lat, lon)
        if z_terrain is not None:
            diff = alt_ray_corrected - z_terrain  # positive = ray above terrain

            if s >= MIN_FALLBACK_RANGE and diff < min_gap:
                min_gap   = diff
                min_gap_s = s

            if prev_diff is not None and prev_diff > 0.0 and diff <= 0.0:
                t_frac = prev_diff / (prev_diff - diff + 1e-12)
                s_hit  = prev_s + t_frac * (s - prev_s)
                P_hit  = C + s_hit * d
                lat_h, lon_h, _ = enu_to_wgs84(
                    P_hit[0], P_hit[1], P_hit[2], lat0, lon0, alt0)
                z_h = sample_dem_point(data, transform, lat_h, lon_h) or (
                    alt0 + P_hit[2])
                return lat_h, lon_h, z_h, s_hit, "intersection"

            prev_diff = diff
            prev_s    = s

        s += step_m

    if min_gap <= ridge_gap_limit:
        P_close = C + min_gap_s * d
        lat_c, lon_c, _ = enu_to_wgs84(
            P_close[0], P_close[1], P_close[2], lat0, lon0, alt0)
        z_c = sample_dem_point(data, transform, lat_c, lon_c) or (
            alt0 + P_close[2])
        return lat_c, lon_c, z_c, min_gap_s, "ridge_fallback"

    return None


# ---------------------------------------------------------------------------
# Monte Carlo uncertainty estimation
# ---------------------------------------------------------------------------

def monte_carlo_uncertainty(
    fire_x: float,
    fire_y: float,
    frame_heading: float,
    tilt: float,
    hfov: float,
    cost_px: float,
    dem_data: np.ndarray,
    dem_transform,
    lat0: float,
    lon0: float,
    alt0: float,
    img_w: int,
    img_h: int,
    n_samples: int = 500,
    fire_x_std: float = 3.0,
    fire_y_std: float = 5.0,
    dem_step: float = 25.0,
    max_range: float = 40000.0,
    ridge_gap_limit: float = 200.0,
    seed: int = 42,
) -> dict:
    """
    Propagate calibration and annotation uncertainty through the ray-march to
    produce a 90 % confidence radius for each fire location.

    Uncertainty sources modelled:
      - Heading:  N(0, cost_px / fx) degrees  — from calibration RMS cost
      - Tilt:     N(0, cost_px / fx) degrees  — same calibration cost
      - fire_x:   N(0, fire_x_std) pixels     — manual annotation
      - fire_y:   N(0, fire_y_std) pixels     — manual annotation (larger: more
                                                 consequential for range)
    """
    rng = np.random.default_rng(seed)
    fx  = (img_w / 2.0) / math.tan(math.radians(hfov / 2.0))

    sigma_heading = math.degrees(cost_px / fx)
    sigma_tilt    = math.degrees(cost_px / fx)

    lats, lons, ranges = [], [], []

    for _ in range(n_samples):
        p_heading = frame_heading + rng.normal(0.0, sigma_heading)
        p_tilt    = tilt          + rng.normal(0.0, sigma_tilt)
        p_fire_x  = fire_x        + rng.normal(0.0, fire_x_std)
        p_fire_y  = fire_y        + rng.normal(0.0, fire_y_std)

        K_p     = build_K(hfov, img_w, img_h)
        R_p     = build_rotation_matrix(p_heading, p_tilt)
        d_world = pixel_to_world_ray(p_fire_x, p_fire_y, K_p, R_p)

        hit = intersect_ray_dem(
            dem_data, dem_transform,
            lat0, lon0, alt0, d_world,
            step_m=dem_step,
            max_range_m=max_range,
            ridge_gap_limit=ridge_gap_limit,
        )
        if hit is not None:
            lat_h, lon_h, _, range_h, _ = hit
            lats.append(lat_h)
            lons.append(lon_h)
            ranges.append(range_h)

    n_valid = len(lats)
    empty   = {"mc_n_valid": n_valid, "mc_lat_std_m": None,
               "mc_lon_std_m": None, "mc_range_std_m": None,
               "mc_conf90_m": None}
    if n_valid < 10:
        return empty

    lats_a   = np.array(lats)
    lons_a   = np.array(lons)
    ranges_a = np.array(ranges)

    lat_std_m = float(np.std(lats_a))   * WGS84_R * math.pi / 180.0
    lon_std_m = float(np.std(lons_a))   * WGS84_R * math.cos(math.radians(lat0)) * math.pi / 180.0

    mean_lat  = float(np.mean(lats_a))
    mean_lon  = float(np.mean(lons_a))
    dlat_m    = (lats_a - mean_lat) * WGS84_R * math.pi / 180.0
    dlon_m    = (lons_a - mean_lon) * WGS84_R * math.cos(math.radians(lat0)) * math.pi / 180.0
    conf90    = float(np.percentile(np.sqrt(dlat_m**2 + dlon_m**2), 90))

    return {
        "mc_n_valid":     n_valid,
        "mc_lat_std_m":   round(lat_std_m,              1),
        "mc_lon_std_m":   round(lon_std_m,              1),
        "mc_range_std_m": round(float(np.std(ranges_a)), 1),
        "mc_conf90_m":    round(conf90,                  1),
    }


# ---------------------------------------------------------------------------
# CSV helpers
# ---------------------------------------------------------------------------

def read_fire_pixels(csv_path: Path) -> dict[str, Tuple[float, float]]:
    out = {}
    with csv_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            frame = (row.get("frame") or "").strip()
            try:
                x, y = float(row["x"]), float(row["y"])
            except (KeyError, ValueError):
                continue
            if frame:
                out[frame] = (x, y)
    return out


def read_frame_headings(csv_path: Path) -> dict[str, float]:
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
# Importable run() entry point
# ---------------------------------------------------------------------------

def run(
    camera_lat: float,
    camera_lon: float,
    tower_height: float,
    calibration: str,
    fire: str,
    dem: str,
    img_w: int = 640,
    img_h: int = 480,
    camera_alt: Optional[float] = None,
    tilt_override: Optional[float] = None,
    dem_step: float = 25.0,
    max_range: float = 40000.0,
    ridge_gap_limit: float = 200.0,
    frame_headings: Optional[str] = None,
    n_mc: int = 500,
    out: str = "output/fire_locations.csv",
) -> Tuple[str, list]:
    """
    Run fire localization and return (output_csv_path, rows).
    Importable by pipeline.py.
    """
    if rasterio is None:
        raise RuntimeError("rasterio is required. Run: pip install rasterio")

    with open(calibration, "r", encoding="utf-8") as f:
        cal = json.load(f)
    heading  = float(cal["heading_deg"])
    tilt     = float(cal["tilt_deg"]) if tilt_override is None else tilt_override
    hfov     = float(cal["hfov_deg"])
    cost_px  = float(cal.get("cost_px", 10.0))
    print(f"Calibration: heading={heading:.3f}°  tilt={tilt:.3f}°  "
          f"HFOV={hfov:.3f}°  cost={cost_px:.2f} px")

    fire_pixels = read_fire_pixels(Path(fire))
    print(f"Fire pixels loaded: {len(fire_pixels)} frame(s)")

    fh_map: dict[str, float] = {}
    if frame_headings:
        fh_map = read_frame_headings(Path(frame_headings))
        print(f"Per-frame headings loaded: {len(fh_map)} frame(s)")

    K        = build_K(hfov, img_w, img_h)
    lat0     = camera_lat
    lon0     = camera_lon

    print(f"Loading DEM: {dem}")
    dem_data, dem_transform = load_dem_to_memory(dem)

    if camera_alt is not None:
        alt0 = camera_alt
        print(f"Camera altitude (provided): {alt0:.2f} m")
    else:
        alt0 = get_camera_alt_from_dem(dem_data, dem_transform,
                                       lat0, lon0, tower_height)
        print(f"Camera altitude (DEM + tower): {alt0:.2f} m")

    out_path = Path(out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    rows = []

    for frame, (fire_x, fire_y) in sorted(fire_pixels.items()):
        frame_heading = fh_map.get(frame, heading)
        R_c2w         = build_rotation_matrix(frame_heading, tilt)
        d_world       = pixel_to_world_ray(fire_x, fire_y, K, R_c2w)
        bearing, elev = bearing_elev_from_enu(d_world)

        row: dict = {
            "frame":         frame,
            "fire_x":        f"{fire_x:.1f}",
            "fire_y":        f"{fire_y:.1f}",
            "bearing_deg":   f"{bearing:.4f}",
            "elev_deg":      f"{elev:.4f}",
            "range_m":       "",
            "lat":           "",
            "lon":           "",
            "terrain_alt_m": "",
            "method":        "",
            "mc_lat_std_m":  "",
            "mc_lon_std_m":  "",
            "mc_range_std_m":"",
            "mc_conf90_m":   "",
            "mc_n_valid":    "",
        }

        hit = intersect_ray_dem(
            dem_data, dem_transform,
            lat0, lon0, alt0, d_world,
            step_m=dem_step,
            max_range_m=max_range,
            ridge_gap_limit=ridge_gap_limit,
        )
        if hit is not None:
            lat_h, lon_h, z_h, range_m, method = hit
            row["range_m"]       = f"{range_m:.1f}"
            row["lat"]           = f"{lat_h:.8f}"
            row["lon"]           = f"{lon_h:.8f}"
            row["terrain_alt_m"] = f"{z_h:.2f}"
            row["method"]        = method

            tag = "" if method == "intersection" else "  [RIDGE FALLBACK]"
            print(f"  {frame}: bearing={bearing:.2f}° elev={elev:.2f}° "
                  f"range={range_m/1000:.2f} km  "
                  f"-> ({lat_h:.5f}, {lon_h:.5f}){tag}")

            if n_mc > 0:
                mc = monte_carlo_uncertainty(
                    fire_x, fire_y, frame_heading, tilt, hfov, cost_px,
                    dem_data, dem_transform, lat0, lon0, alt0,
                    img_w, img_h,
                    n_samples=n_mc,
                    dem_step=dem_step,
                    max_range=max_range,
                    ridge_gap_limit=ridge_gap_limit,
                )
                for k, v in mc.items():
                    row[k] = "" if v is None else str(v)
                if mc["mc_conf90_m"] is not None:
                    print(f"    MC 90% conf radius: {mc['mc_conf90_m']:.0f} m  "
                          f"(n={mc['mc_n_valid']}/{n_mc})")
        else:
            print(f"  {frame}: bearing={bearing:.2f}° elev={elev:.2f}°  "
                  f"[no terrain intersection within {max_range/1000:.0f} km]")

        rows.append(row)

    fields = ["frame", "fire_x", "fire_y", "bearing_deg", "elev_deg",
              "range_m", "lat", "lon", "terrain_alt_m", "method",
              "mc_lat_std_m", "mc_lon_std_m", "mc_range_std_m",
              "mc_conf90_m", "mc_n_valid"]

    with out_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    print(f"\nWrote {len(rows)} result(s) -> {out_path}")
    return str(out_path), rows


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Localize fire origin from calibration + fire pixel + DEM."
    )
    ap.add_argument("--camera-lat",       type=float, required=True)
    ap.add_argument("--camera-lon",       type=float, required=True)
    ap.add_argument("--tower-height",     type=float, default=17.5)
    ap.add_argument("--camera-alt",       type=float, default=None)
    ap.add_argument("--calibration",      required=True)
    ap.add_argument("--fire",             required=True)
    ap.add_argument("--img-w",            type=int,   default=640)
    ap.add_argument("--img-h",            type=int,   default=480)
    ap.add_argument("--dem",              type=str,   default=None)
    ap.add_argument("--dem-step",         type=float, default=25.0)
    ap.add_argument("--max-range",        type=float, default=40000.0)
    ap.add_argument("--ridge-gap-limit",  type=float, default=200.0)
    ap.add_argument("--tilt-override",    type=float, default=None)
    ap.add_argument("--frame-headings",   type=str,   default=None)
    ap.add_argument("--n-mc",             type=int,   default=500,
                    help="Monte Carlo samples for uncertainty (0 = disabled).")
    ap.add_argument("--out",              type=str,   default="output/fire_locations.csv")
    args = ap.parse_args()

    if args.dem is None:
        raise ValueError("--dem is required.")

    run(
        camera_lat=args.camera_lat,
        camera_lon=args.camera_lon,
        tower_height=args.tower_height,
        calibration=args.calibration,
        fire=args.fire,
        dem=args.dem,
        img_w=args.img_w,
        img_h=args.img_h,
        camera_alt=args.camera_alt,
        tilt_override=args.tilt_override,
        dem_step=args.dem_step,
        max_range=args.max_range,
        ridge_gap_limit=args.ridge_gap_limit,
        frame_headings=args.frame_headings,
        n_mc=args.n_mc,
        out=args.out,
    )


if __name__ == "__main__":
    main()
