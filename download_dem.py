"""
download_dem.py

Download SRTM 30m elevation tiles from the Mapzen/Tilezen public AWS S3 bucket
(no account or API key required) and merge them into a single GeoTIFF.

Tiles needed for the Oghi camera (34.53N, 73.00E):
    N34E072  — covers lat 34-35, lon 72-73
    N34E073  — covers lat 34-35, lon 73-74

Usage:
    python download_dem.py --out dem/oghi_dem.tif

The output covers the full two-tile area (34–35°N, 72–74°E) which gives
~40 km margin in every direction from the camera.
"""

import argparse
import gzip
import io
import struct
from pathlib import Path

import numpy as np
import requests
import rasterio
from rasterio.transform import from_bounds
from rasterio.merge import merge


# AWS S3 public bucket — no authentication required
BASE_URL = "https://elevation-tiles-prod.s3.amazonaws.com/skadi"

# Tiles that cover the Oghi camera area
TILES = ["N34E072", "N34E073"]

# SRTM1 HGT format constants
SRTM1_SIZE = 3601  # samples per side (1 arc-second resolution, ~30m)


def download_tile(tile_name: str, cache_dir: Path) -> Path:
    """
    Download and cache a single HGT tile. Returns path to the decompressed HGT file.
    """
    lat_prefix = tile_name[:3]          # e.g. "N34"
    gz_name    = tile_name + ".hgt.gz"
    hgt_name   = tile_name + ".hgt"

    hgt_path = cache_dir / hgt_name
    if hgt_path.exists():
        print(f"  {hgt_name}: already cached.")
        return hgt_path

    url = f"{BASE_URL}/{lat_prefix}/{gz_name}"
    print(f"  Downloading {gz_name} from {url} ...")

    resp = requests.get(url, timeout=120, stream=True)
    if resp.status_code != 200:
        raise RuntimeError(
            f"Download failed for {tile_name} (HTTP {resp.status_code}).\n"
            f"URL tried: {url}"
        )

    raw = resp.content
    decompressed = gzip.decompress(raw)
    hgt_path.write_bytes(decompressed)
    mb = len(decompressed) / 1024 / 1024
    print(f"  {hgt_name}: downloaded and decompressed ({mb:.1f} MB).")
    return hgt_path


def hgt_to_array(hgt_path: Path) -> np.ndarray:
    """
    Read a standard SRTM1 HGT file into a float64 numpy array.
    HGT format: big-endian signed int16, SRTM1_SIZE × SRTM1_SIZE, row-major (N→S).
    """
    raw = hgt_path.read_bytes()
    n   = SRTM1_SIZE * SRTM1_SIZE
    data = np.frombuffer(raw, dtype=">i2", count=n).reshape(SRTM1_SIZE, SRTM1_SIZE)
    data = data.astype(np.float64)
    data[data == -32768] = np.nan   # nodata sentinel
    return data


def parse_tile_name(tile_name: str):
    """
    Parse tile name like 'N34E072' into (lat_min, lon_min).
    The tile covers [lat_min, lat_min+1] × [lon_min, lon_min+1].
    """
    lat_sign = 1 if tile_name[0] == "N" else -1
    lon_sign = 1 if tile_name[3] == "E" else -1
    lat = lat_sign * int(tile_name[1:3])
    lon = lon_sign * int(tile_name[4:7])
    return lat, lon


def tile_to_tif(hgt_path: Path, tif_path: Path) -> None:
    """Convert a decompressed HGT file to a GeoTIFF."""
    tile_name = hgt_path.stem
    lat_min, lon_min = parse_tile_name(tile_name)

    data = hgt_to_array(hgt_path)

    # SRTM1 pixel registration: edges of the tile are at exact integer degrees.
    # The transform maps pixel (0,0) to the top-left corner of the top-left pixel.
    # Top-left corner: (lon_min, lat_min+1), bottom-right: (lon_min+1, lat_min)
    transform = from_bounds(
        lon_min, lat_min,
        lon_min + 1, lat_min + 1,
        SRTM1_SIZE, SRTM1_SIZE,
    )

    with rasterio.open(
        tif_path, "w",
        driver="GTiff",
        height=SRTM1_SIZE, width=SRTM1_SIZE,
        count=1,
        dtype=np.float32,
        crs="EPSG:4326",
        transform=transform,
        nodata=np.nan,
    ) as dst:
        dst.write(data.astype(np.float32), 1)


def main():
    ap = argparse.ArgumentParser(description="Download SRTM DEM tiles for the Oghi area.")
    ap.add_argument("--out",   default="dem/oghi_dem.tif",
                    help="Output merged GeoTIFF (default: dem/oghi_dem.tif).")
    ap.add_argument("--cache", default="dem/cache",
                    help="Directory to cache raw HGT files (default: dem/cache).")
    args = ap.parse_args()

    out_path   = Path(args.out)
    cache_dir  = Path(args.cache)
    cache_dir.mkdir(parents=True, exist_ok=True)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # --- Download tiles ---
    print("Downloading SRTM tiles ...")
    hgt_paths = []
    for tile in TILES:
        hgt_path = download_tile(tile, cache_dir)
        hgt_paths.append(hgt_path)

    # --- Convert each HGT to a temporary GeoTIFF ---
    print("\nConverting to GeoTIFF ...")
    tif_paths = []
    for hgt_path in hgt_paths:
        tif_path = cache_dir / (hgt_path.stem + ".tif")
        tile_to_tif(hgt_path, tif_path)
        tif_paths.append(tif_path)
        print(f"  {tif_path.name}: done.")

    # --- Merge tiles ---
    print("\nMerging tiles ...")
    datasets = [rasterio.open(p) for p in tif_paths]
    mosaic, transform = merge(datasets)
    meta = datasets[0].meta.copy()
    meta.update({
        "driver":    "GTiff",
        "height":    mosaic.shape[1],
        "width":     mosaic.shape[2],
        "transform": transform,
        "compress":  "lzw",
    })

    with rasterio.open(out_path, "w", **meta) as dst:
        dst.write(mosaic)

    for ds in datasets:
        ds.close()

    print(f"\nDone. Merged DEM saved → {out_path}")

    # --- Quick verification ---
    with rasterio.open(out_path) as ds:
        print(f"  Bounds : {ds.bounds}")
        print(f"  CRS    : {ds.crs}")
        print(f"  Shape  : {ds.height} × {ds.width} px")
        # Sample elevation at camera location
        cam_lat, cam_lon = 34.534508, 73.003801
        val = list(ds.sample([(cam_lon, cam_lat)]))[0][0]
        print(f"  Elevation at camera ({cam_lat}, {cam_lon}): {val:.1f} m")


if __name__ == "__main__":
    main()
