"""
download_copernicus_dem.py

Download Copernicus GLO-30 DEM tiles from the public AWS S3 bucket
(no account or API key required) and merge them into a single GeoTIFF.

Why Copernicus instead of SRTM?
    SRTM 30m systematically underestimates sharp mountain peaks due to
    radar wave penetration and canopy/snow surface averaging. Copernicus
    uses TanDEM-X interferometric SAR which more accurately captures
    ridgelines and peaks — critical when localizing fires that appear
    near or above a ridgeline from the camera's perspective.

Tiles needed (same coverage as oghi_dem.tif):
    N34_E072  covers 34-35 N, 72-73 E
    N34_E073  covers 34-35 N, 73-74 E

Usage:
    python download_copernicus_dem.py --out dem/copernicus_dem.tif
"""

import argparse
from pathlib import Path

import requests
import rasterio
from rasterio.merge import merge


BASE_URLS = [
    "https://copernicus-dem-30m.s3.amazonaws.com",
    "https://copernicus-dem-30m.s3.eu-central-1.amazonaws.com",
]

# 1-degree tiles covering the Oghi and Danna Top camera areas
TILES = [
    (34, 72),
    (34, 73),
]


def tile_name(lat: int, lon: int) -> str:
    return f"Copernicus_DSM_COG_10_N{lat:02d}_00_E{lon:03d}_00_DEM"


def download_tile(lat: int, lon: int, cache_dir: Path) -> Path:
    name     = tile_name(lat, lon)
    tif_path = cache_dir / f"{name}.tif"

    if tif_path.exists():
        print(f"  {name}: already cached.")
        return tif_path

    last_err = None
    for base in BASE_URLS:
        url = f"{base}/{name}/{name}.tif"
        print(f"  Trying {url} ...")
        try:
            resp = requests.get(url, timeout=300, stream=True)
            if resp.status_code != 200:
                print(f"  HTTP {resp.status_code}, trying next URL ...")
                continue
            total = 0
            with tif_path.open("wb") as f:
                for chunk in resp.iter_content(chunk_size=1024 * 1024):
                    f.write(chunk)
                    total += len(chunk)
            print(f"  {name}: done ({total/1024/1024:.1f} MB).")
            return tif_path
        except Exception as e:
            last_err = e
            print(f"  Error: {e}, trying next URL ...")
            if tif_path.exists():
                tif_path.unlink()

    raise RuntimeError(f"All download URLs failed for {name}. Last error: {last_err}")


def main():
    ap = argparse.ArgumentParser(
        description="Download Copernicus GLO-30 DEM for the Oghi/Danna Top area."
    )
    ap.add_argument("--out",   default="dem/copernicus_dem.tif",
                    help="Output merged GeoTIFF (default: dem/copernicus_dem.tif).")
    ap.add_argument("--cache", default="dem/cache",
                    help="Tile cache directory (default: dem/cache).")
    args = ap.parse_args()

    out_path  = Path(args.out)
    cache_dir = Path(args.cache)
    cache_dir.mkdir(parents=True, exist_ok=True)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    print("Downloading Copernicus GLO-30 tiles ...")
    tif_paths = []
    for lat, lon in TILES:
        tif_paths.append(download_tile(lat, lon, cache_dir))

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

    print(f"\nDone. Merged DEM saved -> {out_path}")

    with rasterio.open(out_path) as ds:
        print(f"  Bounds : {ds.bounds}")
        print(f"  CRS    : {ds.crs}")
        print(f"  Shape  : {ds.height} x {ds.width} px")
        for label, lat, lon in [
            ("Danna Top", 34.439466, 73.347998),
            ("Oghi",      34.534508, 73.003801),
        ]:
            val = list(ds.sample([(lon, lat)]))[0][0]
            print(f"  Elevation at {label} ({lat}, {lon}): {val:.1f} m")


if __name__ == "__main__":
    main()
