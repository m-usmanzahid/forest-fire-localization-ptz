# Forest Fire Localization from a Fixed PTZ Camera

## 1. Purpose

This repository implements a geometry-based pipeline to estimate ground coordinates (latitude/longitude) of a fire or smoke origin observed in a fixed PTZ camera feed. It works on extracted PNG frames using:

- manually annotated Ground Control Points (GCPs)
- camera horizontal field of view (HFOV) calibration
- per-frame pose estimation via solvePnPRansac
- automatic GCP tracking across nearby frames
- ray-terrain intersection using a DEM GeoTIFF

## 2. Problem Statement

Smoke can be detected in image space, but operational response needs a real-world location. Given a smoke-origin pixel `(x, y)` in a frame, the system estimates:

- bearing (azimuth) from the camera
- elevation angle
- ground intersection point (lat/lon)

Operational scope: up to 20 km range from the camera with an early-stage target of <= 500 m reported error radius (conservative).

## 3. Method Overview

1. Extract PNG frames from the PTZ feed (GIF/video).
2. Annotate GCPs (pixel to known landmark coordinates + elevation).
3. Estimate global HFOV and per-frame headings.
4. Annotate fire/smoke origin pixels.
5. Track GCPs automatically across adjacent frames.
6. Estimate per-frame camera pose (rotation) using solvePnPRansac.
7. Convert the fire pixel into a viewing ray (bearing/elevation).
8. Intersect the ray with a terrain DEM to obtain lat/lon.
9. Fuse multi-frame intersections into a final estimate (auto-reject pose flips).

## 4. Repository Structure

```
forest_fire_localisation/
  data/
    frame1.gif
    frame2.gif
    frame3.gif
    frame4.gif
  frames/
    frame4_frame_0001.png ... frame4_frame_0015.png
    gcp_annotator.py              # Manual annotation tool (GCPs + fire pixels)
    estimate_heading_and_hfov.py  # Global HFOV + per-frame heading calibration
    pixel_to_bearing.py           # Bearing-only validation utility (optional)
    track_gcps.py                 # Automatic GCP tracking across neighboring frames
    localize_fire.py              # Pose estimation + DEM ray-terrain intersection
    final_fire_report.py          # Auto-reject pose flips + fuse to final coordinate
    gcp_frame4_0006.csv           # Manual GCPs (example seed)
    gcp_frame4_0009.csv           # Manual GCPs (example seed)
    gcp_frame4_0011.csv           # Manual GCPs (example seed)
    gcp_frame4_tracked.csv        # Auto-tracked GCPs (generated)
    fire_pixels.csv               # Fire/smoke origin pixel annotations
    frame_heading_hfov.csv        # HFOV + heading calibration output (generated)
    frame_poses.csv               # Per-frame pose summary (generated)
    fire_intersections.csv        # Per-frame fire localization (generated)
    fire_final_report.txt         # Final fused output (generated)
  dem/
    oghi_dem.tif                  # DEM GeoTIFF (required for final lat/lon)
  locations.txt                   # Camera coordinates for each frame set
  requirements.txt
  README.md
```

## 5. Camera Configuration (Frame 4 / Oghi)

- Focused sequence: Frame 4 (daytime).
- Camera location (from `locations.txt`): 34.534508, 73.003801.
- Altitude components:
  - Ground elevation at camera location (Google Earth): 1374.13 m
  - Tower height: 35 ft = 10.668 m
  - Camera optical center altitude: 1384.798 m
- Frame size: 640 x 480 px

## 6. Data Requirements

### 6.1 Ground Control Points (GCPs)

A GCP is a visible landmark with:

- pixel location (x, y)
- known world coordinates (lat, lon, alt_m)

CSV schema: `frame,id,x,y,lat,lon,alt_m`

Operational guidance:

- Use stable features (settlements, ridge corners, mountain peaks).
- Avoid vegetation edges, moving objects, and ambiguous textures.
- Use 6-12 GCPs per frame, spread across the image (left/right + top/bottom).
- `alt_m` is required for 3D pose estimation.

### 6.2 Fire / smoke origin pixels

A fire-origin pixel is the base of the smoke plume (closest visible source point). Use the same CSV schema and leave lat/lon/alt_m blank with `id=fire_origin`.

### 6.3 DEM (required for final coordinates)

To convert a ray into a ground coordinate you need a DEM:

- format: GeoTIFF (.tif)
- recommended path: `dem/oghi_dem.tif`
- coverage: at least 20 km radius around the camera (plus buffer)

If no DEM is provided, the pipeline outputs bearing/elevation only.

## 7. Environment Setup (Windows / PowerShell)

Install dependencies:

```
pip install -r requirements.txt
```

## 8. Operational Workflow

### 8.1 Frame extraction (if required)

Extract PNG frames from the GIF/video feed. If `frames/frame4_frame_*.png` already exist, this step can be skipped. Expected: `frame4_frame_0001.png ... frame4_frame_0015.png`.

### 8.2 Annotate GCPs (manual)

Create a new GCP file for a frame:

```
python frames\gcp_annotator.py --frame frame4_frame_0009.png --out frames\gcp_frame4_0009.csv
```

Append additional points later:

```
python frames\gcp_annotator.py --frame frame4_frame_0009.png --out frames\gcp_frame4_0009.csv --append
```

Minimum: 6 GCPs per frame. Recommended: 8-12 well-distributed GCPs.

### 8.3 Estimate global HFOV and per-frame headings

This estimates a single global HFOV (zoom assumed constant) and per-frame centerline heading (azimuth):

```
python frames\estimate_heading_and_hfov.py --camera-lat 34.534508 --camera-lon 73.003801 --image-width 640 --csv frames\gcp_frame4_0006.csv frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv --out frames\frame_heading_hfov.csv
```

Output: `frames/frame_heading_hfov.csv`.

### 8.4 Annotate fire / smoke origin pixels

Click the smoke base and record it with `id=fire_origin`:

```
python frames\gcp_annotator.py --frame frame4_frame_0009.png --out frames\fire_pixels.csv --append
python frames\gcp_annotator.py --frame frame4_frame_0011.png --out frames\fire_pixels.csv --append
```

Output: `frames/fire_pixels.csv`.

### 8.5 Bearing-only validation (optional)

Validate horizontal consistency before running full pose + DEM:

```
python frames\pixel_to_bearing.py --headings frames\frame_heading_hfov.csv --pixels frames\fire_pixels.csv --out frames\fire_bearings.csv
```

Output: `frames/fire_bearings.csv`.

### 8.6 Automatic GCP tracking (recommended)

Propagate GCPs from seed frames to nearby frames to reduce manual work:

```
python frames\track_gcps.py --frames-dir frames --prefix frame4_frame_ --start 8 --end 14 --seed frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv --out frames\gcp_frame4_tracked.csv --min-points 6 --min-score 0.40 --search-radius 220
```

Output: `frames/gcp_frame4_tracked.csv`.
If later frames drift or flip: add another manual seed (e.g., `gcp_frame4_0012.csv`) and rerun tracking with three seeds.

### 8.7 DEM acquisition (GeoTIFF)

A DEM is required for final lat/lon output. Recommended: QGIS + OpenTopography DEM downloader, dataset SRTM 30 m, coverage >= 20 km radius around the camera (plus buffer).

Validate DEM:

```
python -c "import rasterio; ds=rasterio.open(r'dem\oghi_dem.tif'); print(ds.crs); print(ds.bounds); print(ds.width, ds.height); ds.close()"
```

### 8.8 Per-frame localization (pose + DEM intersection)

Without DEM (angles only):

```
python frames\localize_fire.py --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 --hfov 52.760104 --img-w 640 --img-h 480 --gcp frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv frames\gcp_frame4_tracked.csv --fire frames\fire_pixels.csv
```

With DEM (final per-frame coordinates):

```
python frames\localize_fire.py --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 --hfov 52.760104 --img-w 640 --img-h 480 --gcp frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv frames\gcp_frame4_tracked.csv --fire frames\fire_pixels.csv --dem dem\oghi_dem.tif
```

Outputs: `frames/frame_poses.csv`, `frames/fire_intersections.csv`.

### 8.9 Final fused output (auto-reject pose flips)

Automatically rejects pose-flip frames (e.g., sudden 180-degree heading changes and unrealistic near-camera intersections):

```
python frames\final_fire_report.py --camera-lat 34.534508 --camera-lon 73.003801 --fire frames\fire_intersections.csv --poses frames\frame_poses.csv --out frames\fire_final_report.txt
```

Output: `frames/fire_final_report.txt`.

## 9. Outputs and Interpretation

### 9.1 frames/frame_poses.csv

Per-frame pose estimation summary:

- n_gcps, n_inliers: number of GCPs and RANSAC inliers
- reproj_rms_px: reprojection RMS error (pixels)
- bearing_center_deg, elev_center_deg: camera centerline direction

Quality guidance: reproj_rms_px < 3 strong; 3-4 usable; > 4 review GCP quality and tracking stability.

### 9.2 frames/fire_intersections.csv

Per-frame localization result:

- bearing_deg, elev_deg: fire ray direction
- range_m: estimated distance to terrain intersection
- lat, lon: estimated fire-origin coordinates (requires DEM)
- terrain_alt_m: DEM elevation at intersection
- reproj_rms_px, n_inliers: pose quality indicators

### 9.3 frames/fire_final_report.txt

Single final output including final fused (lat, lon), accepted vs rejected frames (with reasons), empirical spread (meters), and conservative radius (default 500 m).

## 10. Quality Control and Operational Guidance

- Do not average all frames blindly.
- Prefer frames with stable bearings across adjacent smoke frames.
- Accept if reproj_rms_px <= 4 and n_inliers >= 6 (preferred).
- Reject frames with bearing flips or unrealistically small range_m (tens of meters).
- To improve robustness, add another seed GCP frame near problematic regions and rerun `track_gcps.py`.

## 11. Reproducibility Checklist

A new user can reproduce the end-to-end workflow with:

1. `frames/frame4_frame_*.png`
2. `frames/gcp_frame4_*.csv` (with alt_m)
3. `frames/fire_pixels.csv`
4. `dem/oghi_dem.tif`
5. `requirements.txt`

Then run in order:

1. `estimate_heading_and_hfov.py`
2. `track_gcps.py`
3. `localize_fire.py`
4. `final_fire_report.py`

## 12. Common Issues

### 12.1 solvePnPRansac fails

Cause: too few GCPs, poor landmark matches, or inconsistent tracked points.
Fix: annotate more GCPs (6-12), use stable landmarks with image-wide spread, add another manual seed frame and rerun tracking.

### 12.2 lat/lon blank in fire_intersections.csv

Cause: DEM not provided; DEM does not cover the ray path; ray does not intersect terrain within max range (default 20 km).
Fix: provide `--dem dem\oghi_dem.tif`; verify DEM coverage; confirm camera altitude configuration; adjust `--max-range` if required.

## 13. Optional Vertical FOV (--vfov) Note

`localize_fire.py` supports `--vfov` to override the vertical FOV. Use only if the video stream is vertically cropped/rescaled or VFOV is explicitly calibrated. Default behavior (when omitted) assumes square pixels (fy = fx).
