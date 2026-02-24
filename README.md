# Forest Fire Localization from a Fixed PTZ Camera

## 1. Purpose

This project estimates the **ground coordinates (latitude/longitude)** of a fire/smoke origin observed in a **fixed-location PTZ camera feed**.

The system operates on extracted video/GIF frames and uses:

- manually annotated landmarks (Ground Control Points, GCPs),
- camera calibration (horizontal field of view),
- per-frame pose estimation, and
- terrain intersection using a DEM (Digital Elevation Model)

to map a smoke-origin pixel in an image to a real-world ground location.

---

## 2. Problem Statement

A smoke plume can be detected in image space (pixel coordinates), but this alone does not provide a real-world location.  
The objective is to convert a selected smoke-origin pixel into a physically meaningful estimate of:

- **bearing (azimuth) from the camera**
- **elevation angle**
- **ground intersection point (latitude/longitude)**

with a practical operational target of high precision and a defensible error radius (e.g., ~500 m during early deployment).

---

## 3. Method Overview

The localization workflow is implemented in the following stages:

1. **Frame extraction** from the source feed (GIF/video)
2. **GCP annotation** (pixel ↔ known landmark coordinates)
3. **Global HFOV estimation and per-frame heading calibration**
4. **Fire-origin pixel annotation**
5. **Automatic GCP tracking across adjacent frames** (scaling manual annotations)
6. **Per-frame 3D camera pose estimation (solvePnP)**
7. **Ray construction from the fire pixel**
8. **Ray–terrain intersection using DEM** (final coordinate estimation)

---

## 4. Repository Structure

```text
forest_fire_localisation/
├── data/
│   ├── frame1.gif
│   ├── frame2.gif
│   ├── frame3.gif
│   └── frame4.gif
│
├── frames/
│   ├── frame4_frame_0001.png ... frame4_frame_0015.png
│   │
│   ├── gcp_annotator.py              # Manual annotation tool (landmarks + fire pixels)
│   ├── estimate_heading_and_hfov.py  # Global HFOV + per-frame heading calibration
│   ├── pixel_to_bearing.py           # Bearing-only validation utility
│   ├── track_gcps.py                 # Automatic GCP tracking across neighboring frames
│   ├── localize_fire.py              # Pose estimation + DEM ray-terrain intersection
│   │
│   ├── gcp_frame4_0006.csv           # Manual GCPs for frame 0006
│   ├── gcp_frame4_0009.csv           # Manual GCPs for frame 0009
│   ├── gcp_frame4_0011.csv           # Manual GCPs for frame 0011
│   ├── gcp_frame4_tracked.csv        # Auto-tracked GCPs (generated)
│   ├── fire_pixels.csv               # Fire/smoke origin pixel annotations
│   ├── frame_heading_hfov.csv        # HFOV + heading calibration output
│   ├── frame_poses.csv               # Pose estimation summary (generated)
│   └── fire_intersections.csv        # Fire localization output (generated)
│
├── dem/
│   └── oghi_dem.tif                  # DEM GeoTIFF (user-provided; required for final lat/lon)
│
├── locations.txt                     # Camera coordinates for frame sets
├── README.md
└── requirements.txt
```

---

## 5. Camera Configuration (Frame 4 / Oghi)

This repository currently focuses on **Frame 4** (daytime sequence).

### Camera location (from `locations.txt`)

- **Frame 4 (Oghi):** `34.534508, 73.003801`

### Camera altitude used for localization

- Ground elevation at camera location (Google Earth): `1374.13 m`
- Tower height: `35 ft = 10.668 m`
- **Camera optical center altitude:** `1384.798 m`

### Frame size

- Confirmed frame dimensions: **640 × 480 px**

---

## 6. Data Requirements

### 6.1 Ground Control Points (GCPs)

A GCP is a visible landmark with:

- image pixel position `(x, y)`
- known world coordinates `(lat, lon, alt_m)`

**CSV format**

```csv
frame,id,x,y,lat,lon,alt_m
```

### 6.2 Fire-Origin Pixels

A fire-origin pixel is a clicked image point representing the **base of the smoke plume** (closest visible point to the ground).

Use the same CSV schema and leave `lat/lon/alt_m` blank for `id=fire_origin`.

### 6.3 DEM (Required for Final Coordinates)

A **DEM GeoTIFF** is required to compute the final ground intersection:

- Format: `.tif` (GeoTIFF)
- Recommended path: `dem/oghi_dem.tif`

Without a DEM, the system outputs **bearing and elevation angle only**.

---

## 7. Environment Setup

### 7.1 Install Dependencies (Windows PowerShell)

```powershell
pip install -r requirements.txt
```

---

## 8. Operational Workflow

### 8.1 Step 1 — Frame Extraction (if required)

Extract PNG frames from the source GIF/video feed.
If frames already exist in `frames/`, this step can be skipped.

Expected output example:

- `frame4_frame_0001.png` ... `frame4_frame_0015.png`

---

### 8.2 Step 2 — Annotate GCPs (Manual Landmark Registration)

Use `gcp_annotator.py` to click visible landmarks and record:

- landmark ID
- latitude
- longitude
- elevation (`alt_m`)

#### Example (create a new GCP file)

```powershell
python frames\gcp_annotator.py --frame frame4_frame_0009.png --out gcp_frame4_0009.csv
```

#### Example (append more points later)

```powershell
python frames\gcp_annotator.py --frame frame4_frame_0009.png --out gcp_frame4_0009.csv --append
```

#### Recommended practice

- Use stable landmarks (settlements, ridge corners, mountain tops)
- Avoid vegetation edges and moving objects
- Use **6–12 GCPs per frame**
- Include **`alt_m` for every GCP** (required for 3D pose estimation)

---

### 8.3 Step 3 — Estimate Global HFOV and Per-Frame Headings (Horizontal Calibration)

This step estimates:

- a single **global horizontal field of view (HFOV)** for the camera (constant zoom assumption)
- per-frame camera centerline headings (azimuth)

#### Example

```powershell
python frames\estimate_heading_and_hfov.py --camera-lat 34.534508 --camera-lon 73.003801 --image-width 640 --csv frames\gcp_frame4_0006.csv frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv --out frames\frame_heading_hfov.csv
```

#### Output

- `frames/frame_heading_hfov.csv`

This file contains:

- estimated heading per frame
- global HFOV
- RMS residual (calibration quality metric)

---

### 8.4 Step 4 — Annotate Fire/Smoke Origin Pixels

For each frame containing smoke, click the **base of the smoke plume** (closest visible source point).

#### Example

```powershell
python frames\gcp_annotator.py --frame frame4_frame_0009.png --out fire_pixels.csv --append
python frames\gcp_annotator.py --frame frame4_frame_0011.png --out fire_pixels.csv --append
```

When prompted:

- Set `id = fire_origin`
- Leave `lat/lon/alt_m` blank

#### Output

- `frames/fire_pixels.csv`

---

### 8.5 Step 5 — Optional Bearing-Only Validation

Use `pixel_to_bearing.py` to convert annotated fire pixels into bearings using the calibrated HFOV and frame headings. This is a validation step before full 3D localization.

#### Example

```powershell
python frames\pixel_to_bearing.py --headings frames\frame_heading_hfov.csv --pixels frames\fire_pixels.csv --out frames\fire_bearings.csv
```

#### Output

- `frames/fire_bearings.csv`

A consistent bearing across smoke frames indicates stable fire-origin selection and horizontal calibration.

---

### 8.6 Step 6 — Automatic GCP Tracking Across Neighboring Frames (Recommended)

To reduce manual annotation effort, use `track_gcps.py` to propagate GCPs from seed frames (e.g., `0009`, `0011`) to nearby frames.

#### Example

```powershell
python frames\track_gcps.py --frames-dir frames --prefix frame4_frame_ --start 8 --end 14 --seed frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv --out frames\gcp_frame4_tracked.csv --min-points 6 --min-score 0.40 --search-radius 220
```

#### Output

- `frames/gcp_frame4_tracked.csv`

#### Notes

- The tracker uses a hybrid approach (optical flow + template matching)
- It selects the best seed frame per target frame
- If tracking degrades in later frames, add another manual seed frame closer to the problematic frames

---

### 8.7 Step 7 — Add DEM (Required for Final Latitude/Longitude Output)

A DEM is required to convert a ray direction into a ground coordinate.

#### Required file

- `dem/oghi_dem.tif`

#### Coverage recommendation

For an operational radius of approximately **20 km** around the Oghi camera, ensure the DEM covers at least this area (with buffer).

---

### 8.8 Step 8 — Run Final Localization (Pose + Ray–Terrain Intersection)

#### 8.8.1 Without DEM (Ray Angles Only)

Use this mode to validate pose estimation and ray directions.

```powershell
python frames\localize_fire.py --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 --hfov 52.760104 --img-w 640 --img-h 480 --gcp frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv frames\gcp_frame4_tracked.csv --fire frames\fire_pixels.csv
```

#### 8.8.2 With DEM (Final Coordinates)

```powershell
python frames\localize_fire.py --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 --hfov 52.760104 --img-w 640 --img-h 480 --gcp frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv frames\gcp_frame4_tracked.csv --fire frames\fire_pixels.csv --dem dem\oghi_dem.tif
```

#### Outputs

- `frames/frame_poses.csv`
- `frames/fire_intersections.csv`

---

## 9. Output Files and Interpretation

### 9.1 `frames/frame_poses.csv`

Per-frame pose estimation summary:

- `n_gcps`: total GCPs used
- `n_inliers`: RANSAC inliers
- `reproj_rms_px`: reprojection RMS error (pixels)
- `bearing_center_deg`: camera centerline azimuth
- `elev_center_deg`: camera centerline elevation angle

#### Quality guidance

- `reproj_rms_px < 3`: strong fit
- `3–5 px`: usable
- `> 5 px`: review GCP quality and tracking consistency

---

### 9.2 `frames/fire_intersections.csv`

Per-frame fire localization result:

- `bearing_deg`, `elev_deg`: fire ray direction
- `range_m`: estimated distance to terrain intersection
- `lat`, `lon`: estimated fire-origin coordinates (requires DEM)
- `terrain_alt_m`: DEM elevation at intersection
- `reproj_rms_px`, `n_inliers`: pose quality indicators (use for frame filtering)

---

## 10. Frame Selection and Quality Control

Do not average all frames blindly when generating a final operational estimate.

### Recommended filtering criteria

Use only frames that satisfy:

- consistent `bearing_deg` across adjacent smoke frames
- reasonable `elev_deg` continuity
- `reproj_rms_px <= 4`
- preferably `n_inliers >= 6`

Frames with abrupt bearing flips or unrealistic pose jumps should be excluded and re-annotated/re-tracked.

---

## 11. Final Coordinate Estimation (Multi-Frame Fusion)

After obtaining per-frame coordinates from `fire_intersections.csv`:

1. Remove outlier frames
2. Compute a fused estimate (mean or weighted mean)
3. Compute spatial spread (meters) to report confidence

### Recommended reporting format

- **Estimated fire origin:** `(lat, lon)`
- **Confidence radius:** e.g., `500 m` (or empirical radius from frame spread)

---

## 12. Common Issues and Corrective Actions

### 12.1 `solvePnPRansac` Fails

**Cause:**

- Too few GCPs
- Poor landmark matches
- Incorrect tracked points

**Fix:**

- Add more GCPs (6–12)
- Use stable landmarks
- Add an additional seed frame for tracking near problematic frames

---

### 12.2 `lat/lon` Remain Blank in `fire_intersections.csv`

**Cause:**

- DEM not provided
- DEM does not cover the target region
- Ray does not intersect terrain within configured max range

**Fix:**

- Add `--dem dem\oghi_dem.tif`
- Increase `--max-range`
- Verify DEM coverage and camera altitude configuration

---

## 13. Legacy Scripts (Not Required for Final Pipeline)

Some earlier scripts may remain in `frames/` for exploratory work (e.g., yaw plotting). These are not part of the final localization workflow and can be ignored for operational use.

Examples:

- `click_landmarks.py`
- `estimate_yaw.py`
- `plot_yaw_path.py`
- `plot_frames_on_path.py`

---

## 14. Reproducibility and Handover Requirements

To reproduce the final localization workflow, a new user requires:

1. This repository
2. `dem/oghi_dem.tif` (DEM GeoTIFF)
3. GCP CSV files with `alt_m`
4. `fire_pixels.csv`

After setup, run `localize_fire.py` and inspect:

- `frames/frame_poses.csv`
- `frames/fire_intersections.csv`

for per-frame pose quality and fire-origin coordinate estimates.

```

```
