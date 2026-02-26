# Forest Fire Localization from a Fixed PTZ Camera

## 1. Purpose

This project estimates the **ground coordinates (latitude/longitude)** of a fire/smoke origin observed in a **fixed-location PTZ camera feed**.

The system operates on extracted video/GIF frames and uses:

- manually annotated landmarks (**Ground Control Points, GCPs**),
- camera calibration (**horizontal field of view**, HFOV),
- per-frame pose estimation (solvePnP), and
- ray–terrain intersection using a **DEM** (Digital Elevation Model)

to map a smoke-origin pixel in an image to a real-world ground location.

---

## 2. Problem Statement

A smoke plume can be detected in image space (pixel coordinates), but this alone does not provide a real-world location.  
The objective is to convert a selected smoke-origin pixel into a physically meaningful estimate of:

- **bearing (azimuth) from the camera**
- **elevation angle**
- **ground intersection point (latitude/longitude)**

with a practical operational target of high precision and a defensible error radius (e.g., **~500 m** during early deployment).

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
9. **Multi-frame fusion with automatic rejection of pose-flip/outlier frames** (final single coordinate + confidence radius)

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
│   ├── pixel_to_bearing.py           # Bearing-only validation utility (optional)
│   ├── track_gcps.py                 # Automatic GCP tracking across neighboring frames
│   ├── localize_fire.py              # Pose estimation + DEM ray-terrain intersection
│   ├── final_fire_report.py          # Auto reject pose flips + fuse frames to final coordinate
│   │
│   ├── gcp_frame4_0006.csv           # Manual GCPs for frame 0006 (example)
│   ├── gcp_frame4_0009.csv           # Manual GCPs for frame 0009 (example)
│   ├── gcp_frame4_0011.csv           # Manual GCPs for frame 0011 (example)
│   ├── gcp_frame4_tracked.csv        # Auto-tracked GCPs (generated)
│   ├── fire_pixels.csv               # Fire/smoke origin pixel annotations
│   ├── frame_heading_hfov.csv        # HFOV + heading calibration output (generated)
│   ├── frame_poses.csv               # Pose estimation summary (generated)
│   ├── fire_intersections.csv        # Per-frame fire localization output (generated)
│   └── fire_final_report.txt         # Final fused estimate (generated)
│
├── dem/
│   └── oghi_dem.tif                  # DEM GeoTIFF (required for final lat/lon)
│
├── locations.txt                     # Camera coordinates for frame sets
├── README.md
└── requirements.txt


⸻

5. Camera Configuration (Frame 4 / Oghi)

This repository currently focuses on Frame 4 (daytime sequence).

5.1 Camera location (from locations.txt)
	•	Frame 4 (Oghi): 34.534508, 73.003801

5.2 Camera altitude used for localization
	•	Ground elevation at camera location (Google Earth): 1374.13 m
	•	Tower height: 35 ft = 10.668 m
	•	Camera optical center altitude: 1384.798 m

5.3 Frame size
	•	Confirmed frame dimensions: 640 × 480 px

5.4 Operational range requirement
	•	The localization system is designed to operate within 20 km of the camera location.
	•	localize_fire.py should be run with --max-range 20000 (meters) to enforce this constraint.

⸻

6. Data Requirements

6.1 Ground Control Points (GCPs)

A GCP is a visible landmark with:
	•	image pixel position (x, y)
	•	known world coordinates (lat, lon, alt_m)

CSV format

frame,id,x,y,lat,lon,alt_m

Recommended practice:
	•	Use stable landmarks (settlements, ridge corners, mountain tops)
	•	Avoid vegetation edges and moving objects
	•	Use 6–12 GCPs per frame
	•	Include alt_m for every GCP (required for 3D pose estimation)

6.2 Fire-Origin Pixels

A fire-origin pixel is a clicked image point representing the base of the smoke plume (closest visible point to the ground).

Use the same CSV schema and leave lat/lon/alt_m blank for id=fire_origin.

6.3 DEM (Required for Final Coordinates)

A DEM GeoTIFF is required to compute the final ground intersection:
	•	Format: .tif (GeoTIFF)
	•	Recommended path: dem/oghi_dem.tif

Without a DEM, the system outputs bearing and elevation angle only.

Coverage recommendation:
	•	For an operational radius of approximately 20 km around the Oghi camera, ensure the DEM covers at least this area (buffer recommended).

⸻

7. Environment Setup

7.1 Install Dependencies (Windows PowerShell)

pip install -r requirements.txt


⸻

8. Operational Workflow

8.1 Step 1 — Frame Extraction (if required)

Extract PNG frames from the source GIF/video feed.
If frames already exist in frames/, this step can be skipped.

Expected output example:
	•	frame4_frame_0001.png … frame4_frame_0015.png

⸻

8.2 Step 2 — Annotate GCPs (Manual Landmark Registration)

Use gcp_annotator.py to click visible landmarks and record:
	•	landmark ID
	•	latitude
	•	longitude
	•	elevation (alt_m)

Example (create a new GCP file)

python frames\gcp_annotator.py --frame frame4_frame_0009.png --out frames\gcp_frame4_0009.csv

Example (append more points later)

python frames\gcp_annotator.py --frame frame4_frame_0009.png --out frames\gcp_frame4_0009.csv --append


⸻

8.3 Step 3 — Estimate Global HFOV and Per-Frame Headings (Horizontal Calibration)

This step estimates:
	•	a single global horizontal field of view (HFOV) for the camera (constant zoom assumption)
	•	per-frame camera centerline headings (azimuth)

Example

python frames\estimate_heading_and_hfov.py --camera-lat 34.534508 --camera-lon 73.003801 --image-width 640 --exclude-id gps_tetoli --csv frames\gcp_frame4_0006.csv frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv --out frames\frame_heading_hfov.csv

Output
	•	frames/frame_heading_hfov.csv

⸻

8.4 Step 4 — Annotate Fire/Smoke Origin Pixels

For each frame containing smoke, click the base of the smoke plume (closest visible source point).

Example

python frames\gcp_annotator.py --frame frame4_frame_0009.png --out frames\fire_pixels.csv --append
python frames\gcp_annotator.py --frame frame4_frame_0011.png --out frames\fire_pixels.csv --append

When prompted:
	•	Set id = fire_origin
	•	Leave lat/lon/alt_m blank

Output
	•	frames/fire_pixels.csv

⸻

8.5 Step 5 — Optional Bearing-Only Validation

Use pixel_to_bearing.py to convert annotated fire pixels into bearings using the calibrated HFOV and frame headings. This is a validation step before full 3D localization.

Example

python frames\pixel_to_bearing.py --headings frames\frame_heading_hfov.csv --pixels frames\fire_pixels.csv --out frames\fire_bearings.csv

Output
	•	frames/fire_bearings.csv

A consistent bearing across smoke frames indicates stable fire-origin selection and horizontal calibration.

⸻

8.6 Step 6 — Automatic GCP Tracking Across Neighboring Frames (Recommended)

To reduce manual annotation effort, use track_gcps.py to propagate GCPs from seed frames (e.g., 0009, 0011) to nearby frames.

Example

python frames\track_gcps.py --frames-dir frames --prefix frame4_frame_ --start 8 --end 14 --seed frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv --out frames\gcp_frame4_tracked.csv --min-points 6 --min-score 0.40 --search-radius 220

Output
	•	frames/gcp_frame4_tracked.csv

Notes
	•	The tracker uses a hybrid approach (optical flow + template matching)
	•	It selects the best seed frame per target frame
	•	If tracking degrades in later frames, add another manual seed frame closer to the problematic frames and re-run tracking (e.g., gcp_frame4_0012.csv)

⸻

8.7 Step 7 — Add DEM (Required for Final Latitude/Longitude Output)

A DEM is required to convert a ray direction into a ground coordinate.

Required file
	•	dem/oghi_dem.tif

⸻

8.8 Step 8 — Run Final Localization (Pose + Ray–Terrain Intersection)

8.8.1 Without DEM (Ray Angles Only)
Use this mode to validate pose estimation and ray directions.

python frames\localize_fire.py --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 --hfov 52.760104 --img-w 640 --img-h 480 --max-range 20000 --gcp frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv frames\gcp_frame4_tracked.csv --fire frames\fire_pixels.csv

8.8.2 With DEM (Final Coordinates)

python frames\localize_fire.py --camera-lat 34.534508 --camera-lon 73.003801 --camera-alt 1384.798 --hfov 52.760104 --img-w 640 --img-h 480 --max-range 20000 --gcp frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv frames\gcp_frame4_tracked.csv --fire frames\fire_pixels.csv --dem dem\oghi_dem.tif

Outputs
	•	frames/frame_poses.csv
	•	frames/fire_intersections.csv

⸻

8.9 Step 9 — Produce a Single Final Estimate (Auto-Reject Pose Flips)

This step produces a final fused estimate (lat/lon) with frame rejection and a conservative confidence radius.

python frames\final_fire_report.py --camera-lat 34.534508 --camera-lon 73.003801 --fire frames\fire_intersections.csv --poses frames\frame_poses.csv --out frames\fire_final_report.txt

Output:
	•	frames/fire_final_report.txt

⸻

9. Output Files and Interpretation

9.1 frames/frame_poses.csv

Per-frame pose estimation summary:
	•	n_gcps: total GCPs used
	•	n_inliers: RANSAC inliers
	•	reproj_rms_px: reprojection RMS error (pixels)
	•	bearing_center_deg: camera centerline azimuth
	•	elev_center_deg: camera centerline elevation angle

Quality guidance:
	•	reproj_rms_px < 3: strong fit
	•	3–5 px: usable
	•	> 5 px: review GCP quality and tracking consistency

⸻

9.2 frames/fire_intersections.csv

Per-frame fire localization result:
	•	bearing_deg, elev_deg: fire ray direction
	•	range_m: estimated distance to terrain intersection (meters)
	•	lat, lon: estimated fire-origin coordinates (requires DEM)
	•	terrain_alt_m: DEM elevation at intersection
	•	reproj_rms_px, n_inliers: pose quality indicators (use for filtering)

⸻

10. Frame Selection and Quality Control

Do not average all frames blindly when generating a final operational estimate.

Recommended filtering criteria:
	•	consistent bearing_deg across adjacent smoke frames
	•	reasonable elev_deg continuity
	•	reproj_rms_px <= 4
	•	preferably n_inliers >= 6

Frames with abrupt bearing flips or unrealistic pose jumps should be excluded and re-annotated/re-tracked.

⸻

11. Final Coordinate Estimation (Multi-Frame Fusion)

The final fused estimate is produced by final_fire_report.py, which:
	1.	rejects pose-flip/outlier frames automatically,
	2.	computes a weighted fused coordinate, and
	3.	reports empirical spread (meters) and a conservative radius (default 500 m).

Recommended reporting format:
	•	Estimated fire origin: (lat, lon)
	•	Confidence radius: 500 m (or empirical radius if larger)

⸻

12. Common Issues and Corrective Actions

12.1 solvePnPRansac Fails

Cause:
	•	too few GCPs
	•	poor landmark matches
	•	incorrect tracked points

Fix:
	•	add more GCPs (6–12)
	•	use stable landmarks
	•	add an additional seed frame for tracking near problematic frames

⸻

12.2 lat/lon Remain Blank in fire_intersections.csv

Cause:
	•	DEM not provided
	•	DEM does not cover the target region
	•	ray does not intersect terrain within configured max range

Fix:
	•	add --dem dem\oghi_dem.tif
	•	verify DEM coverage and camera altitude configuration
	•	confirm --max-range 20000 is appropriate (increase only if required and DEM covers it)

⸻

13. Legacy Scripts (Not Required for Final Pipeline)

Some scripts may remain in frames/ for exploratory work. These are not part of the operational pipeline and can be ignored.

Examples:
	•	click_landmarks.py
	•	estimate_yaw.py
	•	plot_yaw_path.py
	•	plot_frames_on_path.py

⸻

14. Reproducibility and Handover Requirements

To reproduce the full localization workflow, a new user requires:
	1.	this repository
	2.	dem/oghi_dem.tif (DEM GeoTIFF)
	3.	GCP CSV files with alt_m
	4.	frames/fire_pixels.csv

Then run, in order:
	1.	estimate_heading_and_hfov.py
	2.	track_gcps.py
	3.	localize_fire.py
	4.	final_fire_report.py

Inspect:
	•	frames/frame_poses.csv
	•	frames/fire_intersections.csv
	•	frames/fire_final_report.txt
```
