# Forest Fire Localization — Skyline Matching Pipeline

Estimates the GPS coordinates of a forest fire from a **single fixed PTZ camera** whose
position is known but whose pan angle (heading) is not.

| Camera | Best result | Range | Method |
|--------|-------------|-------|--------|
| Oghi (frame4) | **91 m error** | 1.84 km | Direct terrain intersection, SRTM DEM |
| Danna Top (frame5) | **~1,037 m error** | 9.35 km | Ridge fallback, Copernicus DEM |

---

## How It Works

1. A synthetic horizon profile is computed from a Digital Elevation Model (DEM) — what the
   ridge silhouette *should* look like from the camera's GPS position in every direction.
2. The observed ridge silhouette in a real camera frame is matched against the synthetic
   profile to find the camera's heading, tilt, and HFOV. This is **skyline matching**.
3. Once the camera is calibrated, fire pixel positions are back-projected through the
   calibration and intersected with the DEM to get GPS coordinates.
4. If the ray does not intersect terrain (fire is hidden behind a ridge), a **ridge fallback**
   reports the closest-approach point as an estimated location.

---

## Accuracy Results

### Frame4 — Oghi Camera (34.534508, 73.003801)

| Session | Approach | Error vs NASA FIRMS |
|---|---|---|
| Baseline (manual landmark clicking) | Pixel offset → yaw → position | ~800–900 m |
| Previous session | Skyline matching, single heading | ~461 m |
| Current best | Calibrate on fire frame + heading_from_fire | **~91 m** |

- Ground truth: NASA FIRMS — **34.53385, 73.02354**
- Fire range: 1.84 km
- DEM used: SRTM (`dem/oghi_dem.tif`)
- All three annotated frames converged within 2 m of each other.

### Frame5 — Danna Top Camera (34.439466, 73.347998)

| Approach | Error vs NASA FIRMS |
|---|---|
| SRTM DEM | Ray misses all terrain — no result |
| Copernicus DEM + ridge fallback | **~1,037 m** |

- Ground truth: NASA FIRMS — **34.51177, 73.32583**
- Fire range: ~8.3 km (predicted ridge fallback range: 9.35 km)
- DEM used: Copernicus GLO-30 (`dem/copernicus_dem.tif`)
- Both annotated frames produce identical predicted coordinates.

---

## Current Known Limitations

### Ridge Occlusion (Frame5)

The ~1,037 m error on frame5 is not a calibration failure — it is a **fundamental geometric
limit** of single-camera observation when the fire is hidden behind a ridge.

**What is happening:**
- The fire is at ~8.3 km, bearing ~345.8° from Danna Top.
- A mountain ridge sits between the camera and the fire.
- Only smoke that has risen above the ridge is visible in the image (pixel y ≈ 82, top of frame).
- The ray at that elevation angle (+5.98°) flies over all terrain, reaching ~2,378 m altitude
  at 9.35 km. The fire terrain is ~1,400–2,000 m. The smoke visible to the camera has
  already risen 400–1,000 m above the fire base before cresting the ridge.
- The **ridge fallback** reports the closest-approach point (9.35 km) as the best estimate,
  but the fire is actually ~1.1 km closer (8.28 km).

**Error decomposition:**
| Source | Contribution |
|--------|-------------|
| Range overshoot (ridge fallback vs fire range) | ~1,102 m |
| Bearing error (1.92° from calibration limit) | ~280 m lateral |
| **Total** | **~1,037 m** |

**Why the bearing cannot be improved further:**
The skyline geometry looking NNW from Danna Top has a nearly flat cost landscape between
344° and 346°. Every calibration attempt (single-frame, multi-frame averaged, constrained
search, all 15 frames) converges to ~344.5–344.7°. The true bearing is 345.82°. The
residual 1.9° is irreducible with skyline matching for this camera/scene.

**What would actually fix it:**
- A **second camera** at a different location — triangulating two bearing lines eliminates
  the range ambiguity entirely and would reduce error to <200 m.
- The fire becoming directly visible (clearing smoke/ridge) — would give a clean terrain
  intersection instead of a ridge fallback.

### track_heading.py — Large Pan Angle Bug

The default `--max-shift 80` is too small when the camera has panned more than ~3° from the
reference frame. The cross-correlation finds a spurious local peak and reports a wrong
heading. Always estimate the expected pixel shift (`pan_degrees / HFOV * image_width`) and
set `--max-shift` to at least that value.

For frame5, the camera panned ~7.3° between frames (199 px at HFOV=23.97°) but
`--max-shift 80` caused track_heading to report only +0.3 px. Fix: use
`--max-shift 250` or larger, or use the fire-pixel consistency method below.

**Workaround for large pans:** infer the heading from fire-pixel consistency:
```python
# Same fire visible in two frames → fire bearing must be identical
# heading_B = fire_bearing_from_A - pixel_offset_B
import math
HFOV, W = 23.97, 640
fx = (W/2) / math.tan(math.radians(HFOV/2))
offset_A = math.degrees(math.atan((x_A - W/2) / fx))
fire_bearing = heading_A + offset_A
offset_B = math.degrees(math.atan((x_B - W/2) / fx))
heading_B = fire_bearing - offset_B
```

### DEM Selection — SRTM vs Copernicus

| Scenario | Recommended DEM |
|----------|----------------|
| Short range (<5 km), clean terrain intersection | SRTM (`dem/oghi_dem.tif`) |
| Long range, fire near/behind ridgeline | Copernicus (`dem/copernicus_dem.tif`) |
| Ray gives `method=ridge_fallback` | Copernicus (ridgeline height accuracy matters) |
| Ray gives `method=intersection` | Either; prefer whichever gives lower multi-frame spread |

Copernicus GLO-30 uses TanDEM-X interferometric SAR and more accurately captures sharp
mountain peaks vs SRTM which systematically underestimates ridgelines due to radar
penetration and canopy averaging.

**Important:** switching DEM also changes the horizon profile and calibration. Always
rebuild `horizon_profile_*.csv` and re-run `calibrate_camera.py` when changing DEMs.
Tested on frame4 (Oghi): Copernicus made results **worse** (130 m avg vs 91 m with SRTM)
because at 1.84 km the calibration drifted with the different horizon shape. SRTM is
sufficient for short-range direct intersections.

**Multi-frame spread as calibration quality gate:** after localizing, if predicted fire
locations across frames spread > 150 m, the calibration landed in a wrong local minimum.
Re-run calibrate_camera.py with tighter `--heading-min/max` bounds.

---

## Copernicus DEM Download

```bash
python download_copernicus_dem.py --out dem/copernicus_dem.tif
```

Downloads tiles N34E072 and N34E073 from the public AWS S3 bucket (no account required),
merges them into a single GeoTIFF. Output: 3600×7200 px, bounds 72–74°E 34–35°N, ~74 MB.

---

## Nighttime Operation

Skyline matching requires a visible sky/terrain boundary. At night the sky/terrain
contrast is much lower than daytime, and camera systems (e.g. LUMS) draw blue bounding
boxes directly onto saved frames — these are not real scene content and confuse the
gradient detector.

Both issues are solved by the `--night` flag in `calibrate_camera.py`, which:
1. Erases blue overlay pixels (B channel >> R and G) before detection.
2. Applies CLAHE to amplify the faint sky/terrain boundary.

The rest of the pipeline (detect → grid-search → Nelder-Mead) is unchanged.

**Night calibration for frame3 (Danna Top):**

```bash
python build_horizon.py \
    --camera-lat 34.439466 \
    --camera-lon 73.347998 \
    --tower-height 17.5 \
    --dem dem/oghi_dem.tif \
    --out horizon_profile_danna.csv

python calibrate_camera.py \
    --frame frames/frame3_frame_0008.png \
    --horizon horizon_profile_danna.csv \
    --night \
    --sky-frac 0.45 \
    --heading-min 150 --heading-max 270 \
    --hfov-min 20 --hfov-max 60 \
    --out calibration_danna.json
```

**Full nighttime pipeline for frame3:**

```bash
python annotate_fire.py \
    --frames frames/frame3_frame_0008.png frames/frame3_frame_0009.png \
    --out fire_pixels_danna.csv

python heading_from_fire.py \
    --fire fire_pixels_danna.csv \
    --calibration calibration_danna.json \
    --anchor-frame frames/frame3_frame_0008.png \
    --out frame_headings_danna.csv

python localize_fire.py \
    --camera-lat 34.439466 \
    --camera-lon 73.347998 \
    --tower-height 17.5 \
    --calibration calibration_danna.json \
    --fire fire_pixels_danna.csv \
    --dem dem/oghi_dem.tif \
    --frame-headings frame_headings_danna.csv \
    --out output/fire_locations_danna.csv
```

---

## Project Structure

```
forest-fire-localization-ptz/
│
├── data/
│   ├── frame3.gif              Raw GIF feeds
│   ├── frame4.gif
│   └── frame5.gif
│
├── dem/
│   ├── oghi_dem.tif            SRTM DEM — Oghi/Danna Top area
│   ├── copernicus_dem.tif      Copernicus GLO-30 DEM (same coverage, higher accuracy at peaks)
│   └── cache/                  Raw tile cache for download scripts
│
├── frames/                     PNG frames extracted from GIFs
│   └── *.png
│
├── output/                     Localization results
│   ├── fire_locations.csv          Frame4 (Oghi)
│   ├── fire_locations_frame5.csv   Frame5 (Danna Top)
│   └── fire_locations_danna.csv    Frame3 (Danna Top, nighttime)
│
├── extract_frames.py           Step 1 — Extract PNG frames from GIFs
├── build_horizon.py            Step 2 — Build synthetic horizon from DEM
├── calibrate_camera.py         Step 3 — Match observed skyline to synthetic horizon
├── track_heading.py            Step 3b — Track heading change via skyline cross-correlation
├── heading_from_fire.py        Step 3c — Derive per-frame heading from fire pixel position
├── annotate_fire.py            Step 4 — Click to mark fire/smoke pixel (only manual step)
├── localize_fire.py            Step 5 — Compute fire GPS coordinates (with ridge fallback)
├── download_copernicus_dem.py  Utility — Download Copernicus GLO-30 tiles from AWS S3
│
├── calibration.json                Oghi camera calibration (SRTM)
├── calibration_frame5.json         Danna Top calibration (Copernicus)
├── calibration_danna.json          Danna Top calibration (nighttime, SRTM)
├── horizon_profile.csv             Oghi horizon (SRTM)
├── horizon_profile_danna_cop.csv   Danna Top horizon (Copernicus)
├── horizon_profile_danna.csv       Danna Top horizon (SRTM, nighttime)
├── fire_pixels.csv                 Frame4 fire annotations
├── fire_pixels_frame5.csv          Frame5 smoke annotations
├── fire_pixels_danna.csv           Frame3 fire annotations (night)
├── frame_headings.csv              Frame4 per-frame headings
├── frame_headings_frame5.csv       Frame5 per-frame headings
├── frame_headings_danna.csv        Frame3 per-frame headings (night)
├── locations.txt                   Camera GPS coordinates reference
└── requirements.txt
```

---

## Setup

```bash
pip install -r requirements.txt
```

Requires: `numpy`, `opencv-python`, `Pillow`, `scipy`, `rasterio`, `requests`

---

## Full Pipeline — Step by Step

### Step 1 — Extract Frames

```bash
python extract_frames.py --input data/frame4.gif --out frames/
```

**Output:** `frames/frame4_frame_0001.png` … `frame4_frame_0015.png`

---

### Step 2 — Build Horizon Profile

Compute the expected ridge elevation angle at every azimuth from the camera position.
Run **once per camera location per DEM**. Reuse for all future events.

```bash
# Oghi camera — SRTM
python build_horizon.py \
    --camera-lat 34.534508 --camera-lon 73.003801 \
    --tower-height 17.5 \
    --dem dem/oghi_dem.tif \
    --out horizon_profile.csv

# Danna Top camera — Copernicus
python build_horizon.py \
    --camera-lat 34.439466 --camera-lon 73.347998 \
    --tower-height 10 \
    --dem dem/copernicus_dem.tif \
    --out horizon_profile_danna_cop.csv
```

| Argument | Description |
|---|---|
| `--camera-lat` / `--camera-lon` | Camera GPS coordinates |
| `--tower-height` | Height of the camera mast in metres |
| `--dem` | DEM GeoTIFF — must cover at least 40 km radius |
| `--az-step` | Azimuth resolution in degrees (default: 0.1°, gives 3600 rows) |
| `--max-range` | Maximum ray-cast range in metres (default: 40000) |
| `--out` | Output CSV path |

---

### Step 3 — Calibrate Camera

Determine heading, tilt, and HFOV by matching the observed ridge silhouette against the
synthetic horizon. Now supports **multiple frames** — their skylines are averaged before
optimisation to reduce per-frame noise.

```bash
# Single frame (standard)
python calibrate_camera.py \
    --frame frames/frame4_frame_0009.png \
    --horizon horizon_profile.csv \
    --hfov-min 16 --hfov-max 26 \
    --heading-min 75 --heading-max 110 \
    --out calibration.json

# Multiple frames averaged (reduces noise; use only frames at the same pan position)
python calibrate_camera.py \
    --frame frames/frame5_frame_0004.png frames/frame5_frame_0006.png \
    --horizon horizon_profile_danna_cop.csv \
    --sky-frac 0.25 \
    --heading-min 330 --heading-max 360 \
    --hfov-min 18 --hfov-max 30 \
    --out calibration_frame5.json
```

| Argument | Description |
|---|---|
| `--frame` | One or more PNG frames. Multiple frames are averaged before optimisation — use only frames at the same camera pan position |
| `--horizon` | `horizon_profile.csv` from Step 2 |
| `--hfov-min` / `--hfov-max` | Search range for HFOV (default: 20–90°). Narrow this if zoom is known |
| `--tilt-min` / `--tilt-max` | Search range for camera tilt (default: −20 to +20°) |
| `--heading-min` / `--heading-max` | **Always narrow to ±40° around known camera direction** to avoid locking onto a wrong ridge |
| `--sky-frac` | Fraction of image height to search for skyline from the top (default: 0.5). Use 0.25 if foreground trees contaminate the detection |
| `--night` | Erase blue overlay boxes and apply CLAHE before detection |
| `--out` | Output JSON path |

**Quality check — `cost_px`:**
| Value | Meaning |
|---|---|
| < 5 px | Excellent |
| 5–10 px | Good |
| 10–15 px | Acceptable |
| > 15 px | Poor — try clearer frame or narrower search |

> **Note:** low `cost_px` does not guarantee correct heading. The optimizer can find a
> visually similar ridge at the wrong azimuth. Always constrain `--heading-min/max`.

---

### Step 3b — Track Per-Frame Heading

For panning cameras: estimate heading change between a reference frame and fire frames
via horizontal skyline cross-correlation.

```bash
python track_heading.py \
    --reference frames/frame4_frame_0001.png \
    --frames frames/frame4_frame_0009.png frames/frame4_frame_0010.png \
    --calibration calibration.json \
    --sky-frac 0.35 \
    --max-shift 250 \
    --out frame_headings.csv
```

| Argument | Description |
|---|---|
| `--reference` | Clear reference frame at the calibration pan position |
| `--frames` | Fire frames to compute headings for |
| `--sky-frac` | Top image fraction to use (default: 0.35) |
| `--max-shift` | **Maximum pixel shift to search.** Default 80 is too small for large pans. Estimate: `pan_degrees / HFOV * image_width`. Use 250+ for safety |
| `--out` | Output CSV: `frame, heading_deg, pixel_shift, confidence` |

Check the `confidence` column — values below 0.3 are unreliable. For large pans or heavy
smoke, use Step 3c instead.

---

### Step 3c — Per-Frame Heading from Fire Pixel

Most accurate for multi-frame panning sequences. The fire is a fixed world point, so its
bearing from one anchor frame constrains the heading for every other frame.

```bash
python heading_from_fire.py \
    --fire fire_pixels.csv \
    --calibration calibration.json \
    --anchor-frame frames/frame4_frame_0009.png \
    --out frame_headings.csv
```

| Argument | Description |
|---|---|
| `--anchor-frame` | Frame whose heading is treated as ground truth — use the calibration frame |
| `--anchor-heading` | Override anchor heading in degrees if you have a better estimate |

**Output:** `frame, heading_deg, fire_x, offset_deg`

---

### Step 4 — Annotate Fire Pixel

Click the **base of the visible smoke/fire plume** in each fire frame.

```bash
python annotate_fire.py \
    --frames frames/frame5_frame_0003.png frames/frame5_frame_0005.png \
    --out fire_pixels_frame5.csv
```

| Key | Action |
|---|---|
| Left-click | Place / move the marker |
| S | Save and move to next frame |
| D | Skip frame |
| Q | Quit |

> For ridge-occluded fires, click the lowest visible smoke pixel — the point where smoke
> first becomes visible above the ridgeline. This gives the smallest elevation angle and the
> closest ridge-fallback range estimate.

---

### Step 5 — Localize Fire

Cast a ray from the camera through the annotated pixel and intersect with the DEM. If the
ray does not hit terrain (ridge occlusion), the `ridge_fallback` method reports the point
of closest approach to terrain.

```bash
# Frame4 (Oghi, direct intersection)
python localize_fire.py \
    --camera-lat 34.534508 --camera-lon 73.003801 \
    --tower-height 17.5 \
    --calibration calibration.json \
    --fire fire_pixels.csv \
    --dem dem/oghi_dem.tif \
    --frame-headings frame_headings.csv \
    --out output/fire_locations.csv

# Frame5 (Danna Top, ridge fallback)
python localize_fire.py \
    --camera-lat 34.439466 --camera-lon 73.347998 \
    --tower-height 10 \
    --calibration calibration_frame5.json \
    --fire fire_pixels_frame5.csv \
    --dem dem/copernicus_dem.tif \
    --frame-headings frame_headings_frame5.csv \
    --out output/fire_locations_frame5.csv
```

| Argument | Description |
|---|---|
| `--dem` | DEM GeoTIFF for ray-terrain intersection |
| `--ridge-gap-limit` | Maximum ray-terrain gap (m) to trigger ridge fallback (default: 200 m) |
| `--max-range` | Maximum ray range in metres (default: 40000) |
| `--frame-headings` | Per-frame headings from Step 3b or 3c. Required for panning cameras |

**Output columns:** `frame, fire_x, fire_y, bearing_deg, elev_deg, range_m, lat, lon, terrain_alt_m, method`

The `method` column is either `intersection` (ray hit terrain) or `ridge_fallback`
(ray missed terrain; closest-approach point reported).

---

## Accuracy Expectations

| Source of error | Typical impact |
|---|---|
| Skyline calibration heading | ±0.5–2° → ±90–350 m at 10 km |
| DEM resolution (SRTM 30 m) | ±50–150 m at terrain intersection |
| Fire pixel annotation jitter | ±1–3 px → ±30–100 m at range |
| Smoke obscuring ridge during calibration | 5–20° heading error |
| Ridge occlusion (smoke only visible) | 500 m – 2 km range overshoot |

Practical expected accuracy:
- **Direct terrain intersection:** 100–400 m at 2–10 km
- **Ridge fallback:** 500–1,500 m depending on how far the fire is behind the ridge

---

## Common Issues

### `cost_px` is high (> 15) after calibration
- Use a frame with a clean ridge and no smoke near the horizon
- Narrow `--heading-min/max` to ±40° around the known camera direction
- Try `--sky-frac 0.25` to exclude foreground trees
- Try `--hfov-min/max` bounds if zoom level is known

### track_heading gives wrong headings / large spread across frames
- **Most common cause:** `--max-shift` is too small for the actual pan. Estimate the
  expected shift: `pan_deg / HFOV * W` and set `--max-shift` to at least that value.
  Default 80 fails for pans larger than ~3°.
- Check the `confidence` column — below 0.3 is unreliable
- For panning cameras with fire visible, use `heading_from_fire.py` instead

### Ray does not intersect terrain (`method=ridge_fallback`)
- Fire may be hidden behind a ridge — this is expected for long-range fires
- Switch to Copernicus DEM which better captures sharp ridgeline heights
- The fallback reports the best single-camera estimate; a second camera is needed for
  accurate range estimation in this scenario

### Multi-frame calibration gives worse results
- Only average frames at the **same camera pan position**. Frames from different pan
  positions produce a blurred skyline that forces HFOV to be over-estimated and
  degrades localization on off-center frames.

### `lat/lon` blank in output
- Add `--dem` to the localize command
- Check DEM covers both camera location and fire area

### Nighttime calibration fails
- Store daytime `calibration.json` and reuse it — skip Step 3 at night
- Go directly to Step 4 → Step 3c → Step 5

### Copernicus DEM download fails / corrupted tile
- Delete the partial `.tif` file from `dem/cache/` before re-running
- The download script tries a fallback AWS region automatically
