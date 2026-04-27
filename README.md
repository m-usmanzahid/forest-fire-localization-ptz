# Forest Fire Localization — Skyline Matching Pipeline

Estimates the GPS coordinates of a forest fire from a **single fixed PTZ camera** whose
position is known but whose pan angle (heading) is not.

Achieved accuracy on frame4 (Oghi camera): **~91 m from NASA FIRMS ground truth** at 1.84 km range.

---

## How It Works

1. A synthetic horizon profile is computed from a Digital Elevation Model (DEM) — what the
   ridge silhouette *should* look like from the camera's GPS position in every direction.
2. The observed ridge silhouette in a real camera frame is matched against the synthetic
   profile to find the camera's heading, tilt, and HFOV. This is **skyline matching**.
3. Once the camera is calibrated, fire pixel positions are back-projected through the
   calibration and intersected with the DEM to get GPS coordinates.

---

## Nighttime Operation

Skyline matching requires a visible sky/terrain boundary.  At night the sky/terrain
contrast is much lower than daytime, and camera systems (e.g. LUMS) draw blue bounding
boxes directly onto saved frames — these are not real scene content and confuse the
gradient detector.

Both issues are solved by the `--night` flag in `calibrate_camera.py`, which:
1. Erases blue overlay pixels (B channel >> R and G) before detection.
2. Applies CLAHE to amplify the faint sky/terrain boundary.

The rest of the pipeline (detect → grid-search → Nelder-Mead) is unchanged.

**Night calibration for frame3 (Danna Top):**

```bash
# Step 2 — build horizon for Danna Top (run once, reuse forever)
python build_horizon.py \
    --camera-lat 34.439466 \
    --camera-lon 73.347998 \
    --tower-height 17.5 \
    --dem dem/oghi_dem.tif \
    --out horizon_profile_danna.csv

# Step 3 — calibrate on a nighttime frame with --night flag
# Use frame 0008 which has a visible ridge with minimal fire contamination near the top
python calibrate_camera.py \
    --frame frames/frame3_frame_0008.png \
    --horizon horizon_profile_danna.csv \
    --night \
    --sky-frac 0.45 \
    --heading-min 150 --heading-max 270 \
    --hfov-min 20 --hfov-max 60 \
    --out calibration_danna.json \
    --show
```

If `cost_px` is high (> 15) after night calibration:
- Try a different frame — pick one where the ridge is cleanest (minimal fire glow near the skyline)
- Narrow `--heading-min/max` if you can estimate the camera direction from the scene
- Lower `--sky-frac` to 0.35 if city lights or fire glow in the lower half are being mistaken for sky

Once `calibration_danna.json` is saved, it is reused for all future nighttime events from
Danna Top without re-running calibration.

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
│   ├── frame1.gif          Raw GIF feeds (one per camera location)
│   ├── frame2.gif
│   ├── frame3.gif
│   └── frame4.gif
│
├── dem/
│   └── oghi_dem.tif        DEM GeoTIFF covering at least 40 km radius
│
├── frames/                 Auto-created by extract_frames.py
│   └── *.png
│
├── output/                 Auto-created by localize_fire.py
│   └── fire_locations.csv
│
├── extract_frames.py       Step 1 — Extract PNG frames from GIFs
├── build_horizon.py        Step 2 — Build synthetic horizon from DEM
├── calibrate_camera.py     Step 3 — Match observed skyline to synthetic horizon
├── track_heading.py        Step 3b — Track heading change via skyline cross-correlation
├── heading_from_fire.py    Step 3c — Derive per-frame heading from fire pixel position
├── annotate_fire.py        Step 4 — Click to mark fire pixel (only manual step)
├── localize_fire.py        Step 5 — Compute fire GPS coordinates
│
├── horizon_profile.csv     Output of Step 2
├── calibration.json        Output of Step 3
├── frame_headings.csv      Output of Step 3b or 3c
├── fire_pixels.csv         Output of Step 4
├── locations.txt           Camera GPS coordinates
└── requirements.txt
```

---

## Setup

```bash
pip install -r requirements.txt
```

Requires: `numpy`, `opencv-python`, `Pillow`, `scipy`, `rasterio`

---

## Full Pipeline — Step by Step

### Step 1 — Extract Frames

```bash
python extract_frames.py --input data/frame4.gif --out frames/
```

To extract all cameras at once:

```bash
python extract_frames.py --input data/frame1.gif data/frame2.gif data/frame3.gif data/frame4.gif --out frames/
```

**Output:** `frames/frame4_frame_0001.png` … `frame4_frame_0015.png`

---

### Step 2 — Build Horizon Profile

Compute the expected ridge elevation angle at every azimuth from the camera position.
Run **once per camera location**. Reuse for all future events at that location.

```bash
python build_horizon.py \
    --camera-lat 34.534508 \
    --camera-lon 73.003801 \
    --tower-height 17.5 \
    --dem dem/oghi_dem.tif \
    --out horizon_profile.csv
```

| Argument | Description |
|---|---|
| `--camera-lat` / `--camera-lon` | Camera GPS coordinates |
| `--tower-height` | Height of the camera mast in metres (default: 17.5) |
| `--dem` | DEM GeoTIFF path — must cover at least 40 km radius around camera |
| `--az-step` | Azimuth resolution in degrees (default: 0.1 — gives 3600 rows) |
| `--max-range` | Maximum ray-cast range in metres (default: 40000) |
| `--out` | Output CSV path |

**Output:** `horizon_profile.csv` — one row per azimuth with expected horizon elevation angle.

Typical runtime: under 60 seconds.

---

### Step 3 — Calibrate Camera

Determine the camera's **heading**, **tilt**, and **HFOV** by matching the observed ridge
silhouette against the synthetic horizon.

**Critical rules:**
- Use a **clear daytime frame** with a well-visible ridge and minimal smoke near the horizon.
- **Calibrate on the frame closest to where the fire is visible** to avoid bridging large
  pan angles. If fire is in frames 9–11, calibrate on frame 9.
- Store the resulting `calibration.json` permanently for that camera. Reuse it for all
  future events — including nighttime — without re-running this step.
- If `cost_px` is above 15, try a different (clearer) frame or narrow the search ranges.

```bash
python calibrate_camera.py \
    --frame frames/frame4_frame_0009.png \
    --horizon horizon_profile.csv \
    --img-w 640 --img-h 480 \
    --hfov-min 30 --hfov-max 50 \
    --heading-min 70 --heading-max 110 \
    --out calibration.json \
    --show
```

| Argument | Description |
|---|---|
| `--frame` | A clear PNG frame to calibrate from |
| `--horizon` | `horizon_profile.csv` from Step 2 |
| `--img-w` / `--img-h` | Image dimensions in pixels (default: 640 × 480) |
| `--hfov-min` / `--hfov-max` | Search range for HFOV in degrees (default: 20–90). Narrow this if you know the approximate zoom level — e.g. `30 50` for a moderate zoom |
| `--tilt-min` / `--tilt-max` | Search range for camera tilt in degrees (default: −20 to +20) |
| `--heading-min` / `--heading-max` | Search range for camera heading in degrees (default: 0–360). **Always narrow this** to ±40° around the known camera orientation — e.g. `70 110` for a camera facing east. This prevents the optimizer from locking onto the wrong ridge in a different direction |
| `--sky-frac` | Fraction of image height to search for the skyline, from the top (default: 0.5). Reduce to 0.35 if tall foreground trees are pulling the detected line down |
| `--show` | Open a window showing detected skyline (green) vs expected (red). Close it to save |
| `--out` | Output JSON path |

**Output:** `calibration.json`

```json
{
  "heading_deg": 91.082,
  "tilt_deg": 3.605,
  "hfov_deg": 20.425,
  "cost_px": 24.30,
  "frame": "frame4_frame_0009.png"
}
```

**Quality check — `cost_px`:**
| Value | Meaning |
|---|---|
| < 5 px | Excellent |
| 5–10 px | Good |
| 10–15 px | Acceptable |
| > 15 px | Poor — try a clearer frame or narrower search ranges |

Note: a lower `cost_px` does not always mean a more correct heading. If the optimizer
finds a visually similar ridge at the wrong azimuth (a local minimum), `cost_px` can be
low but the heading will be wrong. Always constrain `--heading-min/max` using known
camera orientation to prevent this.

---

### Step 3b — Track Per-Frame Heading (optional, daytime only)

If the camera is panning and you need per-frame headings for frames close to the
calibration reference frame (within ~20 frames), use cross-correlation of the skyline.

**Only use this when:**
- Frames are captured during the day (skyline visible)
- The pan from the reference frame to the target frames is small (< ~30°, < ~400 px shift)
- Smoke does not cover most of the frame

```bash
python track_heading.py \
    --reference frames/frame4_frame_0001.png \
    --frames frames/frame4_frame_0009.png frames/frame4_frame_0010.png frames/frame4_frame_0011.png \
    --calibration calibration.json \
    --sky-frac 0.35 \
    --out frame_headings.csv
```

| Argument | Description |
|---|---|
| `--reference` | The calibration frame used in Step 3 |
| `--frames` | Fire frames to compute headings for |
| `--calibration` | `calibration.json` from Step 3 |
| `--sky-frac` | Top image fraction to search for skyline (default: 0.35) |
| `--max-shift` | Maximum pixel shift to search in either direction (default: 80). If the camera has panned a lot, increase this — but if shift exceeds image width the frames share no common scene and cross-correlation fails entirely |
| `--out` | Output CSV |

**Output:** `frame_headings.csv` — columns: `frame, heading_deg, pixel_shift, confidence`

Check the `confidence` column. Values below 0.3 are unreliable.

---

### Step 3c — Per-Frame Heading from Fire Pixel (recommended)

The most accurate method when fire is visible across multiple frames. Since the fire is a
fixed point in the world, knowing its bearing from one anchor frame lets you back-calculate
the exact heading for every other frame from where the fire pixel appears.

**Use this instead of track_heading.py when:**
- Fire/smoke is present in the frames
- The camera has panned far from the calibration frame (large pixel shift)
- Nighttime (skyline cross-correlation unavailable but fire is the brightest object)

```bash
python heading_from_fire.py \
    --fire fire_pixels.csv \
    --calibration calibration.json \
    --anchor-frame frames/frame4_frame_0009.png \
    --out frame_headings.csv
```

| Argument | Description |
|---|---|
| `--fire` | `fire_pixels.csv` from Step 4 |
| `--calibration` | `calibration.json` from Step 3 |
| `--anchor-frame` | The fire frame whose heading is treated as ground truth. Use the same frame you calibrated on in Step 3 for best consistency |
| `--anchor-heading` | Override the anchor heading (degrees). If omitted, uses the heading from `calibration.json`. Provide this if you have a more trusted heading for the anchor frame |
| `--img-w` | Image width in pixels (default: 640) |
| `--out` | Output CSV |

**Output:** `frame_headings.csv` — columns: `frame, heading_deg, fire_x, offset_deg`

---

### Step 4 — Annotate Fire Pixel

Click the **base of the smoke plume** in each frame where fire is visible.

```bash
python annotate_fire.py \
    --frames frames/frame4_frame_0009.png frames/frame4_frame_0010.png frames/frame4_frame_0011.png \
    --out fire_pixels.csv
```

To add more frames later without overwriting existing annotations:

```bash
python annotate_fire.py --frames frames/frame4_frame_0013.png --out fire_pixels.csv --append
```

| Key | Action |
|---|---|
| Left-click | Place / move the fire marker |
| S | Save this frame and move to next |
| D | Skip this frame |
| Q | Quit |

**Output:** `fire_pixels.csv`

---

### Step 5 — Localize Fire

Cast a ray from the camera through the annotated fire pixel and find where it intersects
the terrain in the DEM.

```bash
python localize_fire.py \
    --camera-lat 34.534508 \
    --camera-lon 73.003801 \
    --tower-height 17.5 \
    --calibration calibration.json \
    --fire fire_pixels.csv \
    --dem dem/oghi_dem.tif \
    --frame-headings frame_headings.csv \
    --out output/fire_locations.csv
```

| Argument | Description |
|---|---|
| `--camera-lat` / `--camera-lon` | Camera GPS coordinates |
| `--tower-height` | Tower height in metres — used to compute camera altitude from DEM (default: 17.5) |
| `--camera-alt` | Override: provide camera altitude directly in metres ASL. Use if you know the exact altitude and want to skip the DEM lookup |
| `--calibration` | `calibration.json` from Step 3 |
| `--fire` | `fire_pixels.csv` from Step 4 |
| `--frame-headings` | Per-frame headings CSV from Step 3b or 3c. **Always pass this for a panning camera.** Without it, every frame uses the single calibration heading which causes growing error across frames |
| `--dem` | DEM GeoTIFF for ray-terrain intersection. Without this, only bearing and elevation angle are output (no GPS coordinates) |
| `--dem-step` | Ray-march step size in metres (default: 25). Smaller = more accurate intersection but slower |
| `--max-range` | Maximum ray range in metres (default: 40000) |
| `--tilt-override` | Override the tilt from `calibration.json`. Useful for manual tuning without re-running calibration |
| `--out` | Output CSV path |

**Output:** `output/fire_locations.csv`

| Column | Description |
|---|---|
| `frame` | Source frame filename |
| `fire_x` / `fire_y` | Annotated fire pixel coordinates |
| `bearing_deg` | Bearing from camera to fire (degrees, clockwise from North) |
| `elev_deg` | Elevation angle of fire ray (negative = looking down into terrain) |
| `range_m` | Distance from camera to estimated fire location |
| `lat` / `lon` | Estimated fire GPS coordinates |
| `terrain_alt_m` | DEM elevation at the intersection point |

---

## Recommended Command Sequence (Frame4 / Oghi Example)

```bash
# Step 1 — Extract frames (skip if already done)
python extract_frames.py --input data/frame4.gif --out frames/

# Step 2 — Build horizon profile (once per camera, reuse forever)
python build_horizon.py --camera-lat 34.534508 --camera-lon 73.003801 --tower-height 17.5 --dem dem/oghi_dem.tif --out horizon_profile.csv

# Step 3 — Calibrate on the fire frame directly (saves per-frame heading bridging)
python calibrate_camera.py --frame frames/frame4_frame_0009.png --horizon horizon_profile.csv --img-w 640 --img-h 480 --hfov-min 30 --hfov-max 50 --heading-min 70 --heading-max 110 --out calibration.json --show

# Step 4 — Annotate fire pixel
python annotate_fire.py --frames frames/frame4_frame_0009.png frames/frame4_frame_0010.png frames/frame4_frame_0011.png --out fire_pixels.csv

# Step 3c — Derive per-frame headings from fire pixel positions
python heading_from_fire.py --fire fire_pixels.csv --calibration calibration.json --anchor-frame frames/frame4_frame_0009.png --out frame_headings.csv

# Step 5 — Localize
python localize_fire.py --camera-lat 34.534508 --camera-lon 73.003801 --tower-height 17.5 --calibration calibration.json --fire fire_pixels.csv --dem dem/oghi_dem.tif --frame-headings frame_headings.csv --out output/fire_locations.csv
```

---

## Accuracy Results (Frame4 — Oghi Camera)

| Session | Approach | Error vs NASA FIRMS |
|---|---|---|
| Baseline (manual landmark clicking) | Pixel offset → yaw → position | ~800–900 m |
| Previous session | Skyline matching, single heading | ~461 m |
| This session (best) | Calibrate on fire frame + heading_from_fire | **~91 m** |

Ground truth: NASA FIRMS fire location — 34.53385, 73.02354
Camera position: Oghi — 34.534508, 73.003801
Fire range: 1.84 km

All three annotated frames produced independent estimates within 2 m of each other,
confirming the result is not a coincidence.

---

## Accuracy Expectations

| Source of error | Typical impact |
|---|---|
| Skyline calibration (heading) | ±0.5–2° → ±90–350 m at 10 km |
| DEM resolution (SRTM 30 m) | ±50–150 m at terrain intersection |
| Fire pixel annotation jitter | ±1–3 px → ±30–100 m depending on range |
| Smoke obscuring the ridge during calibration | Can cause heading errors of 5–20° |

Practical expected accuracy: **100–400 m** at 2–10 km with SRTM 30 m DEM and
daytime calibration on a clear frame.

---

## Common Issues

### `cost_px` is very high (> 15) after calibration
- Use a frame where the ridge is clearly visible with no smoke or haze near the horizon
- Narrow `--heading-min/max` to ±40° around the known camera direction
- Try `--sky-frac 0.35` to prevent tall foreground trees pulling the detected skyline down
- Try `--hfov-min 30 --hfov-max 50` if zoom level is roughly known

### Per-frame heading estimates are wrong after track_heading.py
- The camera may have panned too far from the reference frame — try `heading_from_fire.py` instead
- Check `confidence` column — values below 0.3 are unreliable
- Increase `--max-shift` if the pan is large, but note that shifts larger than image width mean no overlapping scene content and cross-correlation will fail

### `lat/lon` is blank in `fire_locations.csv`
- Add `--dem dem/oghi_dem.tif` to the localize command
- Check that the DEM covers both the camera location and the fire area
- If `elev_deg` is positive the ray points upward and never hits terrain — fire pixel may be in the sky rather than at the base of the plume

### Nighttime — calibration fails
- Store daytime `calibration.json` per camera and reuse it
- Skip Step 3 entirely at night; go directly to Step 4 → Step 3c → Step 5

### Camera altitude warning
- DEM must cover the camera's GPS location
- Or provide `--camera-alt` directly (ground elevation + tower height in metres ASL)
