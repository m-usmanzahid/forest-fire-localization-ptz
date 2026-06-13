# Session Summary — Forest Fire Localization Pipeline

## Project Overview
Single-camera PTZ pipeline that estimates GPS coordinates of forest fires using skyline matching against a DEM. Camera position is known, pan angle (heading) is not — heading is recovered by matching the observed ridge silhouette to a synthetic horizon profile.

---

## Camera Locations

| Camera | Lat | Lon | Tower Height |
|--------|-----|-----|--------------|
| Oghi | 34.534508 | 73.003801 | 17.5m |
| Danna Top | 34.439466 | 73.347998 | 10m |

---

## Results Summary — All 4 Tested Frames

### Frame 4 — Oghi Camera
- **DEM:** SRTM (`dem/oghi_dem.tif`)
- **Method:** Direct terrain intersection
- **Calibration:** `calibration.json` — heading 95.1°, tilt ~3.6°, HFOV ~20.4°, cost ~3px
- **Fire pixels:** `fire_pixels.csv`
- **Per-frame headings:** `frame_headings.csv`
- **Output:** `output/fire_locations.csv`
- **Predicted fire:** avg (34.5330, 73.0237), range 1.84km, bearing 95.1°
- **FIRMS ground truth:** (34.53385, 73.02354)
- **Error: 91m** ✅

### Frame 5 — Danna Top Camera, Nov 14 2023
- **DEM:** Copernicus (`dem/copernicus_dem.tif`)
- **Method:** Ridge fallback (fire hidden behind ridge, only smoke visible)
- **Calibration:** `calibration_frame5.json` — heading 344.6°, tilt 0.0°, HFOV 23.97°, cost 11.6px
- **Fire pixels:** `fire_pixels_frame5.csv` — smoke at y≈82 (top of frame, above ridge)
- **Per-frame headings:** `frame_headings_frame5.csv`
  - frame5_0004: 344.6237° (anchor)
  - frame5_0003: 344.6237°
  - frame5_0005: 352.0941° (inferred via fire-pixel consistency, 199.4px shift)
- **Output:** `output/fire_locations_frame5.csv`
- **Predicted fire:** avg (34.5197, 73.3199), range 9.35km, bearing 343.9°
- **FIRMS ground truth:** (34.51177, 73.32583) — same fire as FIRMS cluster visible in FIRMS map
- **Error: ~1,037m** — hard floor due to ridge occlusion, NOT a calibration failure
- **Error decomposition:** range overshoot 1,102m + bearing error 280m lateral

### Frame 6 — Danna Top Camera, Nov 14 2023 daytime (~12:45)
- **DEM:** Copernicus (`dem/copernicus_dem.tif`)
- **Method:** Direct terrain intersection
- **Calibration:** `calibration_frame6.json` — heading 119.22°, tilt -6.15°, HFOV 42.74°, cost 14.97px
- **Fire pixels:** `fire_pixels_frame6.csv`
- **Per-frame headings:** `frame_headings_frame6.csv` (heading_from_fire, anchor=frame6_0008)
  - frame6_0006: 115.796°, frame6_0008: 119.217°, frame6_0010: 121.816°
- **Output:** `output/fire_locations_frame6.csv`
- **Predicted fire:** avg (34.4300, 73.3620), range ~1.67km, bearing 129.5° (ESE)
- **FIRMS ground truth:** None — fire too small for VIIRS detection
- **Error: Unknown** (no ground truth)
- **Note:** 3 frames agree within 12m of each other — calibration is stable

### Frame 7 — Danna Top Camera, Nov 14 2023 nighttime (~21:27)
- **DEM:** Copernicus (`dem/copernicus_dem.tif`)
- **Method:** Ridge fallback (ray at +2° elevation misses all terrain)
- **Calibration:** `calibration_frame7.json` — heading 145.89°, tilt -9.21°, HFOV 47.93°, cost 9.33px (night mode)
- **Fire pixels:** `fire_pixels_frame7.csv` — fire near top of frame (y≈93-97), looking slightly up
- **Per-frame headings:** `frame_headings_frame7.csv` (heading_from_fire, anchor=frame7_0003)
  - frame7_0001: 130.238°, frame7_0003: 145.889°, frame7_0005: 160.749°
- **Output:** `output/fire_locations_frame7.csv`
- **Predicted fire:** avg (34.3690, 73.4142), range ~9.97km, bearing 142.4° (SSE)
- **FIRMS ground truth:** None
- **Error: Unknown**
- **Note:** Localized with `--ridge-gap-limit 1000`. 3 frames agree within 100m and 0.55° bearing — self-consistent

---

## Frame 6 vs Frame 7 — Two Different Fires
Frame 6 (bearing 129.5°, 1.67km) and frame 7 (bearing 142.4°, 10km) are 8.3km apart. Almost certainly two separate fires on Nov 14 2023, not the same fire at different times.

---

## FIRMS Ground Truth Status for Nov 14 2023
FIRMS (VIIRS/NOAA-20) detected fires at ~(34.511, 73.326) on Nov 14 2023. These are at bearing **346° (NNW)** from Danna Top — that is the frame 5 fire. The frame 6 and frame 7 fires (ESE and SSE) had no FIRMS detection — they were below the 375m VIIRS threshold.

---

## Key Technical Issues & Decisions

### Ridge Occlusion (frames 5 and 7)
Fire hidden behind a ridge — only smoke above ridgeline is visible. Single camera cannot resolve range. Ridge fallback gives closest-approach estimate but overshoots by 500m–2km. Fix requires a second camera for triangulation.

### DEM Selection Rule
- `method=intersection`, short range (<5km) → SRTM fine
- `method=ridge_fallback` or long range → Copernicus GLO-30
- Copernicus tested on frame4: made results worse (130m vs 91m) because calibration drifted with the different horizon shape at short range

### track_heading max-shift Bug
Default `--max-shift 80` fails for pans >~3°. For frame5_0005 the true shift was 199px but the correlator found a spurious peak. Fix: estimate expected shift = `pan_deg / HFOV * image_width`, set max-shift to at least that. For large pans use `heading_from_fire.py` instead (fire-pixel consistency).

### Multi-frame Calibration Caveat
Only average frames at the **same camera pan position**. Frames at different pan positions blur the skyline → HFOV over-estimated → localization error grows at off-center pixels.

### Night Mode
`--night` flag in `calibrate_camera.py`: erases blue LUMS overlay boxes (B >> R,G pixels), applies CLAHE to amplify faint sky/terrain boundary. Used for frame7 calibration.

### frame5_0005 Heading (fire-pixel consistency method)
Cross-correlation failed (spurious peak). Heading inferred as:
```
fire_bearing_from_0003 = 343.902°
pixel_offset_0005 = atan((103 - 320) / fx) = -8.192°
heading_0005 = 343.902 - (-8.192) = 352.094°
```

---

## Pending / Next Steps
1. Get ground truth for frame 6 fire at (34.430, 73.362) — field reports, MODIS archive, or Google Earth historical imagery for ESE direction from Danna Top on Nov 14 2023
2. Get ground truth for frame 7 fire at (34.369, 73.414) — same date, SSE direction
3. Commit frame 6 and frame 7 pipeline files to git branch `ayaan` (not yet committed)

---

## Git Status
- Branch: `ayaan`
- Not yet committed: calibration_frame6.json, calibration_frame7.json, fire_pixels_frame6.csv, fire_pixels_frame7.csv, frame_headings_frame6.csv, frame_headings_frame7.csv, output/fire_locations_frame6.csv, output/fire_locations_frame7.csv
- `.gitignore` excludes `dem/copernicus_dem.tif` (104MB, over GitHub limit) — regenerate with `python download_copernicus_dem.py --out dem/copernicus_dem.tif`

---

## Pipeline Commands Reference (frame 6 example)

```bash
# Build horizon (once per camera)
python build_horizon.py --camera-lat 34.439466 --camera-lon 73.347998 --tower-height 10 --dem dem/copernicus_dem.tif --out horizon_profile_danna_cop.csv

# Calibrate
python calibrate_camera.py --frame frames/frame6_frame_0003.png --horizon horizon_profile_danna_cop.csv --heading-min 100 --heading-max 140 --hfov-min 30 --hfov-max 60 --out calibration_frame6.json

# Annotate fire pixels (manual)
python annotate_fire.py --frames frames/frame6_frame_0006.png frames/frame6_frame_0008.png frames/frame6_frame_0010.png --out fire_pixels_frame6.csv

# Per-frame headings from fire pixel consistency
python heading_from_fire.py --fire fire_pixels_frame6.csv --calibration calibration_frame6.json --anchor-frame frames/frame6_frame_0008.png --out frame_headings_frame6.csv

# Localize
python localize_fire.py --camera-lat 34.439466 --camera-lon 73.347998 --tower-height 10 --calibration calibration_frame6.json --fire fire_pixels_frame6.csv --dem dem/copernicus_dem.tif --frame-headings frame_headings_frame6.csv --out output/fire_locations_frame6.csv

# For ridge fallback cases add: --ridge-gap-limit 1000
```
