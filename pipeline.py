"""
pipeline.py  —  Forest Fire Localization: single-command runner

Orchestrates all pipeline steps in sequence:
  1. Build horizon profile from DEM          (cached — skipped if already exists)
  2. Calibrate camera via skyline matching
  3. Compute per-frame headings from fire pixels
  4. Localise fire with DEM ray-march + Monte Carlo uncertainty

Usage — inline arguments:
    python pipeline.py ^
        --camera-lat 34.439466 --camera-lon 73.347998 --tower-height 10 ^
        --dem dem/copernicus_dem.tif ^
        --calibration-frame frames/frame6_frame_0003.png ^
        --heading-min 100 --heading-max 140 ^
        --hfov-min 30 --hfov-max 60 ^
        --fire fire_pixels_frame6.csv ^
        --anchor-frame frames/frame6_frame_0008.png ^
        --name frame6

Usage — config file (recommended for repeated runs):
    python pipeline.py --config pipeline_config.json

Config JSON schema (all CLI flags map to the same keys with underscores):
{
    "name":               "frame6",
    "camera_lat":         34.439466,
    "camera_lon":         73.347998,
    "tower_height":       10.0,
    "dem":                "dem/copernicus_dem.tif",
    "calibration_frame":  ["frames/frame6_frame_0003.png"],
    "heading_min":        100.0,
    "heading_max":        140.0,
    "hfov_min":           30.0,
    "hfov_max":           60.0,
    "fire":               "fire_pixels_frame6.csv",
    "anchor_frame":       "frames/frame6_frame_0008.png",
    "img_w":              640,
    "img_h":              480,
    "night":              false,
    "ridge_gap_limit":    200.0,
    "n_mc":               500,
    "out_dir":            "output"
}

Outputs (all written to --out-dir / name_*):
    {out_dir}/{name}_horizon.csv
    {out_dir}/{name}_calibration.json
    {out_dir}/{name}_frame_headings.csv
    {out_dir}/{name}_fire_locations.csv
"""

import argparse
import json
import sys
import time
from pathlib import Path

import build_horizon
import calibrate_camera
import heading_from_fire
import localize_fire


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _elapsed(t0: float) -> str:
    s = time.time() - t0
    return f"{s:.1f}s" if s < 60 else f"{s/60:.1f}m"


def _print_banner(title: str) -> None:
    print(f"\n{'='*60}")
    print(f"  {title}")
    print(f"{'='*60}")


def _print_summary(rows: list, name: str, out_dir: str) -> None:
    _print_banner("RESULTS SUMMARY")
    valid = [r for r in rows if r.get("lat")]
    if not valid:
        print("  No fire locations resolved.")
        return

    lats   = [float(r["lat"])   for r in valid]
    lons   = [float(r["lon"])   for r in valid]
    ranges = [float(r["range_m"]) for r in valid]

    avg_lat   = sum(lats)   / len(lats)
    avg_lon   = sum(lons)   / len(lons)
    avg_range = sum(ranges) / len(ranges)

    print(f"  Frames resolved : {len(valid)} / {len(rows)}")
    print(f"  Average location: ({avg_lat:.6f}, {avg_lon:.6f})")
    print(f"  Average range   : {avg_range/1000:.2f} km")

    methods = set(r.get("method", "") for r in valid)
    if "ridge_fallback" in methods:
        print("  Method          : ridge_fallback (fire behind ridge — "
              "range estimate is lower bound only)")
    else:
        print("  Method          : direct terrain intersection")

    conf90_vals = [float(r["mc_conf90_m"]) for r in valid
                   if r.get("mc_conf90_m")]
    if conf90_vals:
        avg_conf = sum(conf90_vals) / len(conf90_vals)
        print(f"  MC 90% conf radius (avg): {avg_conf:.0f} m")

    print(f"\n  Output: {out_dir}/{name}_fire_locations.csv")


# ---------------------------------------------------------------------------
# Core pipeline
# ---------------------------------------------------------------------------

def run_pipeline(cfg: dict) -> None:
    t_total = time.time()

    name     = cfg["name"]
    out_dir  = Path(cfg.get("out_dir", "output"))
    out_dir.mkdir(parents=True, exist_ok=True)
    force    = cfg.get("force", False)

    # ------------------------------------------------------------------ #
    # Step 1 — Horizon profile                                            #
    # ------------------------------------------------------------------ #
    _print_banner("Step 1 / 4 — Horizon profile")
    horizon_path = out_dir / f"{name}_horizon.csv"

    if horizon_path.exists() and not force:
        print(f"  Reusing cached horizon: {horizon_path}")
        print("  (pass --force to rebuild)")
    else:
        t0 = time.time()
        build_horizon.run(
            camera_lat=cfg["camera_lat"],
            camera_lon=cfg["camera_lon"],
            tower_height=cfg["tower_height"],
            dem=cfg["dem"],
            out=str(horizon_path),
            az_step=cfg.get("az_step", 0.1),
            range_step=cfg.get("range_step", 50.0),
            max_range=cfg.get("max_range", 40000.0),
        )
        print(f"  Done in {_elapsed(t0)}")

    # ------------------------------------------------------------------ #
    # Step 2 — Calibrate camera                                           #
    # ------------------------------------------------------------------ #
    _print_banner("Step 2 / 4 — Camera calibration")
    cal_path = out_dir / f"{name}_calibration.json"
    t0       = time.time()

    cal = calibrate_camera.run(
        frame=cfg["calibration_frame"],
        horizon=str(horizon_path),
        img_w=cfg.get("img_w", 640),
        img_h=cfg.get("img_h", 480),
        heading_min=cfg.get("heading_min", 0.0),
        heading_max=cfg.get("heading_max", 360.0),
        hfov_min=cfg.get("hfov_min", 20.0),
        hfov_max=cfg.get("hfov_max", 90.0),
        tilt_min=cfg.get("tilt_min", -20.0),
        tilt_max=cfg.get("tilt_max",  20.0),
        sky_frac=cfg.get("sky_frac", 0.5),
        night=cfg.get("night", False),
        show=False,
        out=str(cal_path),
    )
    print(f"  Done in {_elapsed(t0)}")

    # ------------------------------------------------------------------ #
    # Step 3 — Per-frame headings                                         #
    # ------------------------------------------------------------------ #
    _print_banner("Step 3 / 4 — Per-frame headings")
    headings_path = out_dir / f"{name}_frame_headings.csv"
    t0            = time.time()

    _, headings = heading_from_fire.run(
        fire=cfg["fire"],
        calibration=str(cal_path),
        anchor_frame=cfg["anchor_frame"],
        anchor_heading=cfg.get("anchor_heading"),
        img_w=cfg.get("img_w", 640),
        out=str(headings_path),
    )
    print(f"  Done in {_elapsed(t0)}")

    # ------------------------------------------------------------------ #
    # Step 4 — Localise fire                                              #
    # ------------------------------------------------------------------ #
    _print_banner("Step 4 / 4 — Fire localization")
    locations_path = out_dir / f"{name}_fire_locations.csv"
    t0             = time.time()

    _, rows = localize_fire.run(
        camera_lat=cfg["camera_lat"],
        camera_lon=cfg["camera_lon"],
        tower_height=cfg["tower_height"],
        calibration=str(cal_path),
        fire=cfg["fire"],
        dem=cfg["dem"],
        img_w=cfg.get("img_w", 640),
        img_h=cfg.get("img_h", 480),
        camera_alt=cfg.get("camera_alt"),
        tilt_override=cfg.get("tilt_override"),
        dem_step=cfg.get("dem_step", 25.0),
        max_range=cfg.get("max_range", 40000.0),
        ridge_gap_limit=cfg.get("ridge_gap_limit", 200.0),
        frame_headings=str(headings_path),
        n_mc=cfg.get("n_mc", 500),
        out=str(locations_path),
    )
    print(f"  Done in {_elapsed(t0)}")

    # ------------------------------------------------------------------ #
    # Summary                                                             #
    # ------------------------------------------------------------------ #
    _print_summary(rows, name, str(out_dir))
    print(f"\nTotal pipeline time: {_elapsed(t_total)}\n")


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def _load_config(args: argparse.Namespace) -> dict:
    """Merge a JSON config file (if given) with CLI overrides."""
    cfg: dict = {}

    if args.config:
        config_path = Path(args.config)
        if not config_path.exists():
            print(f"ERROR: config file not found: {config_path}", file=sys.stderr)
            sys.exit(1)
        with config_path.open("r", encoding="utf-8") as f:
            cfg = json.load(f)

    # CLI arguments override config file values
    cli_map = {
        "name":              args.name,
        "camera_lat":        args.camera_lat,
        "camera_lon":        args.camera_lon,
        "tower_height":      args.tower_height,
        "dem":               args.dem,
        "calibration_frame": args.calibration_frame,
        "heading_min":       args.heading_min,
        "heading_max":       args.heading_max,
        "hfov_min":          args.hfov_min,
        "hfov_max":          args.hfov_max,
        "tilt_min":          args.tilt_min,
        "tilt_max":          args.tilt_max,
        "fire":              args.fire,
        "anchor_frame":      args.anchor_frame,
        "anchor_heading":    args.anchor_heading,
        "img_w":             args.img_w,
        "img_h":             args.img_h,
        "night":             args.night if args.night else None,
        "ridge_gap_limit":   args.ridge_gap_limit,
        "n_mc":              args.n_mc,
        "out_dir":           args.out_dir,
        "force":             args.force if args.force else None,
    }
    for k, v in cli_map.items():
        if v is not None:
            cfg[k] = v

    # Validate required fields
    required = ["name", "camera_lat", "camera_lon", "tower_height",
                "dem", "calibration_frame", "fire", "anchor_frame"]
    missing = [r for r in required if not cfg.get(r)]
    if missing:
        print(f"ERROR: missing required fields: {missing}", file=sys.stderr)
        print("Provide them via --config or as CLI arguments.", file=sys.stderr)
        sys.exit(1)

    # Normalise calibration_frame to a list
    if isinstance(cfg["calibration_frame"], str):
        cfg["calibration_frame"] = [cfg["calibration_frame"]]

    return cfg


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="Forest fire localization pipeline — single command, all steps.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )

    ap.add_argument("--config", type=str, default=None,
                    help="Path to a JSON config file. CLI args override it.")

    # Camera
    ap.add_argument("--name",          type=str,   default=None,
                    help="Run name — used to name all output files.")
    ap.add_argument("--camera-lat",    type=float, default=None)
    ap.add_argument("--camera-lon",    type=float, default=None)
    ap.add_argument("--tower-height",  type=float, default=None,
                    help="Tower/mast height in metres.")

    # DEM
    ap.add_argument("--dem",           type=str,   default=None,
                    help="DEM GeoTIFF path.")

    # Calibration
    ap.add_argument("--calibration-frame", type=str, nargs="+", default=None,
                    help="Frame(s) to calibrate from.")
    ap.add_argument("--heading-min",   type=float, default=None)
    ap.add_argument("--heading-max",   type=float, default=None)
    ap.add_argument("--hfov-min",      type=float, default=None)
    ap.add_argument("--hfov-max",      type=float, default=None)
    ap.add_argument("--tilt-min",      type=float, default=None)
    ap.add_argument("--tilt-max",      type=float, default=None)
    ap.add_argument("--night",         action="store_true", default=None,
                    help="Night mode: CLAHE + erase blue overlay boxes.")

    # Fire / headings
    ap.add_argument("--fire",          type=str,   default=None,
                    help="fire_pixels CSV from annotate_fire.py.")
    ap.add_argument("--anchor-frame",  type=str,   default=None,
                    help="Anchor frame name for heading_from_fire.py.")
    ap.add_argument("--anchor-heading",type=float, default=None,
                    help="Known heading for anchor frame (degrees). "
                         "If omitted, uses calibration heading.")

    # Image dimensions
    ap.add_argument("--img-w",         type=int,   default=None)
    ap.add_argument("--img-h",         type=int,   default=None)

    # Localization
    ap.add_argument("--ridge-gap-limit", type=float, default=None,
                    help="Ridge fallback gap limit in metres (default: 200). "
                         "Use 1000 for fire-behind-ridge cases.")
    ap.add_argument("--n-mc",          type=int,   default=None,
                    help="Monte Carlo samples for uncertainty (default: 500, 0=off).")

    # Output
    ap.add_argument("--out-dir",       type=str,   default=None,
                    help="Directory for all outputs (default: output).")
    ap.add_argument("--force",         action="store_true", default=None,
                    help="Rebuild horizon profile even if cached.")

    args = ap.parse_args()
    cfg  = _load_config(args)

    print("\nForest Fire Localization Pipeline")
    print(f"  Run name : {cfg['name']}")
    print(f"  Camera   : ({cfg['camera_lat']}, {cfg['camera_lon']}), "
          f"tower {cfg['tower_height']} m")
    print(f"  DEM      : {cfg['dem']}")
    print(f"  Fire CSV : {cfg['fire']}")

    run_pipeline(cfg)


if __name__ == "__main__":
    main()
