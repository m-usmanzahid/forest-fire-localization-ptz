"""
track_gcps.py

Purpose
-------
Automatically propagate (track) Ground Control Point (GCP) pixel locations from one or more "seed" frames
to neighboring frames, producing enough GCPs per frame to estimate camera pose.

This script uses a HYBRID tracker:
  - LK optical flow predicts motion.
  - Template matching refines/reacquires each landmark.

Why multiple seeds?
-------------------
When the PTZ camera pans, different landmarks enter/leave the field of view.
A landmark set from seed frame 0009 might not exist in frame 0014, and vice-versa.
So we allow multiple seeds (e.g., 0009 and 0011) and, for each target frame, we
choose the SINGLE "best" seed (closest in time and with enough tracked points).
This avoids mixing unrelated landmarks from distant seeds (which can break solvePnP).

Inputs
------
Seed CSV(s):
    frame,id,x,y,lat,lon,alt_m
Frame range:
    --start .. --end (inclusive)

Output
------
A combined CSV:
    frame,id,x,y,lat,lon,alt_m
where for each frame we output points coming from one chosen seed.

Example (PowerShell)
--------------------
python frames\track_gcps.py `
  --frames-dir frames --prefix frame4_frame_ --start 8 --end 14 `
  --seed frames\gcp_frame4_0009.csv frames\gcp_frame4_0011.csv `
  --out frames\gcp_frame4_tracked.csv `
  --min-points 6 --min-score 0.40 --search-radius 220
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import cv2


@dataclass
class GCP:
    id: str
    lat: float
    lon: float
    alt_m: float
    x: float
    y: float


def frame_path(frames_dir: Path, prefix: str, idx: int, ext: str) -> Path:
    return frames_dir / f"{prefix}{idx:04d}.{ext}"


def load_gray(path: Path) -> np.ndarray:
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not read image: {path}")
    return img


def read_seed_gcps(csv_path: Path) -> Tuple[str, List[GCP]]:
    """Read a seed CSV and return (seed_frame_filename, list_of_GCPs_for_that_frame)."""
    seed_frame = None
    gcps: List[GCP] = []
    seen = set()

    with csv_path.open("r", encoding="utf-8") as f:
        r = csv.DictReader(f)
        for row in r:
            fr = (row.get("frame") or "").strip()
            if not fr:
                continue
            if seed_frame is None:
                seed_frame = fr
            if fr != seed_frame:
                continue

            gid = (row.get("id") or "").strip()
            if not gid or gid in seen:
                continue

            lat = (row.get("lat") or "").strip()
            lon = (row.get("lon") or "").strip()
            alt = (row.get("alt_m") or "").strip()
            if lat == "" or lon == "" or alt == "":
                continue

            gcps.append(GCP(
                id=gid,
                x=float(row["x"]),
                y=float(row["y"]),
                lat=float(lat),
                lon=float(lon),
                alt_m=float(alt),
            ))
            seen.add(gid)

    if seed_frame is None or len(gcps) == 0:
        raise ValueError(f"No usable seed points in {csv_path}. Need frame,id,x,y,lat,lon,alt_m.")
    return seed_frame, gcps


def clamp(v: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, v))


def extract_patch(img: np.ndarray, x: float, y: float, size: int) -> np.ndarray:
    """Extract square patch centered at (x,y). Returns empty array if out of bounds."""
    h, w = img.shape[:2]
    r = size // 2
    cx, cy = int(round(x)), int(round(y))
    x0, x1 = cx - r, cx + r + 1
    y0, y1 = cy - r, cy + r + 1
    if x0 < 0 or y0 < 0 or x1 > w or y1 > h:
        return np.array([], dtype=img.dtype)
    return img[y0:y1, x0:x1]


def template_search(next_img: np.ndarray, tmpl: np.ndarray, x_pred: float, y_pred: float,
                    search_radius: int) -> Tuple[bool, float, float, float]:
    """Search for tmpl around predicted point. Returns (ok, x_new, y_new, score)."""
    if tmpl.size == 0:
        return False, 0.0, 0.0, -1.0

    h, w = next_img.shape[:2]
    th, tw = tmpl.shape[:2]
    cx, cy = int(round(x_pred)), int(round(y_pred))

    x0 = clamp(cx - search_radius, 0, w - 1)
    y0 = clamp(cy - search_radius, 0, h - 1)
    x1 = clamp(cx + search_radius, 0, w - 1)
    y1 = clamp(cy + search_radius, 0, h - 1)

    roi = next_img[y0:y1+1, x0:x1+1]
    if roi.shape[0] < th or roi.shape[1] < tw:
        return False, 0.0, 0.0, -1.0

    res = cv2.matchTemplate(roi, tmpl, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(res)
    x_new = x0 + max_loc[0] + tw / 2.0
    y_new = y0 + max_loc[1] + th / 2.0
    return True, float(x_new), float(y_new), float(max_val)


def lk_predict(prev_img: np.ndarray, next_img: np.ndarray, pts: np.ndarray,
               win: int, levels: int) -> Tuple[np.ndarray, np.ndarray]:
    """LK prediction for multiple points. Returns (pts_next, ok_mask)."""
    lk_params = dict(
        winSize=(win, win),
        maxLevel=levels,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    )
    p_next, st, _ = cv2.calcOpticalFlowPyrLK(prev_img, next_img, pts, None, **lk_params)
    return p_next, st.reshape(-1).astype(bool)


def track_direction(frames: List[Path], seed_pos: int, seed_gcps: List[GCP],
                    step: int, min_points: int,
                    template_size: int, search_radius: int, min_score: float,
                    lk_win: int, lk_levels: int) -> Dict[str, Dict[str, GCP]]:
    """Track from seed_pos in a single direction."""
    results: Dict[str, Dict[str, GCP]] = {}

    prev_img = load_gray(frames[seed_pos])
    active: Dict[str, GCP] = {g.id: g for g in seed_gcps}

    i = seed_pos + step
    while 0 <= i < len(frames):
        next_img = load_gray(frames[i])
        ids = list(active.keys())
        pts_prev = np.array([[active[g].x, active[g].y] for g in ids], dtype=np.float32).reshape(-1, 1, 2)

        pts_pred, ok_lk = lk_predict(prev_img, next_img, pts_prev, lk_win, lk_levels)
        pts_pred2 = pts_pred.reshape(-1, 2)
        pts_prev2 = pts_prev.reshape(-1, 2)

        new_active: Dict[str, GCP] = {}
        for j, gid in enumerate(ids):
            x0, y0 = float(pts_prev2[j, 0]), float(pts_prev2[j, 1])
            x_pred, y_pred = (float(pts_pred2[j, 0]), float(pts_pred2[j, 1])) if ok_lk[j] else (x0, y0)

            tmpl = extract_patch(prev_img, x0, y0, template_size)
            ok, x_new, y_new, score = template_search(next_img, tmpl, x_pred, y_pred, search_radius)
            if not ok or score < min_score:
                continue

            g0 = active[gid]
            new_active[gid] = GCP(
                id=gid, lat=g0.lat, lon=g0.lon, alt_m=g0.alt_m,
                x=x_new, y=y_new
            )

        if len(new_active) < min_points:
            break

        results[frames[i].name] = new_active
        active = new_active
        prev_img = next_img
        i += step

    return results


def track_from_seed(frames: List[Path], seed_pos: int, seed_gcps: List[GCP], min_points: int,
                    template_size: int, search_radius: int, min_score: float,
                    lk_win: int, lk_levels: int) -> Dict[str, Dict[str, GCP]]:
    """Track forward and backward from seed, include seed itself."""
    out: Dict[str, Dict[str, GCP]] = {}
    seed_name = frames[seed_pos].name
    out[seed_name] = {g.id: g for g in seed_gcps}

    out.update(track_direction(frames, seed_pos, seed_gcps, +1, min_points,
                              template_size, search_radius, min_score, lk_win, lk_levels))
    out.update(track_direction(frames, seed_pos, seed_gcps, -1, min_points,
                              template_size, search_radius, min_score, lk_win, lk_levels))
    return out


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-dir", type=str, default="frames")
    ap.add_argument("--prefix", type=str, default="frame4_frame_")
    ap.add_argument("--ext", type=str, default="png")
    ap.add_argument("--start", type=int, required=True)
    ap.add_argument("--end", type=int, required=True)

    ap.add_argument("--seed", nargs="+", required=True)
    ap.add_argument("--out", type=str, default="frames/gcp_frame4_tracked.csv")

    ap.add_argument("--min-points", type=int, default=6)
    ap.add_argument("--template-size", type=int, default=31)
    ap.add_argument("--search-radius", type=int, default=220)
    ap.add_argument("--min-score", type=float, default=0.40)

    ap.add_argument("--lk-win", type=int, default=41)
    ap.add_argument("--lk-levels", type=int, default=5)
    args = ap.parse_args()

    if args.template_size % 2 == 0:
        raise ValueError("--template-size must be odd.")

    frames_dir = Path(args.frames_dir)
    frames: List[Path] = []
    for idx in range(args.start, args.end + 1):
        p = frame_path(frames_dir, args.prefix, idx, args.ext)
        if not p.exists():
            raise FileNotFoundError(f"Missing frame: {p}")
        frames.append(p)

    name_to_pos = {p.name: i for i, p in enumerate(frames)}

    # Track separately per seed
    per_seed_tracks: List[Tuple[str, int, Dict[str, Dict[str, GCP]]]] = []
    for seed_csv in args.seed:
        seed_frame, seed_gcps = read_seed_gcps(Path(seed_csv))
        if seed_frame not in name_to_pos:
            raise ValueError(
                f"Seed frame '{seed_frame}' from {seed_csv} is not within selected range "
                f"{frames[0].name}..{frames[-1].name}."
            )
        seed_pos = name_to_pos[seed_frame]
        tracked = track_from_seed(frames, seed_pos, seed_gcps, args.min_points,
                                  args.template_size, args.search_radius, args.min_score,
                                  args.lk_win, args.lk_levels)
        per_seed_tracks.append((seed_frame, seed_pos, tracked))

    # For each frame: choose ONE seed (closest, and with enough points)
    chosen: Dict[str, Dict[str, GCP]] = {}
    for fr in name_to_pos.keys():
        best = None  # (distance, -num_points, seed_frame, points)
        fr_pos = name_to_pos[fr]
        for seed_frame, seed_pos, tracked in per_seed_tracks:
            if fr not in tracked:
                continue
            pts = tracked[fr]
            npts = len(pts)
            if npts < args.min_points:
                continue
            dist = abs(fr_pos - seed_pos)
            cand = (dist, -npts, seed_frame, pts)
            if best is None or cand < best:
                best = cand
        if best is not None:
            chosen[fr] = best[3]

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    with out_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "id", "x", "y", "lat", "lon", "alt_m"])
        for fr in sorted(chosen.keys(), key=lambda n: name_to_pos[n]):
            pts = chosen[fr]
            for gid in sorted(pts.keys()):
                g = pts[gid]
                w.writerow([fr, g.id, f"{g.x:.2f}", f"{g.y:.2f}", f"{g.lat:.8f}", f"{g.lon:.8f}", f"{g.alt_m:.2f}"])

    print(f"Wrote: {out_path}")
    for fr in sorted(chosen.keys(), key=lambda n: name_to_pos[n]):
        print(f"{fr}: {len(chosen[fr])} points (pose-ready)")

    missing = [fr for fr in name_to_pos.keys() if fr not in chosen]
    if missing:
        print("\nFrames not pose-ready (not enough tracked points):")
        for fr in missing:
            print(f"  {fr}")
        print("\nTip: add another seed frame nearer to those frames, or lower --min-score slightly (careful).")


if __name__ == "__main__":
    main()