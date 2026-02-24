from __future__ import annotations

from pathlib import Path


def iter_gif_frames_pil(gif_path: Path):
    from PIL import Image, ImageSequence

    with Image.open(gif_path) as img:
        for frame in ImageSequence.Iterator(img):
            yield frame.convert("RGBA")


def iter_gif_frames_imageio(gif_path: Path):
    import imageio.v2 as imageio

    for frame in imageio.mimread(gif_path):
        yield frame


def save_frames(gif_path: Path, out_dir: Path) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = gif_path.stem
    count = 0

    try:
        frames = iter_gif_frames_pil(gif_path)
        use_pil = True
    except Exception:
        frames = iter_gif_frames_imageio(gif_path)
        use_pil = False

    for idx, frame in enumerate(frames, start=1):
        out_name = f"{stem}_frame_{idx:04d}.png"
        out_path = out_dir / out_name
        if use_pil:
            frame.save(out_path, format="PNG")
        else:
            import imageio.v2 as imageio

            imageio.imwrite(out_path, frame)
        count += 1

    return count


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    data_dir = root / "data"
    out_dir = root / "frames"

    gifs = sorted(data_dir.glob("*.gif"))
    if not gifs:
        raise SystemExit(f"No .gif files found in {data_dir}")

    total = 0
    for gif_path in gifs:
        extracted = save_frames(gif_path, out_dir)
        total += extracted
        print(f"{gif_path.name}: {extracted} frames")

    print(f"Done. Extracted {total} frames to {out_dir}")


if __name__ == "__main__":
    main()
