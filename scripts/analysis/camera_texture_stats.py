#!/usr/bin/env python3
"""Image-quality statistics of a recording's camera stream (texture / sharpness / HEVC blockiness).

Purpose: explain why the photometric VIO update is weaker on one truck than on another. Decodes N
evenly spaced frames from [start, start+duration] and prints medians of
  grad_mean      mean Sobel gradient magnitude (8-bit gray, full resolution)
  textured_pct   percentage of pixels with gradient magnitude > 12 (usable photometric patches)
  lap_var        variance of the Laplacian (sharpness / motion blur / compression smoothing)
  blockiness     mean |dI| across 8-px block borders divided by mean |dI| inside blocks (1.0 = none)
  brightness/contrast  mean and std of the gray image
Usage: /usr/bin/python3 scripts/analysis/camera_texture_stats.py --bag DIR [--start S --duration D --frames 12]
"""
import argparse, pathlib, sys, tempfile
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools" / "calib"))
from livo_calib import dump_camera_stream, decode_hevc  # noqa: E402


def stats(img):
    import cv2
    g = img.astype(np.float32)
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3) / 8.0
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3) / 8.0
    mag = np.sqrt(gx * gx + gy * gy)
    lap = cv2.Laplacian(g, cv2.CV_32F)
    dx = np.abs(np.diff(g, axis=1))
    cols = np.arange(dx.shape[1])
    border = (cols % 8) == 7
    blockiness = float(dx[:, border].mean() / max(dx[:, ~border].mean(), 1e-6))
    return dict(grad_mean=float(mag.mean()), textured_pct=float((mag > 12).mean() * 100.0),
                lap_var=float(lap.var()), blockiness=blockiness,
                brightness=float(g.mean()), contrast=float(g.std()))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--start", type=float, default=0.0, help="seconds after the first image")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--frames", type=int, default=12)
    a = ap.parse_args()
    out_dir = pathlib.Path(tempfile.mkdtemp(prefix="camtex_"))
    img_stamps, stream = dump_camera_stream(pathlib.Path(a.bag), out_dir, duration=a.duration, start=a.start)
    n = len(img_stamps)
    keep = set(int(k) for k in np.linspace(0, max(n - 1, 0), a.frames).round())
    rows = []
    for k, img in decode_hevc(stream, 1.0, keep=keep):
        rows.append(stats(img))
    if not rows:
        sys.exit("no frames decoded")
    keys = list(rows[0].keys())
    print(f"{a.bag}: {len(rows)} frames from {n} packets, window start {a.start:.0f}s dur {a.duration:.0f}s")
    for k in keys:
        v = np.array([r[k] for r in rows])
        print(f"  {k:13s} median {np.median(v):9.3f}  p10 {np.percentile(v, 10):9.3f}  p90 {np.percentile(v, 90):9.3f}")


if __name__ == "__main__":
    main()
