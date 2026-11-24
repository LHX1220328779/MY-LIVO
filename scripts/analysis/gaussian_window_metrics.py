#!/usr/bin/env python3
"""Compare saved map renders on the same recorded cameras and static LiDAR pixels."""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

RANGE_BINS = ('le30m', '30to60m', 'gt60m')
RANGE_METRICS = ('psnr_db', 'ssim', 'background_like_at_returns')
ALPHA_REGIONS = ('static_lidar_neighborhood', 'nonsky_static', 'static_returns')
ALPHA_METRICS = ('alpha_mean', 'alpha_below_0_05_fraction', 'alpha_below_0_1_fraction',
                 'alpha_below_0_5_fraction')
STATIC_DEPTH_METRICS = ('coverage', 'mae_m', 'median_m')


def stationary_pixels(dynamic, width):
    kernel = max(3, round(11 * width / 640) | 1)
    return cv2.dilate(dynamic, np.ones((kernel, kernel), np.uint8)) < 128


def ssim_map(image, reference):
    blur = lambda pixels: cv2.GaussianBlur(pixels, (11, 11), 1.5)
    left, right = blur(image), blur(reference)
    covariance = blur(image * reference) - left * right
    variance = blur(image ** 2) - left ** 2 + blur(reference ** 2) - right ** 2
    return (2*left*right + 6.5025) * (2*covariance + 58.5225) / (
        (left ** 2 + right ** 2 + 6.5025) * (variance + 58.5225))


def static_scores(image, reference, depth, dynamic, background):
    width = depth.shape[1]
    kernel = max(3, round(9 * width / 640) | 1)
    measured = (depth > 0).astype(np.uint8)
    stationary = stationary_pixels(dynamic, width)
    region = (cv2.dilate(measured, np.ones((kernel, kernel), np.uint8)) != 0) & stationary
    hits = (measured != 0) & stationary
    if not hits.any():
        raise ValueError('No static measured pixels in held-out view')
    image, reference = image.astype(np.float64), reference.astype(np.float64)
    mse = np.mean((image[region] - reference[region]) ** 2)
    ssim = ssim_map(image, reference)
    return dict(static_psnr_db=float(-10*np.log10(max(mse, 1e-9)/255**2)), static_ssim=float(ssim[region].mean()),
                background_like_at_returns=float((abs(image[hits] - background).max(axis=1) <= 2).mean()))


def measured_range_scores(image, reference, depth, dynamic, background):
    """Depth is measured camera-axis Z; only exact return pixels select a range, without dilation."""
    hits = (depth > 0) & np.isfinite(depth) & stationary_pixels(dynamic, depth.shape[1])
    masks = (hits & (depth <= 30), hits & (depth > 30) & (depth <= 60), hits & (depth > 60))
    image, reference = image.astype(np.float64), reference.astype(np.float64)
    ssim = ssim_map(image, reference)
    out = {}
    for name, mask in zip(RANGE_BINS, masks):
        pixels = int(mask.sum())
        scores = dict(pixels=pixels, **{key: None for key in RANGE_METRICS})
        if pixels:
            mse = np.mean((image[mask] - reference[mask]) ** 2)
            scores.update(psnr_db=float(-10*np.log10(max(mse, 1e-9)/255**2)),
                          ssim=float(ssim[mask].mean()),
                          background_like_at_returns=float((abs(image[mask] - background).max(axis=1) <= 2).mean()))
        out[name] = scores
    return out


def render_support_scores(render_path, depth, dynamic, sky):
    """Measure saved alpha directly; old RGB-only renders provide no alpha evidence."""
    width = depth.shape[1]
    stationary = stationary_pixels(dynamic, width)
    hits = (depth > 0) & np.isfinite(depth) & stationary
    kernel = max(3, round(9 * width / 640) | 1)
    region = (cv2.dilate(((depth > 0) & np.isfinite(depth)).astype(np.uint8),
                        np.ones((kernel, kernel), np.uint8)) != 0) & stationary
    masks = dict(static_lidar_neighborhood=region,
                 nonsky_static=stationary & (sky < 128) if sky is not None else None,
                 static_returns=hits)
    near = hits & (depth <= 30)
    out = dict(available=False, alpha_regions={
        name: dict(pixels=int(mask.sum()) if mask is not None else None,
                   **{key: None for key in ALPHA_METRICS}) for name, mask in masks.items()},
        static_depth_30m=dict(measured_pixels=int(near.sum()), valid_pixels=None,
                             **{key: None for key in STATIC_DEPTH_METRICS}))
    alpha_path = render_path.with_name(render_path.stem + '_alpha.png')
    depth_path = render_path.with_name(render_path.stem + '_depth.bin')
    if not alpha_path.is_file() or not depth_path.is_file():
        return out
    alpha = cv2.imread(str(alpha_path), cv2.IMREAD_UNCHANGED)
    rendered_depth = np.fromfile(depth_path, '<f4')
    if (alpha is None or alpha.dtype != np.uint16 or alpha.shape != depth.shape or
            rendered_depth.size != depth.size):
        raise ValueError(f'Invalid saved alpha or depth: {render_path}')
    alpha = alpha.astype(np.float64) / 65535
    rendered_depth = rendered_depth.reshape(depth.shape)
    out['available'] = True
    for name, mask in masks.items():
        if mask is not None and mask.any():
            values = alpha[mask]
            out['alpha_regions'][name].update(
                alpha_mean=float(values.mean()),
                alpha_below_0_05_fraction=float((values < .05).mean()),
                alpha_below_0_1_fraction=float((values < .1).mean()),
                alpha_below_0_5_fraction=float((values < .5).mean()))
    valid = near & np.isfinite(rendered_depth) & (rendered_depth > 0)
    scores = out['static_depth_30m']
    scores.update(valid_pixels=int(valid.sum()),
                  coverage=float(valid.sum() / near.sum()) if near.any() else None)
    if valid.any():
        error = np.abs(rendered_depth[valid].astype(np.float64) - depth[valid])
        scores.update(mae_m=float(error.mean()), median_m=float(np.median(error)))
    return out


def summarize_render_support(rows):
    available = [row['render_support'] for row in rows if row['render_support']['available']]
    regions = {}
    for name in ALPHA_REGIONS:
        valid = [row['alpha_regions'][name] for row in available
                 if row['alpha_regions'][name]['alpha_mean'] is not None]
        regions[name] = dict(valid_views=len(valid), pixels=sum(row['pixels'] for row in valid),
                             **{key: float(np.mean([row[key] for row in valid])) if valid else None
                                for key in ALPHA_METRICS})
    depths = [row['static_depth_30m'] for row in available]
    scores = dict(available_views=len(depths), measured_pixels=sum(row['measured_pixels'] for row in depths),
                  valid_pixels=sum(row['valid_pixels'] for row in depths))
    for key in STATIC_DEPTH_METRICS:
        values = [row[key] for row in depths if row[key] is not None]
        scores[key] = float(np.mean(values)) if values else None
    return dict(available_views=len(available), alpha_regions=regions, static_depth_30m=scores)


def baseline_comparison(baseline, model):
    reference = {row['frame']: row for row in baseline['rows']}
    rows, ranked = [], {name: [] for name in ('static', *RANGE_BINS)}
    for row in model['rows']:
        base = reference[row['frame']]
        differences = {key: row['scores'][key] - base['scores'][key]
                       if row['scores'][key] is not None and base['scores'].get(key) is not None else None
                       for key in row['scores']}
        ranges = {}
        for name in RANGE_BINS:
            left, right = base['measured_ranges'][name], row['measured_ranges'][name]
            ranges[name] = dict(pixels_delta=right['pixels'] - left['pixels'], **{
                key: right[key] - left[key] if right[key] is not None and left[key] is not None else None
                for key in RANGE_METRICS})
        support = {}
        for name in ALPHA_REGIONS:
            left = base['render_support']['alpha_regions'][name]
            right = row['render_support']['alpha_regions'][name]
            support[name] = {key: right[key] - left[key]
                             if left[key] is not None and right[key] is not None else None
                             for key in ALPHA_METRICS}
        left, right = base['render_support']['static_depth_30m'], row['render_support']['static_depth_30m']
        depth_delta = {key: right[key] - left[key]
                       if left[key] is not None and right[key] is not None else None
                       for key in ('measured_pixels', 'valid_pixels', *STATIC_DEPTH_METRICS)}
        rows.append(dict(frame=row['frame'], window=row['window'], scores=differences, measured_ranges=ranges,
                         render_support=dict(alpha_regions=support, static_depth_30m=depth_delta)))
        for name in ranked:
            left = base['scores']['static_psnr_db'] if name == 'static' else base['measured_ranges'][name]['psnr_db']
            right = row['scores']['static_psnr_db'] if name == 'static' else row['measured_ranges'][name]['psnr_db']
            if left is not None and right is not None:
                ranked[name].append(dict(frame=row['frame'], window=row['window'], baseline_psnr_db=left,
                                         model_psnr_db=right, delta_psnr_db=right-left))
    diagnostics = {}
    for name, values in ranked.items():
        ordered = sorted(values, key=lambda item: item['delta_psnr_db'])
        diagnostics[name] = dict(valid_views=len(values), worst=ordered[:5],
                                 degraded=[item for item in ordered if item['delta_psnr_db'] < -0.1])
    return dict(delta_direction='model minus baseline', psnr_degradation_threshold_db=0.1,
                rows=rows, diagnostics=diagnostics)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared', type=Path, help='Prepared input directory with submap_*')
    parser.add_argument('models', type=Path, nargs='+', help='Map directories with map_info.json and saved renders')
    parser.add_argument('--out', type=Path, required=True, help='New comparison JSON file')
    parser.add_argument('--baseline', type=Path, help='Compare every model against these saved renders; included in models')
    args = parser.parse_args()
    prep = json.loads((args.prepared.parent/'preparation.json').read_text())
    poses = json.loads((args.prepared.parent/'camera_poses.json').read_text())
    with (Path(prep['fusion'])/'frames.csv').open() as stream:
        origin = float(next(csv.DictReader(line for line in stream if not line.startswith('#')))['stamp'])
    windows = (prep['options'].get('intervals') or '').split(',')
    bins = [tuple(map(float, win.split(':'))) for win in windows if win]
    report = dict(prepared=str(args.prepared.resolve()), windows=windows if bins else [], models={},
                  measured_range_definition='Camera-axis Z in metres: (0,30], (30,60], (60,infinity); '
                  'exact finite measured return pixels, same dynamic exclusion; SSIM uses 11x11 image neighborhoods. '
                  'Summary quality metrics average valid views equally; pixels counts their summed support.',
                  render_support_definition='Alpha is the saved uint16 composited opacity divided by 65535, '
                  'never inferred from RGB. Static LiDAR neighborhoods use the existing scaled 9x9 dilation; '
                  'all regions exclude the same scaled 11x11 dynamic dilation. nonsky_static additionally '
                  'excludes sky pixels; it is null when the prepared sky mask is absent. Static 30m depth uses '
                  'finite exact measured 0<Z<=30 pixels and finite positive rendered depth, without alpha filtering; '
                  'MAE/NumPy median and coverage use that support. Both alpha/depth sidecars are required, otherwise '
                  'new scores are null. Summary scores average valid views equally and counts sum their support.')
    models = [args.baseline, *args.models] if args.baseline else args.models
    for model in dict.fromkeys(path.resolve() for path in models):
        info = json.loads((model/'map_info.json').read_text())
        saved = {row['frame']: row for row in json.loads((model/'camera_poses.json').read_text())}
        metrics = {row['frame']: row for row in info['metrics'] if row['split'] == 'heldout' and row['stage'] == 'merged'}
        views = model/'views' if (model/'views').is_dir() else model/'shared/views'
        rendered = {path.stem.removeprefix('merged_'): path for path in views.glob('batch_*/merged_*.png')
                    if not path.stem.endswith('_alpha')}
        background = np.rint(np.array(info['background_rgb'])[::-1] * 255)
        rows = []
        for pose in poses:
            if not pose['validation']:
                continue
            name = pose['frame']
            other = saved[name]
            if (not other['validation'] or pose['stamp'] != other['stamp'] or pose['scan_stamp'] != other['scan_stamp'] or
                    not np.allclose(pose['T_world_camera'], other['T_world_camera'], rtol=0, atol=1e-7)):
                raise ValueError(f'Not the same held-out camera: {model}/{name}')
            source, = args.prepared.glob(f'submap_*/{name}.bin')
            reference = cv2.imread(str(source.with_suffix('.png')))
            image = cv2.imread(str(rendered[name]))
            dynamic = cv2.imread(str(source.with_name(name+'_dynamic.png')), cv2.IMREAD_GRAYSCALE)
            if (image is None or reference is None or dynamic is None or image.shape != reference.shape or
                    dynamic.shape != image.shape[:2]):
                raise ValueError(f'Missing mask or inconsistent image dimensions: {name}')
            depth = np.fromfile(source, '<f4', count=dynamic.size).reshape(dynamic.shape)
            sky_path = source.with_name(name + '_sky.png')
            sky = cv2.imread(str(sky_path), cv2.IMREAD_GRAYSCALE) if sky_path.is_file() else None
            if sky_path.is_file() and (sky is None or sky.shape != depth.shape):
                raise ValueError(f'Invalid sky mask: {sky_path}')
            scores = static_scores(image, reference, depth, dynamic, background)
            scores.update({key: float(metrics[name][key]) for key in ('psnr_full_db', 'ssim_full', 'depth_coverage',
                          'depth_mae_m', 'depth_30m_mae_m', 'depth_30m_median_m')})
            offset = pose['stamp'] - origin
            window = next((label for label, (lo, hi) in zip(windows, bins) if lo-1e-4 <= offset < hi-1e-4), None)
            if bins and window is None:
                raise ValueError(f'Frame outside requested windows: {name}')
            rows.append(dict(frame=name, window=window, scores=scores,
                             measured_ranges=measured_range_scores(image, reference, depth, dynamic, background),
                             render_support=render_support_scores(rendered[name], depth, dynamic, sky)))
        if not rows:
            raise ValueError('No held-out cameras')
        summary = {}
        for label in ['all', *dict.fromkeys(row['window'] for row in rows if row['window'])]:
            selected = [row for row in rows if label == 'all' or row['window'] == label]
            group = [row['scores'] for row in selected]
            summary[label] = dict(n=len(group), **{key: float(np.mean([item[key] for item in group])) for key in group[0]})
            ranges = {}
            for name in RANGE_BINS:
                valid = [row['measured_ranges'][name] for row in selected if row['measured_ranges'][name]['pixels']]
                ranges[name] = dict(valid_views=len(valid), pixels=sum(item['pixels'] for item in valid), **{
                    key: float(np.mean([item[key] for item in valid])) if valid else None for key in RANGE_METRICS})
            summary[label]['measured_ranges'] = ranges
            summary[label]['render_support'] = summarize_render_support(selected)
        report['models'][str(model.resolve())] = dict(gaussians=info['gaussians'], summary=summary, rows=rows)
    if args.baseline:
        name = str(args.baseline.resolve())
        report['baseline'] = name
        report['comparisons'] = {other: baseline_comparison(report['models'][name], model)
                                 for other, model in report['models'].items() if other != name}
    with args.out.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
