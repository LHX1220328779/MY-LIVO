#!/usr/bin/env python3
"""Evaluate fixed recorded heldout image regions from saved Gaussian RGB/alpha/depth renders."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

FRAMES = ('frame_001796', 'frame_001804', 'frame_001812')
REGIONS = dict(door=(400, 20, 600, 135), bright_ground=(360, 170, 620, 300),
               normal_ground=(30, 200, 300, 340))
SCORES = ('psnr_db', 'render_white_fraction', 'original_white_fraction',
          'new_white_fraction', 'alpha_mean', 'alpha_below_0_05_fraction',
          'depth_coverage', 'depth_mae_m', 'depth_median_m', 'depth_front_over_1m_fraction')


def score_region(image, reference, alpha, rendered_depth, measured_depth, stationary, box):
    x0, y0, x1, y1 = box
    sl = (slice(y0, y1), slice(x0, x1))
    mask = stationary[sl]
    pixels = int(mask.sum())
    measured = measured_depth[sl]
    measured_mask = mask & np.isfinite(measured) & (measured > 0)
    row = dict(box=list(box), pixels=pixels, measured_pixels=int(measured_mask.sum()), depth_pixels=None,
               **{key: None for key in SCORES})
    if pixels:
        left, right = image[sl][mask], reference[sl][mask]
        mse = np.square(left.astype(np.float64)-right.astype(np.float64)).mean()
        white = left.min(axis=1) > 245
        original_white = right.min(axis=1) > 245
        row.update(psnr_db=float(-10*np.log10(max(mse, 1e-9)/255**2)),
                   render_white_fraction=float(white.mean()),
                   original_white_fraction=float(original_white.mean()),
                   new_white_fraction=float((white & ~original_white).mean()))
        if alpha is not None:
            values = alpha[sl][mask]
            row.update(alpha_mean=float(values.mean()),
                       alpha_below_0_05_fraction=float((values < .05).mean()))
    if rendered_depth is not None:
        depth = rendered_depth[sl]
        valid = measured_mask & np.isfinite(depth) & (depth > 0)
        row['depth_pixels'] = int(valid.sum())
        if measured_mask.any():
            row['depth_coverage'] = float(valid.sum()/measured_mask.sum())
        if valid.any():
            signed_error = depth[valid].astype(np.float64)-measured[valid]
            error = np.abs(signed_error)
            row.update(depth_mae_m=float(error.mean()), depth_median_m=float(np.median(error)),
                       depth_front_over_1m_fraction=float((signed_error < -1).mean()))
    return row


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('prepared', type=Path)
    parser.add_argument('models', type=Path, nargs='+')
    parser.add_argument('--out', type=Path, required=True, help='New result JSON file')
    args = parser.parse_args()
    if args.out.exists():
        raise FileExistsError(args.out)
    poses = {row['frame']: row for row in json.loads(
        (args.prepared.parent/'camera_poses.json').read_text())}
    prepared = {}
    for name in FRAMES:
        if name not in poses or not poses[name]['validation']:
            raise ValueError(f'Region evaluation requires a heldout camera: {name}')
        source, = args.prepared.glob(f'submap_*/{name}.bin')
        reference = cv2.imread(str(source.with_suffix('.png')))
        dynamic = cv2.imread(str(source.with_name(name+'_dynamic.png')), cv2.IMREAD_GRAYSCALE)
        if reference is None or dynamic is None or dynamic.shape != reference.shape[:2]:
            raise ValueError(f'Missing prepared RGB/dynamic mask: {name}')
        h, w = dynamic.shape
        measured = np.fromfile(source, '<f4', count=h*w).reshape(h, w)
        kernel = max(3, round(11*w/640) | 1)
        stationary = cv2.dilate(dynamic, np.ones((kernel, kernel), np.uint8)) < 128
        boxes = {label: tuple(round(value * scale) for value, scale in zip(box, (w/640, h/360, w/640, h/360)))
                 for label, box in REGIONS.items()}
        prepared[name] = dict(reference=reference, measured=measured, stationary=stationary, boxes=boxes)
    report = dict(
        prepared=str(args.prepared.resolve()), heldout_frames=list(FRAMES), reference_boxes_640x360=REGIONS,
        definitions=dict(
            regions='Same normalized image rectangles at each recorded camera, scaled from 640x360.',
            masks='Same prepared input and width-scaled 11x11 dilated dynamic exclusion for every model.',
            new_white_fraction='ROI static pixels with every rendered RGB channel >245 and original RGB not all >245.',
            depth='Finite positive exact prepared return pixels in each ROI; depth errors require finite positive '
                  'saved rendered depth. Coverage records the valid rendered fraction of those measured pixels. '
                  'front_over_1m means rendered depth < measured depth - 1 metre.',
            alpha='Saved uint16 composited opacity /65535; missing alpha/depth files produce null corresponding scores.',
            summary='Scores average available views equally; pixel counts sum their supports.'),
        models={})
    for model in dict.fromkeys(path.resolve() for path in args.models):
        info = json.loads((model/'map_info.json').read_text())
        model_poses = {row['frame']: row for row in json.loads((model/'camera_poses.json').read_text())}
        views = model/'views' if (model/'views').is_dir() else model/'shared/views'
        rows = {}
        for name, source in prepared.items():
            pose, other = poses[name], model_poses[name]
            if (not other['validation'] or pose['stamp'] != other['stamp'] or
                    pose['scan_stamp'] != other['scan_stamp'] or
                    not np.allclose(pose['T_world_camera'], other['T_world_camera'], rtol=0, atol=1e-7)):
                raise ValueError(f'Not the same fixed heldout camera: {model}/{name}')
            render_path, = views.glob(f'batch_*/merged_{name}.png')
            image = cv2.imread(str(render_path))
            if image is None or image.shape != source['reference'].shape:
                raise ValueError(f'Inconsistent saved RGB dimensions: {render_path}')
            alpha_path = render_path.with_name(render_path.stem+'_alpha.png')
            alpha = None
            if alpha_path.is_file():
                alpha = cv2.imread(str(alpha_path), cv2.IMREAD_UNCHANGED)
                if alpha is None or alpha.dtype != np.uint16 or alpha.shape != source['measured'].shape:
                    raise ValueError(f'Invalid uint16 saved alpha: {alpha_path}')
                alpha = alpha.astype(np.float64)/65535
            depth_path = render_path.with_name(render_path.stem+'_depth.bin')
            depth = None
            if depth_path.is_file():
                depth = np.fromfile(depth_path, '<f4')
                if depth.size != source['measured'].size:
                    raise ValueError(f'Invalid saved depth length: {depth_path}')
                depth = depth.reshape(source['measured'].shape)
            rows[name] = {label: score_region(image, source['reference'], alpha, depth, source['measured'],
                                              source['stationary'], box)
                          for label, box in source['boxes'].items()}
        summary = {}
        for label in REGIONS:
            selected = [row[label] for row in rows.values()]
            summary[label] = dict(views=len(selected), pixels=sum(row['pixels'] for row in selected),
                                 measured_pixels=sum(row['measured_pixels'] for row in selected),
                                 depth_pixels=sum(row['depth_pixels'] for row in selected
                                                  if row['depth_pixels'] is not None))
            for key in SCORES:
                values = [row[key] for row in selected if row[key] is not None]
                summary[label][key] = float(np.mean(values)) if values else None
        report['models'][str(model)] = dict(gaussians=info['gaussians'], rows=rows, summary=summary)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
