#!/usr/bin/env python3
"""Compare original delivered PCD observations in the same road ROI; no resampling of map coordinates."""
import argparse
import json
import pathlib

import numpy as np


def chunks(path):
    path = pathlib.Path(path)
    return [path] if path.is_file() else sorted(path.glob('map_[0-9]*.pcd'))


def measure(path, center, rotation, bounds, pixel):
    xmin, xmax, ymin, ymax = bounds
    shape = (int(round((ymax-ymin)/pixel)), int(round((xmax-xmin)/pixel)))
    grid = np.full(shape, np.inf, dtype=np.float32)
    # Per half-metre road patch, double-precision sums for z = a + bx + cy.
    cell = .5
    patch_shape = (int(round((ymax-ymin)/cell)), int(round((xmax-xmin)/cell)))
    sums = np.zeros((np.prod(patch_shape), 10), dtype=np.float64)
    points = 0
    files = chunks(path)
    if not files:
        raise ValueError(f'no PCD chunks: {path}')
    for file in files:
        with open(file, 'rb') as f:
            header = []
            while True:
                line = f.readline()
                if not line:
                    raise ValueError(f'invalid PCD: {file}')
                header.append(line)
                if line.startswith(b'DATA '):
                    break
            if line.strip() != b'DATA binary' or b'FIELDS x y z intensity\n' not in header:
                raise ValueError(f'expected binary XYZI PCD: {file}')
            offset = f.tell()
        data = np.memmap(file, dtype='<f4', mode='r', offset=offset).reshape(-1, 4)
        for first in range(0, len(data), 500000):
            p = data[first:first+500000]
            xy = (p[:, :2].astype(np.float64)-center) @ rotation
            valid = ((xy[:, 0]>=xmin) & (xy[:, 0]<xmax) & (xy[:, 1]>=ymin) & (xy[:, 1]<ymax)
                     & np.isfinite(p[:, 2]))
            xy = xy[valid]-[xmin, ymin]
            z = p[valid, 2].astype(np.float64)
            points += len(z)
            ij = np.floor(xy/pixel).astype(np.int64)
            np.minimum.at(grid, (ij[:, 1], ij[:, 0]), z)
            ij = np.floor(xy/cell).astype(np.int64)
            ids = ij[:, 1]*patch_shape[1]+ij[:, 0]
            dx, dy = (xy-ij*cell).T
            # Subtract an arbitrary common height for stable second moments.
            z = z-180.
            terms = [np.ones(len(z)), dx, dy, z, dx*dx, dx*dy, dx*z, dy*dy, dy*z, z*z]
            for i, term in enumerate(terms):
                sums[:, i] += np.bincount(ids, weights=term, minlength=len(sums))
    n,x,y,z,xx,xy,xz,yy,yz,zz = sums.T
    good = n>=20
    ids = np.flatnonzero(good)
    a = np.array([[n[good], x[good], y[good]], [x[good], xx[good], xy[good]],
                  [y[good], xy[good], yy[good]]]).transpose(2,0,1)
    b = np.array([z[good], xz[good], yz[good]]).T
    sol = np.einsum('nij,nj->ni', np.linalg.pinv(a), b)
    rms = np.sqrt(np.maximum(0, (zz[good]-np.sum(sol*b, axis=1))/n[good]))
    ground = (np.linalg.norm(sol[:, 1:], axis=1)<.5) & (rms<.15)
    return grid, dict(zip(ids[ground].tolist(), rms[ground].tolist())), points


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('before'); ap.add_argument('after'); ap.add_argument('output')
    ap.add_argument('--center', nargs=2, type=float, required=True)
    ap.add_argument('--angle-deg', type=float, default=0)
    ap.add_argument('--bounds', nargs=4, type=float, default=[-60,60,-5,10])
    ap.add_argument('--pixel', type=float, default=.05)
    args = ap.parse_args()
    out = pathlib.Path(args.output); out.mkdir(parents=True, exist_ok=True)
    angle=np.deg2rad(args.angle_deg)
    rot=np.array([[np.cos(angle),-np.sin(angle)],[np.sin(angle),np.cos(angle)]])
    results=[measure(p,np.array(args.center),rot,args.bounds,args.pixel) for p in [args.before,args.after]]
    before,after=[np.isfinite(r[0]) for r in results]
    common=sorted(set(results[0][1]) & set(results[1][1]))
    stats={'center':args.center,'angle_deg':args.angle_deg,'bounds':args.bounds,'pixel_m':args.pixel,
           'common_ground_patches':len(common), 'new_pixels':int(np.sum(after & ~before)),
           'lost_pixels':int(np.sum(before & ~after))}
    for tag,r in zip(['before','after'],results):
        vals=np.array([r[1][i] for i in common])
        stats[tag]={'ROI_points':r[2],'occupied_pixels':int(np.isfinite(r[0]).sum()),
                    'plane_rms_median_m':float(np.median(vals)), 'plane_rms_p90_m':float(np.percentile(vals,90))}
        np.save(out/(tag+'_height.npy'),r[0])
    (out/'comparison.json').write_text(json.dumps(stats,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(3,1,figsize=(18,8))
    z=results[0][0][before];lo,hi=np.percentile(z,[2,98])
    for a,tag,r in zip(ax,['Before','After'],results):
        a.imshow(r[0],origin='lower',extent=args.bounds,cmap='terrain',vmin=lo,vmax=hi)
        a.set_title(tag);a.set_aspect('equal')
    gain=np.zeros(before.shape);gain[after & ~before]=1;gain[before & ~after]=-1
    ax[2].imshow(gain,origin='lower',extent=args.bounds,cmap='bwr_r',vmin=-1,vmax=1)
    ax[2].set_title('New coverage: blue; lost coverage: red');ax[2].set_aspect('equal')
    fig.tight_layout();fig.savefig(out/'coverage.png',dpi=160)
    print(json.dumps(stats,indent=2))


if __name__=='__main__':
    main()
