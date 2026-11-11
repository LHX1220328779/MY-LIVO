#!/usr/bin/env python3
"""Compare axle calibrations on identical returns and cross-time plane neighbours.

Both calibrations use exact point-time INS interpolation. Evaluate each choice of
neighbourhood, so a score cannot improve only because the candidate selected it.
This is a consistency diagnostic against recorded INS, not independent ground truth.
"""
import argparse
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import refine_rear_axle_extrinsic as axle


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--before", required=True)
    ap.add_argument("--after", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--output", required=True)
    a = ap.parse_args()
    pathlib.Path(a.output).parent.mkdir(parents=True, exist_ok=True)
    configs = [json.loads(pathlib.Path(p).read_text()) for p in (a.before, a.after)]
    axle.BODY_FILTER = axle.normalize(configs[0])
    Ts = [np.asarray(next(e[axle.REFERENCE_KEY] for e in c["lidar"] if axle.REFERENCE_KEY in e)).reshape(4, 4)
          for c in configs]
    offsets = [c["time_offsets"] for c in configs]
    ins, scans = axle.read_recording(a.bag, offsets[0]["lidar"], (0, 1e9), .25, 3, 80, 2)
    poses = axle.InsPoses(ins, offsets[0]["rtk_position"], offsets[0]["ins_attitude"])
    data, _ = axle.assemble(scans, poses, Ts[0], 1, 5)
    used = np.unique(data["scan"])[::int(np.ceil(len(np.unique(data["scan"])) / 700))]
    keep = np.isin(data["scan"], used)
    P, tau = data["P"][keep], data["tau"][keep]
    del data
    worlds = []
    for c, T, off in zip(configs, Ts, offsets):
        poses = axle.InsPoses(ins, off["rtk_position"], off["ins_attitude"])
        rot, pos = poses.at(tau + off["lidar"] - offsets[0]["lidar"])
        worlds.append(np.einsum("nij,nj->ni", rot, P @ T[:3, :3].T + T[:3, 3]) + pos)
    del rot, pos
    sel = np.sort(np.random.default_rng(20261111).choice(len(P), min(200000, len(P)), replace=False))
    out = {"before": a.before, "after": a.after, "observations": len(P), "queries": len(sel),
           "range_m": [3, 80], "cross_time_gap_s": 1.5, "neighbour_radius_m": .5,
           "note": "Exact point-time poses; shared observations and neighbours; INS consistency only.", "scores": []}
    for ref in range(2):
        tree = axle.cKDTree(worlds[ref])
        residuals, normals = [[], []], []
        for chunk in np.array_split(sel, 8):
            dist, nb = tree.query(worlds[ref][chunk], k=20, distance_upper_bound=.5, workers=2)
            valid = np.isfinite(dist)
            nb = np.where(valid, nb, 0)
            valid &= np.abs(tau[nb] - tau[chunk, None]) > 1.5
            counts = valid.sum(1)
            ok = counts >= 6
            chunk, nb, valid, counts = chunk[ok], nb[ok], valid[ok], counts[ok]
            planes, masks = [], []
            for world in worlds:
                X = world[nb]
                cen = (X * valid[..., None]).sum(1) / counts[:, None]
                D = (X - cen[:, None]) * valid[..., None]
                ev, eigvec = np.linalg.eigh(np.einsum("nki,nkj->nij", D, D) / counts[:, None, None])
                normal = eigvec[:, :, 0]
                masks.append((ev[:, 0] < .04**2) & (ev[:, 1] > 3 * ev[:, 0]))
                planes.append((cen, normal))
            common = masks[0] & masks[1]
            normals.extend(np.abs(planes[ref][1][common, 2]))
            for j in range(2):
                cen, normal = planes[j]
                residuals[j].extend(np.abs(np.einsum("ni,ni->n", normal, worlds[j][chunk] - cen))[common])
        normals = np.asarray(normals)
        r0, r1 = map(np.asarray, residuals)
        groups = {}
        for name, mask in (("all", np.ones(len(r0), bool)), ("ground", normals > .8), ("vertical", normals < .3)):
            groups[name] = {"n": int(mask.sum()), "before_median_p95_mm": np.quantile(r0[mask] * 1000, [.5, .95]).tolist(),
                            "after_median_p95_mm": np.quantile(r1[mask] * 1000, [.5, .95]).tolist(),
                            "paired_delta_median_p95_mm": np.quantile((r1[mask] - r0[mask]) * 1000, [.5, .95]).tolist()} if mask.any() else {"n": 0}
        out["scores"].append({"neighbours_from": ("before", "after")[ref], "groups": groups})
        print(json.dumps(out["scores"][-1]), flush=True)
        del tree
    pathlib.Path(a.output).write_text(json.dumps(out, indent=2) + "\n")


if __name__ == "__main__":
    main()
