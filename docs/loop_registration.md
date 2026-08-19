# Multi-resolution loop registration

This stage converts every geometrically plausible loop candidate into a raw
point-cloud registration measurement. It intentionally does not decide whether
that measurement is trustworthy and has no reference to the GTSAM graph.

## Coordinate convention and submap

For a candidate pair `historical <- current`, the target submap is constructed
in the historical candidate body frame:

```text
T_candidate_neighbor =
    inverse(T_map_candidate) * T_map_neighbor

P_candidate = T_candidate_neighbor * P_neighbor_body
```

The current keyframe body cloud is the NDT source. Its initial transform is the
detector output `T_candidate_current_initial`, and the final NDT result is
`T_candidate_current`. Keeping NDT coordinates local avoids unnecessary float
precision loss when the mine ENU coordinates become large.

The historical target uses keyframes in a configurable `+/-40` ID window with
stride 4, matching the useful submap idea in Lightning-LM. The current source
remains one keyframe so the later verification stage can interpret overlap
consistently and registration cost stays bounded.

## Coarse-to-fine NDT

The default resolutions follow Lightning-LM:

```text
10 m -> 5 m -> 2 m -> 1 m
```

Each level receives the previous transform as its initial estimate. Source and
target are independently voxelized with:

```text
leaf = max(0.5 m, resolution * 0.25)
```

The 0.5 m lower bound matches the already-downsampled backend keyframe clouds
and avoids wasting time on a nominal 0.1 m filter that cannot recover removed
detail.

For each level the backend records convergence, iteration count,
transformation probability, fitness, point counts, and elapsed time. After the
last level it additionally computes source-to-target overlap and inlier RMSE
with a 1 m nearest-neighbor radius, plus the translation/rotation correction
from the candidate initial estimate.

## Runtime isolation

NDT jobs run FIFO on one background worker. The FAST-LIVO2 thread only freezes
the needed pose snapshots and cloud shared pointers, then enqueues the job.
Submaps are not copied into the queue. The default queue holds 64 candidates,
which is larger than the 30 candidates observed in the current Truck 29 run.
Any queue rejection is logged as an error rather than silently losing a loop.

Shutdown drains pending jobs before closing the CSV files. Wait until the ROS
process exits completely before running validation.

## Outputs

```text
/backend/loop_registrations
Log/backend/loop_registrations.csv
Log/backend/loop_registration_levels.csv
```

RViz displays the registered loop link and the initial-to-final NDT correction.
These markers are diagnostic, not accepted loop edges.

Validate after a complete bag run with:

```bash
./scripts/validate_loop_registration.py
```

The validator requires one processed job per candidate, verifies all four NDT
levels and every transform/metric field, and rejects worker failures or missing
rows. It deliberately does not impose acceptance thresholds on fitness,
overlap, or correction size. Those high-precision gates and neighbor
consistency checks belong to the separate Loop Verification stage.
