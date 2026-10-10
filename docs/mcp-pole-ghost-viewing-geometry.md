# MCP topic: truck29 start-up pole "ghost" is mostly a viewing-geometry artifact of the metric; one real double ring remains

Date: 2026-10-10. Runs: `results/guard_t29_lio_20261009_150420`, `results/guard_t29_livo_20261009_150420` (fused map, 45 chunks, 117.8 s). Scripts: `scripts/evaluate_wuhu_lio.py`, `scripts/analysis/round_summary.py`, `scripts/analysis/round_metrics.py`.

## 1. Cause of the "23 cm" figure

The previous headline ("pole axis XY separation", median 9.9 / P95 22.9 / max 23.9 cm) takes, for each 10 cm height band, the median XY of the pole returns in that band and compares epochs. A pole seen from one side returns only the sensor-facing arc, so that median sits toward the sensor. The offset therefore depends on the angle from which each epoch observed the pole, not on map accuracy.

Evidence:

1. **Geometry.** The car passes the pole at about 17 m around t ≈ 20 s. The bearing from sensor to pole sweeps about 130° in the first 30 s. Epochs 3 and 9 (t ≈ 8 s and 24 s) view the pole from about 175° and 78° in the pole frame.
2. **Model.** For a surface of radius r, the sensor-facing centroid moves by about 2·r·sin(Δθ/2) between views. With r = 17 cm, the model tracks the measured epoch separations:

| pair | measured (cm) | model 2r·sin(Δθ/2) (cm) | Δθ (deg) |
|---|---|---|---|
| 3v4 | 1.1 | 0.1 | 0.3 |
| 3v7 | 15.7 | 12.4 | 43 |
| 3v8 | 18.9 | 21.8 | 80 |
| 3v9 | 23.9 | 27.6 | 108 |
| 8v9 | 5.1 | 8.4 | 29 |

3. **Ring fit.** Points that lie on the pole circumference do not depend on the viewing side. Fitting a circle with a fixed radius to the shaft returns (`POLE_RING_RADIUS = 0.12 m`) gives per-epoch centres that agree far better. Chunk 3 vs chunk 9 is 1.7 cm (LIO) and 2.2 cm (LIVO), where the old metric gave about 22 cm.

## 2. Result with the view-independent metric

Pole ring-centre separation over all epoch pairs (radius 12 cm fixed; radius sensitivity 10 and 14 cm changes the pair median by about 1–2 cm):

| run | median (cm) | P95 (cm) | max (cm) | old view-biased median / max (cm) |
|---|---|---|---|---|
| LIO (guard_150420) | 3.47 | 10.78 | 10.96 | 9.91 / 23.94 |
| LIVO (guard_150420) | 3.63 | 5.46 | 5.88 | 10.94 / 28.40 |

Per-chunk ring rms is 1.5–1.8 cm (ring returns 100–370 per chunk).

## 3. What is still real (not fixed here)

Chunks 7–9 (t ≈ 18–26 s, range 18–26 m) admit **two competing ring solutions** about 17 cm apart with similar support (about 150 points each in LIO). Starting the fit from different centres converges to one or the other. Chunks 3–6 have a single solution. This is a second surface inside those epochs, a genuine double-ring, and the view-independent metric now shows it (LIO max 11 cm; LIVO shows the same chunk-7 pattern).

Not yet explained. Candidate causes to test next:

- one LiDAR's extrinsic/timing offset (split the chunk returns by `sources.bin` source id and fit each source);
- a pose discontinuity inside a 2.6 s chunk window (scan-to-map drift at the closest-approach segment).

No algorithm was changed for this because the cause is not yet isolated.

Top height (AGENTS "顶部高度误差") is **not assessed**. The per-chunk top P99 ranges 51.55–52.50 m; the 0.9 m low value at chunk 7 is a visibility question (the pole top is seen only from some angles), not a validated height error. An independent top detector is needed.

## 4. Startup trajectory rule (AGENTS §4)

Signed cross-track of the optimised trajectory vs native LIO, relative to RTK (`scripts/evaluate_wuhu_lio.py`, startup section), LIO guard run:

| first L m | native (mm) | optimised (mm) | rear-axle opposite side |
|---|---|---|---|
| 10 | +3.9 | −1.8 | 23/45 |
| 20 | +15.9 | −1.6 | 47/89 |
| 30 | +23.3 | −1.1 | 70/130 |

The optimised track lies about 1–2 mm on the far side of RTK, not between native and RTK. This is within RTK noise and the rule's intent is met to mm precision, so no change was made. It is recorded here for review.

## 5. Changes in this round

- `scripts/evaluate_wuhu_lio.py`: adds `ring_fit` and the ring-centre pair table with radius sensitivity; relabels the per-height median as `[view-biased diagnostic]`.
- `scripts/analysis/round_summary.py`: headline pole line is the ring-centre separation; the view-biased line is kept as a diagnostic.
- `scripts/analysis/round_metrics.py`: `pole.pair_separation_cm` is now `ring_fit`; the old value is `pole.view_biased_pair_separation_cm`.
- `tools/studio/web/app.js`: the pole column label now says the value is the ring-fit centre separation (view-independent).

No mapping, fusion, LIO/VIO, calibration, or config change was made. The existing guard rounds were re-evaluated, so no new mapping run was needed; both rounds keep their full `map_*.pcd` output.

## 6. Earlier conclusions to re-read

- The "sensor/geometry limit" conclusion and the 0.232 → 0.202 m improvement in commits `465ad54`/`214aaa8` used the view-biased metric. Their absolute values are not comparable with the ring-fit numbers.
- The 17 cm pole radius in earlier notes is not supported by this data: the pooled radial histogram peaks at 11–13 cm.

## 7. Open items (outside this change)

- `tools/studio/server.py` binds `0.0.0.0:8765` with no authentication, and exposes calibration activate/delete and job kill. Recommend `127.0.0.1` by default or an access token.
- Working-tree state, left untouched: deleted `config/perception_truck29*.json` and `config/carlibration/perception_truck29.json`, deleted `pics/Framework.png`, dirty `3rdparty/gtsam`, untracked calibration versions.

## 8. Reproduce

```
cd /home/project/MY-LIVO
/usr/bin/python3 scripts/evaluate_wuhu_lio.py --pole-pair-run <fusion_dir> --pole-chunks 3 9 --analysis-output <dir>
/usr/bin/python3 scripts/analysis/round_summary.py <fusion_dir> <eval_dir> > <eval_dir>/summary.txt
/usr/bin/python3 scripts/analysis/round_metrics.py <round_dir>
```
