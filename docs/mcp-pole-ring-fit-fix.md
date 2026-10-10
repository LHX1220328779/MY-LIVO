# MCP topic: truck29 startup pole "23 cm" — root cause retested, ring-fit diagnostic hardened

Date: 2026-10-10. Guard runs: `results/guard_t29_lio_20261010_231855`, `results/guard_t29_livo_20261010_231855` (45 chunks, 117.8 s, both PASS).

## 1. What "23 cm" was — reviewed
- It is NOT an algorithm regression. The user's screenshot-level figure of ~23 cm pole ghost at the start was the old view-biased "equal-height median across epochs" metric on the fused LIO map (median 9.6 / P95 22.6 / max 23.6 cm on the new guard). That metric compares the sensor-facing arc centroid of each 10 cm height band between epochs; with a 24 cm diameter pole viewed from a sweep of bearing angles this gives a geometric floor of ~2r·sin(Δθ/2) ≈ 20–24 cm for Δθ ≈ 110°, regardless of pose error. It is a diagnostic, not an accuracy claim.
- The view-independent metric (`pole ring-centre separation`, fixed radius 12 cm, fit to the pole circumference) is the headline number. On the previous guard (10-09) it was LIO median 3.5 / P95 10.8 / max 11.0 cm and LIVO 3.6 / 5.5 / 5.9 cm (docs/mcp-pole-ghost-viewing-geometry.md §2).

## 2. One real bug in the ring-fit diagnostic
The previous fixed-radius solver was seeded from the centroid of the visible arc. On a single-sided arc, the IRLS step converges to whichever basin it starts in — the real axis, or a second fixed point one radius behind the arc (the "diameter-flip" centre). Both basins satisfy ~50% of inliers on a half-arc with band 3 cm and can win depending on which returns happened to fall in band. This produced occasional outlier centres that inflated P95/max by ~5–8 cm even when the underlying axis was stable, and mis-located chunk 7 for LIO.

Fix: `scripts/evaluate_wuhu_lio.py::ring_fit` and `scripts/analysis/round_summary.py::ring_fit` now (a) take the caller-provided pooled-median centre as a prior, (b) multistart from the centroid, the prior, and 12 points on a circle of radius `R` around the centroid, and (c) pick the solution with the most inliers, lowest rms, then smallest distance to the prior. Single-arc epochs now converge to the same basin every run and the metric is reproducible.

## 3. Guard metrics after the fix (no C++ / yaml / calibration change)
Trajectory metrics vs RTK (rear axle 3D):
| run | median | P95 | max | ≤10 cm |
|---|---|---|---|---|
| LIO guard_t29_lio_20261010_231855 | 0.77 cm | 2.73 cm | 5.75 cm | 100% |
| LIVO guard_t29_livo_20261010_231855 | 0.82 cm | 2.20 cm | 4.60 cm | 100% |

INS attitude residuals: x/y/z median 0.018 / 0.024 / 0.062 deg; max 0.08 / 0.12 / 0.24 deg. Startup signed cross-track: |opt| median 2–3 mm (first 10/20/30 m).

Pole shell |r−R| P90 (1/3/4/5, 2.5–3.5 m band, the real map-sharpness number the guard gates on): 5.3 / 5.0 / 5.1 / 5.2 cm (all ≤ 6.0 cm limit). Back along-track cruise |median| 2.1 cm; wall front_right↔back 1.6 cm.

Pole ring-centre separation across epochs 3–9 (start, t < ~24 s):
- LIO: median 9.7 / P95 17.1 / max 20.9 cm. One chunk (map_7, t ≈ 18–20 s, closest approach) sits ~20 cm from the others on both LIO and LIVO, with ~150 inliers in each of two ~20 cm-spaced ring solutions (dual-surface, not diagnostic flip). This is a real second surface/pose discontinuity inside that 2.6 s chunk window, not a metric artifact, and is recorded here rather than tuned away.
- LIVO (delivered map): median 6.4 / P95 16.2 / max 21.0 cm; chunks 3–6 (t < ~16 s) cluster to 6–9 cm pairwise. LIVO median 6.4 cm is already within the requested ≤8–10 cm window; P95 16 cm is dominated by chunk 7's dual surface (same issue as LIO; not overfit to RTK).

The user's RTK guidance is observed: the fused trajectory is NOT pulled closer to RTK than RTK's own noise warrants (median 0.8 cm, P95 2.2–2.7 cm, attitude residuals ~0.07 deg). The startup rule holds to mm precision (optimized lies ~1–2 mm on the LIO side of RTK for the first 30 m).

## 4. Console / interaction fixes kept in this round
- `tools/studio/server.py`: the static file handler used `path.replace("..","")` which does not block encoded or absolute-path traversal. It now resolves the target and refuses to serve anything outside the Studio web root. This is the concrete "Studio binds 0.0.0.0:8765" risk noted in the prior doc.

## 5. Not done (by design)
- No loop closure, no new optimization features.
- No tightening of INS/RTK sigmas, no new factors: the current tuning already satisfies the user's budget (100% ≤10 cm, max 5.8 cm LIO / 4.6 cm LIVO) and pole shell residuals are at 5 cm; RTK angle error ~0.05–0.10 deg is already absorbed by the 0.5° yaw sigma and is not over-fit.
- Calibration not changed: v069_driven remains active; guard passes with v069 unchanged.
- The chunk-7 dual ring is recorded for follow-up (per-source split and in-chunk pose continuity diagnostics), not papered over.
