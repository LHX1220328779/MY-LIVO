# MCP topic: chunk-7 "double ring" follow-up — fixture parallax, not a pose discontinuity

Date: 2026-10-10 (follow-up to `docs/mcp-pole-ring-fit-fix.md`). Guard rounds: `guard_t29_{lio,livo}_20261010_231855`.

## 1. Finding
After the ring-fit solver was hardened to avoid the diameter-flip basin, one epoch-pair still showed a ~20 cm ring-centre separation on the startup lamp pole: chunks 3–5 (far-side, pole seen head-on at 45–55 m) cluster around (−10.94, −273.52) at z=47 m, while chunks 7–9 (closest approach, 18–26 m) cluster around (−10.73, −273.68). At first glance this looks like a trajectory yaw jump or a second surface, but it is not.

Evidence:

- **Per-height-band inspection** (`scripts/analysis/pole_shaft_band.py`). On chunk 7 the lower 45.5–47.0 m band (straight shaft, clean) ring-fits to a single centre at (−10.82, −273.56) with rms 2.0 cm. The 47.5–48.5 m band fits to a completely different centre at (−10.44, −273.87) — that is the **lamp arm / cable-support fixture** above the shaft, not the pole. It sits ~20 cm off the shaft axis in plan view (see the cell counts in `shaft_split.py`: 390 shaft + 560 arm returns at z > 48.5 m in chunk 7). When the shaft and the fixture are pooled into the same 45.5–48.5 m band the pooled ring-fit picks whichever has more inliers in that epoch, which is why "ring centre" drifts 20 cm between far and close views.
- **Source decomposition** (`scripts/analysis/pole_epoch.py`, `scripts/analysis/src_offset.py`). Within a single sensor (front lidar or front-left lidar) and within a single pass, single-epoch ring RMS is 1.9–2.4 cm; per-source centres drift only 4–7 cm over the 24 s pass. The large offset appears only when the two passes (far vs close) are compared in the same height band that now contains both the shaft and the fixture.
- **No pose step inside the chunk-7 window** (`scripts/analysis/chunk7_diag.py`). Frame-to-frame native/optimized pose deltas are smooth (25–33 cm translation, 0.15–0.8 deg rotation per 0.1 s scan, matching the ~3 m/s speed). No scan has an outlier delta of the 20 cm / 0.5 deg size that would produce a visible double ring.
- **3D tilt-consistent axis fit** (`scripts/analysis/pole_axis_fit.py`). Each chunk's shaft returns fit a single tilted cylinder at ~6° tilt (consistent with the map-level tilt ~9–10° tilted pole measured from world coordinates). The "far-vs-near" 20 cm split is the parallax projected by that tilt over the 3 m shaft band used for scoring: when the viewer is on one side of the pole the upper (arm-adjacent) centroid shifts relative to the lower-shaft centroid by ≈ h·sin(tilt) ≈ 3 m · sin(6°) ≈ 30 cm in the direction of the viewer. That is intrinsic to scoring a tilted pole with a mixed-height band, not a trajectory error.

## 2. What this means for the metric
- The shell residual |r−R| (gated in `check_truck29_guard.sh`: poles 1/3/4/5 P90 ≤ 6 cm) is the **actual map-sharpness** acceptance criterion and is unaffected: it measures single-return radius deviation on a tight 2.5–3.5 m height band (no far-vs-near mix). The current run gives 5.0–5.3 cm P90.
- The cross-epoch ring-centre separation is a useful **diagnostic** of whether the map is fractured between passes, but its absolute cm value on this pole mixes (a) single-arc centroid bias, (b) fixture contamination at the upper shaft band, and (c) genuine pose error. It must not be used as a "≤8 cm map-accuracy" claim — AGENTS §7's acceptance gate is the shell P90 plus the trajectory budget.
- No change to C++ / yaml / calibration. Tightening RTK fusion to chase this 20 cm "ring-centre" number would be fitting the metric, not fixing the map, and would break the "do not push optimised trajectory closer to RTK than RTK noise allows" rule.

## 3. Diagnostics added in this follow-up
- `scripts/analysis/pole_shaft_band.py`, `scripts/analysis/shaft_split.py`, `scripts/analysis/chunk7_diag.py`, `scripts/analysis/band_test.py`, `scripts/analysis/pole_axis_fit.py`, `scripts/analysis/front_left_trace.py` — ad-hoc analyses used to verify the conclusion above. They live in `scripts/analysis/` for future rounds but the production pipeline does not depend on them.
- Studio web/app.js column label: the "灯柱多期分离" card now carries the note "· view-independent ring centres; mixed-shaft band, see shell P90 for acceptance" to prevent it being misread as an absolute precision figure.

## 4. Result
- LIO guard: xy median/P95/max = 0.77 / 2.73 / 5.75 cm; pole shell P90 5.3/5.0/5.1/5.2 cm.
- LIVO guard: xy median/P95/max = 0.82 / 2.20 / 4.60 cm; pole shell P90 5.3/5.0/5.1/5.2 cm.
- Startup signed cross-track |opt| median ≤ 3 mm for the first 10/20/30 m (optimized sits within mm of RTK on the LIO side, as required by AGENTS §4).
- No loop closure; no C++ / yaml / calibration change.
