# MY-LIVO RTK 融合实验延续状态

> **2026-09-19 更新（run84-89 完成）**：当前基线 **db90565**（run89=局部近邻边+yaw运动σ×3，最优存档）。
> 最优指标：灯柱 0v10 中位 15.27cm（原生 16.0）、位置 100%≤10cm max 9.50cm、地面 5-20m 41.58mm（原生 44.69）、IMU 异侧 52/77。
> 已证伪勿重试：yaw multistart（84）、时代池化加边（85）、sigma 0.035（86，尾部恶化）。
> 指标解读三要点：~8cm 视角本底、z≈48 灯臂簇混构、重影主发于 t≈215-227（0v5=5.2cm vs 5v10=16.4cm）。
> 灯柱顶部 era0 无共视回波，无法跨时代评估。详见 docs/mcp-experiment-log.md 与 results/wuhu_rtk_89_local_yaw3x/RUN.md。
> 以下为历史状态（run83 时代），事实性诊断记录仍有效：

## Protected baseline
- Commit 79304f4 is protected. Do not amend/reset/damage it.
- Production baseline currently considered valid: run81 (2 Hz graph, gain=2, adaptive RTK, unified -0.1 s RTK/INS fusion-time semantics, 144 startup stationary INS samples).
- Run81 fusion: results/wuhu_rtk_81_observable_motion_gain2_full/Log/fusion/1789630332307204894

## Fixed evaluation metrics
- Position target: 80% route <=10 cm, none >20 cm; run81 is substantially better than target.
- Pole XY: map_0 vs map_10 fixed cylinder (-10.75,-274,r=2.5,z=43..54.5), shaft 45.5..48.5.
- Pole vertical metrics are first-class: fixed ground-grid Z delta and fixed lamp-head Z delta, plus position-Z vs attitude-Z correction decomposition.
- Run81 current-evaluator baseline: equal-height XY median 13.61 cm; fixed ground-grid median map10-map0 +20.94 cm; fixed lamp-head mean +21.21 cm.
- Canonical-support comparisons keep exact run81 original point-row support so ROI membership changes cannot fake improvement.

## Important completed conclusions
- Adaptive RTK from smoothed trans_mid observability is useful and retained in run81.
- NativeMotion observability gain=2 is the current actual-map choice; gain=3 did not improve pole Z and worsened some later windows.
- RViz /imu_reference_path had a real 0.1 s time-semantic bug. Fusion graph uses raw odometry sample at graph_time-0.1 s; corrected reference publishing must use the same fusion timestamp. Independent scans support ~-0.1 s for position and attitude on this bag.
- Pole vertical error is not purely Z-position error: correction decomposition shows large opposing position-Z and attitude-induced-Z contributions.
- 5 Hz dense-key run84 is rejected: metre-scale RTK failures and no pole-Z benefit.
- Adding 31 strict continuous-time non-keyframe long-baseline factors symmetrically to run81 is rejected: it pulled reliable early map_0 and worsened canonical lamp-head Z from +21.2 cm to ~+40.3 cm.
- Anchoring early source keys with those 31 factors improved pole Z only ~1 cm; insufficient.
- 10 Hz direct map0->map10 partial pairs exist, but their 24 surfaces are far from pole (nearest ~23 m, median ~59 m).
- Raising single-scan max_planes 800->1600/3200 changed nothing; budget bottleneck only appears for multi-frame submaps.
- A 4.2 s / 10 s-stride / 3200-plane submap test produced a strong direct early->late closure centered ~204.9->224.9 s: 166 candidates, 153 consistent, registered RMS ~9.6 cm, change 13.5 cm / 0.45 deg, RTK baseline diff -4.6 cm with ~6.1 cm sigma. Multi-frame submap is useful as association capture/validation, not as a frozen-pose factor.

## Current hypothesis / active experiment
Offline only: use validated multi-frame submap registration ONLY as association seed. Re-run matching/crop/refit on real single scans in the early/late windows. Final residuals stay tied to each real scan time via the 4-pose InterpolatedScanGeometryAuditFactor. No new high-rate RTK/motion states and no frozen submap pose.

## Source/code state
- Production source must not be changed by this experiment.
- tests/rtk_fusion_math.cpp contains offline-only diagnostics including InterpolatedScanGeometryAuditFactor.
- continuousGeometryPartialPair() was modified offline-only to accept optional association_seed; default callers remain unseeded.
- generateSeededContinuousWindow(...) was inserted before main().
- CLI insertion for --seeded-continuous-window was being attempted when the prior conversation interrupted; it may be incomplete or absent. Inspect before build.
- Offline CONTINUOUS_MAX_PLANES diagnostic exists; default behavior remains production max_planes.

## Next exact action
1. Inspect tests/rtk_fusion_math.cpp around generateSeededContinuousWindow and main() for partial/incomplete CLI insertion.
2. Repair ONLY offline test CLI if needed.
3. Build rtk_fusion_math and run math checks if shared diagnostic code compiles.
4. Execute seeded window around centers ~1785910204.9 and ~1785910224.9, submap window 4.2 s, scan window initially 2.2 s, submap max planes 3200.
5. Before fixed-graph optimization, inspect accepted real-scan pairs/surfaces and their world distance to pole. Proceed only if seed recovers materially more local/direct support than unseeded partial mode.

## Safety
- Do not lower production 12-surface or 3 deg / 1.2 m safety gates just to pass this bag.
- Do not use final error or hand-picked route segment as an online gating signal.
- Do not stage offline diagnostic code as production without full validation.
- Do not reset/amend protected commit 79304f4.
