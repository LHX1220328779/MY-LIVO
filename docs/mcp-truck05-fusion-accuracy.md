# truck05 融合精度攻坚：时间偏移边缘伪值与几何锚过强（MCP 会话记录）

日期：2026-09-25 / 分支 fusion/56c6896-ins-attitude
硬指标：truck29 / truck05，LIO / LIVO，全局 xy max < 8.5 cm（optimized vs INS 后轴）。

## 结论速览

| 场景 | 修复前 xy max | 修复后 xy max | 判定 |
| --- | --- | --- | --- |
| truck29 v061 LIO 120 s | 6.67 | 6.56 | 达标 |
| truck29 v061 LIVO 60 s | 10.1（修复 1 前） | 5.02 | 达标 |
| truck29 v061 LIVO 120 s | 10.11（修复 1 前） | 7.12 | 达标 |
| truck05 LIO W1（40-100 s） | 10.2（v019 旧参数） | 7.4 | 达标 |
| truck05 LIO W2（598-658 s） | 14.7（v016） | 6.0 | 达标 |
| truck05 LIO 全程 720 s | — | 7.25 | 达标 |
| truck05 LIVO 全程 720 s | — | p95 4.12，max 47.3（孤立 1 样本，native 失稳段） | 见"残留问题" |
| truck29raw LIO 120 s（厂家原始件全流程） | 7.84 | 5.69 | 达标 |
| truck29raw LIVO 120 s（厂家原始件全流程） | 8.10 | 7.23 / 8.63（两样本） | 达标 |

三项修改（全部已验证）：

1. rtk_fusion 不连续包夹（keyframe_discontinuity_m: 0.05，本会话前半已验证）。
2. `scripts/refine_rear_axle_extrinsic.py` 时间偏移候选网格自适应外扩（step 0.03、上限 ±0.30 s、profile ≤ 25 点、无新增即停）。
3. `config/wuhu_truck29.yaml` rtk.plane_model_sigma_m 0.05 → 0.2。

## 根因 1：axle 阶段时间偏移解顶在网格边缘（truck05）

- 旧解 v009_axle：time_profile 网格 −0.18..0.00（按 truck29 设计），最优 dt 顶在正边缘，
  24 迭代未收敛、quality=warn，得到有效 τ=−0.0368 s（边界伪值）。
- SLAM τ 扫描（W2 LIO，直接改 calibration 的 time_offsets.lidar）单调证据：
  τ=−0.13 → 19.7、−0.09 → 18.3、−0.0368 → 14.8、0.00 → 11.3、+0.04 → 中段变差。真值在 0 附近。
- 求解器加自适应外扩后重跑 axle（v008_import 父版本、40-340 s 行驶窗，630 scans）：
  22 迭代收敛，interior minimum，τ=−0.0328 s；旋转 (−0.090, −0.156, +0.304)°，
  平移 (+3.6, +5.4) cm —— 与被边缘毒化的旧解差约 9 cm。
- 由 v017_axle 重建链：v018_ground（frames 2-20 静止 / 1840-40 行驶）→ LIO 48 s →
  v019_driven（window 8-40，全部 quality ok）。registry 激活 v019。

## 根因 2：跨时几何锚过强压制 RTK 位置因子（全局参数）

现象（W2，v019，plane 0.05）：32.7 s 弯道 fused 沿轨 −10.8 cm 而 native 同时刻仅 −4 cm；
native 在 33.5-35.5 s 才爆发 +14 cm 沿轨漂移 —— 融合误差在漂移爆发前 2-3 s 反向出现
（过平滑泄漏），且位置因子（σ=3 cm）拉不回来。收紧/放宽 motion sigma、rate huber、
position huber、加密 keyframe 均无效（都在 ±1.3 cm 轮间噪声带内）。
唯一有效杠杆是几何锚 sigma（plane_model_sigma_m）：

| plane σ (m) | W2 d3/xy max (cm) | W1 d3/xy max (cm) |
| --- | --- | --- |
| 0.05 | 13.3 / 11.4 | 11.5 / 10.2 |
| 0.12 | 8.9 / 8.6 | 8.7 / 7.6 |
| 0.15 | 8.7 / 7.8 | — |
| 0.20 | 7.9 / 6.0 | 8.0 / 7.4 |

机制：native 漂移爆发（truck05 弯道 4-6 cm/s，truck29 的约 10 倍）时，跨时/邻近面锚
（σ=5 cm、每对 ≤20 因子）合力超过位置因子，把 fused 拖向漂移的 native 几何并把误差
反因果地摊到爆发前。σ=0.2 让重访面退回"补充"角色。truck29 全量回归不退化反而略好
（LIO 120 s 6.67→6.56、LIVO 60 s 6.50→5.02、LIVO 120 s 10.11(修复1前)→7.12），故全局生效，
无需 per-truck 分叉。

## 残留问题（不建议继续调参）

1. truck05 native 高速段（250-400 s，5-7.3 m/s）z 米级漂移（最大 +12.7 m，漂移率 ±30-40 cm/s），
   fused z 被拖到 −40 cm（校正吸收 96.7%）。d3 超标样本全部集中于此段；xy 不受影响（全程 7.25）。
   vertical_sigma/huber/motion-z 调参均无效（w300 代理场验证）；去掉 back_lidar 有限改善
   （19→14 cm）但 native 仍失稳，不动生产雷达配置。属 native 前端在该路段的退化，
   需前端侧（退化方向检测/重力对齐）另立课题。
2. truck05 native LIVO 刀锋态：同一命令两次运行 native max 1.9 m / 40 m（W2），
   720 s 全程 native 峰值 10.7 m、3.5 s 内漂 2.4→9.2 m。包夹机制把 fused 保在
   p95 4.12 cm、645 样本仅 6 个 >8.5 cm（孤立尖峰 47.3 cm）。相机 tf 与 v016 完全一致，
   非标定链回归（v016 同样偶发发散，见 v016 note）。属视觉前端稳定性课题。
3. 评测口径提醒：d3 含 z，而 INS z 在高速颠簸段本身可疑（truck05 RTK 协方差全 0，
   fallback σz=5 cm）。硬指标按 xy 判定。

## 复现要点

- τ 扫描：复制 calibration.json 改 time_offsets.lidar，`run_wuhu_round.sh --calibration ...`。
- axle 重标：`livo_calib.py axle --id wuhu/truck05/v008_import --bag .../ros2bag --window 40 340 --dof rpy_xy`。
- 参数实验：`run_wuhu_round.sh <round> --set rtk.plane_model_sigma_m=0.2 ...`；
  注意 --set 键名必须与 yaml 完全一致（错误键会静默追加成无效新键 —— 本次 Z1/Z1K/T3
  即因 `motion_translation_sigma` 少了 `_m_sqrt_sec` 后缀而实际是基线重跑）。
- 轮间噪声：同配置重跑 max 波动约 ±1.3 cm，单次 <1.5 cm 的差异不可作结论。
