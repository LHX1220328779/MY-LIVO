# MCP 第 91–99 轮：根因分析与前后端修复记录（基线 56c6896 → 当前 run99）

分支 `fusion/56c6896-ins-attitude`（基于 `56c6896`）。所有运行结果、全部 `map_*.pcd`、评估与 RUN.md 均在 `results/wuhu_rtk_9x_*/`；本文件只记录事实、方法与结论。

## 1. 基线（run90 = 56c6896）诊断出的四个根因

| # | 现象 | 证据 | 根因 |
| --- | --- | --- | --- |
| R1 | 原生 LIO 俯仰以 ~0.01°/m 线性漂移（75 s 达 3.8°），加计零偏被迫增长到 0.32 m/s²，77 s 后在退化直道上速度发散到 19 m/s（真值 5 m/s），原生位置误差 380 m，后段地图完全失真 | `Log/mat_out.txt`、`scripts/analysis/round_summary.py` 的原生姿态 vs INS 表；陀螺零偏初值正确与否不改变漂移（run92） | 扫描-地图配准只能观测相对自建地图的姿态；EKF 内重力约束远弱于每帧数千条点面行，系统性配准俯仰偏置无界积累，被加计零偏吸收后在无前向观测路段泄漏为速度 |
| R2 | 四雷达合并帧地面分层，地面"厚"，远处结构层内涂抹 | 静止段共视地面点到面：front_left 相对 front 横滚差 **-1.31°**（12–27 m 处高度差中位 16 cm）、front_right +0.37°、back +0.23/-0.13°；行驶窗口 6 自由度配准：两侧雷达前移 6.5–7 cm、back 高度 -10 cm | 手工外参（`multi_lidar_to_rear_axle_manual`）误差 |
| R3 | 转弯段原生偏航滞后 INS，去畸变/传播用错 IMU 窗口 | 原生姿态与（与原始陀螺同步、滞后仅 -0.01 s 的）INS 姿态之差对机体角速度回归：偏航通道零点在 **+0.125 s**（run90/93 一致），配准 RMS 与地面厚度随该补偿同步改善 | LiDAR 帧头相对 IMU/INS 时钟晚 ~0.125 s；旧配置只在后端用 -0.1 s 补偿了位置关联 |
| R4 | 优化后横滚相对 INS 中位 0.6°（P95 1.5°）、后段偏航 2.5°，IMU 轨迹 P95 22 cm；灯柱顶部（距传感器 ~10 m）随姿态差移动 | run90 `summary.txt`；4.68 m 杆臂 × 2.5° = 20 cm | 后端只有后轴 RTK 位置因子，姿态仅经 ~5 m 杆臂间接可观 |

另查明：INS 位置输出与其姿态/陀螺存在 ~0.04 s 的相对时延（位置需 -0.10 s、姿态需 ~-0.14 s 才与扫描对齐），单天线 INS 静止/起步阶段航向为陀螺保持，开始运动后才重新对准。

## 2. 改动（按轮次，均为单变量或明确组合）

| 轮 | 改动 | 关键结果 |
| --- | --- | --- |
| 91 | 后端 `InsAttitudeFactor`（残差 `Log((R_ins·M)^T R_i)`，单个安装角变量 `M`，σ 0.15/0.15/0.3°，Huber 2.8，与位置因子同门控/时序/时间权重） | 姿态 vs INS：横滚中位 0.60→0.12°，偏航 P95 2.85→0.37°；IMU 轨迹 P95 22→6.4 cm |
| 92 | `initial_gyr_bias_sigma` 0.0005→0.0001 | 初值保持正确，但漂移不变 → 证伪"零偏初值"假设 |
| 93 | 雷达外参 v1（`scripts/refine_multi_lidar_extrinsics.py`，静止段地面共视，仅 roll/pitch/z） | 原生 67 s 误差 8.1→5.8 m；地面厚度 map_00-10 24→14 cm |
| 94 | 前端 IEKF 加 INS 姿态伪观测，绝对 σ=0.3° | 无效（点面行角信息 ~1e9 rad⁻²，伪观测占比 1e-5） |
| 95 | 伪观测权重 = max(1/σ², f/(1-f)·tr(H_rot)/3)，f=0.05 | **前端不再发散**：原生最大误差 368→6 m，末段速度 5.5/5.5，加计零偏 <0.08 m/s²，陀螺零偏保持 1.9e-3 |
| 96 | 前端牵引只作用于 roll/pitch（掩码 [1,1,0]），后端偏航 σ 1.0° | 指示牌各时代 std 4.5→3.0 cm |
| 97 | `lidar_time_offset=-0.125`，后端 `time_offset_sec=+0.025`，新增 `attitude_time_offset_sec=0.0` | 配准 RMS 各窗口下降（60–70 s：5.6→4.8 cm），后段地面 12.3→9.2 / 10.6→8.3 cm |
| 98 | 姿态因子运动门控（≥1 m/s 且运动 ≥5 s），偏航 σ 2.0° | 偏航过松：后段 -0.75°，IMU ≤10 cm 占比 92.6% |
| 99 | 外参 v2（`scripts/refine_multi_lidar_yaw.py`，行驶窗口逐点线性去畸变 6 自由度），偏航 σ 0.5° | 见第 3 节，当前最优 |

## 3. run99 与基线 run90 对照（全部来自实际落盘数据）

| 指标 | run90（56c6896） | run99 | 目标 |
| --- | --- | --- | --- |
| 后轴 3D vs RTK（路程加权） | 100% ≤10 cm，max 9.7 cm | **100% ≤10 cm，max 8.1 cm**，中位 1.9 cm | 80% ≤10 cm，≤20 cm |
| 物理 IMU 轨迹 vs INS 参考 | 67% ≤10 cm，P95 22 cm，max 26 cm | **99.99% ≤10 cm**，P95 5.6 cm，max 10.1 cm | 蓝线不得远离绿线 |
| 优化姿态 vs INS（中位/P95） | 俯仰 0.16/0.48°，横滚 0.60/1.50°，偏航 0.34/2.85° | 俯仰 0.11/0.21°，横滚 0.11/0.20°，偏航 0.29/0.47° | — |
| 原生 LIO 末段 | 速度 19 m/s，误差 381 m | 速度 5.5 m/s，误差 ≤4.7 m | 前端健康 |
| 全程多时代地面厚度 P50（1 m 格 P90-P10，27k 格） | 21.0 cm；后段 map_40-50 17.2 cm（帧内 16.1） | **9.9 cm；后段 6.6 cm（帧内 5.7）** | 接近原生 LIO（原生参照本身含外参分层） |
| 校正场每关键帧旋转步长（中位/P95） | 0.048/0.30° | 0.012/0.035° | 平滑 |
| 起步 10 m 后轴异侧 | 5/46 | 0/46（横偏中位 +0.98 cm，原生 +2.5 cm） | 位于原生与 RTK 之间 |
| 灯柱轴心跨时代（相近方位角，`multi_pole.py`） | 5.0 cm | 6.9 cm（噪声本底 ±3 cm） | 越小越好 |
| 单帧内灯柱 P90 半径 | 11.0 cm | 10.9 cm | — |

## 4. 关于灯柱"重影"的量化结论

1. 灯柱杆径约 17 cm（自由圆拟合半径 7.4–9.9 cm）。单帧内 25–30 个回波的 P90 半径 ~11 cm ≈ 物理半径 + 3 cm 测距噪声，帧间质心抖动仅 2–3 cm；原生地图与校正地图在同一时代的散布完全一致。即杆身"粗"主要是物理尺寸与 20–50 m 处光斑，不是轨迹误差。
2. 接近段（map_5–8，front/front_right 观测）与离开段（map_10–12，front_left 观测）方位角相差 105°，各自只看到杆的一侧，可见弧质心必然相差 ≈1.0×r ≈ 8 cm；固定半径圆拟合显示两侧弧心仍差 ~17–21 cm，因此约 10 cm 为真实误差，且同一帧内不同雷达对同一牌面/杆身的水平偏差达 5–12 cm（`plate_per_lidar.py`、`pole_per_lidar.py`），指向残余雷达间 yaw/xy/时延标定误差，而非轨迹。
3. 视角无关的指示牌法向偏移、多灯柱轴心统计在 run90–99 之间差异 ≤3 cm，处于方法噪声内；基线 run90 在该指标上并不劣于新版本。
4. 灯柱顶部 Z：era0（55 m 外）无共视顶面，无法跨时代逐点评估；姿态一致性提升（横滚 P95 1.5→0.2°）使 10 m 高处的姿态致位移由 ~26 cm 降到 ~3.5 cm 量级。

## 5. 当前配置要点（`config/wuhu_truck29.yaml`）

- `multi_lidar.calibration_file`: `config/perception_truck29_refined_v2.json`（原始 JSON 未改）
- `time_offset.lidar_time_offset: -0.125`；`rtk.time_offset_sec: 0.025`；`rtk.attitude_time_offset_sec: 0.0`
- `rtk.frontend_attitude_aiding: true`，`pull_fraction 0.05`，`axis_mask [1,1,0]`，Huber 1.0°
- `rtk.attitude_enabled: true`，`attitude_sigma_deg [0.15,0.15,0.5]`，`attitude_min_speed_mps 1.0`，`attitude_motion_age_sec 5.0`，安装角先验 [0.3,0.3,0.5]°
- `imu.initial_gyr_bias_sigma: 0.0001`

## 6. 工具

- `scripts/run_wuhu_rtk_round.sh <round>`：一键回放 + 优雅停止 + 导出全部 PCD + 评估；`scripts/evaluate_wuhu_round.sh` 自动从 `parameters.txt` 读取时间偏差。
- `scripts/analysis/round_summary.py`（INS 一致性/漂移/校正平滑/灯柱时代）、`ground_thickness.py`（全程多时代地面厚度）、`sign_plane.py`、`pole_circle.py`、`multi_pole.py`。
- `scripts/refine_multi_lidar_extrinsics.py`、`scripts/refine_multi_lidar_yaw.py`、`scripts/dump_bag_reference.py`。

## 7. 未解决与建议

- 雷达间残余水平/时延标定（同帧 5–12 cm）是灯柱重影的主要剩余来源，建议用标定场或更长的静止多目标数据重做四雷达标定（含各雷达点时间基准）。
- 后端仍按关键帧 RTK（3 cm σ）逐点跟随；若 RTK 存在 5 cm 级慢变误差会直接印入地图。原生链现已健康，可考虑放宽位置 σ、加紧运动链以进一步保留 LiDAR 局部一致性（未在本次验证）。

