# MCP 专题：zongyang truck07 建图精度劣化根因 — 前端时间基沿用 truck29 模板

日期：2026-10-08 · 分支 `fusion/56c6896-ins-attitude` · 前置文档：`docs/mcp-colour-only-and-time-offset-audit.md`（时间语义）、`docs/mcp-studio-data-assessment.md`（数据评估）

## 1. 现象

zongyang truck07（2026-10-07 录制，360 s / 1394 m / 无相机）在 v003_ground 标定下的 48 s LIO 轮
（`results/calib_truck07_lio_1008_172457`）：

- 原生 LIO 后轴 vs RTK：median 0.30 m，P95 2.05 m，max 2.25 m；yaw 漂移到 1.1°，误差随速度/转弯增长；
- 优化后虽然 `<=10 cm 100%`，但修正节点旋转步长 max 0.237°（truck29 常态的 ~20 倍），点云可见重影、整体发糊。

## 2. 根因链

1. `tools/calib/livo_calib.py` 的 `DEFAULT_TIME_OFFSETS` 把 truck29 的硬件常数
   `lidar_frontend = -0.125 s`（扫描↔IMU，前端去畸变/传播用）作为 import 模板写进每台新车；
2. axle 阶段只求解并覆盖 `time_offsets.lidar`（扫描↔INS 对齐，truck07 实测 **-0.0041 s**，
   时移候选扫描在 +0.097..+0.121 s 处有明确谷底），`lidar_frontend` 无任何阶段校验；
3. truck07 的四路 LiDAR 均为 GPS 整拍对齐打戳（front 在 100 ms 整拍边界，left/right +0.2 ms，
   back +16 ms 相位、67 ms 扫窗；点时间为帧内相对值，语义自洽），IMU 与 INS 同钟
   （gyro-z vs INS 偏航角速率相关 0.999，滞后仅 +12.5 ms）——真实扫描↔IMU 偏移 ≈ 0；
4. 前端因此拿 **121 ms 之前的 IMU 段**去畸变/传播每一帧：直线慢速几乎无感，速度/角速率上来后
   系统性错位 → 原生 LIO 偏航漂移、后端被迫做大幅修正 → 地图糊。

RTK/INS 参考系统反而是干净的（数据评估：固定解 100%、位置阶跃 0 次、xy 一致性 P95 1.90 cm、
卫星 P10 32 颗），"RTK 跳变"假设排除。

## 3. 修复

### 3.1 标定数据（立即生效）

`zongyang/truck07/v005_manual`（已激活）：`lidar_frontend = -0.0041 + 0.0125 = +0.0084 s`。

### 3.2 标定工具（一般化，防复发）

`livo_calib.py` 新增 `measure_imu_ins_clock_lag()`（imu_data gyro-z vs imu_data/odometry 偏航率
互相关，±0.3 s 搜索）；axle 阶段求解完 `time_offsets.lidar` 后自动执行**前端时间基同步**：

    candidate = lidar(扫描↔INS 解) + imu/ins 时钟差
    相关性 >=0.95 且偏航激励 >=0.02 rad/s 才判定；
    |candidate - 模板 lidar_frontend| > 20 ms 则替换并写 lidar_frontend_source，否则确认模板不动。

`--no-frontend-sync` / `--keep-time-offset` 可关闭。对既有车队为空操作（见 §4 验证），
只有模板明显错误（如异车模板 >100 ms）才触发替换。

公式对既有两台车的手工验证值是自再现的：
truck29 `-0.1459 + 0.0125 = -0.1334`（与手调 -0.125 差 8.4 ms，小于门限→不动）；
truck05 `-0.0267 + 0.0125 = -0.0142`（与手调 -0.0119 差 2.3 ms→不动）。

## 4. 验证

### 4.1 truck07 A/B（48 s LIO，同窗同数据）

| 指标 | v003(-0.125) | fix(-0.0041) | fix2(+0.0109) |
|---|---|---|---|
| native 后轴 median / P95 / max (m) | 0.300 / 2.046 / 2.253 | 0.207 / 0.591 / 0.678 | **0.191 / 0.545 / 0.639** |
| native yaw P95 (deg) | 1.125 | 0.238 | 0.236 |
| optimized 后轴 median / P95 / max (cm) | 1.77 / 4.73 / 9.07 | 1.14 / 2.70 / 5.22 | **1.14 / 2.57 / 4.77** |
| optimized yaw P95 (deg) | 0.554 | 0.211 | **0.158** |
| 修正节点旋转步长 max (deg) | 0.237 | 0.013 | ~0.013 |

### 4.2 truck07 全程 360 s（v005_manual，`results/zy07_lio_full_v005`）

- optimized 后轴 vs RTK：**median 1.23 cm，P95 2.94 cm，max 6.04 cm，100% <=10 cm，0% >20 cm**；
- optimized 姿态 vs INS：pitch P95 0.18°，roll P95 0.13°，yaw P95 0.23°；
- 723 个修正节点旋转步长 median 0.009°，max 0.071°；
- 地图锐度（wall_ghost，fused，竖直平面体素 PCA）：
  前段 15-45 s 平面厚度 median 2.9 cm，雷达间法向偏移 median 1.1 cm；
  后段 300-340 s 各对雷达偏移 median 0.9-2.7 cm（back_lidar 残留 ~2 cm 常数偏置，
  属后续更好场景下 driven 阶段的改进空间，非重影级）。
- driven 复跑（v006 尝试）被质量门正确拒绝：时间基修正后地面残差已到 1.9-3.2 cm，
  8-40 s 窗内竖直结构不足（355/570/49 条竖直匹配），不具备 6-DoF 精修条件——保持 v005。

### 4.3 truck29 回归守护

`scripts/check_truck29_guard.sh`（LIO+LIVO 两轮 + 地图/轨迹阈值）——结果见 §6。
运行时路径零改动：C++ 二进制、`config/wuhu_truck29.yaml`、truck29 标定文件与 registry 条目均未动；
`livo_calib.py` 的新逻辑对 truck29 实测为"模板确认、不替换"。

## 5. 数据评估结论（tools/livo/assess.py，42 项）

truck07 数据整体可用，3 项关注级不可标定补救：无相机（仅 LIO）、front_lidar 录制丢帧 0.44%、
起步前静止仅 2.6 s（建议下次采集静止 >=10 s）。多雷达扫描起始离散 P99 16.6 ms 为 GPS 整拍相位差，
合帧容差 20 ms 下全程仅 6 帧被丢（起步对齐），不是缺陷。IMU 最大断点 20 ms，自动补偿范围内。

## 6. 守护结果与归档

（guard 运行记录：`results/t29_guard_after_fix.log`，轮次 `results/guard_t29_*`）

2026-10-08 守护（commit 1e15d3c + 本次改动，active truck29 标定 v066_driven）：**GUARD PASS**

| 检查 | 实测 | 门限 |
|---|---|---|
| LIO fused xy median / P95 / max (cm) | 0.80 / 2.95 / 6.38 | <=1.0 / 3.5 / 7.5 |
| LIVO fused xy median / P95 / max (cm) | 0.86 / 2.46 / 4.96 | <=1.0 / 3.5 / 7.0 |
| 灯柱壳层 \|r-R\| P90（4 根，cm） | 4.9-5.5 | <=6.0 |
| back LiDAR 沿迹偏移 cruise \|median\| (cm) | 1.3 | <=3.0 |
| 墙面 front_right vs back median (cm) | 1.9 | <=3.0 |

起步灯柱点云截面与基线带一致，无重影级偏移。

## 7. 后续建议

1. 下次 zongyang 采集：起步静止 >=10 s、带几处竖直结构（挡墙/灯柱）的转弯窗，driven 阶段即可把
   back_lidar 残留 ~2 cm 偏置收掉；
2. 若更换 INS/IMU 链路固件，重跑 axle 阶段即可——前端时间基现在随标定自动核对；
3. `front_lidar` 0.44% 录制丢帧与半数点量（13.6k/帧）建议排查该路硬件/带宽。

