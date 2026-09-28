# 标定链路复审：colour-only 模式、时间偏移审计与流水线精简（2026-09-27）

范围：`/home/lhx/docker/MY-LIVO`，分支 `fusion/56c6896-ins-attitude`（基线 3dbf7e7）。目标是 29/05 两台车全局误差 ≤ 8.5 cm、彩色点云上色正确、外参与时间偏移在控制台可见、时间偏移逻辑经得起审计、在不牺牲精度前提下删掉多余的流水线步骤。本文只记录实测结论；未经实测的推断都标明"未证实"。

## 决策更新（2026-09-28，用户裁定）

**图像必须参与前端状态估计**：LIVO 引入视觉的目的就是在 LiDAR 退化环境下增强鲁棒性，"图像只上色"违背这一目的。因此：

- `config/wuhu_truck29.yaml` 默认已改回 `vio.ekf_update_en: true`、`camera.frame_divider: 1`（原始 LIVO 行为，`--mode livo` 即全速 VIO 参与 EKF）。`ekf_update_en=false` 仅保留为**诊断开关**，不作为生产模式，也不再是流水线建议。
- 下文 colour-only 的数字（truck29 5.02 / truck05 4.32、5.65 cm）保留，但定位变为**诊断证据**：同一数据、同一标定，去掉视觉更新后误差降到一半，说明当前 VIO 更新在矿区富纹理/动态场景里注入了误差。结论不是关掉它，而是让视觉更新"该出手时出手"（§8 方案）。
- §5 中依赖 colour-only 的精简建议（删 `camloop`、`camera-time` 降为可选、LIO/LIVO 合并）**撤回**；视觉参与前端时相机时间与外参直接影响轨迹，这两个阶段保留。
- 回退验证：`mcp_t29_livo_default`（默认配置、v063、同一二进制）见 §6：truck29 LIVO **5.86 cm**、LIO **7.09 cm**，两种模式均满足 ≤ 8.5 cm。
- **truck05**：用户裁定"若是数据本身（时间戳、原始外参不准、动态障碍）导致，则不改"。§9 逐项排查后的结论：视觉拖累 truck05 的根因是 **数据/标定输入**（强光过曝、动态矿卡、从未标定且在本数据中无法验证的相机模型、相机视场内 LiDAR 点少）与原始 LIVO **固定视觉权重** 的组合；LIO 与标定链路本身在该段是健康的（视觉关闭或降权后 5.65–8.6 cm）。因此本轮不为 truck05 改算法，只给出运行建议与补数据清单（§9.4）。

## 决策更新 2（2026-09-28，§10：视觉权重对等界 + 10 Hz 整帧 LIO，新默认）

在"图像必须参与前端"的前提下继续改进，验收改为：max ≤ 10 cm（RTK 本身有数厘米/≥0.05° 误差，不追求贴近 RTK）且 **truck29 起步段灯柱不得比纯 LIO 粗/重影**。§10 的结论：原 LIVO 起步段灯柱重影（第 2 根灯柱弥散 13.6 cm，LIO 5.2）来自两处设计缺陷——30 Hz 图像把 4 雷达 10 Hz 合帧切成 1/3 片让 LIO 变弱，以及固定视觉权重比同帧 LiDAR 信息大约 1000 倍。新默认 `camera.frame_divider: 3`（LIO 整帧 10 Hz，VIO 仍在前端 ESIKF 内以 10 Hz 更新并上色）+ `vio.lidar_parity_ratio: 1.0`（每帧视觉信息不得超过同帧 LiDAR 最弱方向的信息，LiDAR 退化帧自动保留全权重；0 = 原固定权重）。truck29：LIVO 后轴 xy max 5.86 → **5.21 / 4.87 cm**（两次回放），灯柱弥散 fused 3.9 / 4.8 cm（≤ LIO 5.2），LIO 路径逐帧不变（7.09）。truck05：W598 LIVO 16.49 → **6.12 cm**（前端不再发散）；720 s 全程 LIVO 47.3 → **9.89 cm**（xy max，≤ 10 cm 验收线；z 39 cm 出在 340–380 s 扬尘段，与 LIO 同源，见 §10.6）。§0 表中"精度"一行以本节为准。

## 0. 结论速览

| 项 | 结论 |
| --- | --- |
| 精度 | **最终默认配置（原始 LIVO，VIO 参与 EKF）在 truck29 上：LIVO 5.86 cm、LIO 7.09 cm（后轴 xy max，120 s / 580 m），均 ≤ 8.5 cm，且与历史 v063 LIVO 5.92 一致。** truck05：LIO 全程 7.25 cm（v019）；全速 LIVO 在 W598 为 16.49 cm（前端发散 14.7 m），根因是数据/标定输入（§9），按用户裁定不改算法。colour-only 诊断模式的数字（truck29 5.02、truck05 W40 4.32 / W598 5.65、全程 13.47）保留为诊断证据；其中全程 720 s 的超差全部来自回放丢帧→多雷达合帧器失配的 5 处帧空洞（§5.1、§6）。|
| 上色 | 彩色点云由图像上色（`rgb_map_*.pcd`），默认配置下 VIO 同时参与状态更新（原始 LIVO）。truck29 相机时间偏移经独立几何方法验证（LiDAR 深度边缘 vs 图像梯度，联合峰值 **−0.7 ms**，对比度 0.053）：−0.115 s 正确。truck05 该方法在 W40 无峰（对比度 0.011），原因是场景缺少静止竖直结构 + 相机外参只有供应商标称（见 §4.3）；投影叠加图目测外参无度级错误。|
| 控制台 | 启动与结束时打印 `CALIBRATION IN EFFECT` 摘要：四雷达→后轴 6 自由度、相机→后轴、内参/畸变、后轴→IMU 杆臂、四个时间偏移及其"作用到哪一路"的解释、后端估计的 INS 安装角、相机模式。|
| 时间偏移审计 | 语义链路已核对无误（§3.1）。发现并修掉两处评估侧错误（§3.4）。用四种信号级方法交叉检验：**只有 truck29 的三项偏移得到独立证实**；truck05 的 `lidar_frontend`（−0.0119）与 INS 位置关联（−0.0078 绝对）**无法用信号级方法在 ±15 ms 内判定**，几何法（§3.3）此前也是平坦剖面。因此本轮不改 truck05 时间偏移，明确写出"不可辨识"而不是拍一个数。|
| 效率 | 默认配置下 truck29 LIVO 全程实时（VIO 平均 15 ms/帧，30 Hz）。诊断模式（分频 3 + 关 VIO 更新）LIO 回到 10 Hz、单帧 42–85 ms，truck05 720 s 全程实时。流水线精简建议见 §5（依赖 colour-only 的部分已撤回）。|
| 确定性 | truck05 运行期 extrinsic precheck 每轮都对 front_left/front_right 施加 +5.5/+4.6 cm 的 z 修正，且 front_right 处在 4 cm 施加阈值边缘（W40 未施加、W598 施加）→ 同一标定不同起点得到不同外参。已用 `from-precheck` 固化为 **v024_precheck**，运行期应回到"confirmed"。|

## 1. 近 10 个标定版本回顾（做了什么、精度走到哪）

| 车 | 版本 | 阶段 | 关键改动 | 后轴 xy max（窗口） |
| --- | --- | --- | --- | --- |
| truck29 | v061 | LIO | 6-DoF driven 外参 | 6.56 cm（120 s） |
| truck29 | v062 | LIVO | 相机时间 −0.115 | 6.93 cm |
| truck29 | v063 (active) | LIVO | 相机旋转微调 | 5.92 cm |
| truck29 | v063 + colour-only | — | 本轮，无标定改动 | **5.02 cm** |
| truck29raw | v014 (active) | LIO/LIVO | 原始外参重对齐 | 5.69 / 7.2–8.6 cm |
| truck05 | v019 (active) | LIO | driven 6-DoF | W1 7.4 / W2 6.0 / 720 s 7.25 cm（LIVO 720 s 47.3） |
| truck05 | v020 | manual | lidar_frontend=−0.125（照搬 29） | 720 s 12.59 cm（**恶化**） |
| truck05 | v021 | manual | 相机偏移试验 | W2 LIO 5.81 / LIVO 9.13 |
| truck05 | v022 | manual | lidar_frontend −0.0119、camera −0.115 | LIVO W598 19.6 / 14.2 cm（3D） |
| truck05 | v023 | manual | camera −0.0019（同源时钟规则） | LIVO W598 11.3；**colour-only 5.65** |
| truck05 | v024_precheck | precheck | v023 + 固化 precheck z/roll/pitch | 720 s 见 §6 |

判断：LiDAR 外参与 INS 对齐部分（axle→ground→driven）已经收敛，两台车 LIO 级精度都在 6–7.4 cm；把精度拖到 8.5 cm 以上的从来不是 LiDAR 外参，而是 **VIO 光度更新** 在矿区（弱纹理、动态车辆、曝光变化）注入的误差，以及 truck05 上未被固化的运行期外参修正。原有算法"不是最优"的地方就在这两点，本轮改的也只是这两点，未动 LIO/后端求解。

## 2. 本轮改动清单

代码（未提交部分在工作树，提交见 §7）：

- `src/LIVMapper.cpp` / `include/LIVMapper.h`：`printCalibrationSummary()`（启动 + 结束各一次，结束时附后端估计的 INS 安装角）；`vio.ekf_update_en`（false = colour-only）、`debug.print_frame_timing`；`[status]` 行每 10 s 一条（LIO/VIO 均值、点数、模式）。`parameters.txt` 新增 `effective.camera_mode` / `effective.camera_frame_divider`。
- `src/vio.cpp` / `include/vio.h`：`ekf_update_en=false` 时跳过光度 EKF 更新与视觉子图维护，仅保留图像→点着色。
- `src/rtk_fusion.cpp`：`appendParameters` 写入 `effective.*`（实际生效的四个时间偏移、杆臂、标定 id）。
- `config/wuhu_truck29.yaml`：新增 `vio.ekf_update_en`（默认 **true**，见"决策更新"）、`debug.print_frame_timing: false`；`camera.frame_divider` 保持 1。
- `scripts/analysis/round_summary.py`、`round_metrics.py`：**修复** `parameters.txt` 键取"最后一次出现"（yaml 转储值 `rtk.time_offset_sec=0.025`）而不是首行的生效值 → `summary.txt`/`error_series.csv` 误差此前被高估（truck29 优化后中位 11 cm vs 真实 0.9 cm）。杆臂改读 `effective.imu_to_rear_axle`。修复后三轮重评估，`summary.txt` 与 `metrics.json["trajectory"]` 一致。
- `tools/calib/livo_calib.py`：`set` 新增 `--rtk-position-offset/--ins-attitude-offset`（做关联时间试验用）；`decode_hevc` 加 `-nostdin`（ffmpeg 曾吞掉通过 stdin 传入的脚本剩余部分）。
- 新脚本 `scripts/analysis/time_sync_check.py`、`camera_sync_check.py`、`camera_overlay_check.py`（§3.2、§4）。
- `tools/livo/player.py`：按建图进度节流（`--pace-topic/--max-lead`）、按消息类型分配发布历史深度（§5.1）。
- 标定 `config/carlibration/wuhu/truck05/v024_precheck/`（§2.1）。

### 2.1 v024_precheck 的来历

`Log/extrinsic_precheck.json`（静止起步时以 front_lidar 为参考的地面拟合）在 truck05 三轮中稳定给出：front_left dz +5.49/+5.85 cm、roll −0.07/−0.11°、pitch +0.09/+0.11°；front_right dz +4.57/+3.35 cm。front_right 的 3.35 cm 低于 `min_apply_height_m=0.04` 而未施加，4.57 cm 则施加——同一 v023 在不同起点上实际生效的外参相差 4.6 cm。`livo_calib.py from-precheck --run-dir results/mcp_t05_colour_w598 --id wuhu/truck05/v023_manual` 将 W598 的修正写进 v024；truck29 v063 的 precheck 为 confirmed（−0.4/+3.0 cm，未施加），不需要固化。

## 3. 时间偏移逻辑审计

### 3.1 语义（代码核对，`src/LIVMapper.cpp` L586–625、`src/rtk_fusion.cpp` `finalizeOptimization`）

| 标定键 | 作用 | truck29 | truck05 |
| --- | --- | --- | --- |
| `time_offsets.lidar_frontend` | 加到扫描头戳上，用于 IMU 去畸变/传播（scan↔IMU） | −0.125 | −0.0119 |
| `time_offsets.lidar` | axle 阶段求得的 scan↔INS 对齐；与上式之差 `lidar − lidar_frontend` 作为 reference shift 加进 INS 关联 | −0.1459（shift −0.0209） | −0.0328（shift −0.0209） |
| `time_offsets.rtk_position` / `ins_attitude` | INS 位置/姿态关联的额外偏移（覆盖 yaml `rtk.time_offset_sec`） | 0.025 / 0 → 生效 +0.0041 / −0.0209 | 同 |
| `time_offsets.camera` | 加到图像戳上（image↔IMU） | −0.115 | −0.0019 |

符号约定：lag>0 = 第二路信号的时间戳偏晚；生效值全部打印在控制台摘要和 `parameters.txt` 的 `effective.*` 中。由于标定文件的 `rtk_position/ins_attitude` 覆盖 yaml，`--set rtk.time_offset_sec` 不再起作用，要改关联时间必须建标定版本（`livo_calib.py set --rtk-position-offset`）。

### 3.2 信号级交叉检验（`scripts/analysis/time_sync_check.py`，不需重跑 bag）

| 检验 | truck29（colour-only 120 s） | truck05 W598 | truck05 W40 | 解读 |
| --- | --- | --- | --- | --- |
| [1] INS 姿态角速度 vs 原始陀螺 | +10.0 ms | −4 ms | −4.9 ms | INS 消息内部链路两车不同，但都在 10 ms 内 |
| [1] INS 位置微分速度 vs INS 速度 | −2.3 ms | −6 ms | +5.9 ms | 一致 |
| [2] 原生 LIO 偏航角速度 vs 陀螺 | −1.9 ms | +10.2 ms | +4.7 ms | **不能用来判 `lidar_frontend`**：truck05 用 −0.0119 与 −0.125 两个值跑，滞后仅 +10.2→+8.6 ms，说明 LIO 姿态在更新之间跟随陀螺，对扫描时间偏移不敏感。脚本已改为"仅供参考"，不再输出推荐值 |
| [3] 原生后轴沿迹误差对 INS 速度回归（20 s 窗、去线性漂移） | −2.3 ms（MAD 14.5） | −13.5 ms（MAD 15） | 无效（起步段） | 分辨率 ~15 ms，只能说 truck29 关联正确、truck05 可能偏晚 10–15 ms |
| [3b] 速度剖面互相关（已删除） | — | — | — | 全窗/高通/分段三种做法互相矛盾（±20 ms），无法达到 10 ms 分辨率，已从脚本移除 |

truck29 的三项偏移由此得到独立证实（[2]、[3] 与相机边缘法 §4 全部 ≈ 0）。truck05 的 `lidar_frontend` 与位置关联只能说"与当前值差在 ±15 ms 内不可辨识"；链路推断（INS 早 4 ms + axle τ）给出 −0.029 / −0.026，但同一推断在 truck29 上偏差 30 ms，所以**不采信**。10 ms 关联误差 ≈ 5 m/s 时 5 cm 沿迹误差，是 truck05 剩余误差预算里最大的未知项，但本轮 W40/W598/720 s 结果说明它没有把误差推过 8.5 cm。

### 3.3 几何仲裁（应做而未在本轮完成）

要把 truck05 的两个偏移定到 5 ms，需要以地图厚度/灯杆重影为目标，在 colour-only LIO 上扫 `lidar_frontend ∈ {−0.012, −0.022, −0.032}` × `rtk_position` 两档（各 60 s W40 窗），每轮约 4 min。此前 axle 阶段的 τ 剖面在 truck05 上平坦（3.68–3.72 cm），可能与位置/姿态分裂的 dt（0.025 / 0）有关，`scripts/refine_rear_axle_extrinsic.py --no-split` 可复核。列为后续项。

### 3.4 评估侧的两处错误（已修）

1. `round_summary.py`/`round_metrics.py` 读 `parameters.txt` 用了最后一次出现的键（yaml 转储），导致 INS 关联用 +0.025 而非生效值，所有旧 `summary.txt`/`error_series.csv` 数字偏大（`metrics.json["trajectory"]` 一直正确，来自 `evaluate_wuhu_lio.py` 首次匹配）。
2. `camera_sync_check.py` 第一版（跨帧光度一致性）在匀速旋转下对相机时间偏移不敏感（两侧投影同步平移），单对散布 ±150 ms，已弃用；第二版改为 LiDAR 深度边缘 vs 图像梯度（对 τ 一阶敏感）。

## 4. 相机：上色正确性与时间偏移

### 4.1 方法（`scripts/analysis/camera_sync_check.py`）

取转弯中的 30 帧（|yaw rate| > 4 °/s），累积 ±0.6 s 扫描（`scans.bin`，运行位姿）到世界系，用插值位姿 pose(t+τ) 投影到 0.25 倍图像（z-buffer），深度边缘 = 深度比 > 1.25 的近侧 + 天际线；得分 = 边缘像素处归一化图像梯度均值；τ 网格 ±150 ms/5 ms，联合曲线抛物线细化。

### 4.2 结果

| 车/窗口 | 生效相机偏移 | 联合峰值 | 对比度 | 单帧中位 | 结论 |
| --- | --- | --- | --- | --- | --- |
| truck29 120 s | −0.115 | **−0.7 ms** | 0.053 | −3.8 ms（MAD 36） | 证实。第一段转弯（7–15 s）逐帧 −10…+21 ms 一致；83–99 s 段场景弱，散布大 |
| truck05 W40 | −0.0019 | −65 ms | 0.011（无峰） | −19 ms（MAD 106） | **不可判**。曲线在 0.995–1.003 间随机 |

### 4.3 truck05 为何无峰，与上色的关系

`scripts/analysis/camera_overlay_check.py` 把 LiDAR 点按深度着色投到原图（`results/.cache/mcp_camsync/t05w40_overlay/`）：t+5 s（洒水车横穿）、t+25 s（近处停放矿卡）两帧中，矿卡轮廓、路缘、远处建筑与点云对齐在 ~10 px（640 宽）以内，即外参无度级错误，彩色地图不会出现"颜色贴到别的物体上"。但 truck05 相机外参仍是供应商标称（`camera-lidar` 阶段因无灯杆未跑过），内参 [1849.37 1849.37 1003.31 502.72]、畸变全零，边缘法要求亚像素级对齐才能出峰；加上 W40 转弯段视野里多是开阔路面，深度边缘主要来自地面起伏噪声。要在 truck05 上定相机时间，先要有一次相机→LiDAR 旋转细调（用停放矿卡/路缘做目标，`livo_calib.py camera-lidar` 的目标检测需要放宽到非杆状结构），列为后续项。

## 5. 流水线精简建议（精度不降的前提下）— 部分撤回，见"决策更新"

当前：`import → windows → axle → ground → LIO round → driven → camera-time → LIVO round → camera-lidar → camloop → validate/activate`。

| 步骤 | 处置 | 依据 |
| --- | --- | --- |
| `camera-time`（光流 vs 陀螺） | **降为可选**。colour-only 模式下相机偏移只影响上色不影响轨迹；truck05 上该阶段本来就过不了门限。改用 `camera_sync_check.py` 在 LIVO 轮次产物上离线做（truck29 已验证），不再单独跑 | §4 |
| `camloop`（相机偏移闭环再跑 LIVO） | **删除**。它的目标是让 VIO 收敛，VIO 已不参与状态更新 | §1 |
| `LIVO round` 与 `LIO round` | **合并为一轮 colour-only**：同一轮既出 LIO 级轨迹又出彩色地图；`driven` 用它的 `frames.csv/scans.bin` 即可 | 三轮实测 LIO 42–85 ms/帧，10 Hz 实时 |
| `camera-lidar`（灯杆） | 保留，但要扩展目标类型，truck05 才有输入 | §4.3 |
| axle 阶段 720 s 全量 | 限制到 ~300 s 行驶段（此前 7 min 且不收敛） | 既有记录 |
| 运行期 precheck | 首轮后立即 `from-precheck` 固化，之后应为 confirmed；若再次 applied 说明起点静止段地面不一致，要人工看 | §2.1 |

精简后：`import → windows → axle(≤300 s) → ground → colour-only round → driven → from-precheck → colour-only round(验证) → camera_sync_check/camera_overlay_check → validate/activate`，少两轮 bag 回放（每轮 120–720 s 实时 + 评估 1.5–5 min）。

## 5.1 720 s 全程回放暴露的两处稳定性问题（已修，`tools/livo/player.py`）

| 现象 | 根因 | 修复 |
| --- | --- | --- |
| 第 1 次全程：t=537 s `IMU stream has a 0.100000 s gap` FATAL，而录包 IMU 在该处完全规则（`imu_raw.csv` 只有 77.2/429.2/586.2 s 的 40/60/30 ms 空洞） | 建图节点在 250–450 s 密集路段 LIO 80–95 ms/帧，累计落后实时 48 s；节点用 `spin_some` 在处理线程内取消息，读端历史 LiDAR 200 帧（20 s）/IMU 5000（50 s）溢出后丢样，`IMU and LiDAR delta time is 45 s` 告警即此。IMU 丢 10 样 = 恰好 0.100 s | 播放器按建图进度节流：订阅 `/aft_mapped_to_init`，数据时间领先建图 > `--max-lead 3 s` 即暂停发布（暂停时长计入进度基准，不追赶）；实测 mapper_lead 保持 0.2–0.3 s |
| 第 2 次全程（图像发布历史加深到 300）：t=93 s 帧率跌到 2.5 Hz、`Invalid LIO pose or scan interval` FATAL | 解码器（republish）落后时，深历史把迟到的 HEVC 包全部送达，图像晚到数秒，LIVO 按图像切帧的节奏崩掉 | 图像/HEVC 发布历史保持 10（迟到即丢，解码器在下一个 I 帧恢复）；IMU/INS/状态等小消息历史 3000，点云 20 |

| 第 3 次全程（节流 max-lead 3 s、点云历史 20，= `mcp_t05_colour_full`）：跑完，但 `frames.csv` 有 32 个 0.2–2.0 s 的帧空洞（184–199 s、277 s、330 s、352–372 s，其中一个 2.0 s），对应融合误差 xy 13.5 cm 尖峰（190 s）与 z 44 cm（330–390 s，LIO 无更新只靠 IMU 传播） | 多雷达合帧要求四路扫描起始对齐 ≤ 1 ms；任一路一帧迟到/丢失后合帧器失配，逐帧 `Dropping unmatched … scan-start span 97 ms exceeds 1.0 ms`，直到自行重新对齐（0.2–2.0 s） | 第 4 次尝试把点云历史加到 60、max-lead 1.5 s：建图反而慢 2–3 倍（可靠传输对 1.5 MB 样本的重传流量），合帧器在 365 s 失配后**永不恢复**，与节流形成死锁（无新数据→不出帧→播放器一直等）。已回退到历史 20 / 3 s 并加 `--hold-timeout 20 s` 死锁保护。**合帧器失配后应丢弃最旧一路的旧帧重新对齐，这是 C++ 侧的首要遗留项** |

失败/缺陷轮次保留为 `results/mcp_t05_colour_full_{imugap_aborted,imgdepth_aborted,lag_aborted,r1_framegaps}` 供复核。附带结论：`imu.maximum_time_gap: 0.1` 的严格中止本身没有问题（录包内 ≤ 60 ms 的空洞都能桥接），问题全部在回放侧；60 s 窗口轮次无任何帧空洞。**遗留（C++）**：任何原因的 LiDAR 帧空洞都应在后端体现为里程计因子 σ 放大或 INS 位置因子 z 权重不被 Huber（δ=2.8σ）压掉——第 3 次全程里 RTK 正常（mode 4、24 星）而 LIO 漂了 44 cm，鲁棒核保护了错的一方。

## 6. 实测轮次

| 轮次 | 车/窗口 | 标定 | 模式 | 后轴 xy 中位/P95/max（cm） | 3D max | 原生 xy 末端 | 地面 5–20 m 厚度 前→后 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `mcp_t29_colour_120` | truck29 全程 117.7 s / 580 m | v063 | colour-only | 0.92 / 2.39 / **5.02** | 5.91 | 77 cm（z 3.0 m） | 24.9→25.2 mm |
| `mcp_t05_colour_w40` | truck05 40–100 s（急弯段） | v023 | colour-only | 1.25 / 3.08 / **4.32** | 7.38 | 20 cm | 45.0→29.4 mm（plane offset） |
| `mcp_t05_colour_w598` | truck05 598–658 s | v023 | colour-only | 0.87 / 2.50 / **5.65** | 7.87 | 26.6 cm | 25.5→28.4 mm |
| `mcp_t05_v022_livo_w598_r2` | 同上 | v022 | 全速 LIVO | — / — / 11.3 | 14.2 | — | — |
| `mcp_t05_colour_full` | truck05 全程 717.5 s / 1538 m | v024 | colour-only（回放节流 3 s；仍有 32 个帧空洞） | 0.72 / 2.69 / **13.47** | 44.7 | 6.75 m（z 0.2 m） | 38.7→55.0 mm |
| `mcp_t05_colour_full_r2_merger_deadlock` | 同上 | v024 | 点云历史 60、max-lead 1.5 s | 未完成（§5.1 第 4 行） | | | |
| `mcp_t29_livo_default` | truck29 全程 117.7 s | v063 | **默认配置（VIO 参与 EKF，30 Hz）**，回退验证 | 0.97 / 2.67 / **5.86**（= 历史 v063 LIVO 5.92，二进制改动未改变 LIVO 行为）| 6.93 | 2.72 m（原生 3D 漂移 max 3.15 m / 580 m） | — |
| `mcp_t29_lio_default` | truck29 全程 117.7 s | v063 | `--mode lio`（默认配置） | 0.78 / 3.04 / **7.09** | 7.10 | 原生 3D 漂移 max 2.98 m | — |
| `mcp_t05_livo_w598` | truck05 598–658 s | v024 | 全速 LIVO 默认配置（§9） | 1.27 / 4.47 / **16.49** | 16.50 | 原生 3D 漂移 max **14.66 m**（t=25–35 s 前端发散） | — |
| `mcp_t05_livo_w598_expo` | 同上 | v024 | LIVO + `vio.exposure_estimate_en=true`（§9 诊断） | 1.51 / 4.48 / **10.13** | 16.96（z 16.7） | 原生 max 0.96 m | — |
| `mcp_t05_livo_w598_cov1000` | 同上 | v024 | LIVO + `vio.img_point_cov=1000`（视觉权重 1/10，§9 诊断） | 1.07 / 2.74 / **8.62** | 8.71 | 原生 max 0.26 m（= LIO 前端水平） | — |
| `mcp_t05_livo_w598_expo_cov1000` | 同上 | v024 | LIVO + 曝光估计 + 权重 1/10（诊断） | 1.11 / 2.66 / **8.59** | 8.67 | 原生 max 0.25 m（与仅降权相同：降权起决定作用） | — |
| `mcp_t29_livo_expo` | truck29 全程 117.7 s | v063 | LIVO + `vio.exposure_estimate_en=true`（回归，**不采用**） | 1.02 / 2.98 / **6.99**（差于默认 5.86） | 7.80 | 原生 max 1.99 m；视觉子图检索点数 70 s 后跌到 0（曝光状态漂移，视觉实际失效） | — |

全程的分段 xy/z 最大误差（cm，每 30 s）：0–180 s xy ≤ 8.3（30–60 s 洒水车横穿段 8.3）、180–210 s xy 13.5（帧空洞）、240–300 s z 16–23、330–390 s z 42–44（帧空洞 0.5/0.5/2.0/0.8 s，帧内点数最低 7–9 k，即用户所说其他矿卡经过/扬尘段）、420–720 s xy ≤ 4.2 / z ≤ 6.4。此前 v019 LIO 模式全程 xy max 7.25 cm。

所有轮次都保存了完整 `map_*.pcd` / `rgb_map_*.pcd`（`results/<round>/Log/fusion/<stamp>/`）供目视；未在建图期间打开大点云。

## 7. 提交与后续

提交：见 `git log`（本轮一次提交，含上述代码、v024 标定与激活、本文档、开发计划记录）。未提交：工作树中 `config/perception_truck29*.json`、`config/carlibration/perception_truck29.json`、`pics/Framework.png` 的删除与 `3rdparty/gtsam` 子模块状态不是本轮改动，保持原样——注意 `config/wuhu_truck29.yaml` 的默认 `multi_lidar.calibration_file` 仍指向已删除的 `config/perception_truck29_refined_v2.json`，不带 `--calibration` 直接 launch 会失败（`run_wuhu_round.sh` 总是显式传标定，不受影响）。

## 8. 视觉更新的下一步（按用户裁定：参与前端，但要鲁棒）

诊断证据：截掉视觉 EKF 更新后 truck05 W598 xy max 11.3 → 5.65 cm、truck29 5.92 → 5.02 cm，而 LIO 本身在这两段并不退化（有效特征 1.5–1.8 万/帧，配准残差 ~2 cm）。也就是说当前 VIO 在 **LiDAR 不退化** 的场景里仍以固定权重（`img_point_cov: 100`）修正状态，把光度噪声（曝光、动态车辆、弱纹理、相机外参/时间的残余误差）带进了轨迹。符合原始 LIVO 思路的修法是让视觉权重随 LiDAR 可观测性自适应，而不是恒定：

1. **按 LiDAR 退化程度加权视觉更新**：用 LIO 迭代更新里已有的 Hessian 最小特征值（`lio.min_eigen_value` 同源量）或有效特征数/方向分布，构造 0–1 的退化因子 d；视觉测量协方差取 `img_point_cov / max(d, d_min)`。LiDAR 富约束时视觉只起微弱先验作用，退化时（长直巷道、扬尘、近距大车遮挡）恢复全权重——这正是 LIVO 想要的"退化时视觉兜底"。
2. **视觉更新质量门**：光度残差中位数/内点率低于阈值（曝光突变、动态物体占比高）时跳过该帧视觉更新，仍上色。
3. **验证口径**：在 truck05 W598（富约束）要求接近 colour-only 的 5.65 cm，在 truck05 240–390 s（扬尘/大车遮挡，帧内点数 7–9 k）要求优于纯 LIO；两者同时满足才算"视觉增强而非拖累"。
4. 前提修正：truck05 相机→LiDAR 旋转仍是供应商标称（§4.3），视觉权重再自适应也修不了外参错——应先做相机→LiDAR 细调。

后续（按优先级）：
0. **多雷达合帧器失配恢复**（C++）：某路扫描丢失/迟到后按时间戳丢弃最旧一路的旧帧立即重新对齐，而不是逐帧丢弃直到偶然对齐；同时 colour-only 模式下 LIO 不应因图像缺失断更。这是 truck05 全程 720 s 达标（≤ 8.5 cm）唯一剩余的障碍：去掉 5 处帧空洞后其余 700 s 的 xy ≤ 8.3 cm、z ≤ 9.7 cm。复现：`bash scripts/run_wuhu_round.sh <name> --mode livo --dataset …/05/20260914/ros2bag --calibration config/carlibration/wuhu/truck05/v024_precheck/calibration.json --start-offset 320 --duration 70`。
1. truck05 几何仲裁 `lidar_frontend` / 位置关联（§3.3），目标 5 ms。
2. truck05 相机→LiDAR 旋转细调 + 边缘法定相机时间（§4.3），然后目视 `rgb_map` 边界上色。
3. 把 `time_sync_check.py`/`camera_sync_check.py` 接到 `evaluate_wuhu_round.sh` 的可选步骤。
4. 原生 z 漂移（truck29 120 s 3.0 m）与融合后姿态差（truck05 yaw P95 0.45–0.47° vs INS）作为下一个精度项：远距离（40–70 m）平面偏移优化后反而增大（W40 wall 23→46 mm），说明后端在远处引入了姿态/沿迹抖动。

## 9. truck05 加入视觉后精度下降的根因分析（数据/标定输入，还是算法）

用户裁定：truck05 达不到 8.5 cm 若是数据本身导致（时间戳、原始外参不准、动态障碍等）则不改。本节用同一段数据（598–658 s，91 m，含一次转弯）、同一标定（v024_precheck）、同一二进制，把"视觉为什么拖累 truck05"逐项拆开。所有轮次见 §6。

### 9.1 现象

| 配置（W598） | 后轴 xy max | 原生（前端）3D 漂移 max | 说明 |
| --- | --- | --- | --- |
| LIO 前端（colour-only 诊断） | 5.65 cm | 0.285 m（0.3 % 路程） | LIO 健康：有效特征 1.5–1.8 万/帧，配准残差 ~2 cm |
| 原始 LIVO 默认（`img_point_cov 100`，曝光估计关） | **16.49 cm** | **14.66 m**（t=25–35 s 从 0.3 m 跳到 7 m 再到 14 m，之后不再恢复） | 前端发散，靠 RTK 后端拉回 |
| LIVO + `vio.exposure_estimate_en=true` | 10.13 cm（z 16.7） | 0.96 m | 发散消失，仍在同一段注入 0.7 m 级误差 |
| LIVO + `vio.img_point_cov=1000`（视觉权重 1/10） | 8.62 cm | 0.26 m | 回到 LIO 前端水平 |
| LIVO + 曝光估计 + 权重 1/10 | 8.59 cm | 0.25 m | 与仅降权相同，降权起决定作用 |
| 对照：truck29 LIVO 默认 / LIO | 5.86 / 7.09 cm | 3.15 / 2.98 m（580 m，0.5 %） | 视觉参与后精度**提高** 1.2 cm |

发散段（窗口 t=25–35 s，bag 623–633 s）的逐 5 s 剖面（`mcp_t05_livo_w598`）：车速 3.0→3.7→3.1 m/s（窗口内最快段），LiDAR 帧内点数 21 k→16 k（最低），视觉子图检索点数 157→108→67→40（最低），前端 RTK 残差 0.17→0.69→2.97→11.1 m。同一段在 LIO 前端下原生误差只从 0.12 m 走到 0.17 m。

### 9.2 逐项排查

1. **时间戳（排除）**。`scripts/analysis/camera_stream_audit.py`（新增）：两台车相机 header 间隔均为 33.33 ms、0 处 >50 ms 间隙、接收时间 = header 时间、无近重复帧；IMU 间隔干净（truck05 仅已知的 3 处 20–40 ms 间隙）。truck05 全程回放里的 828 条 `dropped_duplicate` 来自 48 段分片 bag 的重叠，不是传感器。相机→IMU 偏移值本身也不是原因：此前 −35/+20/+40 ms 的扫描全部发散（§4.3）。
2. **相机模型（无法验证，是主要嫌疑）**。truck05 内参 `[1849.37 1849.37 1003.31 502.72]`、畸变全零，来自供应商地面距离曲线反演，从未用标定板标定；相机→LiDAR 旋转为标称值（`camera-lidar` 阶段因无立杆从未跑过）。新增 `scripts/analysis/camera_extrinsic_check.py`（直行帧上 LiDAR 深度边缘 vs 图像梯度的 (pitch,yaw) ±1.2° 网格搜索）：truck29 联合峰 (−0.93°, +0.03°)、对比度 0.029、单帧 MAD 0.9°；truck05 W40 对比度 0.010、单帧估计散布在整个网格（yaw MAD 1.56°）——**该数据里没有足够的静止竖直结构来验证或修正相机外参**。对 8×8 光度块而言 0.5° 旋转误差 = 16 px 的系统性偏差，会被固定权重的 VIO 当作状态误差吸收。truck29 的内参（fx≠fy：1642/1855）与外参是标定过的。
3. **光照与动态障碍（成立）**。抽帧目视：628 s 画面中央一辆迎面矿卡（动态）；634 s 转弯时路面大面积过曝（高光溢出）；全程直射阳光、植被深阴影。`scripts/analysis/camera_texture_stats.py`（新增）12 帧统计：truck05 亮度 P10–P90 95–113（摆幅 18 灰阶）、对比度 63；truck29 阴天均匀光照，亮度 129–135（摆幅 6）、对比度 47。光度 VIO 的前提是亮度恒定；我们的 yaml 里 `exposure_estimate_en: false`，曝光变化直接进入残差。实测打开曝光估计后前端不再发散（14.66→0.96 m），但仍剩 10.13 cm——曝光只是因素之一。同一开关在 truck29 上回归为 6.99 cm（差于默认 5.86），且 70 s 后视觉子图检索点数跌到 0（逆曝光状态漂移把所有块判为离群），因此**不改默认值**；曝光问题应在相机侧（锁定曝光）或按 §8 的质量门处理。注意 truck05 图像并不"弱纹理"（梯度均值 10.4 vs 4.8，纹理像素 30 % vs 8 %），是碎石重复纹理 + 饱和 + 动态，对块匹配不利。
4. **视觉信息量少（成立）**。视觉子图检索点数中位 truck05 95 vs truck29 219；<20 点的帧占 20 % vs 4 %；成功更新点数中位 34 vs 186，19.5 % 的帧为 0。发散段只有 40–67 点。原因是相机视场内可用的 LiDAR 点少（开阔路面、远坡、天空占大半画面，帧内点数同时跌到 16 k）。
5. **码率（次要）**。truck05 HEVC 码率约低 40 %（包中位 12.4 vs 20.8 kB，关键帧每 30 vs 60 包），块效应 1.055 vs 0.973，影响小。
6. **融合权重（放大器）**。原始 LIVO 用固定 `img_point_cov` 把每个块残差当同等可信；把权重降到 1/10 后，同一数据回到 LIO 水平（8.62 cm，原生 0.26 m）。说明前 5 项造成的光度误差本身不大，是固定权重把它放大成了前端发散。

### 9.3 结论

truck05 加视觉后精度下降是 **数据/标定输入的问题**，不是标定链路或 LIO 的错误：同一段、同一标定，视觉关闭或降权即 5.65–8.6 cm；四雷达外参、杆臂、时间偏移在 LIO 模式下全程 7.25 cm（v019）。具体输入缺陷按贡献排序：(a) 相机内参/相机→LiDAR 旋转未标定且本数据无法验证；(b) 强光过曝 + 动态矿卡违反亮度恒定；(c) 相机视场内 LiDAR 点少；(d) 码率低。这些都不能从这份录制离线补救（没有标定板、没有立杆场景），所以 **truck05 LIVO ≤ 8.5 cm 不能仅靠这份数据达成**；LIVO 在 truck29（相机标定过、阴天）上视觉是正增益（LIO 7.09 → LIVO 5.86）。

### 9.4 建议

- 现阶段 truck05 生产运行：`--mode lio`（全程 7.25 cm），或 LIVO 加 `--set vio.img_point_cov=1000`（W598 8.62 cm，原生漂移与 LIO 相同）。不改默认 yaml，truck29 默认配置已验证。
- 补齐 truck05 相机输入后再启用全权重 LIVO：标定板内参 + 畸变；在有立杆/建筑的路段录一段用于 `camera-lidar` 阶段（或沿 truck29 路线录）；相机曝光锁定或提高码率。
- 算法侧（§8）仍然成立且更具体了：视觉权重按 LiDAR 退化程度与视觉子图规模自适应（检索 <50 点或曝光估计跳变时降权/跳过），使 truck29 保持 5.86、truck05 不低于 LIO 水平。

## 10. 视觉权重的 LiDAR 对等界（parity bound）与 10 Hz 整帧 LIO：起步段灯柱重影的修复

用户要求（2026-09-28）：继续按 §8 改进；RTK 本身有几厘米 / ≥0.05° 的误差，不必贴近 RTK，最大误差 ≤ 10 cm 即可；重点看 truck29 刚起步的高特征路段——纯 LIO 时灯柱很锐，融合后不能变粗/重影。

### 10.1 度量：起步段灯柱锐度（新增 `scripts/analysis/start_pole_sharpness.py`）

不依赖外部参考，直接用轮次自己的归档：fused 点 = 保存的 `map_*.pcd`（用户在 CloudCompare 看到的），native 点 = 同一批回波按各帧原生 LIO 位姿重投影（逐帧施加 T_native·T_fused⁻¹，LiDAR/IMU 约定互相抵消）。在前 30 s 的 fused 点云上自动检出细高竖直结构（≥3 m 高、≤1.2 m 宽），只取离地 1–4 m 的杆身：
- 杆身径向 P90（含物理半径），native / fused；
- **中心弥散半径**：按 1 s 时间桶、共享半径拟合圆心，取回波 ≥100 的桶，圆心到加权平均圆心距离的 P90——这就是观察者看到的"杆变粗/重影"；对 native 与 fused 分别计算，二者之差是融合引入的部分。
另有已有的多历元灯柱轴线分离度（`round_summary`，chunk 间同一灯柱轴线 XY 距离）作为全程口径。

### 10.2 诊断：为什么 LIVO 的起步段灯柱比纯 LIO 粗

| truck29 轮次（v063，同一二进制） | 后轴 xy max | 前 40 s 原生 xy 漂移 | 起步段灯柱中心弥散 native / fused（3 根 7.4 m 灯柱中位；第 2 根） | 多历元轴线分离中位 |
| --- | --- | --- | --- | --- |
| `mcp_t29_lio_default`（LIO 10 Hz） | 7.09 | 21 cm | 4.7 / 5.2（4.7 / 5.2） | 9.0 |
| `mcp_t29_livo_default`（原始 LIVO，30 Hz 切片，固定权重） | 5.86 | **87 cm** | 6.0 / 5.1（**11.2 / 13.6**，拟合半径 11.7 vs 8.1 cm：杆"胖"了 40 %） | 12.2 |
| `mcp_t29_colour_div1`（30 Hz 切片，关视觉更新） | 6.71 | 73 cm | 5.6 / 6.6 | 10.5 |
| `mcp_t29_livo_cov1000`（30 Hz，视觉权重 1/10） | 6.71 | 65 cm | 6.0 / 6.1（6.0 / 6.1） | 11.6 |
| `mcp_t29_livo_msig012`（30 Hz，后端 xy 运动先验 0.012） | 6.32 | 88 cm | 5.8 / 6.0 | 11.2 |

两个独立原因：
1. **30 Hz 图像切片削弱了 LIO**（主因）。LIVO 模式把四雷达合帧扫描按图像时刻切成 33 ms 切片做 LIO 更新，每片只有 1/3 回波且只覆盖部分扇区；关掉视觉更新（`colour_div1`）漂移仍有 73 cm，与开视觉的 87 cm 同量级，而整帧 10 Hz 的 LIO 只有 21 cm。原始 FAST-LIVO2 的切片设计针对单台非重复扫描 Livox，不适合矿卡的四雷达 10 Hz 合帧。
2. **固定视觉权重远超 LiDAR 信息**（次因，但决定了 truck05 的发散）。用 `Log/lio_geometry.csv` 的 LiDAR 信息矩阵算，truck29 一帧 LIO 更新的最弱方向 σ ≈ 1–2 mm（富特征段）、3–6 mm（开阔段）；而 200 个视觉点 × 64 像素、固定 `img_point_cov=100`（σ=10 灰阶）注入的信息比它大约 3 个数量级——前端位姿实际由光度对齐决定，LiDAR 只是"建议"，任何光度模型误差（时间/外参残差、曝光、动态物体）直接变成位姿误差。

### 10.3 修改（`762b01d` 之后，见 `git log`）

- **视觉更新的 LiDAR 对等界**（`src/vio.cpp` `VIOManager::parityCovarianceScale`，`vio.lidar_parity_ratio`）：每帧每金字塔层，用第一次迭代的 H 计算视觉信息矩阵 HᵀH/img_point_cov 的旋转块、平移块最大特征值，与同一帧 LIO 的信息矩阵（`voxelmap_manager->lidar_information_body_`，handleLIO 刚算完的）最弱特征值之比；比值超过 `lidar_parity_ratio` 的部分按倍数放大该帧的 `img_point_cov`。LiDAR 退化的帧比值天然小，视觉保持原始全权重——正是 LIVO 想要的"退化时视觉兜底"。特征值极值与坐标系无关，所以平移块 world/body 的差别不影响这个界。`lidar_parity_ratio: 0` 恢复原始固定权重。控制台每帧打印 `[ VIO ] LiDAR parity: vision covariance x…`，`CALIBRATION IN EFFECT` 与 `parameters.txt` 打印 `effective.vio_lidar_parity_ratio`。实测放大倍数 truck29 中位 ×400–600、P10 ×20–180、P90 ×4000–5000，即视觉在 90 % 的帧被大幅降权，在 LiDAR 最弱的 10 % 帧接近全权重。
- **相机分频默认 3**（`camera.frame_divider: 3`）：LIO 回到 10 Hz 整帧，视觉更新与上色按 10 Hz 图像进行。这不是"砍相机"：视觉仍在前端 ESIKF 里更新状态，只是不再把 LiDAR 扫描切碎。

### 10.4 结果（truck29，v063，新二进制）

| 轮次 | 配置 | 后轴 xy 中位/P95/max | z max | 前 40 s 原生 xy 漂移 | 起步段灯柱弥散 native / fused（第 2 根） | 多历元轴线分离中位 |
| --- | --- | --- | --- | --- | --- | --- |
| `mcp_t29_livo_parity1` | 30 Hz 切片 + 对等界 1.0 | 0.85 / 2.32 / 6.53 | 4.03 | 61 cm | 4.6 / 5.5（5.9 / 6.6） | 11.2 |
| `mcp_t29_livo_parity01` | 30 Hz 切片 + 对等界 0.1 | 0.86 / 2.50 / 6.43 | 6.71 | 71 cm | 5.9 / 6.3（5.9 / 6.3） | 11.2 |
| **`mcp_t29_livo_parity1_div3`** | **分频 3 + 对等界 1.0（新默认）** | **0.77 / 2.04 / 5.21** | 5.18 | **25 cm** | **3.5 / 3.9（3.8 / 4.5）** | **7.2** |
| **`mcp_t29_livo_newdefault`** | 同上，yaml 默认、不带 `--set`（复现 + 存图） | **0.90 / 2.29 / 4.87** | 4.02 | 24 cm | 3.6 / 4.8（4.8 / 6.9） | 9.2 |

新默认同时满足三项：全程 xy max 5.21 / 4.87 cm（两次独立回放，历史最好；P95 2.04 / 2.29 也是最好）、起步段灯柱弥散 3.9 / 4.8 cm（两次都不差于纯 LIO 的 5.2，原 LIVO 第 2 根灯柱 13.6）、多历元灯柱轴线分离 7.2 / 9.2 cm（原 12.2，LIO 9.0）。灯柱指标的回放间波动约 1 cm，因此只能下"不差于 LIO、明显好于原 LIVO"的结论，不能再细分。RTK 侧的 max 由 5.86 变 ~5 只是顺带，评价依据是灯柱与前端漂移。

### 10.5 truck05 用同一默认（对等界 1.0 + 分频 3，v024_precheck）

| 轮次 | 后轴 xy 中位/P95/max | 3D max | 前端原生漂移 | 视觉放大倍数（中位 / P10） |
| --- | --- | --- | --- | --- |
| `mcp_t05_livo_w598`（原始 LIVO） | 1.27 / 4.47 / 16.49 | 16.50 | 14.7 m（发散） | — |
| **`mcp_t05_livo_w598_parity1_div3`** | **0.86 / 2.82 / 6.12** | 7.01 | 0.28 m（= LIO 水平 0.29） | ×56 / ×11 |

truck05 在 LiDAR 信息相对更弱的帧里视觉保留了更多权重（放大倍数中位 ×56，truck29 为 ×400–600），仍然不再发散：§9 列出的数据/标定输入缺陷没有变，但固定权重这个"放大器"被拿掉后，它们不再能把前端拉偏。§9 的结论因此更新为：**truck05 的相机输入缺陷依旧存在（内参/外参未标定、强光、动态车），但在对等界下 60 s 窗口已回到 ≤ 8.5 cm；全程 720 s 见 §10.6。**

`--start-offset 320 --duration 70`（扬尘/大车段）单独起跑无效：320 s 时车已在行驶，IMU 在运动中初始化，前端从第 1 帧起就偏 7 m 并线性发散（任何配置都一样），该段只能在全程回放里评估。

### 10.6 truck05 全程 720 s：LIVO 与 LIO 同一二进制、同一标定（v024_precheck）对照

| 轮次 | 模式 | 后轴 xy 中位/P95/max | z max | 3D max | 前端帧数（10 Hz 应为 7170） | 帧空洞 >0.15 s |
| --- | --- | --- | --- | --- | --- | --- |
| `mcp_t05_colour_full`（§5.1，旧二进制，图像只上色） | ≈LIO | 0.72 / 2.69 / 13.47 | 44 | — | — | 32 |
| **`mcp_t05_lio_full_newbuild`** | LIO | 0.75 / 2.78 / **7.31** | 38.4 | 38.4 | 6046 | 282 |
| **`mcp_t05_livo_full_newdefault`** | LIVO（新默认） | 0.83 / 2.99 / **9.89** | 39.3 | 39.8 | 6338 | 92 |
| 历史 LIVO 全程（v019，原始 LIVO） | LIVO | — / — / 47.3 | — | — | — | — |

逐 60 s 的 3D max（cm）两条曲线几乎重合——LIO `[8.1, 9.1, 3.6, 5.2, 26.6, 37.2, 36.8, 2.0, 0.8, 2.4, 3.4, 1.8]`，LIVO `[8.7, 5.8, 3.8, 7.7, 28.2, 39.6, 35.3, 2.4, 0.7, 3.8, 7.2, 2.0]`：**加入视觉后 truck05 的误差已与纯 LIO 同级**（xy max 9.89 vs 7.31，差值出在同一路段，且在 §6 记录的 0.5 cm 级回放噪声之外，但两者的超差都不是视觉引起的）。历史 LIVO 全程 47.3 cm 的"视觉拖累"在新默认下不存在。

两个模式共同的超差段 **340–395 s** 的机理（用 `trajectory_lio.txt` / `keyframes.csv` 逐 5 s 对齐 RTK 得到）：

| t（s） | 前端 z 误差相对 330 s 的增量 | 融合后 z 误差增量 | RTK 高程变化 |
| --- | --- | --- | --- |
| 345 | −0.25 m | −0.51 m | −1.1 m |
| 355 | −3.20 m | −0.66 m | −5.3 m |
| 365 | −6.15 m | −0.72 m | −10.5 m |
| 380 | −8.37 m | −0.45 m | −16.5 m |
| 395 | −9.27 m | −0.27 m | −20.0 m |

即车辆以 5–7 m/s 下一段 20 m 落差的坡道（扬尘 + 大车段），**前端 LIO 在 55 s 内高程漂了 9.3 m**（少算了约 45% 的下降量；LIO/LIVO 两种模式一样，RTK 全程 fixed、22–23 星），后端把 9.3 m 压到 0.3–0.7 m 但没有压到 ≤ 10 cm：位置因子的 Huber（δ = 2.8σ）与相对运动因子（布朗 σ_z = 0.028 m/√s）互相"鲁棒"之后的平衡点。§5.1 里"帧空洞导致"的解释因此修正为：帧空洞（363 s 一处 2 s）只是叠加项，主因是坡道扬尘段的前端高程漂移。这是数据（扬尘、坡道、动态车）触发的前端失效，不是标定或视觉问题；按用户裁定不在本轮追（后端对"RTK 健康而 LIO 粗大失效"的自适应降权是唯一可能的算法侧改进，见 §10.8）。

**帧空洞的来源**：`dataset.json` 显示录包每 30 s 段四路 LiDAR 各 300 帧完整无缺，空洞全部产生在回放链路：本机上 LIO 单帧 55–75 ms、VIO 24–29 ms，LIO/LIVO 都略慢于实时（实时倍率 LIO 1.05–1.13，LIVO 1.08–1.24），播放器按 `--max-lead 3 s` 节流后四路点云到达不均，合帧器待配队列（`multi_lidar.queue_size: 5`）溢出即丢帧（LIVO 1923 条 `Dropping queued …`，560 s 后集中），随后逐帧失配直到重新对齐。LIO 模式空洞反而更多（282 vs 92），说明空洞数量由回放抖动而非 VIO 负载决定（未再细查）。这解释了 §5.1 第 3 次全程"32 个帧空洞"的来源，并说明 truck05 全程在本机的评估受计算吞吐限制（truck29 120 s 不受影响）。

### 10.7 回归与不变量

- LIO 路径未被本轮改动触及：`mcp_t29_lio_newbuild`（新二进制）与 `mcp_t29_lio_default`（762b01d）后轴 xy 0.78 / 3.04 / 7.09 cm 逐位一致，前 40 s 原生漂移 21.2 cm 一致。
- `vio.lidar_parity_ratio: 0` 恢复原固定权重，`camera.frame_divider: 1` 恢复 30 Hz 切片，两者都仍可用 `--set` 复现旧行为。
- 上色路径不变（VIO 仍以 10 Hz 处理图像并给 `rgb_map_*.pcd` 上色）；控制台新增每帧 `[ VIO ] LiDAR parity: vision covariance x…` 与 `CALIBRATION IN EFFECT` 中的 `vio_lidar_parity_ratio`。
- `--start-offset 320` 单独起跑对 truck05 无效（车在行驶中初始化 IMU），该段只能在全程里评估。

### 10.8 遗留（按优先级）

1. 后端：RTK fixed 且 LIO/RTK 增量失配远超两者 σ 时（truck05 坡道段 17 cm/s vs σ_z 2.8 cm/√s），相对运动因子应自适应放大 σ（可切换约束），否则 Huber 会保护粗大失效的前端。预期把 truck05 全程 340–395 s 的 0.4–0.7 m 压到 10 cm 级；对 truck29 无影响（那里 LIO 漂移 21 cm/120 s）。
2. 吞吐：本机 LIO 55–75 ms/帧 + VIO 25–29 ms/帧，truck05 全程略慢于实时导致回放丢帧；候选是 LIO 迭代次数 / 下采样叶子的实测权衡（须以 truck29 灯柱与 xy max 双指标验收），以及合帧器待配队列 5 → 30 帧以吸收回放抖动。
3. §9 的 truck05 相机内参/外参标定与 §3.3 的 `lidar_frontend` 几何仲裁不变。
