# 标定链路复审：colour-only 模式、时间偏移审计与流水线精简（2026-09-27）

范围：`/home/lhx/docker/MY-LIVO`，分支 `fusion/56c6896-ins-attitude`（基线 3dbf7e7）。目标是 29/05 两台车全局误差 ≤ 8.5 cm、彩色点云上色正确、外参与时间偏移在控制台可见、时间偏移逻辑经得起审计、在不牺牲精度前提下删掉多余的流水线步骤。本文只记录实测结论；未经实测的推断都标明"未证实"。

## 决策更新（2026-09-28，用户裁定）

**图像必须参与前端状态估计**：LIVO 引入视觉的目的就是在 LiDAR 退化环境下增强鲁棒性，"图像只上色"违背这一目的。因此：

- `config/wuhu_truck29.yaml` 默认已改回 `vio.ekf_update_en: true`、`camera.frame_divider: 1`（原始 LIVO 行为，`--mode livo` 即全速 VIO 参与 EKF）。`ekf_update_en=false` 仅保留为**诊断开关**，不作为生产模式，也不再是流水线建议。
- 下文 colour-only 的数字（truck29 5.02 / truck05 4.32、5.65 cm）保留，但定位变为**诊断证据**：同一数据、同一标定，去掉视觉更新后误差降到一半，说明当前 VIO 更新在矿区富纹理/动态场景里注入了误差。结论不是关掉它，而是让视觉更新"该出手时出手"（§8 方案）。
- §5 中依赖 colour-only 的精简建议（删 `camloop`、`camera-time` 降为可选、LIO/LIVO 合并）**撤回**；视觉参与前端时相机时间与外参直接影响轨迹，这两个阶段保留。
- 回退验证：`mcp_t29_livo_default`（默认配置、v063、同一二进制）见 §6 末行。

## 0. 结论速览

| 项 | 结论 |
| --- | --- |
| 精度 | **colour-only LIVO**（`camera.frame_divider=3` + `vio.ekf_update_en=false`）在两台车上都是当前最优：truck29 120 s 全程后轴 xy max **5.02 cm**（3D 5.91），truck05 W40 **4.32 cm**（3D 7.38）、W598 **5.65 cm**（3D 7.87）。此前最优：truck29 LIVO 5.92 / LIO 6.56；truck05 LIO 6.0–7.4 / LIVO 11.3。**truck05 全程 720 s 尚未达标**：xy 中位 0.72 / P95 2.69 / max 13.47 cm，z max 44 cm；超差全部落在回放丢帧→多雷达合帧器失配→LIO 断更的 5 处帧空洞上（§5.1、§6），60 s 窗口无一处空洞；此前 LIO 模式全程 xy max 7.25 cm。|
| 上色 | 彩色点云仍由图像上色（`rgb_map_*.pcd`），VIO 只是不再参与状态更新。truck29 相机时间偏移经独立几何方法验证（LiDAR 深度边缘 vs 图像梯度，联合峰值 **−0.7 ms**，对比度 0.053）：−0.115 s 正确。truck05 该方法在 W40 无峰（对比度 0.011），原因是场景缺少静止竖直结构 + 相机外参只有供应商标称（见 §4.3）；投影叠加图目测外参无度级错误。|
| 控制台 | 启动与结束时打印 `CALIBRATION IN EFFECT` 摘要：四雷达→后轴 6 自由度、相机→后轴、内参/畸变、后轴→IMU 杆臂、四个时间偏移及其"作用到哪一路"的解释、后端估计的 INS 安装角、相机模式。|
| 时间偏移审计 | 语义链路已核对无误（§3.1）。发现并修掉两处评估侧错误（§3.4）。用四种信号级方法交叉检验：**只有 truck29 的三项偏移得到独立证实**；truck05 的 `lidar_frontend`（−0.0119）与 INS 位置关联（−0.0078 绝对）**无法用信号级方法在 ±15 ms 内判定**，几何法（§3.3）此前也是平坦剖面。因此本轮不改 truck05 时间偏移，明确写出"不可辨识"而不是拍一个数。|
| 效率 | 相机分频 3 使 LIO 更新从 30 Hz 回到 10 Hz，单帧 LIO 42–85 ms，720 s 全程实时；关掉 VIO 更新后 VIO 耗时 0。流水线可删/合并的步骤见 §5。|
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
| `mcp_t29_livo_default` | truck29 全程 117.7 s | v063 | **默认配置（VIO 参与 EKF，30 Hz）**，回退验证 | 0.97 / 2.67 / **5.86**（= 历史 v063 LIVO 5.92，二进制改动未改变 LIVO 行为）| 6.93 | 2.72 m | — | | | |

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
