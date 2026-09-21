# MY-LIVO：LIVO 精度问题诊断、标定体系重构与 LIVO Studio 工作台

日期：2026-09-21 分支：`fusion/56c6896-ins-attitude`（基线 run99，commit 5d68d5d/33c8858）
数据：`results/prepared/wuhu_livo_camera`（芜湖 29 号车，120 s，4×LiDAR + IMU/INS + 1080p30 相机）

本文替代早先同名文档中关于 `scripts/auto_precorrect_truck.py` / `scripts/carlib_densify_and_calibrate.py`
的描述：这两个脚本未接入 SLAM，前者是 `refine_multi_lidar_extrinsics.py` 第一阶段的复制品，后者是空壳；
它们的职责已分别由算法内静态自检模块和 `tools/calib/livo_calib.py` 承担（见 §2.3）。

## 0. 结论速览

| 问题 | 结论 | 证据 |
| --- | --- | --- |
| LIVO 红线（原生前端）偏离、点云重影、上色错位 | 主因是相机时间戳比 IMU/INS 时钟晚 **115 ms**（`img_time_offset` 原为 0）。VIO 用错时刻的位姿做光度更新，把姿态误差逐帧注入前端；LiDAR 本身已配置 -125 ms，相机未配置 | 光流角速度 vs 陀螺互相关：-115 ms（4 段各 30 s 一致，±3 ms，corr 0.92）；run100→run101 仅改此项：原生轨迹中位/最大 6.30/12.75 m → 2.00/4.31 m，原生 yaw 最大 3.86° → 0.73°，优化轨迹最大 15.2 → 7.6 cm |
| 相机外参/内参是否准确 | 内参 `fx=1642.6 ≠ fy=1855.5` 是正确的（不是标定错误）；相机→后轴外参在 0.1° 量级内，但 LIVO 原生漂移对这 0.1° 非常敏感：两份出厂文件的相机矩阵相差 0.08°，原生漂移 2.0 m vs 3.4 m；边缘对齐得到的 0.2° 修正反而使漂移升到 5.2 m | 焦距扫描 fx×1.00 得分 4.55 px，fx×1.12（=fy）6.49 px；闭环轮次 run101/106/107/108/105/109/110/111（§1.4） |
| LiDAR 外参微调放在独立工具还是算法静态初始化模块 | 两者分工：**离线版本化工具**产出正式外参（全部 6 自由度、相机、时间偏移，带报告与版本）；**算法内静态自检**只做起步静止段的地面 roll/pitch/z 核对，小量自动修正、超限告警，绝不改写标定文件 | run102：原始人工外参 + 自检 → 优化轨迹 100% ≤10 cm（max 10.2 cm），front_left 横滚 -1.30° 被自动修正；run103/105：精修外参 + 自检 → “确认无需修正” |
| 标定流程繁琐 | `tools/calib/livo_calib.py`：import → ground → driven → camera-time → camera-lidar，每步生成新版本 `config/carlibration/<矿区>/<车号>/<版本>/`，`registry.json` 记录当前版本；SLAM 只需一个 `calibration_file` | 本次 v001_import … v006_camlidar 全链路跑通，结果与人工精修 `perception_truck29_refined_v2.json` 一致 |
| 企业级 UI | `tools/studio/`（LIVO Studio）：数据准备 / 标定管理 / 一键建图 / 实时精度，纯标准库 HTTP 服务 + 无依赖前端，所有路径与选项持久化 | run105 由 UI 后端 API 启动并完成评估 |
| LIVO 与 LIO 剩余差距 | 时间偏移修复后 LIVO 轨迹指标已与 LIO 持平；地面点云厚度仍比 LIO 高约 15–20%（5–20 m：29–33 mm vs 25.7 mm）。这是 VIO 光度更新本身的噪声/权重问题，不再是同步或外参问题 | §1.6 给出后续实验（`vio.img_point_cov`、帧分频） |

## 1. LIVO 问题诊断

### 1.1 数据事实（`results/.cache/mcp_probe/ts_probe.py`）

- 所有话题 `log_time == header.stamp`；图像间隔恰好 33.333 ms（合成时间戳，非采集时刻），LiDAR 100 ms（抖动 0.02–0.24 ms），IMU 10 ms。
- 相机流为 HEVC（`ffmpeg_image_transport_msgs/FFMPEGPacket`），LIVO 运行时由 `image_transport republish` 解码为 `/midrange_camera/image_raw`。
- 静止段投影检查（`proj_check.py`）：路灯杆点云落在杆左侧约 5–10 px，说明外参误差量级 ~0.2–0.3°，不是主因。

### 1.2 相机时间偏移测定

方法（已固化为 `livo_calib.py camera-time`）：解码整段视频到 480×270，LK 光流取上半幅特征的中位位移，除以焦距与帧间隔得到相机坐标系角速度；
以 1 ms 步长搜索滞后，对每个滞后做带截距的最小二乘回归到陀螺 `gz`（yaw）与 `gx`（pitch），取残差最小的滞后并做抛物线内插。

结果：yaw 通道 **-115.0 ms**（corr 0.92，n=3599），pitch 通道 -115.2 ms；0–30/30–60/60–90/90–120 s 分段 -115/-111/-112/-118 ms（离散 6.5 ms）。
解释：真实曝光时刻 = header + (-0.115 s)。相机与 LiDAR 均比 IMU/INS 时钟晚约 120 ms（LiDAR -125 ms 已在 run97 修正），二者相对差仅 ~10 ms。

### 1.3 验证（同一份配置只改 `time_offset.img_time_offset`）

| 轮次 | img_time_offset | 原生后轴 vs RTK 中位/最大 | 原生姿态误差最大 yaw/roll/pitch | 优化后轴 中位/P95/最大 | 路线 ≤10 cm | 末帧关键帧 RTK 残差 |
| --- | --- | --- | --- | --- | --- | --- |
| run100 LIVO 基线 | 0 | 6.30 / 12.75 m | 3.86° / 2.14° / 2.01° | 2.5 / 6.1 / 15.2 cm | 99.2% | 0.121 m |
| run101 LIVO | -0.115 | 2.00 / 4.31 m | 0.73° / 1.07° / 0.83° | 1.9 / 4.1 / 7.6 cm | 100% | 0.046 m |
| run99 LIO 基线 | — | 2.38 / 4.65 m | — | 1.9 / 4.5 / 8.1 cm | 100% | — |

run100 的原生误差在 90 s 后迅速增大（yaw -3.6°、roll -1.9°），即错误图像时刻造成的 VIO 拖拽是累积性的，这正是 RViz 中红线越走越偏、点云重影的来源。

### 1.4 相机外参与内参验证

代码链路核对：`cameraextrinR/T = inverse(transforms.midrange_camera_to_rear_axle)`（相机←后轴=虚拟 LiDAR），VIO 内 `Rci = Rcl·Rli`，与 LiDAR 外参链一致；
18 元素的 `midrange_camera[0].tf_lidar_to_camera` 从未被 SLAM 读取（正确的恢复方式是丢弃前两个元素，各变体差异 <0.1°≈3 px）。

`livo_calib.py camera-lidar`（run103 fusion 目录，12 帧，±1 s 累积点云，深度边缘 → 图像长边缘距离场）：

- 6 自由度最优改动：旋转 (-0.10°, +0.10°, +0.20°)，平移 <1 mm；得分 4.63 → 4.55 px（半分辨率）。外参已在方法分辨率之内。
- 焦距扫描（fy 固定、每个 fx 重新优化旋转）：fx×0.92 4.98，×0.96 4.73，**×1.00 4.55**，×1.04 4.95，×1.08 5.74，×1.12 6.49，×1.16 7.12 px。
  即 `fx=1642.6, fy=1855.5` 的各向异性内参与激光几何一致；光流增益暗示的 fx≈1880 是平移光流污染，不可采信。
- 场景限制：芜湖路段两侧是树林，Canny 边缘密集导致距离场平坦，因此工具只保留 ≥25 px 的长边缘链，并输出前后叠加图供人工确认
  （`config/carlibration/wuhu/truck29/v006_camlidar/overlays/`）。带建筑/标志牌的场景会给出更强的约束。

闭环验证（同一二进制、同一运行包，只换相机→后轴旋转；重复运行 run106/107 原生中位 3.33/3.37 m，说明结果可复现、差异是真实的）：

| 相机旋转（相对 refined_v2，相机系 xyz，°） | 来源 | 轮次 | 原生 中位/最大 | 原生 yaw 最大 | 优化 中位/最大 | 5–20 m 厚度 |
| --- | --- | --- | --- | --- | --- | --- |
| (-0.17, +0.06, +0.20) | v006 边缘对齐 | run105 | 5.16 / 9.50 m | 0.65° | 2.1 / 9.0 cm | 30.5 mm |
| (-0.07, -0.04, 0) | 出厂 schema-5 文件（v001/v005） | run106/107/108 | 3.33–3.44 / 7.65–7.90 m | 1.10–1.21° | 2.0 / 8.7–9.6 cm | 30.5–30.6 mm |
| (0, 0, 0) | refined_v2（schema-4 文件的相机矩阵；生产版本 v007） | run101 / run111 | 2.00 / 4.31 m，2.11 / 4.33 m | 0.73° / 0.67° | 1.9 / 7.6 cm（两轮相同） | — / 29.4 mm |
| (+0.07, +0.04, 0) | v008 | run109 | 2.47 / 4.27 m | 0.56° | 2.0 / 8.0 cm | 28.7 mm |
| (+0.14, +0.08, 0) | v009 | run110 | 3.54 / 6.12 m | 0.81° | 2.0 / 8.2 cm | 27.8 mm |

结论：① 所有配置经 RTK 融合后都满足路线预算（100% ≤10 cm，最大 <10 cm），相机外参 0.1° 级差异只影响原生前端（红线）与地图厚度；
② refined_v2 的相机矩阵是原生漂移的最小值（run111 复现 run101：优化最大 7.6 cm），选它为生产版本（`v007_import`，registry 当前版本）；+0.07/+0.14° 方向原生漂移变差但地面厚度单调变薄（29.4 → 28.7 → 27.8 mm），说明厚度与前端漂移的最优点不重合，属于 VIO 权重问题（§1.6）；
③ 单靠深度边缘对齐（v006）在树林场景会给出错误方向的 0.2° 修正，因此 `camera-lidar` 阶段的输出只作建议，必须用一轮 LIVO 闭环（原生 yaw/漂移、地面厚度）确认后才能设为当前版本；
UI 中该阶段默认不勾选“设为当前版本”，`livo_calib.py set --camera-rot-delta` 提供闭环微调入口。

### 1.5 算法逻辑相关发现

- LIVO 模式下 LIO 以图像节奏（33 ms 子扫描）更新，RTK 关键帧提交延后到 VIO 上色之后（`pending_rtk_livo_frame_`）；前端姿态拉回（pull_fraction）按更新次数生效，因此每秒作用强度是 LIO 的 3 倍。这些不是错误，但意味着 LIVO 的原生轨迹统计不能与 LIO 直接逐项比较。
- 实时性：30 Hz 1080p VIO 在本机不能实时（2 min 结束时 “IMU and LiDAR delta time” 滞后 12–13 s，图像缓存随时长线性增长）。新增 `camera.frame_divider`（每 N 帧处理 1 帧）：run104 取 3（10 Hz）滞后降到 1.3 s，路线预算仍通过（中位 2.3 cm，最大 8.3 cm），原生漂移略增（3.05/6.54 m）。长时间在线运行建议 2。
- 评估脚本 `scripts/evaluate_wuhu_lio.py` 之前不能读取 LIVO 的 XYZRGB PCD（“Expected binary float XYZI PCD”），已修复 `read_pcd`；LIVO 轮次同时导出几何 `map_*.pcd` 与 `rgb_map_*.pcd`。

### 1.6 剩余差距与后续实验

| 轮次 | 配置 | 优化后轴 中位/最大 | 5–20 m 地面厚度 | 20–40 m 地面厚度 |
| --- | --- | --- | --- | --- |
| run99 LIO | refined_v2 | 1.9 / 8.1 cm | 25.7 mm | 35.5 mm |
| run103 LIVO | -0.115，自检 | 1.9 / 8.0 cm | 31.5 mm | 45.5 mm |
| run104 LIVO | 同上 + frame_divider 3 | 2.3 / 8.3 cm | 30.8 mm | 39.1 mm |
| run105 LIVO | v006 标定文件（相机内参/时间偏移由文件提供） | 2.1 / 9.0 cm | 30.5 mm | 42.9 mm |
| run109 LIVO | v008（相机 +0.07/+0.04°） | 2.0 / 8.0 cm | 28.7 mm | 44.7 mm |

LIVO 地图比 LIO 厚 12–20%，且 10 Hz VIO 时同样存在，说明来源是 VIO 光度更新注入的位姿噪声（稀疏视觉地图、8×8 patch、相机外参 0.1° 级残差——§1.4 已证明前端对其高度敏感），而非同步。
建议下一步（各 1 轮，约 4 min，可直接在 LIVO Studio 一键建图页用“额外参数覆盖”执行）：① `vio.img_point_cov` 100 → 300/1000 降低 VIO 权重，比较厚度与原生姿态误差；② `frame_divider 2`；③ 用 `set --camera-rot-delta` 做 ±0.05° 网格的闭环微调；④ 在有建筑物的路段重做 camera-lidar 阶段。

## 2. 标定架构决策

### 2.1 为什么不是二选一

- 静止起步段只能观测各 LiDAR 相对地面的 roll/pitch/z（3 个自由度），且依赖“地面在 40 m 内近似平面”的假设，精度上限约 0.1–0.2° / 2–3 cm（run103 对已精修外参给出 0.09–0.15°、2–3.5 cm 的“修正”即是该本底）。
- yaw/xy、相机外参、时间偏移需要行驶数据、累积点云与人工可核查的报告，属于离线、可复现、可回滚的工作。
- 因此：算法内模块负责“上车即检、明显装错立即发现并小量兜底”；离线工具负责“产出正式外参并版本化”。

### 2.2 分工

| | 算法内静态自检 `src/extrinsic_precheck.cpp` | 离线工具 `tools/calib/livo_calib.py` |
| --- | --- | --- |
| 触发 | 每次建图启动，陀螺 <0.5°/s 的前 5–12 帧 | 操作员在 UI/命令行执行 |
| 观测 | 各 LiDAR 地面 vs 参考 LiDAR（/front_lidar）地面 | 静止地面 + 行驶累积点云 + 图像光流 + 深度边缘 |
| 求解 | 每个 LiDAR 的 rot_x/rot_y/dz（Huber 加权高斯牛顿，绕 LiDAR 自身位置旋转） | 全部 6 自由度、相机外参、相机时间偏移、内参核查 |
| 动作 | 修正量在 (0.2°/4 cm, 1°/15 cm) 之间：应用到本次运行；小于下限：确认；大于上限：WARN 不应用 | 生成新版本目录，永不改写输入 |
| 产物 | 日志 + `results/<轮次>/Log/extrinsic_precheck.json` | `calibration.json` + `meta.json` + `report.md` (+ 叠加图) |
| 固化 | `livo_calib.py from-precheck --run-dir results/<轮次>` 可把自检修正转为正式版本 | `registry.json` 当前版本 |

### 2.3 遗留脚本处置

- `scripts/auto_precorrect_truck.py`：与 `refine_multi_lidar_extrinsics.py` 第一阶段重复，未被 SLAM 调用；其“开机静止即修正”的意图由 `extrinsic_precheck` 在算法内完成。建议删除。
- `scripts/carlib_densify_and_calibrate.py`：空壳（无实际求解）。相机-LiDAR 精修由 `livo_calib.py camera-lidar` 实现（运动累积致密化 + 深度边缘/图像边缘对齐 + 焦距扫描 + 叠加图）。建议删除。
- `config/perception_truck29_auto_refined.json`、`config/perception_truck29_cam_sanitized.json`：未被任何配置引用，建议删除；正式文件在 `config/carlibration/wuhu/truck29/`。
- `scripts/refine_multi_lidar_extrinsics.py`、`scripts/refine_multi_lidar_yaw.py` 保留，作为工具的 ground/driven 阶段被子进程调用（它们保留未知 JSON 字段，因此 schema-6 的新增段不会丢失）。

## 3. 算法内静态初始化外参自检

文件：`include/extrinsic_precheck.h`、`src/extrinsic_precheck.cpp`，接入点 `src/LIVMapper.cpp`（多 LiDAR 合并之后、送入 LIO 之前）。

流程：
1. 起步静止判定：IMU 陀螺模长 <`static_gyro_deg_s`，收集 `min_frames`–`max_frames` 帧（每帧保留各源在后轴坐标系的原始返回）。
2. 参考 LiDAR 地面模型：1.5 m 网格取最低点带（p5+0.35 m）做局部平面，要求 ≥12 点、法向 z>0.9、厚度 <6 cm。
3. 对每个非参考 LiDAR：其地面候选点到参考平面的点到面残差，Huber 加权高斯牛顿求 (rot_x, rot_y, dz)，旋转绕该 LiDAR 自身位置（保证 yaw/xy 不动）。
4. 门限：`min_apply_rotation_deg 0.2 / min_apply_height_m 0.04`（低于则“确认”）、`max_rotation_deg 1.0 / max_height_m 0.15`（高于则 WARN，不应用）；残差不下降则不应用。
5. 产物：日志每源一行；`Log/extrinsic_precheck.json`（status applied/confirmed/gated/skipped、配置、每源前后残差、修正量、前后矩阵）。

配置（`config/wuhu_truck29.yaml` → `multi_lidar.precheck`），UI 中为“静态初始化外参自检”开关。

结果：

| 轮次 | 输入外参 | 自检结论 | 优化后轴 中位/最大 | 路线 ≤10 cm | 5–20 m 地面厚度 |
| --- | --- | --- | --- | --- | --- |
| run102 LIO | 原始人工 `perception_truck29.json` | front_right (-0.14°, +0.17°, +1.4 cm)、front_left (-1.30°, +0.03°, +1.5 cm) 应用；back 无重叠未检 | 2.55 / 10.2 cm | 100% | 27.3 mm |
| run98 LIO（对照，离线第一阶段） | refined（stage1） | — | 2.49 / 9.6 cm | 100% | — |
| run99 LIO（对照，stage1+2） | refined_v2 | — | 1.90 / 8.1 cm | 100% | 25.7 mm |
| run103/105 LIVO | refined_v2 / v006 | 修正量 0.09–0.15°、2–3.5 cm，低于下限 → 确认 | 1.9 / 8.0 cm | 100% | 31.5 / 30.5 mm |

结论：自检把“外参明显装错”的车（front_left 横滚 -1.3°，16 cm 地面错位）自动拉回到离线第一阶段的水平；剩余到 run99 的差距（yaw/xy、后雷达）必须由离线 driven 阶段完成。
后雷达在静止时与前雷达无地面重叠，静止自检对其无效，这是几何限制而非缺陷。

## 4. 离线标定工具 `tools/calib/livo_calib.py`

运行：容器内 `/usr/bin/python3 tools/calib/livo_calib.py <子命令>`（自动 re-exec 到系统 python 并加载 ROS 2 环境）。

目录与版本：`config/carlibration/<mine>/<truck>/vNNN_<stage>/{calibration.json, meta.json, report.md, stage.log, overlays/}`；
`config/carlibration/registry.json` 记录每辆车的当前版本；失败的阶段不会留下半成品目录。

`calibration.json`（schema 6）= 原 schema-5 内容（`lidar[]` 后轴外参矩阵、`transforms.midrange_camera_to_rear_axle`）+
`camera{width,height,intrinsics,distortion}` + `time_offsets{lidar,camera,rtk_position,ins_attitude}` + `identity` + `history`。
SLAM 读取 `multi_lidar.calibration_file` 时，若存在 `camera`/`time_offsets` 段则覆盖 yaml 中的相机内参与 `lidar/img_time_offset`（日志 “Camera model from calibration file” / “Time offsets from calibration file”），换车只需换文件。

| 子命令 | 输入 | 作用 | 本次结果 |
| --- | --- | --- | --- |
| `import --source --mine --truck [--lidar-offset --camera-offset]` | 出厂 JSON | 正交化矩阵、修复 18 元素相机矩阵、补 camera/time_offsets 段 | v001_import（4 个矩阵正交偏差 5e-5 被修复） |
| `ground --id --bag` | 运行包 | 静止段 roll/pitch/z（`refine_multi_lidar_extrinsics.py`） | v002_ground：front_left -1.31°，与人工精修完全一致 |
| `driven --id --bag --run` | 运行包 + 用父版本跑过的 LIO fusion 目录 | 行驶窗口 6 自由度（`refine_multi_lidar_yaw.py`） | v003_driven：back 残差 8.3 → 3.8 cm |
| `camera-time --id --bag` | 含相机的运行包 | 光流 vs 陀螺时间偏移 | v005_camtime：-0.115 s，corr 0.92 |
| `camera-lidar --id --bag --run [--solve-intrinsics]` | 含相机运行包 + LIVO fusion 目录 | 深度边缘/图像边缘对齐精修相机外参；焦距扫描；叠加图 | v006_camlidar：改动 0.1–0.2°，fx 扫描确认内参 |
| `from-precheck --id --run-dir` | 某轮 `Log/extrinsic_precheck.json` | 固化算法内自检修正 | — |
| `set --id [--camera-offset --lidar-offset --intrinsics --distortion]` | 人工值 | 生成新版本 | — |
| `list / show / activate / export` | — | 版本管理 | — |

耗时（本机）：import <1 s，ground ~50 s，driven ~70 s，camera-time ~2 min（含解码），camera-lidar ~3 min。

## 5. LIVO Studio（`tools/studio/`）

启动（容器内，host 网络）：`/usr/bin/python3 tools/studio/server.py --port 8765`，浏览器打开 `http://localhost:8765`。
标准库 `ThreadingHTTPServer`，前端 `web/{index.html, style.css, app.js}` 无外部依赖；所有输入路径与选项持久化在 `config/carlibration/studio_state.json`。

页面：
- 总览：磁盘/负载/GPU/二进制时间、当前标定、正在运行的作业、最近评估结果表。
- 数据准备：选择原始 mcap 目录与输出目录（目录浏览器），可选“仅 LIO 话题”，调用 `scripts/prepare_wuhu_bag.py`，列出已准备的运行包（时长、话题、是否含相机、大小）。
- 标定管理：按矿区/车号分组的版本树、版本详情（各 LiDAR 与相机的 t/rpy、内参、时间偏移、阶段指标、叠加图、report.md）、导入出厂外参、执行任一阶段（父版本 + 运行包 + fusion 目录下拉选择）、设为当前版本。
- 一键建图：轮次前缀、LIO/LIVO、运行包、标定版本、RTK 融合 / 保存地图 / RViz / 静态自检 / 自动评估开关、回放速率、相机帧分频、额外 `--set` 覆盖；启动 `scripts/run_wuhu_round.sh`（新增 `--bag`、`--calibration`）。
  状态卡片每 2 s 刷新：帧数、关键帧、前端 vs RTK、优化轨迹 vs INS 参考、速度/延迟、两张曲线（10 cm 虚线）、算法事件（含自检结论表）、结束后的评估指标、地图分片数与运行日志。
  “提前结束并导出” = 停止回放，让轮次脚本正常排空/优化/导出/评估；“强制终止”= 杀掉全部进程。
- 结果与精度：历史轮次列表与同一状态视图。

新车上线 SOP：数据准备（含相机）→ 标定管理导入出厂 JSON → ground → 一键建图 LIO（用 v002）→ driven（选该轮 fusion 目录）→ camera-time → 一键建图 LIVO → camera-lidar → 设为当前版本 → 生产建图（自检开启，任何 WARN 回到标定页处理）。

## 6. 验证轮次汇总

| 轮次 | 模式 | 关键配置 | 原生 中位/最大 | 优化 中位/P95/最大 | 路线 ≤10 cm | 5–20 m 厚度 | 备注 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| run99 | LIO | refined_v2 | 2.38 / 4.65 m | 1.9 / 4.5 / 8.1 cm | 100% | 25.7 mm | 基线 |
| run100 | LIVO | img_offset 0 | 6.30 / 12.75 m | 2.5 / 6.1 / 15.2 cm | 99.2% | — | 用户看到的问题 |
| run101 | LIVO | img_offset -0.115 | 2.00 / 4.31 m | 1.9 / 4.1 / 7.6 cm | 100% | — | 时间偏移修复 |
| run102 | LIO | 原始人工外参 + 自检 | 3.42 / 7.31 m | 2.5 / 6.6 / 10.2 cm | 100% | 27.3 mm | 自检兜底 |
| run103 | LIVO | -0.115 + 自检 | 1.82 / 3.86 m | 1.9 / 4.1 / 8.0 cm | 100% | 31.5 mm | 自检“确认” |
| run104 | LIVO | 同上 + frame_divider 3 | 3.05 / 6.54 m | 2.3 / 4.7 / 8.3 cm | 100% | 30.8 mm | 实时（滞后 1.3 s） |
| run105 | LIVO | v006 标定文件（UI 启动） | 5.16 / 9.50 m | 2.1 / 4.7 / 9.0 cm | 100% | 30.5 mm | 相机边缘精修方向错误 |
| run106/107 | LIVO | v005 标定文件（UI 启动，重复两次） | 3.33 / 7.65 m，3.37 / 7.84 m | 2.0 / 4.3 / 8.7 cm，2.0 / 4.3 / 9.6 cm | 100% | 30.6 mm | 可复现性 |
| run108 | LIVO | v005，自检关闭 | 3.44 / 7.90 m | 2.0 / 4.3 / 8.8 cm | 100% | 30.6 mm | 自检不影响前端 |
| run109 | LIVO | v008（相机 +0.07/+0.04°） | 2.47 / 4.27 m | 2.0 / 4.3 / 8.0 cm | 100% | 28.7 mm | 闭环 |
| run110 | LIVO | v009（相机 +0.14/+0.08°） | 3.54 / 6.12 m | 2.0 / 4.5 / 8.2 cm | 100% | 27.8 mm | 闭环 |
| run111 | LIVO | v007（= refined_v2 + 文件内参/偏移，生产版本） | 2.11 / 4.33 m | 1.9 / 4.1 / 7.6 cm | 100% | 29.4 mm | 生产基线，与 run101 一致 |

## 7. 文件索引

- 算法：`include/extrinsic_precheck.h`、`src/extrinsic_precheck.cpp`、`src/LIVMapper.cpp`（自检接入、`camera.frame_divider`、标定文件 camera/time_offsets 段）、`src/rtk_fusion.cpp`（LIVO 导出修复）、`CMakeLists.txt`。
- 配置：`config/wuhu_truck29.yaml`（`multi_lidar.precheck`、`camera.frame_divider`、`time_offset.img_time_offset: -0.115`）。
- 标定：`tools/calib/livo_calib.py`、`config/carlibration/registry.json`、`config/carlibration/wuhu/truck29/v001_import … v006_camlidar`。
- UI：`tools/studio/server.py`、`tools/studio/web/`。
- 脚本：`scripts/run_wuhu_round.sh`（`--bag`、`--calibration`）、`scripts/evaluate_wuhu_lio.py`（XYZRGB PCD）。
- 探针（不入库）：`results/.cache/mcp_probe/{ts_probe.py, cam_time_offset.py, cam_offset_refine.py, proj_check.py}`。
