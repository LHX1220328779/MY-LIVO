# truck05 相机标定报错修复与标定流程核查 — 内参借用 truck29、窗口落在静止段、时间偏移估计失真是根因，已修复并实测
日期：2026-09-24
范围：`tools/calib/livo_calib.py`、`tools/studio/app/calibration.py`、`tools/studio/app/mapping.py`、`tools/studio/web/app.js`、`scripts/run_wuhu_round.sh`、`scripts/analysis/camera_pitch_fit.py`、`config/wuhu_truck29.yaml`；数据 truck05 `/home/project/data/haibo/wuhu/05/20260914/ros2bag`（720 s，1542 m）与 truck29 `/home/project/data/haibo/wuhu_livo/ros2bag`（120 s）。所有轮次均为 60 s 实测（`--rate 1.0`），W1 = 40–100 s（首段行驶），W2 = 598–658 s（第二段起步）。

## 0. 结论速览

| # | 问题 | 根因 | 改动 | 实测证据 |
|---|---|---|---|---|
| 1 | 截图报错：快速模式 `[5/10]` 已知斜率求零点失败（原生尺度 0.9562 偏离 >2 %） | 快速模式沿用 truck29 斜率 0.105 并在偏离时直接失败，docstring 承诺的回退未实现；斜率查找把快速模式记录的“假设斜率”当成实测值 | 单点拟合被拒时自动执行两轮标准扫描；本车没有实测斜率时自动改走标准扫描；`pitch_slope()` 只认标准扫描（≥3 点、无 `slope_source`）的斜率 | 4 种流水线场景逐步模拟通过；truck05 实跑时界面显示“本车尚无实测俯仰斜率，已自动改用标准扫描” |
| 2 | truck05 相机内参错误（最主要根因） | 厂家原始文件没有 `camera_in_params`，导入静默使用 truck29 默认内参 | 由厂家 `equation_*m` 地面距离曲线反演 fx/fy/cx/cy；与 truck29 默认一致（焦距 ≤1.5 %、主点 ≤15 px）时保留默认；`validate` 对借用内参告警；新增 `set --intrinsics-from-supplier` | truck05：fx 1849.4 vs 1642.6（+12.6 %），主点偏 (+32, −32) px；LIVO 原生终点误差 W1 22.95 m → 1.31 m，W2 1.65 m → 0.71 m；truck29 对照反演与 yaml 相差 fx 0.02 %、fy 0.5 %、cx 2.2 px、cy 1.4 px |
| 3 | 标定窗口落在起步前静止段 | 各阶段/轮次窗口都从录制起点算（按 truck29 设计），truck05 前 45 s 静止 | 新增 `windows` 子命令：按 INS 运动剖面选“静止→行驶”过渡，保留 8 s 静止给 IMU 初始化；LIO/LIVO 轮次、ground 静止帧/直行帧、camera-time 起点全部按窗口；truck29 窗口完全不变 | 旧：LIO 轮次只移动 1.15 m 却报 PASS，driven 竖直匹配 1174/889、后雷达未解（quality warn）；新：各雷达 19–30 万匹配、约 10 万竖直匹配，quality ok |
| 4 | 全程建图在 77 s 中止 | 录制本身有 40 ms（77.2 s）、60 ms（429.2 s）IMU 断档，阈值 0.031 s | `imu.maximum_time_gap` 0.031 → 0.1 s（传播按相邻样本均值积分，即线性插值） | 用户 720 s LIO 任务在 77 s 报 “IMU gap interrupted mapping”；新阈值下跨越 77.2 s 的全部轮次正常 |
| 5 | 相机时间偏移估计失真 | 光流/陀螺相关被转弯时相机横移（7.6 m 杆臂 × 偏航率 / 景深）污染：truck05 光流/陀螺增益 2.1、两段分歧 22.6 ms | camera-time 质量门限（分段 ≤12 ms、增益 ≤1.6、相关 ≥0.8），不通过时不覆盖父版本并明确告警 | W2 LIVO 扫描：−35 ms（光流估计）两次发散（10.8 / 19.6 m），+20 / +40 ms 发散（34.7 / 87.7 m），0 ms 两次 1.06 / 1.01 m；truck29 历史估计全部通过门限（增益 1.15–1.25，分歧 3.9–6.6 ms） |
| 6 | 免存图轮次被提前截断 | 排空只看 `frames.csv`，`--no-save-map`（闭环扫描轮次）没有该文件，回放结束 5 s 即停，建图仍滞后 6–12 s | 无 `frames.csv` 时改看 `launch.log` 是否仍在增长 | truck29 同配置：修复前只建到 48.3 s，修复后 57.7 s（与存图轮次一致），融合精度 1.46/3.38/10.35 cm 与历史 1.45/3.41/10.35 cm 相同 |
| 7 | 流水线零点不一致 | camera 流水线快速模式可能复用父版本（非 camlidar 版本）的轮次作零点；camlidar 未改相机时仍重建零点轮次 | 零点只复用同版本、同窗口轮次；camlidar 未改相机时直接用轮次 A | camera 流水线少跑一轮（约 2.5 min） |
| 8 | truck05 原生 LIVO 不可重复 | 起步时洒水车横穿视野（44–47 s）、急弯 16°/s；同一配置重跑可差两个数量级 | 俯仰闭环在 truck05 当前数据上不可用，拟合残差门限会拒绝并说明原因；truck05 保持厂家俯仰 | 同版本同窗口 v013：374 m vs 3.06 m；LIO 同窗口稳定在 28–37 cm |

truck05 当前激活版本已由 `wuhu/truck05/v004_driven` 切换为 `wuhu/truck05/v016_manual`（新 LiDAR 标定 + 反演内参 + 相机偏移 0，`validate --strict` 无告警）。回退：`livo_calib.py activate --id wuhu/truck05/v004_driven`。

## 1. 截图报错与直接修复

- 报错命令：`camera_pitch_fit.py --point calib_truck05_livo_0924_144152_base=0 --slope 0.1050 --max-abs 0.15`，返回 `current native scale 0.9562 is off by more than 2%`。
- 直接原因：快速模式只测零点，斜率取“本车上一次标准扫描”；truck05 从未做过标准扫描，`pitch_slope()` 回落到 truck29 默认值。v007_camloop 用快速模式写入的 `slope_per_deg=0.105`（`slope_source: model/previous run`，1 个点）又会被下一次当成“实测”。
- 修复（`tools/studio/app/calibration.py`）：
  - 快速拟合加 `--exit-zero`，被拒时 `scan_needed` 置位，两轮 ±0.05° 扫描和三点拟合自动执行；通过时这些步骤显示“已跳过”。
  - 本车无实测斜率时直接走标准扫描，任务元数据写入 `mode_note`。
  - `pitch_slope()` 忽略带 `slope_source` 或点数 <3 的拟合。
  - 闭环判定的尺度裕量随实际路径取值（走了扫描用 0.002，纯快速用 0.0005）。
- 但 truck05 的尺度偏离本身来自下文第 2、3、5 节：内参错误、窗口静止、时间偏移错误；只修回退并不能让闭环收敛。

## 2. 相机内参：由厂家地面距离曲线反演

### 2.1 证据

- truck05 与 truck29 的厂家原始 JSON 均无 `camera_in_params`，只有 `tf_lidar_to_camera` 与 `equation_10m … equation_100m`：每条是半径 d 的地面圆在图像中的行号关于列号的 5 次多项式（尾数/指数对）。
- 这些曲线由厂家内参 + 厂家外参 + 地平面画出，因此固定厂家外参即可反解内参。模型：后轴系地面圆（圆心 x0, y0，高度 zg）→ 相机投影，未知数 fx、fy、cx、cy、x0、y0、zg，10 条曲线 × 48 列。
- truck29 对照：自由拟合 fx 1642.2 / fy 1846.3 / cx 968.8 / cy 532.8（rms 0.07 px），yaml 为 1642.57 / 1855.53 / 971.01 / 534.26；地平面 zg −1.26 m，圆心落在相机正下方 (0.01, 7.53) m，方法成立。
- truck05：方形像素模型与自由模型同样好（均 0.05 px），得 fx = fy = 1849.37、cx 1003.31、cy 502.72，zg −1.249 m（与 truck29 同一约定）。LiDAR 实测地面在后轴系约 −1.19 m，一致。

| 对比（truck05，v004 LiDAR 标定） | 窗口 | 原生 中位/P95/终点 | 原生偏航 P95 | 融合 中位/P95/最大 |
|---|---|---|---|---|
| LIO | W1（38–98 s） | 17.5 / 26.7 / 27.4 cm | 0.31° | 4.1 / 8.3 / 12.1 cm |
| LIVO，truck29 默认内参 | W1 | 2.55 / 23.7 / 22.95 m | 4.45° | 3.5 / 8.0 / 11.9 cm |
| LIVO，反演内参 | W1 | 1.57 / 2.14 / 1.31 m | 1.25° | 3.3 / 7.5 / 12.3 cm |
| LIO | W2（598–660 s） | 11.9 / 37.5 / 32.9 cm | 0.33° | 2.6 / 7.1 / 12.2 cm |
| LIVO，truck29 默认内参 | W2 | 1.13 / 1.98 / 1.65 m | 0.98° | 2.8 / 6.6 / 9.9 cm |
| LIVO，反演内参 | W2 | 23.0 / 71.6 / 71.5 cm | 0.27° | 3.4 / 6.3 / 8.4 cm |

内参修正使原生 LIVO 终点误差降低 2.3–17 倍；融合（RTK）精度不受相机影响，均在 3–4 cm 中位。

### 2.2 改动（`tools/calib/livo_calib.py`）

- `ground_equations()` 解析曲线；`intrinsics_from_ground_equations()` 做自由与方形像素两种拟合，方形像素残差不差于自由模型 10 % 时采用方形像素（fy 与 zg 弱相关），并检查 rms ≤1 px、焦距、主点、相机离地高度的合理性。
- `supplier_camera_model()`：反演值与 truck29 默认一致（焦距 ≤1.5 %、主点 ≤15 px）时保留 yaml 值并注明“已被厂家曲线确认”（truck29raw 导入结果不变）；否则采用反演值，导入说明写明偏差。结果与各项拟合记录在 `camera.intrinsics_check`。
- `validate`：非 truck29 车辆使用未确认的默认内参时告警。
- `set --intrinsics-from-supplier`：对已有版本链，从链首导入版本的 `source.json` 与厂家外参重新反演。

## 3. 运动感知的标定窗口

### 3.1 问题

truck05 运动剖面（INS 1 Hz）：0–45 s 静止，47 s 起步，50–130 s 连续急弯，420–550 s、580–610 s 静止。旧流水线的窗口全部按 truck29（开局即行驶）写死：

| 阶段 | 旧窗口 | truck05 实际 | 后果 |
|---|---|---|---|
| LIO 轮次（driven 输入） | 0–48 s | 仅移动 1.15 m | 原生 2.7/3.5/3.5 cm 的假 PASS |
| driven | 轮次内 8–40 s | 静止 | 竖直匹配 1174/889，后雷达“insufficient matches” |
| ground 后雷达运动帧 | 帧 300–340（30–34 s） | 静止 | 后雷达修正 (−1.93°, −0.04°)、z −13.9 cm，残差 8.70→8.55 cm 基本不降 |
| camera-time | 0–60 s | 仅 30–60 s 一段可用，且含洒水车横穿 | +17.3 ms（单段） |
| 相机闭环各轮 | 0–60 s | 行驶约 13 s | 尺度 0.956–1.004 随机跳动 |

### 3.2 改动

- `livo_calib.py windows --bag … [--round-duration 60]`：输出 JSON，并缓存到 `results/.studio/motion/`。
  - 选取行驶距离与转角最多的“静止 ≥4 s → 行驶”过渡，`round_start = 起步 − 8 s`；起步早于约 13 s 的录制保持 0（truck29 起步 5 s → 0）。
  - ground 静止帧取起步前静止段。运动帧默认 30–34 s；该段不是直行（≥1.5 m/s 且偏航率 ≤2°/s）时，改用起步后最直的 4 s。
  - camera-time 起点为起步前 2 s。
  - 同时列出轮次内的 IMU 断档。
- 流水线（`tools/studio/app/calibration.py`）第一步运行 `windows`，其后 ground、LIO 轮次、camera-time、LIVO 轮次 A/零点/扫描/验证全部使用该窗口；操作员显式 `start_offset` 优先。`mapping.pipeline_round_cmd` 支持 `start_offset`。
- camera-time 新增 `--start`（默认按运动剖面跳过静止段）。截取的码流从关键帧开始，保证帧序号与包序号一一对应（实测 1801 包 / 1801 帧）。

### 3.3 结果（truck05 重新导入 → LiDAR 流水线）

- 窗口：round_start 40 s，起步 48 s，ground 运动帧 1840–1880（184–188 s 直行，最大偏航率 0.22°/s）。
- 后雷达 ground：急弯帧（58–62 s）残差 15.9 → 11.9 cm；直行帧残差 13.1 → 6.1 cm。第一次流水线按急弯帧运行，driven 因后雷达修正 0.54° > 0.5° 门限失败；改为直行帧后通过。
- driven（v012）：前右 / 前左 / 后雷达匹配 28.8 / 30.2 / 19.1 万，竖直匹配 10.1 / 9.1 / 10.5 万，残差 4.32→4.12 / 5.01→4.17 / 4.44→3.62 cm，quality ok。
- LIO 精度与旧 v004 同窗口相当（W2：原生中位/P95/终点 10.2/39.2/37.4 cm vs 11.9/37.5/32.9 cm，俯仰 P95 0.35° vs 0.53°），无退化；旧版本的“精度”是在静止数据上得到的。

## 4. IMU 断档容限

- truck05 录制中 IMU（100 Hz）有 68 次单样本丢失，另有 40 ms @77.2 s、60 ms @429.2 s、30 ms @586.2 s 断档；消息头与接收时间戳一致，是录制本身的缺陷。truck29 数据无任何 >15 ms 断档。
- `imu.maximum_time_gap: 0.031` 导致用户的 720 s LIO 任务 `1a0d2408da9` 在 77 s 报 `IMU stream has a 0.040000 s gap`。
- IMU 传播与去畸变用相邻两样本的均值积分（`IMU_Processing.cpp` 中的 `angvel_avr`/`acc_avr`），跨越断档等价于线性插值。阈值改为 0.1 s，可桥接最多 9 个丢失样本，更长的中断仍终止。

## 5. 相机时间偏移：光流估计在 truck05 上失真

- camera-time 在行驶窗口（46–106 s）得到 −35.4 ms，两段分别为 −51.7 ms 与 −29.1 ms；旧窗口（0–60 s）得到 +17.3 ms。
- LIVO 实测（W2，v012 LiDAR + 反演内参，只改相机偏移）：

| 相机偏移 | 原生终点误差 | 说明 |
|---|---|---|
| −35 ms（光流估计） | 10.8 m / 19.6 m | 两次均在 90° 急弯处发散 |
| 0 ms | 1.06 m / 1.01 m | 可重复 |
| +20 ms | 34.7 m | 发散 |
| +40 ms | 87.7 m | 发散 |

- 原因：光流/陀螺增益 2.1（truck29 为 1.15–1.25）。相机在后轴前 7.6 m，转弯时横移速度 = 偏航率 × 7.6 m，近处坡面（8–15 m）的平移光流与旋转同相叠加，但权重随景深变化，使相关滞后偏移。只取地平线附近特征时增益仍为 2.0、滞后 −31…−39 ms。本质矩阵分离旋转的方案在窄视场、小基线下病态（增益 5.3），已放弃。
- 改动：`camera_time_quality()` 门限为分段分歧 ≤12 ms、增益 ≤1.6、相关 ≥0.8。不通过时版本仍生成，但记录 `estimated_offset_s`、`applied: false` 与原因，并保留父版本偏移；阶段输出 `quality: warn … NOT applied`。truck29 的全部历史估计通过门限，行为不变。
- `set --camera-offset` 同时写入 `camera_source`，经实测确定的 0 偏移不再被 `validate` 当成“未验证”。

## 6. 免存图轮次的排空修复

- `run_wuhu_round.sh` 的排空只统计 `frames.csv` 行数；`--no-save-map` 轮次没有该文件，回放结束 5 s 就发 SIGINT，而 LIVO 建图此时仍滞后 6–12 s。
- 影响：闭环扫描轮次（`save_map=False`）与零点轮次（存图）覆盖的时间段不同，俯仰拟合比较的是不同长度路径的原生尺度。这是闭环噪声的来源之一（truck29 同样受影响）。
- 修复：无 `frames.csv` 时监视 `launch.log` 大小（建图每帧输出计时表），停止增长 5 s 才结束。truck29 v061 同配置 60 s：修复前 48.3 s / 1449 帧，修复后 57.7 s / 1732 帧，与存图轮次完全一致。

## 7. truck05 原生 LIVO 的可重复性与俯仰闭环

- W1 起步时（44–47 s）一辆洒水车从右到左横穿相机视野，占据约 40 % 画面；本车 46.8 s 起步，LIVO 在起步 3.5 s 内沿行驶方向少估 1.7 m（LIO 同期误差 3 cm）。
- 同一版本、同一窗口重跑差异巨大：v013 在 W1 为 374 m vs 3.06 m；起点平移 2 s，v004+反演内参由 1.31 m 变为 6.66 m。W2 起步干净，但 90° 急弯对相机偏移极敏感（见第 5 节）。
- 结论：truck05 当前数据上原生尺度不可重复，俯仰闭环（前提是约 0.03 % 的重复性，来自 truck29）无法分辨 0.05° 级俯仰。标准扫描的拟合残差门限（>0.004）会拒绝，拒绝信息已改为说明可能原因与处置。truck05 保持厂家俯仰。
- camera-lidar（灯柱偏航）在 truck05 首段行驶中找不到灯柱（17 帧），未修改相机。

## 8. truck05 最终版本与推荐用法

- `wuhu/truck05/v016_manual`（已激活）。版本链：v008_import（反演内参）→ v009_axle（LiDAR 时间偏移 −0.0368 s）→ v011_ground（直行帧）→ v012_driven（quality ok）→ v016_manual（相机偏移 0，来源写明 LIVO 扫描依据）。
- 中间版本：
  - v010_ground：急弯帧，被 v011 取代。
  - v013_camtime：光流偏移 −35 ms，门限上线前生成，勿用。
  - v014_camlidar：无灯柱，与 v013 相同。
  - v015_scan：中止的扫描点。
  - 以上可按需删除（`livo_calib.py delete`，本次未执行删除）。
- truck05 建图：
  - RTK 良好时 LIO 与 LIVO 融合精度相同（中位 3–4 cm）。
  - RTK 退化路段原生 LIVO 仍比 LIO 差（W2 1.0 m vs 0.37 m），LIO 更稳。
  - 若用 LIVO，务必使用 v016（旧 v004 内参错误）。

## 9. truck29 回归

- 窗口、导入内参、camera-time 门限、ground 帧对 truck29 全部是空操作（见第 2、3、5 节的对照输出）。
- v061_camloop 60 s LIVO（新 yaml + 新排空）：融合 1.46 / 3.38 / 10.35 cm，原生终点 1.43 m，57.7 s / 1732 帧。历史同配置 `calib_truck29_livo_0924_134213_verify` 为 1.45 / 3.41 / 10.35 cm、1.44 m、57.7 s / 1732 帧，结果一致。

## 10. 遗留问题

- LIVO 在急弯与大动态目标下的稳健性属 VIO 核心问题（相机偏移 ±20 ms 即发散），不属标定范围，建议另立专题：急弯时降低视觉权重、动态目标剔除、按增益/残差门控视觉更新。
- 相机时间偏移需要不受平移光流污染的估计方法，例如用 LiDAR 深度扣除平移光流，或用 LIO 位姿做重投影时间对齐。在此之前，门限阻止错误估计写入。
- axle 阶段读取全程 720 s 约 7 min，报告“24 次迭代未收敛”；可考虑限制为起步后约 300 s 的行驶数据并提高迭代上限。
- 本次实验轮次（`results/mcp_t05w*`、`results/mcp_t29_livo_v061_reg*`，每个约 45 MB）与流水线轮次（`calib_truck05_lio_0924_155726`、`calib_truck05_lio_0924_161043`、`calib_truck05_livo_0924_161609`，约 0.75–1.1 GB）保留供查阅，可按需清理。
