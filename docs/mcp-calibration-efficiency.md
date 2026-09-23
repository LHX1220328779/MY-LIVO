# 标定效率优化：短数据段标定的可行性验证与实现

日期：2026-09-23 分支：`fusion/56c6896-ins-attitude`（基线 commit f589b7c，精度体系 v032_camloop）
数据：芜湖 29 号车原始录制（8 段 120 s，4×LiDAR + IMU/INS + 相机）、池州 10 号车原始录制（15 段 450 s，纯 LIO，泛化验证）

## 0. 结论速览

| 问题 | 结论 | 证据 |
| --- | --- | --- |
| 是否只需最开始的一个包（约 30–60 s）就能标定到同等效果 | **是（60 s 内）**。LiDAR ground+driven 链在前 60 s（前 2 段）产出的外参与全量 120 s 结果差 ≤0.06°/1.7 cm，SLAM 全量闭环 budget PASS、各项指标与生产标定持平；单独 30 s（1 段）ground/camtime 已达标，driven 的 back_lidar 略差（0.13°/2.8 cm），因此**推荐 2 段 ≈60 s 作为下限**，30 s 仅够 ground/camtime | §2、§3 全表 |
| 是否必须像 SLAM 一样从静止开始 | **ground 阶段是，其余阶段否**。静止起步段是 ground 阶段（roll/pitch/z）的观测前提：把"静止帧"换成行驶帧后残差中位 4.2→9.4/13.6 cm、修正方向错误（front_left 转角 -0.26° vs 正确 -0.055°），不可用。driven / camera-time / camloop 本来就用行驶数据，不依赖静止起步；SLAM 侧 IMU 初始化（`MAX_INI_COUNT`）与 extrinsic_precheck 也需要起步静止段。矿区作业车起步前天然静止（芜湖 4 s、池州 24 s），此约束在实际运营中无成本 | §4 |
| 效率提升如何落地 | 三处最小改动（不动算法、不动标定求解本身）：① LiDAR 流水线的 LIO 建图轮次默认只放前 48 s（driven 窗口 8–40 s + 排空余量）；② camera-time 阶段新增 `--duration`，流水线默认 60 s（只解码前 60 s 视频）；③ 相机 camloop 已有 fast 模式（前次会话完成），本次不动 | §5 |
| 提速幅度 | LiDAR 流水线 ground→LIO→driven 实测 **140 s**（原 ≈208 s，其中 LIO 轮次 143→65 s）；camera-time 实测 **9.4 s**（原 ≈120 s，13×）。相机全流水线（标准模式）由 ≈15 min 降至 ≈11 min，快速模式 3.5 min 不变 | §5 实测 |
| SLAM 精度是否保持 | **保持**。三组全量 120 s LIVO 闭环（短包 LiDAR 外参、60 s camtime 偏移、新流程 v034_driven 产物）全部 budget PASS：优化后轴 100% ≤10 cm，max 7.0–8.4 cm（基线 7.0 cm，逐轮噪声 ±1 cm 量级）；地面厚度、灯柱一致性、起步 30 m 均持平或更好 | §3 表 |

## 1. 现状与耗时结构（优化前）

流水线（Studio v2 / `livo_calib.py`）各阶段对数据的真实用量与耗时：

| 阶段 | 实际用的数据 | 耗时主体 | 优化前行为 |
| --- | --- | --- | --- |
| ground | 静止帧 2..22（≈2 s）+ 行驶帧 300..340（t≈30–34 s） | 读包（顺序解包到帧 340） | 已经只读前 ~35 s，无需优化 |
| LIO 建图轮次（为 driven 提供位姿） | driven 窗口只用 8–40 s 的 frames.csv | **实时回放整段 120 s + 排空** | 全长回放是最大浪费 |
| driven | 窗口 8–40 s 的原始扫描 + 该窗口的 LIO 位姿 | 读包+求解（窗口后 1 s 提前 break） | 已窗口化，无需优化 |
| camera-time | 光流 vs 陀螺互相关 | **解码整段 1080p HEVC（~2 min）** | 全段解码是第二浪费 |
| camera-lidar（poles 偏航） | 30 个图像/扫描对（窗口 5–115 s） | 选帧解码，~1 min | 已按帧稀疏化，无需优化 |
| camloop（俯仰闭环） | ≥100 s 的 RTK 里程尺度观测 | 2–4 轮 LIVO 建图 | **物理上不可缩短**（尺度误差需长里程积累，11%/°），fast 模式已把 15–20 min 降到 3.5 min |

关键认识：**标定求解阶段早已只消费开头一小段数据；"跑完全部数据包"的时间几乎全部花在两处——给 driven 供位姿的 LIO 建图轮次全长回放、camera-time 全段视频解码。**

## 2. 短数据标定 vs 全量标定：外参一致性实测

工况说明：芜湖包起步静止仅 ~4 s（t=4 s 起步，v=0.7 m/s），前 30 s 已含起步+直行；池州包起步静止 24 s。

### 2.1 LiDAR 外参（相对生产 v013_driven 的差异）

| 数据量 | front_right | back | front_left | front(参考) |
| --- | --- | --- | --- | --- |
| 1 段=30 s（driven 窗口 8–28 s） | 0.036° / 2.8 cm | 0.133° / 2.8 cm | 0.026° / 0.5 cm | 0 |
| 2 段=60 s（driven 窗口 8–40 s，与生产同参） | **0.020° / 0.9 cm** | **0.057° / 1.7 cm** | **0.019° / 0.8 cm** | 0 |
| 新流程集成测试 v034_driven（ground+48 s LIO+driven） | 0.026° / 1.2 cm | 0.052° / 1.5 cm | 0.019° / 1.2 cm | 0 |

参照：run106/107 重复实验表明同配置重跑本身有 0.03 %/2–3 cm 级别的轮次噪声；60 s 结果已进入该噪声带。ground 阶段单独看：30 s 与 120 s 的静止段修正逐位相同（用的本来就是前 22 帧）。

### 2.2 相机时间偏移（光流 vs 陀螺）

| 数据量 | yaw 通道 lag | corr | 与生产 -0.115 差 |
| --- | --- | --- | --- |
| 30 s | -0.1152 s | 0.93 | 0.2 ms |
| 60 s | -0.1211 s | 0.92 | 6 ms |
| 90 s | -0.1181 s | 0.91 | 3 ms |
| 120 s（生产） | -0.1150 s | 0.92 | — |

历史分段测量本身离散 6.5 ms（-111～-118 ms），30–90 s 的结果全部落在该本底内。60 s 点(-0.1211)恰为历史包络的边缘值，故用它做了最严格的 SLAM 闭环验证（§3 第 3 行），通过。

### 2.3 泛化性（池州 10 号车，不同矿区/车辆/静止时长）

ground 阶段 60 s（2 段）vs 150 s（5 段）：front_right/front_left 修正**逐位相同**（数据充分性在 60 s 内已饱和）；back_lidar 行驶段窗口不同（帧 300 vs 900）时修正差 0.08°/2.6 cm，与芜湖同量级。侧雷达在 60 s 即可解出 yaw/xy（静止 24 s 提供了更多墙面重叠）。

## 3. SLAM 精度闭环验证（全量 120 s LIVO，与生产基线同二进制同配置）

| 指标 | 基线 v032（全量标定） | 短包 LiDAR 外参(60 s) | camtime -0.1211(60 s) | 新流程 v034_driven |
| --- | --- | --- | --- | --- |
| 优化后轴 xy 中位 | 1.17 cm | 1.21 cm | 1.27 cm | 1.25 cm |
| 优化后轴 xy P95 | 3.29 cm | 3.26 cm | 3.16 cm | 3.39 cm |
| 优化后轴 xy max | 6.98 cm | 7.09 cm | 8.13 cm | 8.24 cm |
| 路线 ≤10 cm | 100 % | 100 % | 100 % | 100 % |
| budget | PASS | PASS | PASS | PASS |
| 地面 5–20 m 厚度中位 | 29.6 mm | 31.2 mm | 29.1 mm | 29.9 mm |
| 灯柱 0v10 等高 XY 中位 | 23.0 cm | 24.1 cm | 19.1 cm | 22.0 cm |
| 起步 30 m fused 中位 | 2.25 cm | 2.34 cm | — | 2.29 cm |
| 原生路径尺度 | 1.0001 | 1.0008 | 1.0003 | 0.9998 |

（四组轮次目录：`results/mcpeff_livo_shortcal`、`results/mcpeff_livo_shortcal60`、`results/mcpeff_livo_cam121`、`results/mcpeff_livo_v034`。）

判定：优化后轨迹（交付轨迹/地图所依据）各项指标全部在轮次噪声内，无一超出生产基线的历史带宽（run101/103/105–111 的优化 max 为 7.6–9.6 cm）。短包标定**可以采用**。

注意一个不影响判定的现象：30 s 短包 LiDAR 外参那一轮（shortcal，非 60 s 轮）原生前端中位 1.51 m 明显差于基线 0.39 m——原生红线对雷达间 0.1° 级差异敏感（与 §1.4 相机外参敏感性同机制）；60 s 外参把该差异收回噪声带（原生 0.32 m，反而略优）。这是把推荐下限定为 60 s 而不是 30 s 的直接原因。

## 4. 是否必须从静止开始

- **ground 阶段必须**。它的观测模型是"同一时刻各雷达看同一块地面"，静止聚合 20 帧以消噪。用行驶帧（t≈15 s，v≈2.3 m/s）冒充静止帧实测：聚合点云被运动模糊污染，front_right 残差中位 4.2→9.6 cm，front_left 6.3→13.8 cm 且解出错误的 -0.26° 转角修正，back_lidar 修正 -0.40°/+10.9 cm 与正确值(-0.19°/+9.1 cm)偏离。**结论：不能省略静止段；但只需要 ~2 s（帧 2..22），任何"上电即停在原地"的车都天然满足。**
- **driven / camera-time / camera-lidar / camloop 不需要静止**，它们本来就依赖运动激励（driven 需要行驶窗口、camtime 需要转向段光流、camloop 需要 ≥100 s 里程）。
- **SLAM 本身**：IMU 初始化（重力/零偏，`MAX_INI_COUNT`）与算法内 extrinsic_precheck（陀螺 <0.5°/s 的前 5–12 帧）都在起步静止段完成，这是算法既有设计，本次未改动。
- 若某车录制起步时已在行驶：ground 阶段会因静止判定失败而给出高残差解，此时应换包，而不是放宽静止门限（实测证明放宽即出错）。

## 5. 实现（最小改动，未动标定求解与 SLAM 算法）

1. `tools/studio/app/calibration.py`
   - LiDAR 流水线的 LIO 建图轮次默认 `--duration 48`（driven 窗口上限 40 s + 8 s 排空余量）；`body.lidar_round_duration` 可覆盖，传 0 恢复全长。
   - 相机流水线 camera-time 步骤默认传 `--duration 60`；`body.camtime_duration` 可覆盖。
   - `stage_cmd` 支持给 camera-time 传 `--duration`。
2. `tools/calib/livo_calib.py`
   - `camera-time` 子命令新增 `--duration`（从首帧起截断视频流），`dump_camera_stream`/`estimate_camera_offset` 透传；meta.json 的 inputs 记录 duration。
   - 默认（命令行手动调用不带 --duration 时）仍处理全段，行为完全向后兼容。
3. 未改动：ground/driven/camera-lidar/camloop 的求解逻辑、窗口默认值、门限、`run_wuhu_round.sh`、SLAM 源码、评估脚本。

实测提速（本机，芜湖 120 s 包）：

| 链路 | 优化前 | 优化后 | 备注 |
| --- | --- | --- | --- |
| LiDAR 流水线核心三步 | ≈208 s（11+143+54） | **140 s**（11+65+64） | 集成测试 v033_ground→mcpeff_it_lio48→v034_driven |
| camera-time 阶段 | ≈120 s | **9.4 s** | 60 s 视频 + ffmpeg 解码 |
| 相机流水线（标准） | ≈15 min | ≈11 min | camtime 提速 + LIVO A 轮次不变（camloop 需全长，不可缩） |
| 相机流水线（fast） | 3.5 min | 3.5 min | 前次会话已优化，本次未动 |

对更长的运营录制（如池州 450 s），收益按比例放大：LIO 轮次从 ≈8 min 封顶到 ≈1 min。

## 6. 边界与守则

- 推荐数据下限：**含起步静止段的前 2 个分段（≈60 s），且行驶部分包含一次转向**。1 段 30 s 只可用于 ground/camtime 应急，driven 的 back_lidar 精度会退化到 0.13°/3 cm。
- camloop（相机俯仰）不能用短数据：尺度观测物理上需要 ≥100 s 里程，这由闭环判定门限自动守住，无需人工注意。
- 新车 SOP 不变：仍按 ground→LIO→driven→camtime→LIVO→camera-lidar→camloop 走，只是 LIO 轮次与 camtime 自动变快。
- 任何新产出版本仍需经流水线的 validate/闭环判定才激活，本次改动没有绕过任何验收门。

## 7. 文件与轮次索引

- 代码：`tools/studio/app/calibration.py`（流水线 duration 默认值）、`tools/calib/livo_calib.py`（camera-time --duration）。
- 验证轮次：`results/mcpeff_livo_shortcal`（30 s LiDAR 外参）、`results/mcpeff_livo_shortcal60`（60 s LiDAR 外参）、`results/mcpeff_livo_cam121`（60 s camtime 偏移）、`results/mcpeff_livo_v034`（新流程端到端产物）、`results/mcpeff_it_lio48`（48 s LIO 供 driven）。
- 标定版本：`v033_ground`、`v034_driven`（新流程集成测试产物，未激活；激活版本保持 v032_camloop）。
- 探针（不入库）：`results/.cache/mcp_probe/{speed_probe.py, camtime_seg1.py, cmp_calib.py, assemble_shortcalib.py, seg1/, seg12/, cz2seg/, cz5seg/}`。
