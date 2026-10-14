# MCP 专题：truck29raw 2026-10-13 数据标定中断 — 左前雷达安装位姿已变，ground 阶段增加平面粗对齐

日期：2026-10-14 · 分支 `fusion/56c6896-ins-attitude`（基线 `b5d4ed6`）
数据：`/home/project/data/haibo/wuhu/29/20261013/ros2bag`（870 s，58 段，29.6 GB；手动窗口 0–120 s）
父版本：`wuhu/truck29raw/v001_import`（厂家原始文件，2026-08-05 数据上曾一键标到 v014_manual）
失败任务：Studio `1a13967e2f5`（完整流水线）与 `1a139748bf1`（续跑），均止于 `行驶段 6-DoF 精修 (driven) 退出码 3`
修复后任务：Studio `1a139923a49`，16/16 步完成，`v019_gcons` 已激活
探针脚本与原始输出：`results/.probe/`（`probe_ground_tilt.py`、`ground_test*/`、`reg_old*/`，不入库）

## 0. 结论速览

| 问题 | 结论 |
| --- | --- |
| 是数据问题还是算法问题 | **两者都有，主因在数据侧**：`front_left_lidar` 的实际安装位姿相对 `v001_import` 已经变化（俯仰约 6.0°、横滚约 3°，雷达处高差约 4 cm、远处地面高差 30–50 cm）。8 月 5 日数据上同一文件只差 1.3–1.7°。应是传感器被碰动/重装（或固件 hotfix08 改了输出坐标系），标定文件对这一路已经过期 |
| 算法侧缺陷 | ground 阶段是“小修正”求解器（匹配半径 0.6 m、先验 1°/10 cm），6° 的初值在收敛域之外：它把左前雷达越修越差（地面残差 15.5 → 20.3 cm）**仍然落盘**；driven 阶段的质量门（0.5°/15 cm）随后正确拒绝了 2.4°/41 cm 的后续修正，于是流水线死在中途 |
| 修复 | `scripts/refine_multi_lidar_extrinsics.py`：每路雷达先用**自身 RANSAC 地面平面**与参考雷达平面比对（不依赖点匹配）；超出精修尺度（>0.5° 或 >5 cm）且确实能降低起始残差时，用平面闭式对齐（绕雷达自身位置的最小旋转 + 高差）作为求解起点，先验以该起点为中心；精修结果若比起点更差则不落盘，保留更优矩阵；每路输出 ok/warn 质量结论并写入 calibration.json / meta.json。精修器本身未改：在收敛域内的标定**逐位一致**（旧数据 `--no-coarse` 与历史 v003_ground 矩阵最大差 0.0） |
| 精度 | 同一 120 s LIO 轮次：优化轨迹 vs RTK P95 2.98 → 2.49 cm，姿态差 P95 0.59 → 0.31°，地图多雷达地面平面偏移 111 → 29 mm，起步自检倾角修正 0.31 → 0.08°；driven 阶段左前雷达残差 15.6 → 4.7 cm（与右前 3.7 cm 同级），修正量 0.1°/3 cm，质量 ok |

## 1. 现象

`1a13967e2f5` 的步骤：数据评估 ✓ → 手动窗口 ✓ → axle（v015，quality warn：时间偏移 +89 ms 变化大，候选曲线确认）✓ → ground（v016）✓ → LIO 轮次 ✓ → **driven ✗**：

```
front_left_lidar: pass 1 matches 122582->179612 |res| median 16.29->15.55 cm; ground 21.01->21.50; vertical 10.82->8.49;
    correction rot deg (x,y,z)=(-2.251,+0.844,+0.210) trans cm=(+8.6,+21.5,-34.0); iters 25
    [quality: fail; rotation correction 2.41 deg > 0.5; translation correction 41.1 cm > 15; not converged in 25 iterations]
back_lidar: ... correction rot deg (x,y,z)=(+0.385,-0.495,-0.088) ... [quality: fail; rotation correction 0.63 deg > 0.5]
driven refinement rejected by the quality gate; calibration.json not written
```

v016_ground 自己的日志已经给出征兆（当时没有任何门禁）：

```
front_left_lidar: yaw/xy solved=False; matches 3004->3246; |res| median 15.53->19.46 cm; ground 15.47->20.29 cm; iters 25
```

数据评估也早就标红：`静止段雷达间地面倾角差 6.45°（front_left_lidar：前后向 +6.02° 左右向 +2.30°）`、`高度差 85.9 cm`，提示“需运行离线标定阶段”——但离线阶段（ground）当时做不到。

## 2. 根因：逐路雷达独立地面平面（不经过匹配）

`results/.probe/probe_ground_tilt.py`：每路雷达用 `v001_import` 变换到后轴系后，各自 RANSAC 拟合地面平面，比较与参考雷达（front_lidar）的倾角差/高差。静止段 2–22 帧与行驶段 810–850 帧（约 80 s 后、另一地点）分别统计。

| 数据 | 波段 | front_right ΔFA / ΔLat | **front_left ΔFA / ΔLat** | back ΔFA / ΔLat |
| --- | --- | --- | --- | --- |
| 2026-08-05（旧） | 静止 15–30 m | −0.10° / −0.14° | **+0.40° / +1.34°** | — |
| 2026-08-05（旧） | 行驶 15–30 m | −0.13° / −0.03° | **+0.72° / −1.81°** | +1.88° / −2.32° |
| 2026-10-13（新） | 静止 3–15 m | +0.69° / −0.23° | **−5.32° / −3.10°** | — |
| 2026-10-13（新） | 静止 15–30 m | −0.24° / −0.23° | **−6.40° / −2.90°** | — |
| 2026-10-13（新） | 静止 30–50 m | +0.00° / −0.13° | **−5.98° / −2.55°** | — |
| 2026-10-13（新） | 行驶 15–30 m | −0.22° / −0.13° | **−6.31° / −4.03°** | +0.05° / −0.14° |

- 偏差在所有距离波段、两个不同地点上一致 → 不是场景（坡/路肩）效应，是外参错误。
- 只有 front_left 变了，front_right / back 与参考一致 → 不是参考雷达或整机问题。
- 同一文件在 8 月数据上只差 1.3–1.7°（ground 阶段当时能收敛到 5.1 cm）→ 传感器在两次采集之间被动过。
- 数据本身（点数、丢帧、同步、RTK）均正常：这是“标定文件过期”，而不是“录制坏了”。建议现场复查左前雷达支架；若是固件 hotfix08 改了点云坐标系，需在车队层面核对。

ground 阶段为何失败：匹配半径 0.6 m 下，6° 倾角在 15 m 外地面已偏 1.5 m，只剩近处少量错配；1° 先验把解拉向零；25 次迭代走到更差的局部极小。driven 阶段的门禁是对的——2.4° 不是“精修”，它不该在 driven 阶段被吞掉。

## 3. 修复（`scripts/refine_multi_lidar_extrinsics.py`，`tools/calib/livo_calib.py::cmd_ground`）

1. `fit_ground_plane()`：RANSAC（阈值 8 cm，z<1 m，<40 m）+ 3 次内点最小二乘重拟合，取主导近水平平面。
2. `plane_init()`：对每一对（本路，参考）平面，求绕雷达自身位置把本路法向转到参考法向的最小旋转，以及让雷达离地高度一致的 z 平移；多帧取中位（后向雷达对行驶帧逐帧统计，参考为已精修的两侧雷达）。
3. `refine_one()`：
   - 平面差 >0.5° 或 >5 cm（`--coarse-threshold-deg/-z`，即精修器的先验尺度）才视为“在收敛域外”；
   - 平面起点只有在**确实降低起始地面残差**时才使用（后向雷达看的是车后地面，与两侧地面不共面：本例起点 10.7 → 16.2 cm，被拒绝，保持原路径）；
   - `solve(x0=…)`：先验以起点为中心（历史路径 x0=0，逐位一致）；
   - 从不落盘更差的结果：精修若比“手动矩阵 / 平面起点”都差，保留更优者并 warn；
   - 收敛判断改为看最后一步步长（>0.006° 或 >1 mm 才算未收敛；旧的 1e-6 rad 停止条件常因边界点重匹配进入极限环，25 次迭代并不代表发散）；
   - 每路记录 `quality{level, reasons, ground_before/after_cm, rotation_deg, translation_cm}` 与 `plane_check{tilt, fore_aft, lateral, dz, outside_basin, applied}`，整体 `refinement.quality` ok/warn，报告末尾一行 `quality: …`。
4. `cmd_ground` 把 `quality` 写入版本 metrics（meta.json / report.md / Studio 卡片）。
5. `--no-coarse` 保留历史行为用于回归对照。

未改动：精修器方程、匹配、Huber、迭代上限、driven/gcons 阶段及其门禁、数据评估阈值（其提示“>1° 需 ground/driven 阶段重标；>3° 多为安装变动”现在是真的）。

## 4. 验证（全部在 `calibration` 容器内真实运行）

### 4.1 ground 阶段单独运行（父 v015_axle，新数据）

| 雷达 | 旧代码（v016_ground） | 新代码（v017_ground） |
| --- | --- | --- |
| front_right | 地面 9.60 → 6.68 cm，(−0.119, +0.296, +0.105)°，iters 16 | **逐位相同** |
| front_left | 地面 15.47 → **20.29** cm，(−0.358, +0.750, 0)° z −6.4 cm，iters 25 | 平面起点 6.75°（前后向 −6.23°，横向 −2.60°）使用；地面 15.47 → **8.80** cm，竖直 16.03 → 3.60 cm，匹配 3004 → 6411，yaw/xy 变为可解；(−6.287, +2.627, +0.093)° (+1.7, +2.6, −3.1) cm；quality warn（安装位姿变化） |
| back | 地面 13.23 → 11.30 cm（两侧还是错的） | 平面起点被拒绝（不改善）；地面 10.71 → 9.56 cm，iters 21，quality ok |

修复后用探针复核：front_left 静止段三个波段与参考的倾角差 −0.02/−0.16/+0.46°（前后向）、−0.32/−0.14/−0.01°（横向）。

### 4.2 完整流水线 `1a139923a49`（v015_axle → v017_ground → v018_driven → v019_gcons，激活）

driven（v018）：

```
front_right_lidar: pass 1 |res| 3.79->3.71 cm; vertical 3.65->3.26; final (+0.068,-0.037,+0.058) deg (+1.0,+1.8,-1.9) cm [ok]
front_left_lidar:  pass 1 |res| 4.77->4.74 cm; vertical 3.55->3.21; final (+0.105,-0.051,-0.038) deg (+0.7,+1.5,-2.8) cm [ok]
back_lidar:        pass 1 |res| 8.10->5.34 cm; pass 2 ground 3.55->2.89; final (+0.286,+0.022,-0.059) deg (+3.1,+7.9,+1.5) cm [ok]
quality: ok
```

RTK 关联时间：地图自洽修正 −3.2 ms（se 5.5 ms），沿用 +0.025 s。gcons（v019）：后向雷达 +0.217°/−0.040°，z +0.73 cm，平地 |res| 中位 2.99 → 1.32 cm（warn：两半差 0.108°，与 truck29/truck28 历史同级）。

同一 0–120 s 窗口的 LIO 轮次对比（`results/calib_truck29raw_lio_1014_155432` vs `_164046`，`evaluation/metrics.json`）：

| 指标 | v016_ground（失败流水线） | v017_ground（修复后） | rtk1 轮（v018_driven） |
| --- | --- | --- | --- |
| 优化轨迹 vs RTK 中位 / P95 / 最大 (cm) | 1.06 / 2.98 / 7.46 | 0.99 / 2.49 / 7.63 | 0.99 / 2.63 / 7.56 |
| 路段 ≤10 cm | 100 % | 100 % | 100 % |
| 优化姿态差 P95 (°) | 0.59 | 0.31 | 0.28 |
| 地图地面 5–20 m 中位厚度 before/after (mm) | 29.4 / 43.0 | 31.8 / 31.2 | 31.2 / 31.9 |
| 多雷达地面平面偏移 raw/opt (mm) | 71.7 / 111.4 | 46.9 / 29.4 | 28.8 / 23.0 |
| 起步自检倾角修正中位 (°) | 0.305 | 0.084 | 0.204 |
| 跨时间扫描匹配数 | 2801 | 4485 | 4565 |

本次录制路线上没有标准灯柱裁剪区（`pole_skipped`），灯柱指标不可评估；多雷达一致性以地面平面偏移和跨时间匹配数为准。

### 4.3 回归（2026-08-05 旧数据，父 v002_axle，对照历史 v003_ground）

| 路径 | front_right | front_left | back |
| --- | --- | --- | --- |
| 历史 v003_ground | 6.08 → 4.04 cm | 16.60 → 5.15 cm，(+0.109, −1.308)°，iters 25 | 7.88 → 7.60 cm |
| 新代码 `--no-coarse` | **矩阵逐位相同（max |Δ| = 0.0）** | 同上，附 warn“1.31° 超出精修尺度” | 同上 |
| 新代码默认 | 相同（平面差 <0.5°，不触发） | 平面起点 1.71° 使用；16.60 → **5.13** cm，(+0.147, −1.332)° z −2.2 cm，**iters 21** | 起点被拒绝；8.57 → 7.56 cm（两侧略变） |

即：历史上“iters 25”的 1.3° 修正其实没完全收敛；新起点让它在 21 次内收敛到略优的解，差异 0.04°/1.2 cm。

## 5. 遗留与建议

- `wuhu/truck29raw/v016_ground` 是失败流水线留下的死叶子版本（左前雷达更差），未激活；确认不再需要后可在 Studio 删除。
- axle 阶段 quality warn：LiDAR 时间偏移相对父版本变化 +89 ms（−0.125 → −0.036 s），候选曲线单调确认。固件 hotfix08 很可能改了时间戳语义，与左前雷达变化一起，建议把 20261013 之后的数据统一按新 lineage 标定，不再从 8 月的手调版本续接。
- 相机未标（`time_offsets.camera` 为 0，validate 只给 warning）；需要 LIVO 时另跑相机流水线（父 v019_gcons）。
- 数据评估两条“标定无法补救”警示（back_lidar 丢帧 0.25 %、RTK 位置阶跃 1 次 @119.9 s）与本故障无关。
- 建议现场复查左前雷达支架/螺栓；若雷达确实松动，任何标定都只在松动前后各自有效。

## 6. 文件

- `scripts/refine_multi_lidar_extrinsics.py` — 平面粗对齐起点、以起点为中心的先验、改善门控、回退与质量结论、`--no-coarse`
- `tools/calib/livo_calib.py` — ground 版本 metrics 增加 `quality`
- `config/carlibration/wuhu/truck29raw/v017_ground`、`v018_driven`、`v019_gcons`（激活）；`registry.json`
- `results/.probe/probe_ground_tilt.py` — 逐路独立地面平面探针（诊断用）
- Studio 任务日志：`results/.studio/jobs/1a13967e2f5.log`（失败）、`1a139923a49.log`（修复后）
