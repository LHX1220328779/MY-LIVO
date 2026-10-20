# MCP 专题：地图距离优先改为默认开、清理关闭时数据、再次核查优化空间（2026-10-20 夜）

日期：2026-10-20 夜 · 分支 `fusion/56c6896-ins-attitude`（基线 `5f4fdcb`）
数据：`/home/project/data/haibo/wuhu/29/20261013/ros2bag`；标定 truck29raw `v042_camloop`（LIVO）

## 0. 结论速览

| 项 | 结果 |
| --- | --- |
| 距离优先默认开 | yaml `rtk.save_map_priority_enabled: true`，头文件默认值同步；`run_wuhu_round.sh --no-map-priority` 关闭（`--map-priority` 保留）；Studio 建图页“地图距离优先”默认勾选，当前保存的 Studio 状态也已改为开。不带任何开关的新一轮（mcp24）`launch.log` 显示 `range priority: ON` |
| 删除关闭时数据 | 删除 `results/mcp21_livo_rgb`（20 G）、`results/mcp23_lio_60s`（1.2 G）、`results/.mcp/rgbtest/m21`、`m21b`、`m21old`（关闭轮次的导出，3.5 G）及两份启动日志，共 24 G；删除前确认两轮的 `config.yaml` 均为 `save_map_priority_enabled: false`、`launch.log` 均为 `range priority: off`。删除后磁盘可用 190 G |
| 核查 1：标准地图 `global_map.pcd` 灯柱壳宽 7.2 cm，行级 3.2 cm | 不是重影：多出来的主要是灯杆上的标牌、灯臂、树枝。体素图里每 5 cm 一个点，这些附着物和密集的杆身权重一样；杆身本身与行级一样细。不改 |
| 核查 2：体素级去噪（每体素 ≥2 回波 / ≥2 帧） | 不采纳：开距离优先后 58 % 的 5 cm 体素只有 1 个回波、67 % 只来自 1 帧，规则会删掉 6–60 % 的 0.2 m 占据格 |
| 核查 3：彩色着色比例只有 38 % | 采纳：原来的 6 cm 体素兜底会漏掉隔着体素边界 1 mm 的观测。改为“本 3 cm 体素，否则 6 cm 以内最近的观测”：rgb_map_N 着色 38.3 → 59.5 %，global_map_rgb 9.09 → 13.48 M 点；本体素能着色的行颜色完全不变；内存峰值 5.40 → 5.44 GB，导出耗时不变 |
| 最终轮 mcp24（默认设置） | 轨迹 0.96 / 3.41 / 7.18 cm，100 % ≤10 cm；起步段原生 +1.44、优化 +0.35 cm（在原生与 RTK 之间）；5–20 m 厚度 22.61 mm；轨迹、几何与 mcp22b 在多次运行的波动范围内一致 |

## 1. 默认开

| 位置 | 改动 |
| --- | --- |
| `config/wuhu_truck29.yaml` | `save_map_priority_enabled: true`，注释改为默认开及副作用（远遍分块缺被另一遍更近看到的灯柱 / 对向车道，整图并集完整） |
| `include/rtk_fusion.h` | 成员默认值 `true`（launch 未给参数时也是开） |
| `scripts/run_wuhu_round.sh` | 新增 `--no-map-priority`（`--set rtk.save_map_priority_enabled=false`）；`--map-priority` 保留 |
| Studio | `common.py` 默认 `map_priority: True`；`mapping.py` 只有明确取消勾选时传 `--no-map-priority`，否则传 `--map-priority`；`app.js` 未保存过时显示为勾选 |

`map_export.py` 跟随轮次的开关，不用改。用户当前的 Studio 保存状态（`config/carlibration/studio_state.json`，不入库）已通过 `/api/state` 改为开。

## 2. 删除的数据

只删了关闭开关时产生的数据：mcp21（LIVO，关）、mcp23（LIO 60 s，关）以及 mcp21 的三份导出。下面这些不是本次“关闭时”产生的，未删除，是否清理请确认：

- `mcp2/3/6/8/13`、`wuhu_28_lio_7`、`wuhu_29_livo_11`、`t29raw_v033_lio_full_test`：距离优先出现之前的全部回波轮次（基线对照）。
- `calib_truck08_*`（3 个）：用户自己的 xingye truck08 标定流水线 17:54 产生，当时 yaml 默认关；属于标定数据，不动。
- `mcp14–19`、`mcp22b`：开距离优先的轮次。

（mcp20、mcp22、wuhu_29_lio_9 在本次开始前已不在 `results/` 中。）

## 3. 核查 1：标准地图的灯柱“毛边”

`global_map.pcd`（5 cm 体素质心）灯柱壳宽中位 7.2 cm，`map_N.pcd` 行级 3.2 cm。原型 `results/.mcp/gm_halo.py` 取 mcp22b 7 根灯柱 0.6 m 内的写入行，带帧号、距离、雷达、强度，自己重建 5 cm 体素（复现 global_map：2.7 3.7 9.5 7.2 5.5 11.1 5.6 cm）。偏离圆面 >5 cm 的体素：

| 灯柱 | 偏离体素（占灯柱框内体素） | 主要成分 |
| --- | --- | --- |
| 4、6、7 | 103–238（4–9 %） | 杆上的标牌、灯臂、树枝，近距离（7–10 m）的真实结构（`results/.mcp/halo/pole4.png`、`pole6.png`、`pole7.png`，左为行、右为体素，红色为偏离点）。4 号标牌每体素 3 行 / 2 帧（杆面 3 / 3）；6、7 号的标牌被扫到的次数少，多为 1 行 / 1 帧 |
| 0、1、5、8 | 13–111（0–4 %） | 1 行 / 1 帧、强度偏低（中位 10–26，杆面 27–31）的边缘回波，7–10 m |

两类都不是往返重影（那是距离优先去掉的）。行级指标被密集的杆身主导；体素图每 5 cm 只留一个点，附着物和零星边缘点的占比就变大。杆身在两种产物里一样细，不改。

## 4. 核查 2：体素级去噪（不采纳）

`results/.mcp/gm_stats.py` 统计 mcp22b 整图每个 5 cm 体素的回波数、帧数、最近距离（26.36 M 体素），`gm_rules.py` 评估规则：

| 规则 | 保留体素 | 0.2 m 占据格丢失 | 1 m 列丢失 | 灯柱壳宽 cm（0 1 4 5 6 7 8 中位） |
| --- | --- | --- | --- | --- |
| 现状 | 100 % | 0 | 0 | 5.6 |
| 每体素 ≥2 回波 | 42.3 % | 55.2 % | 12.7 % | 2.5 |
| 每体素 ≥2 帧 | 32.9 % | 60.5 % | 12.9 % | 2.6 |
| ≥2 帧，或最近距离 >20 m | 57.0 % | 24.3 % | 0.12 % | 2.8 |
| ≥2 帧，或 >15 m | 63.7 % | 16.9 % | 0.03 % | 2.8 |
| ≥2 帧，或 >10 m | 78.4 % | 6.0 % | 0.01 % | 2.8 |

开距离优先后整图 57.7 % 的体素只有 1 个回波，67.1 % 只来自 1 帧（0–10 m 内也有 57 %），毛边体素在统计上和正常表面没有区别，任何此类规则都会删掉真实表面。逐帧边缘回波滤波需要线束顺序：存档按时间混排 4 台雷达、且已降采样，而收益只是少数灯柱的零星点，不做。

## 5. 核查 3：彩色着色补全（采纳）

现象：开距离优先后几何行只剩近处观测，rgb_map_N 只有 38.3 % 的行能着色。4 个 20 × 20 m 区域（灯柱 0、4、6、8 周围，mcp22b 数据，`results/.mcp/rgb_cov.py` / `rgb_cov2.py` / `rgb_nb.py`）：

颜色偏差：在已有 3 cm 颜色的行上，假设改用兜底，所得颜色与 3 cm 颜色的平均差（RGB 0–255）。

| 方案 | 着色比例（4 区） | 兜底颜色偏差 | 说明 |
| --- | --- | --- | --- |
| 相机看到过（0.5 m 内有颜色观测） | 80–94 % | — | 上限参考 |
| 原规则：3 cm 体素，否则 6 cm 体素 | 32.0–44.7 % | 7.3–9.2 | 6 cm 体素可取到 10 cm 外的观测，却漏掉隔着边界 1 mm 的 |
| 再加 12 cm 体素一级 | 53.4–70.0 % | 15.8–18.7 | 颜色明显变糊 |
| 本 3 cm 体素，否则 27 邻体素内最近（≤6 cm） | 46.9–64.3 % | — | 比精确球少 3–5 个百分点 |
| **本 3 cm 体素，否则 6 cm 以内最近（采纳）** | **50.2–68.1 %** | **3.5–4.5** | 本体素能着色的行颜色完全不变，兜底颜色也比原 6 cm 体素更准 |

- 建图程序 `ColorLookup`：每个 3 cm 体素保留最佳观测（距离 + 20 m×rad/s 转速）的颜色和位置，排序键数组 + 位置（24 字节/体素，去掉 6 cm 一级）；兜底在排序键里按 x、y 列二分查找 ±6 cm 的体素，取最近。`map_export.py` 用同一规则（`cKDTree`）。
- 自检（`results/.mcp/rgb_check.py`）：mcp24 在灯柱 4 区 52 万行上，建图程序的结果与独立的 Python 实现逐行比对：着色与否 100 % 一致，颜色 99.999 % 一致。
- 侧视、俯视图（`results/.mcp/halo/side46.jpg`、`cov4_06.png`）：标牌、杆身、路面弧线更完整，未见颜色串到别的物体上。

| 指标 | mcp22b（旧规则） | mcp24（新规则） |
| --- | --- | --- |
| rgb_map_N 着色行 | 54.4 M / 142.2 M（38.3 %） | 84.6 M / 142.3 M（59.5 %） |
| rgb_map_N 大小 | 0.83 GB | 1.3 GB |
| global_map_rgb | 9.09 M 点（global_map 的 34.5 %） | 13.48 M 点（51.1 %） |
| 着色表构建 / 最终优化到导出完成 | 26.4 s / 104.6 s | 21.0 s / 107.6 s |
| 进程内存峰值 VmHWM | 5.40 GB | 5.44 GB |
| 灯柱壳宽中位：map_N / rgb_map_N | 3.2 / 4.1 cm | 3.1 / 3.8 cm |
| 灯柱壳宽中位：global_map / global_map_rgb | 7.2 / 3.7 cm | 7.8 / 6.3 cm |

global_map_rgb 的壳宽变大不是变差：彩色图就是几何图里能着色的那部分点，以前没着上色的标牌、灯臂（5、6、7 号）现在着上了，数值向同一轮 global_map（7.8 cm）靠拢；0 号从 5.4 降到 2.8 cm。几何完全没变。

## 6. 最终轮 mcp24（默认设置，无距离优先开关参数）

| 指标 | mcp22b（开） | mcp24（默认） |
| --- | --- | --- |
| 优化 vs RTK 中位 / P95 / 最大 | 0.96 / 3.40 / 7.10 cm | 0.96 / 3.41 / 7.18 cm |
| ≤10 cm / >20 cm | 100 % / 0 | 100 % / 0 |
| 起步前 10 m 横向（原生 / 优化） | +1.44 / +0.35 cm | +1.44 / +0.35 cm |
| 5–20 m 厚度 / 20–40 m 厚度 | 22.60 / 26.00 mm | 22.61 / 26.93 mm |
| 跨遍平面偏差 >5 cm | 14.60 % | 16.16 % |
| 回环接受 | 68 / 92 | 67 / 92 |
| 写入行 | 142.2 M / 353.2 M（40.3 %） | 142.3 M / 353.2 M（40.3 %） |

着色只在导出阶段，不影响轨迹和几何；差异是 LIVO 多线程的运行间波动：同样代码的 mcp16 / 17 / 18 / 22b 跨遍偏差 14.47 / 17.51 / 17.57 / 14.60 %，20–40 m 厚度 26.35 / 26.98 / 26.16 / 26.00 mm。完整 PCD：`results/mcp24_livo_default/Log/fusion/1792497041847240215/`（map_N、rgb_map_N、`map/global_map.pcd`、`map/global_map_rgb.pcd`）。

## 7. 改动文件

| 文件 | 改动 |
| --- | --- |
| `config/wuhu_truck29.yaml` | 距离优先默认开；彩色着色说明 |
| `include/rtk_fusion.h` | 默认值 true；`ColorLookup` 结构 |
| `src/rtk_fusion.cpp` | `ColorLookup::find` 本体素否则 6 cm 内最近；`colorLookup` 只建 3 cm 一级并保存位置；日志 |
| `tools/livo/map_export.py` | `colour_points` 同一规则（`cKDTree`），说明文字 |
| `scripts/run_wuhu_round.sh` | `--no-map-priority` |
| `tools/studio/app/common.py`、`mapping.py`、`web/app.js` | Studio 默认开 |

一次性探针（`results/.mcp/`，不入库）：`gm_halo.py`、`gm_stats.py`、`gm_rules.py`、`halo_png.py`、`order_check.py`、`rgb_cov.py`、`rgb_cov2.py`、`rgb_nb.py`、`rgb_side.py`、`rgb_check.py`。
