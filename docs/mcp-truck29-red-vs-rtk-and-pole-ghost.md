# MCP 专题：truck29 红色原生轨迹偏离 RTK 的根因 与 起步灯柱重影的定量结论

日期：2026-10-09 · 分支 `fusion/56c6896-ins-attitude` · 前置：`docs/mcp-zongyang-truck07-frontend-timebase.md`、`docs/mcp-camera-lidar-audit-and-pole-ghost.md`

## 1. "红色原生轨迹与 RTK 差距大（起步即大）"的根因：跨车标定错配

2026-10-09 早晨的 6 个 wuhu 数据建图轮（`studio_lio_1009_082609/082718/083737/083812/083830/084039`）
**全部用 `zongyang/truck07/v004_driven` 的标定跑 wuhu_livo 数据**：

- truck07 的杆臂/外参不属于 truck29（axle 阶段曾对整机平移 -26.5 cm 量级）；
- truck07 v004 的时间语义（lidar=-0.0041 / lidar_frontend=-0.125 → RTK 参考移位 +0.1209 s）
  对 truck29 数据错 ~125 ms，6.5 m/s 下沿迹 ~80 cm 系统错位；
- 结果即"红色原生轨迹从起步就偏离 RTK 米级"（`mapping.py` 中实测注记：native median 1.83 m、pitch 2.9°）。

对照组（同一早晨、正确标定 v068_ground 的 `calib_truck29_lio_1009_083005`）：
起步前 10 m 原生横向偏差 median **-0.2 cm**；原生漂移 45 s 约 0.6 m 属正常纯 LIO 行为，
融合后 median 1.05 / P95 2.58 / max 5.06 cm，100% ≤10 cm。
**结论：起步大偏差不是算法/标定退化，是建图时选错了车辆的标定文件。**

### 1.1 防护（已实现并测试）

`tools/studio/app/mapping.py start()`：原有检查只验证 标定id↔所选车辆（表单草稿场景），
无法拦截"整个车辆选择器停在另一台车"的情况。新增 **数据路径↔矿区** 校验：数据路径中出现
`config/carlibration/` 下已注册矿区名（wuhu/zongyang/…）而所选标定不属于该矿区时直接拒绝，
错误信息指明原因与处置。实测：wuhu 数据 + zongyang 标定 → 拒绝；正确配对 → 放行。

## 2. 今晨 truck29 重标定链（v067_axle→v068_ground→v069_driven）审计

- v067 axle：时移解 -0.3 ms（-0.1459→-0.1462），平移 -0.1 cm，quality ok —— 与 v066 一致性极好；
  **前端时基自动同步按设计不动模板**：`frontend time base: template confirmed (candidate -0.1337);
  imu/ins lag +12.5 ms, corr 0.998`；
- v069 driven：back_lidar rot z +0.14°、y +5.9、z -7.5 cm（quality warn，竖直匹配 1941 条未收敛）；
- **guard 实证（v069 激活态）：GUARD PASS**，并与 v066 基线对比：

| 指标 | v066 (10-08 守护) | v069 (10-09 守护) |
|---|---|---|
| LIO fused xy median/P95/max (cm) | 0.80 / 2.95 / 6.38 | 0.81 / 2.58 / 6.12 |
| LIVO fused xy median/P95/max (cm) | 0.86 / 2.46 / 4.96 | 0.83 / 2.08 / 4.92 |
| 灯柱壳层 P90（4 根，cm） | 5.5 / 5.3 / 4.9 / 5.2 | 5.4 / 4.9 / 4.8 / 5.1 |
| back 沿迹 cruise \|median\| (cm) | 1.3 | 2.1 |
| 墙面 front_right vs back (cm) | 1.9 | 1.6 |

v069 总体持平略优（壳层/墙面/P95 更好，back 沿迹 +0.8 cm 在门限内）→ 维持 v069 激活。

## 3. 起步灯柱重影：确切成因（定量）

guard LIO 轮（45 m 落图截断后交付地图）的灯柱逐时代分解：

- **相邻时代等高中位距 3v4=2.97、8v9=3.26 cm** —— 这是交付地图中真实的"重影"水平，
  与 9-26 审计的近场 3 cm 一致，肉眼几乎不可见；
- 远距对（3v7≈15、3v9≈23 cm）的轴位随时代**单调**漂移（-10.83,-273.56 → -10.63,-273.68），
  且 v066 与 v069 两套独立标定给出几乎相同的 pairwise 表（median 9.40 vs 10.26 cm）——
  若是标定或轨迹错位不可能两套标定逐对复刻。成因是**物理/几何本底**：
  17 cm 半径灯柱每个时代只见单侧弧，弧拟合的轴心沿视线方向有偏置，视角随行驶扫过即表现为
  "轴心漂移"；叠加远观时 INS/RTK 偏航 0.10-0.15° 的物理极限（9-26 审计已证伪调融合权重）。
- 融合轨迹本身 max ≤6.1 cm、修正节点步长 P95 0.009°，不构成可见重影源；
- 远场点雾已被 `rtk.save_map_max_range=45 m` 排除出交付地图（0acad5c）。

**结论：起步段灯柱已到当前传感/几何本底（相邻时代 ~3 cm，壳层 P90 4.8-5.4 cm，限 6.0），
无需也不应再用算法硬拉。** 唯一注记的远期课题仍是竖直柱状地标因子（灯柱自身做跨时刻闭环，
docs/mcp-camera-lidar-audit-and-pole-ghost.md §4），历史实验（run84-89、种子窗）表明简单加边
反而伤近场，不在本次改动。

## 4. 版本留痕

- 改动：`tools/studio/app/mapping.py`（数据↔矿区防护）；无 C++ / yaml / 标定数值改动；
- 守护轮：`results/guard_t29_lio_20261009_085352`、`results/guard_t29_livo_20261009_085352`（PASS）；
- 错配证据轮：`results/studio_lio_1009_08*`（round.json 的 dataset/calibration 字段）。



## 5. 追加（同日）：--no-rtk 建图失败修复 + 标定下拉框陈旧选择修复

### 5.1 studio_lio_1009_090614 失败根因
`use_rtk:=false` 直接不构建 RTK 后端，而 frames.csv / scans.bin / trajectory_* / map_N.pcd 的
导出器全部住在后端里 -> 建图本体 120 s 正常跑完（标定 v069 正确）却零产物，
run 脚本判 "no optimized trajectory exported"。

### 5.2 修复（测试迭代 2 轮）
- 新参数 `rtk.observations_enabled`（默认 true，不影响融合路径）：false 时后端照常构建并导出，
  但不喂任何 INS/RTK 观测（observe/observeStatus 短路），矿区坐标系初始化照常用 INS 锚定
  （跳过 RTK 质量门），全部产物保持矿区系；`--no-rtk` 在 run_wuhu_round.sh 中接到该参数。
- 迭代 1 实测：图在无锚定时反而扭曲输出（optimized median 3.59 m vs native 1.72 m，
  俯仰窗口漂移 ~2x）—— 雷达几何因子与运动先验互相拉扯、无绝对观测约束。
- 迭代 2：观测关闭时位姿采纳点（updatePoses/initializePose）直接采纳 raw 前端位姿 ->
  optimized == native 完全一致（median 1.7162 / P95 4.0478 / max 4.0776 m，纯 LIO 120 s 正常漂移），
  地图由原生位姿落图。mcp_nortk_lio_fix2：status=done，45 块地图，评测全链路（含灯柱对比）产出。
- 融合路径回归：新二进制守护轮 GUARD PASS，LIO 指标与改动前逐项一致
  （0.81/2.58/6.12 cm；灯柱 5.4/4.9/4.8/5.1；back 2.1；wall 1.6），LIVO 在运行间噪声内。

### 5.3 "下拉框显示 29 号车却用了别的标定"
前端 state 里残留跨车辆的 calibration_id 时，<select> 没有任何 option 命中 selected，
浏览器默认显示第一项"激活版本(vNNN)"——看起来像选了本车激活版，提交的却是陈旧 id；
重新手动选择一次才覆盖。已修：渲染时发现 calibration_id 不属于当前车辆立即清零并持久化
（tools/studio/web/app.js pageMap）；服务端另有两道防线（标定id<->车辆、数据路径<->矿区）。
浏览器需刷新页面加载新 app.js。

### 5.4 留痕
轮次：results/mcp_nortk_lio_fix（迭代1）、results/mcp_nortk_lio_fix2（迭代2，保留完整 PCD 供目检）、
results/guard_t29_{lio,livo}_20261009_093913（回归 PASS）。
改动：include/LIVMapper.h, src/LIVMapper.cpp, include/rtk_fusion.h, src/rtk_fusion.cpp,
scripts/run_wuhu_round.sh, tools/studio/web/app.js。

### 5.1 Phase-3 现场验证与 DDS 探测备忘（2026-10-09 补充）

**蓝线身份**：RViz 显示项 "RTK Fused Trajectory"（topic `/rtk/path`，颜色 0;100;255）。红线 = `/path`（原生 LIO），绿线 = `/imu_reference_path`（INS 参考轨迹，刻意保留不门控——它是参考数据而非融合输出）。

**代码门控**：`src/rtk_fusion.cpp` 中 `publishCurrent()`（~1807）与 `publish()`（~1852，diagnostics.flush() 之后）在 `observations_enabled=false` 时提前返回；publisher 仍创建（RViz 显示项存在但轨迹为空）。

**现场实测**（无RTK轮，回放 50% 时刻 20 s 采样，SHM 探针）：
`/path` 194 条、`/imu_reference_path` 194 条、`/rtk/path` **0** 条、`/rtk/odometry` **0** 条。门控确认生效。

**用户所见蓝线的解释**：轮次 studio_lio_1009_095645 于 09:56 启动，早于修复后二进制（~10:05 构建），蓝线属预期残留；修复后的新轮不再出现。

**精度守护**：t29_guard_rvizfix.log GUARD PASS，LIO 0.81/2.58/6.12 与基线完全一致，LIVO 0.85/2.19/4.75，灯杆/后墙/侧墙全过。

**DDS 探测备忘（重要）**：`tools/livo/rosenv.py` 为全工具链设置 `FASTDDS_BUILTIN_TRANSPORTS=SHM`（纯共享内存传输，发现协议也不走 UDP 多播）。因此任何外部 `docker exec` 会话（默认 UDP+SHM）与运行树**互相完全不可见**——`ros2 topic echo`/rclpy 订阅收 0 条不代表话题静默。正确探测方法：探测端同样 `export FASTDDS_BUILTIN_TRANSPORTS=SHM`。

