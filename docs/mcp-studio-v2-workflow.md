# LIVO Studio v2：控制台与标定工具链重构说明

日期：2026-09-22 · 分支 `fusion/56c6896-ins-attitude` · 范围：`tools/studio`、`tools/calib`、`tools/livo`（新增）、`scripts/*`；**SLAM 前后端算法（`src/`、`include/`、`launch/`、`config/*.yaml`）未改动。**

## 1. 问题与结论

| 现象 | 根因 | 处理 |
|---|---|---|
| 网页流程绕：先“数据准备”再建图，LIVO 与 LIO 各要一个 prepared bag | Humble `ros2 bag play` 不能把多目录按时间合并，因此曾用 `prepare_wuhu_bag.py` 合并成 `results/prepared/*`；建图模式与所选 bag 耦合，选错即 0 帧（`studio_livo_0922_112950` 失败即此） | 新增进程内回放器 `tools/livo/player.py`，直接对原始录制目录按时间戳堆合并发布，全部脚本/工具接受原始根目录，`results/prepared` 不再需要 |
| “运行阶段”报“阶段失败，请查看日志” | 阶段输入（prepared bag 是否含相机、fusion 目录是否有 `scans.bin`）在启动后才失败；日志只在内存里 | 启动前由 `dataset.py` 校验话题、由后端校验 fusion 目录；任务日志落盘 `results/.studio/jobs/`，失败原因显示在任务表 |
| 版本后缀 import/ground/driven/camtime/camlidar/manual 无解释 | 工具与页面都只显示后缀 | `livo_calib.py` 内置阶段目录 `STAGES`（中文名、输入、产出、是否需建图、是否仅供参考），`list --json`/`meta.json` 带 `stage_label`；页面全部用中文阶段名 |
| `v006_camlidar` 多出一组相机图片 | 该阶段把 LiDAR 深度边缘投影到图像生成 `overlays/` before/after 图，供人工确认；其余阶段没有 | 详情页对叠加图给出说明，并把 camlidar 标为“仅供参考”，流水线不自动激活 |

## 2. 数据层（`tools/livo/`）

- `dataset.py`：`Dataset.open(path)` 识别 **原始分段根目录**（`*/metadata.yaml` 或 `*/*/metadata.yaml`）或 **单 bag 目录**；无需 ROS 即可给出分段/话题/时长/`lio_ready`/`livo_ready`/覆盖缺口；`iter_messages()` 惰性堆合并、按话题单调保序；预设话题集 `lio` / `livo` / `all`。CLI：`dataset.py <dir> [--json] [--check]`。
- `player.py`：rclpy 节点 `livo_player`，原样转发序列化消息 + `/clock`；`--preset lio|livo|all --rate --start-offset --duration --wait-subscribers --dry-run`；SIGINT 干净退出（退出码 0），stdout 输出 `[player] t=.. / .. s NN% msgs=.. lag=.. ms` 与最终 `[player] summary {json}`。
- `rosenv.py`：`ensure()` 把任意 python 进程重新以 `/usr/bin/python3` + 已 source 的 ROS 环境执行；`status()` 供系统页显示。

芜湖原始录制（8 段、120.13 s、3.74 GB、56 396 条）：全量读取 2.35 s，0 重复 / 0 乱序，跨段连续；与旧 prepared bag 消息数完全一致。回放稳态延迟 0 ms（首段打开时最大 ~50 ms）。

## 3. 脚本

- `scripts/run_wuhu_round.sh <round> --mode lio|livo [--dataset DIR] [--calibration F] [--no-rtk] [--no-save-map] [--rviz] [--rate] [--start-offset] [--duration] [--set k=v] [--no-eval]`：启动前 `dataset.py --json` 校验（LIVO 缺相机直接退出），写 `results/<round>/round.json`（参数、提交、二进制时间、阶段与状态），回放改为 `player.py --preset <mode>`，其余（排空、SIGINT 优化导出、评估）不变。
- `scripts/evaluate_wuhu_round.sh <round> [--dataset DIR]`：INS 参考从 `round.json`（或 `run.log`）记录的数据源读取。
- `scripts/play_all_bags.sh`：薄封装 `player.py`，参数兼容旧用法并新增 `--dataset`。
- `evaluate_wuhu_lio.py` / `refine_multi_lidar_extrinsics.py` / `refine_multi_lidar_yaw.py` / `dump_bag_reference.py`：读取改为 `livo.dataset`，ROS 自举改为 `rosenv.ensure()`。`prepare_wuhu_bag.py` 标记 DEPRECATED。

## 4. 标定工具（`tools/calib/livo_calib.py`）

- `--bag` 接受原始根目录；每个阶段前 `require_dataset()` 校验话题（相机阶段必须含 `midrange_camera/ffmpeg`），`driven`/`camera-lidar` 校验 fusion 目录含 `frames.csv`。
- `meta.json` / `report.md` 新增 `stage_label`、`advisory`、`lineage`、`changes_vs_parent`（每个 LiDAR/相机的 Δt cm、Δrpy °、时间偏移与内参变化）、输入数据源摘要。
- 新命令：`validate --id|--file [--strict]`（矩阵正交、平移/姿态合理性、相机内参、时间偏移）、`delete --id`（仅未激活叶子）。`list --json` 附 `stages`、`children`、`active`、`parent_missing`。

## 5. 标定职责划分（最终）

| 位置 | 负责 | 不负责 |
|---|---|---|
| 离线标定工具 / Studio 标定页 | 生成并版本化正式外参与时间偏移；LiDAR 流水线（ground → LIO → driven）、相机流水线（camtime → LIVO → camlidar 复核） | 运行时修改 |
| 算法内 `multi_lidar.precheck`（已有） | 起步静止段核对地面 roll/pitch/z，小修正当次生效，大偏差告警，写 `Log/extrinsic_precheck.json` | 改写标定文件（固化用 `from-precheck`） |
| SLAM | 只读取一个激活的 `calibration_file`（含相机内参与时间偏移） | 任何标定搜索 |

## 6. Studio v2

后端 `tools/studio/server.py` + `tools/studio/app/{common,jobs,datasets,calibration,mapping}.py`（仅标准库）：
- 启动即 `rosenv.ensure()`，不再出现“忘了 source ROS 导致回放器起不来”。
- `jobs.py`：任务 = 顺序步骤列表，日志与状态落盘 `results/.studio/jobs/<id>.{json,log}`，重启后保留（未完成者标记“中断”）；停止语义：`finish`（`pkill -INT livo/player.py`，轮次继续导出评估）/ `abort`（杀进程组 + 残留 ROS 进程）。
- 流水线：`POST /api/calibrations/pipeline {kind: lidar|camera}` 自动串起标定阶段与所需建图轮次（`calib_<truck>_<mode>_<stamp>`，强制保存地图、不评估），用上一阶段输出的版本 id / fusion 目录作为下一步输入。
- API：`/api/overview`、`/api/datasets?path=`、`/api/fs`、`/api/vehicles`、`/api/calibrations[/version|/file]`、`/api/jobs[/<id>[/stop]]`、`/api/runs[/<r>/live|/file]`、`/api/mapping/start|stop|kill`、`/api/evaluate`、`/api/calibrations/import|stage|pipeline|activate|validate|delete`、`/api/state`。
- 前端 `tools/studio/web/`（原生 JS，无构建）：工作台 / 数据源 / 标定 / 建图 / 结果 / 系统；进行中轮次的阶段条来自 `round.json`，回放进度来自 `playback.log` 的 `[player]` 行，曲线来自 `Log/fusion/*/diagnostics.csv`、`online_path.csv`。

## 7. 验证记录（容器内）

| 项目 | 结果 |
|---|---|
| `dataset.py --check` 原始根目录 | 56 396 条，0 重复/乱序，2.35 s |
| `run_wuhu_round.sh mcp_test_lio_raw --mode lio --duration 30 --no-save-map --no-eval` | phase done，RTK finalized 279 帧，`round.json` 完整 |
| `evaluate_wuhu_round.sh` 读取原始根目录 | INS 参考装载成功（native/fused IMU 差 0.125/0.020 m）；地图项因未保存地图跳过 |
| `livo_calib.py ground --bag <原始根目录>` | 9.5 s，修正量与旧 prepared 流程一致（z +1.8/+0.3/+9.1 cm），`changes_vs_parent`/`lineage` 正确；`delete` 拒绝删除激活版本 |
| Studio API：启动 LIO 轮次 → 25 s 后 `finish` | 回放中断于 17.2 s，算法完成优化导出，job done、round.json done，任务记录落盘 |
| Studio API：LiDAR 标定流水线（ground → LIO 全程 → driven → validate，未激活） | 见 `results/.studio/jobs/` 最新 pipeline 记录与 `results/calib_truck29_lio_*` |

## 8. 遗留 / 建议

- `results/prepared/`（约 7 GB）可以删除；`prepare_wuhu_bag.py` 仅为兼容保留。
- 未跟踪的旧实验文件 `config/perception_truck29_auto_refined.json`、`config/perception_truck29_cam_sanitized.json`、`scripts/auto_precorrect_truck.py`、`scripts/carlib_densify_and_calibrate.py` 与本次无关，建议按需清理或纳入版本。
- 后续优化 SLAM 时只需保持：`results/<round>/round.json` 字段、`Log/fusion/<stamp>/{trajectory_optimized.txt,diagnostics.csv,online_path.csv,frames.csv,scans.bin}` 以及 `Log/extrinsic_precheck.json` 的约定，控制台无需改动。
