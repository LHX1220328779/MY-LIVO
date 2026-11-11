# 高斯建图

`gaussian` 与 `lio`、`livo` 并列。高斯模式复用原生 LIVO 和 RTK 最终校正，再独立优化地图外观；相机位姿、高斯中心、原有里程计与标定不进入优化。也可直接读取已有轮次，跳过位姿估计重跑。

## 环境与构建

本机复用 `/opt/venvs/my-livo-gaussian`：PyTorch 2.5.1+cu124、CUDA 12.4、RTX 4060 8 GB。系统 ROS Python/OpenCV 不变。依赖包含固定版本 transformers 4.46.3、safetensors 0.4.5，用于本地天空分割。

```bash
bash scripts/setup_gaussian.sh
bash scripts/build_gaussian.sh
```

需具备原工程的 OpenCV/Eigen/CMake/FFmpeg，以及 `python3-venv`、`libglm-dev`。`GAUSSIAN_ENV`、`CUDA_HOME` 可指定安装位置；构建使用 `GAUSSIAN_PYTHON`，GPU 架构由 `GAUSSIAN_CUDA_ARCH` 指定（本机 89）。默认单任务编译，独立构建目录为 `build/gaussian`。OpenCV 图像读写保留独立 ABI 边界。

## 运行

新轮次沿用数据和标定选择流程：

```bash
bash scripts/run_wuhu_round.sh my_gaussian_1 --mode gaussian \
  --dataset /path/to/ros2bag --calibration /path/to/calibration.json
```

2026-11-08 起，运行脚本默认关闭 LIO/LIVO 的分块、全局及彩色建图 PCD，只保留扫描、位姿等存档。原生扫描选点掩码 `map_rows.bin` 独立保存，供高斯几何初始化使用；已有 PCD 不删除。高斯模式仍需要扫描存档，不能使用 `--no-save-map`。此设置只调整结果输出，不改估计器数学逻辑。

2026-11-11：控制台手动执行 LIO/LIVO 并开启“保存地图”时，显式保存分块 PCD；开启“导出标准地图”时，同时生成 `Log/fusion/<stamp>/map/global_map.pcd`（LIVO 另有彩色产品）。脚本手动调用可增加 `--map-chunks --map-export`。开发脚本默认、标定流水线和高斯模式继续只保存原生扫描/位姿存档。

复用已有结果：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/livo/gaussian_map.py results/<已有轮次> \
  --out results/<已有轮次>/gaussian_shared --keep-inputs
```

默认从轮次清单读取录制，从 `parameters.txt` 读取实际标定路径和图像时间偏移，也可指定 `--dataset`、`--calibration`。每次使用新的 `--out`，避免覆盖已有地图。高斯阶段失败时保留原生轨迹与存档并报告失败。

默认宽 640、采样间隔 0.5 秒、初始化体素 0.08 m。训练用一张共享地图，40 帧为一个工作批次，额外回放最多 40 个具有共同可见表面的训练视角，前后各遍历一次，每批最多 12 轮。`--max-active 3000000` 限制 GPU 内高斯数量，超额时继续拆分图像和相机批次。每批训练只加载相机前方 60 m 的表面；最终导出及留出评估包含完整全局地图，输出不会按此距离截断。

```bash
/usr/bin/python3 tools/livo/gaussian_map.py results/<轮次> \
  --out results/<轮次>/gaussian_shared \
  --width 640 --interval 0.5 --voxel 0.08 \
  --shared-epochs 12 --shared-sweeps 2 --max-active 3000000
```

`--start`、`--duration` 限制已有轮次内的相机时间窗口；`--prepare-only` 只生成配对输入；`--keep-inputs` 保留输入；`--no-sky-masks` 关闭语义掩码。旧版独立子图拼接仅通过 `--backend legacy` 保留复现实验，其 `--iterations`、`--refine-passes`、`--max-gaussians`、`--position-lr` 等参数不控制共享表面模式。

## 算法与数据边界

1. 相机使用原始 HEVC 图像、实际图像时间偏移和现有标定进行缩放、去畸变。缩放和 CUDA 投影统一像素中心约定，不修改标定文件。相机姿态仍为 `C(t_image) * T_native_imu * T_imu_rear * T_rear_camera`；禁止跨存档缺口外推。
2. 高斯几何复用原生全局选点结果，不再把各图像时刻的原始远距离回波直接拼成地图。优先只读已有 `map/global_map.pcd`；没有 PCD 时按 `map_rows.bin` 从去畸变扫描和最终 `C(t_point)` 重建，在内部缓存为 NPY，不生成原生 PCD。每个 8 cm 单元保留真实表面样本，避免增加一次质心平滑。
3. 将全局表面投影至训练图像，按像素 Z-buffer 和真实 LiDAR 前景深度检查遮挡。结合原生选点、自车回波、车辆实际通过的内部空间、保守自由空间和语义车辆/人员深度关联，清理跨时段动态残留。地面高度和细长结构保护用于限制误删；全部清理只作用于高斯种子。只使用训练 RGB 初始化颜色，留出 RGB 不参与初始化或优化。全量原生 LiDAR 表面是几何输入，不能把这一评估宣称为完全未观测几何的新场景泛化。
4. 全局高斯行保持固定身份。每批从共享表读取、训练后写回相同行；相邻时段、重复经过区域都更新同一张地图，不再导出多个独立高斯层后直接叠加。
5. 局部 PCA 决定表面方向和初始协方差；路面限制法向厚度，细长结构限制横向尺度，不确定邻域采用紧凑尺度。中心和方向固定，仅训练 SH3 外观、透明度和有上下界的尺度。深度项使用相对残差，CUDA 归一化深度梯度包含 alpha 归一化因子，避免以大薄片或厚路面换取 RGB 指标。
6. 同一个本地 SegFormer 同时提供保守天空掩码和车辆/人员掩码。天空保护图像边缘与真实 LiDAR 像素；车辆/人员需结合深度及离地高度关联，不能把被遮挡的远处静态表面直接当成车辆。训练时屏蔽车辆区域的 RGB/SSIM/深度损失，避免将移动卡车外观写入静态路面。模型和 revision 固定，推理在本机进行；不使用学习深度、第二相机或新位姿源。天空背景色只从训练图像估计；训练使用随机背景减少前景透明度与颜色的歧义。没有几何支持的天空用常量背景表达，不是重建了真实天空纹理。
7. 每八帧留出一帧，最终从完整保存的 PLY 重渲染。评估包含全图 PSNR/SSIM、真实 LiDAR 覆盖邻域 PSNR、双方有深度处的渲染深度 MAE，以及 30 m 内 MAE/中位数。高斯混合期望深度不同于点云表面误差，这些指标不能替代独立轨迹或灯柱顶部厘米级验收。

这是最终轨迹完成后的离线建图阶段。单中距相机视场外、LiDAR 未观测远景和动态遮挡仍有覆盖限制。

## 产品与评估

- `point_cloud.ply`：一张完整的标准 SH3 高斯地图。
- `gaussian_centers.pcd`：高斯中心和 DC 颜色，属于高斯产品。
- `camera_poses.json`：实际固定相机位姿与时间。
- `shared/seed_info.json`、`shared/training.log`、`map_info.json`：几何来源、约束、工作批次和运行参数。
- `shared/views/batch_*/merged_*.png`、`reference_*.png`、`metrics.csv`：保存地图的留出视角重渲染与原图。

Studio 结果可读取高斯产品和预览。评估旧版完整拼接图与新版时，使用相同相机、相同背景和同一组留出视角：

```bash
/usr/bin/python3 scripts/analysis/gaussian_compare.py \
  results/<轮次>/prepared/input results/<轮次>/gaussian_shared NEW_REPORT_DIR \
  --baseline-model /path/to/old_complete/point_cloud.ply
```

报告包含逐视角对比和汇总，不用单子图的训练效果替代最终全局地图效果。共享训练在最终重渲染前写入 `trained_pending_replay` 检查点，已完成的视角批次保留。只渲染时不分配反向传播采样缓存，载入 PLY 时也避免复制一整张 GPU 表；这两处降低显存占用而不改变渲染结果。

## 浏览器查看与 Linux 黑屏

推荐使用专用启动命令：

```bash
bash scripts/view_gaussian.sh results/xingye_13_gaussian_4
```

参数可以换成其他轮次目录、具体 `point_cloud.ply` 或 `.ssproj` 工程。目录内有 `.ssproj` 时优先恢复其显示方向和初始视角；直接打开本项目 PLY 时，根据 `camera_poses.json` 自动选取车辆视角，并将 Z 向上的地图调整为浏览器导航使用的 Y 向上。这只是显示变换。

轮次目录包含 `analysis/preview_regions` 时，默认打开起始区域，减少完整大地图的加载时间。可显式指定 `preview_regions/middle`、`end` 或完整 `point_cloud.ply`。区域预览保留周围 62 m 及当前视角前方 120 m 的原高斯参数，避免截掉远处背景；完整地图单独保留。

首次使用自动在 `build/gaussian/viewer` 安装固定版本 Playwright，并准备其 Chromium；需要 Node.js >=20.19、npm 和网络。本机已配置完成。后续复用该专用浏览器进程，打开其他地图只需再次运行同一命令并更换参数。也可在专用窗口使用“文件 → 打开”恢复 `.ssproj`、“文件 → 导入”选择高斯 PLY。保存为 `.ssproj` 可保留初始视角。

本机已对比实际有界面的 Chromium 启动方式：普通启动能访问 `navigator.gpu`，但 `requestAdapter()` 返回 `null`；专用配置返回 NVIDIA/Lovelace，53.8 万高斯可渲染并支持鼠标导航。当前环境的默认 Vulkan 清单目录缺少 NVIDIA 条目，因此启动器在自己的运行目录生成 NVIDIA ICD 清单，为该浏览器进程配置 Vulkan/WebGPU。普通 Chrome 窗口不会继承这些启动参数，单独在普通窗口再次打开网页可能仍然黑屏。对照记录在 `results/xingye_13_gaussian_3/browser_gpu_comparison.json`。

网格也是 GPU 渲染内容，因此“网格也没有”应先检查浏览器图形链路；PLY 加载完成不代表 WebGPU 渲染设备初始化成功。排查其他浏览器时检查 `chrome://settings/system` 的图形加速、`chrome://gpu` 的 WebGPU/驱动信息，以及控制台中 `navigator.gpu.requestAdapter()` 是否成功。仅检查是否存在 `navigator.gpu` 不足以确认支持。

查看操作：左键拖动旋转、右键拖动平移、滚轮缩放、双击感兴趣的路面或墙面设置观察中心。首次查看用已保存的车辆视角；整体框选后的远距离鸟瞰视角通常看不到与训练视角相同的纹理细节。

此启动器使用独立配置目录，并且只调用本地文件导入接口。网络用于获取 SuperSplat 页面和首次安装查看依赖，不调用地图发布接口。系统 ROS、建图环境及普通浏览器设置不变。

第三方来源及保留的许可见 `gaussian/third_party/gaussian_lic/UPSTREAM.md` 和各源文件、许可文件。

## 接入阶段短窗口验证（历史记录）

使用星业 13 号车原始录制 `/home/project/data/haibo/xingye/13/20261027/ros2bag`，标定 `xingye/truck13/v014_camphoto`。原有 LIVO 基准为 `results/gaussian_livo_reference_1107`（45 秒、427 帧、8 个完整几何 PCD 分块）。新模式与基准生成的估计器配置相同，复用同一个原 `fastlivo_mapping` 可执行文件；`src/`、`include/`、根 CMake/package 未改。

算法迭代使用 `xingye13_livo_1105` 已有存档的第 20–40 秒，40 个相机帧、约 37.9 m 路程，35 帧训练、5 帧留出。结果保留在 `results/gaussian_x13_iteration1` 至 `iteration5`，对应准备数据在 `gaussian_x13_prepare`（60 m）和 `gaussian_x13_far_prepare`（120 m）。

- 原深度权重 0.005、无尺度上限：画面改善，但 60 m 内全范围渲染深度 MAE 从初始化的 0.359 m 增大到 0.954 m，未采用。
- 加强深度与尺度约束后，60 m 数据上的 MAE 降到 0.305 m（权重 0.1、尺度上限 0.5 m），明显减少用大高斯填补空白的问题。
- 远景范围扩展至 120 m 后，山坡覆盖改善。此时深度指标覆盖了更远、更稀疏的表面，不能直接与 60 m 的全范围 MAE 比较。
- 最终锁定高斯中心（`iteration5`），以外观参数优化完成渲染，逐中心对输入训练 LiDAR 点作最近邻检查：338381 个高斯中心全部有限，距离最大值为 0 m（同一 float32 坐标）。

最终 120 m 窗口的 5 个留出视角平均结果：

| 指标 | LiDAR/RGB 初始化高斯 | 最终固定中心高斯 |
|---|---:|---:|
| 激光覆盖邻域 PSNR | 10.25 dB | 22.43 dB |
| 全图 PSNR | 9.85 dB | 20.54 dB |
| 全图 SSIM | 0.239 | 0.737 |
| 全范围渲染深度 MAE | 1.030 m | 0.766 m |
| 30 m 内渲染深度 MAE | 0.543 m | 0.428 m |
| 每视角 30 m 内深度中位误差的平均值 | 0.086 m | 0.121 m |

这里的深度是多层透明高斯合成的期望深度，包含遮挡边缘和动态卡车，不等同于原生点云表面误差或轨迹精度；其平均值改善并不表示所有深度指标均改善。固定中心的几何检查和保留的原生 PCD，才用于确认本次没有通过移动几何换取画面指标。初始化基线是未训练高斯，不能解读为原 LIO/LIVO 的定位精度基线。

端到端轮次 `xingye_13_gaussian_2` 已实际完成 45 秒回放、86 个高斯相机帧、3 个子图、536616 个高斯的合并和原生 8 块 PCD 保存；它是固定中心最终参数之前的集成验证。首个集成轮次在开发期接口切换中遇到旧二进制参数不匹配，被正确标为失败，原轨迹/PCD 保留；后续加入二进制版本预检查。

最终默认参数的端到端证据以 `results/xingye_13_gaussian_3/` 为准，详见其中 `round.json`、`gaussian.log`、`gaussian_gpu.csv` 与高斯目录的 `map_info.json`。验证范围是这个真实 45 秒录制窗口及单独的 20 秒算法迭代窗口，未声称完成所有矿区或整段 1020 秒数据的画质验收。

最终默认轮次已完成：426 个原生存档帧、86 个高斯相机帧、3 个子图、537798 个高斯，高斯阶段 40.44 s，整轮 GPU 显存采样峰值 2145 MiB。11 个留出视角平均全图 PSNR 10.35→20.09 dB、SSIM 0.212→0.758，激光覆盖邻域 PSNR 11.35→24.46 dB。全范围渲染深度 MAE 0.697→0.512 m，30 m 内 MAE 0.370→0.307 m；30 m 内中位误差的视角均值 0.061→0.089 m，说明并非所有深度统计均改善。未据此声称新的几何精度达标。

合并 PLY 的 59 个浮点属性全部有限，计数/文件长度正确；合并中心 PCD 完整且与 PLY 坐标逐点一致；实际最大尺度 0.5 m，实际中心学习率 0。原生 8 个 PCD 分块和标准全局几何/RGB 地图均保存，标准地图分别含 2204455 / 2018403 个点。Studio 的实际结果读取确认了高斯数、产品链接及渲染列表。验证摘要为 `results/xingye_13_gaussian_3/validation.json`，该轮次已在结果管理中保护，避免自动清理其完整 PCD。


## 2026-11-07 完整录制验证

已完整处理星业 13 号车 1020.12 秒录制，复用同一 `11a0d14` 估计器的 10177 帧最终位姿/原生扫描存档。2036 个相机采样帧覆盖 1017.50 秒有效区间（0.5 秒间隔），1782 帧训练、254 帧留出，51 子图共 13944637 个高斯。全程高斯解码/配对/训练/导出耗时 1068.2 秒，训练显存采样峰值 5275 MiB。原有 126 块完整 PCD 与全局几何/RGB 地图保留，结果已保护。

**完整全局图尚未达到高精细外观验收要求。** 254 个留出视角的单子图平均全图 PSNR 15.107 dB、SSIM 0.593；同一组22个视角从单子图 SSIM 0.594 降到直接拼接后的 0.387。实际存在黑色天空/远景缺失、灯头蓝色光晕、路面扫描线状波纹，跨子图重复叠加进一步恶化。不能用接入阶段的单子图短窗口指标替代完整合并图指标。

额外做了只保留空间单元最近可观测子图的合并试验，平均 SSIM 提升到 0.445，但22视角中9个退化（包括起步厂房），因此没有将其替换为正式输出，也未纳入默认建图算法。失败试验数据保留作分析证据。下一步应在高斯层处理可见性/表面支持与跨子图外观一致性，再处理天空/无深度区域损失和地面法向协方差，保留现有位姿链路。

完整文件：`results/xingye13_livo_1105/Log/fusion/1793847427156913686/gaussian/point_cloud.ply`（约3.065 GiB）。

分析入口：`results/xingye13_livo_1105/full_gaussian_analysis/analysis.md`（中文结论）及 `report.html`（曲线、全程代表/低分视角、22视角合并重渲染对照）。另有 `quality_summary.json`、`merge_comparison.json`、`execution.json` 和完整运行日志。

当前电脑可用区域/子图方式浏览：

```bash
bash scripts/view_gaussian.sh results/xingye13_livo_1105/full_gaussian_analysis/preview_regions/start
bash scripts/view_gaussian.sh results/xingye13_livo_1105/Log/fusion/1793847427156913686/gaussian/submap_0025
```

`preview_regions/start|middle|end` 是从完整合并图选取的62 m空间区域，保留原高斯参数；完整地图没有裁剪替换。各子图已附对应 `camera_poses.json`，查看器可直接恢复车辆视角。

本次新增只读重渲染入口（载入本项目导出的SH3二进制PLY，输入相机/图像/深度沿用配对阶段的格式）：

```bash
build/gaussian/my_livo_gaussian --render CAMERA_INPUT_DIR SAVED_MODEL.ply NEW_OUTPUT_DIR
/usr/bin/python3 scripts/analysis/gaussian_full_report.py results/xingye13_livo_1105 --render-merged
```

`--render-merged` 需要预先保留在 `full_gaussian_analysis/replay_inputs` 的配对输入，本次已保留22个全程分布视角。该重渲染不训练、不改位姿或高斯参数，并以单子图重载前后指标完全一致验证了解码约定。


## 2026-11-08 完整共享表面地图

复用星业13完整1020.12秒录制的同一原生存档，2036个相机帧、1782训练/254留出、114个工作批次，导出15204057个固定中心高斯。位姿和时间戳逐帧等于上一版完整地图，标定等于原生来源版本 `11a0d14`。只调整高斯建图及其查看/输出；新结果无 LIO/LIVO PCD。

| 254 个相同留出视角的平均指标 | 旧完整拼接图 | 新共享地图 |
|---|---:|---:|
| 全图 SSIM ↑ | 0.4448 | 0.5446 |
| 激光覆盖邻域 PSNR ↑ | 15.719 | 18.756 |
| 全图 PSNR ↑ | 14.137 | 15.839 |
| 30 m 内渲染深度 MAE / m ↓ | 0.863 | 0.433 |
| 全范围渲染深度 MAE / m ↓ | 1.599 | 1.098 |
| 30 m 内深度中位误差的视角均值 / m ↓ | 0.446 | 0.146 |
| 有真实深度处的渲染深度覆盖率 ↑ | 1.0000 | 0.9861 |

SSIM 改善 228/254 视角，26 个退化，尤其回程厂房近景存在明显条纹/亮斑；退化清单和截图同样保留在分析目录。双方从完整保存 PLY 重渲染，相机/图像/深度/背景相同；旧图52视角复用同条件真实缓存，202视角本次补跑。覆盖率有所降低，且个别深度指标仍会退化，不能据此宣称独立厘米级定位或灯柱高度指标已重新验收。共享训练、导出及新图完整重渲染累计约81.6分钟（复用已准备种子，不含此前准备/排查和旧图对照），训练显存采样峰值5747MiB。

结果：`results/xingye_13_gaussian_4/`；中文分析 `analysis/analysis.md`，254视角对照 `analysis/report.html`，重点视角 `analysis/key_views.png`。完整PLY和高斯中心PCD在 `Log/fusion/1793847427156913686/gaussian/`，已保留训练/恢复日志和参数一致性验证。默认轮次浏览加载起点区域，中段/终点在 `analysis/preview_regions/middle|end`；完整地图独立保留。仍有局部黑点、车辆边缘残留及未观测远景，不宣称照片级重建。
