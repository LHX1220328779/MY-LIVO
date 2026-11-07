# 高斯建图

新增模式名为 `gaussian`，与 `lio`、`livo` 并列。Studio 首页与建图页均可选择。它使用原有 LIVO 估计与 RTK 最终校正，然后运行独立 Gaussian-LIC 建图程序。现有估计器源码、外参、RTK 因子及 LIO/LIVO 参数均不修改。也可以直接读取已有 LIO/LIVO 轮次，完全跳过位姿估计的重跑。

## 环境与构建

本机使用 `/opt/venvs/my-livo-gaussian` 的 PyTorch 2.5.1+cu124、`/opt/cuda-12.4` 编译器和 RTX 4060 8 GB。原有 `data_process` 环境没有 PyTorch，因此另建环境。系统 ROS Python、OpenCV 与原建图程序不变。CUDA 库复用 PyTorch 安装内容。

本机依赖已配置；重新部署需先安装系统 `python3-venv`、`libglm-dev`，并具备原项目的 OpenCV/Eigen/CMake/ROS 与 FFmpeg：

```bash
bash scripts/setup_gaussian.sh
bash scripts/build_gaussian.sh
```

`GAUSSIAN_ENV`、`GAUSSIAN_CONDA`、`CUDA_HOME` 可改变依赖安装位置；构建时用 `GAUSSIAN_PYTHON` 指定 Python。`GAUSSIAN_CUDA_ARCH` 默认为本机的 `89`，其他 GPU 需指定相应架构。默认单任务编译，避免 LibTorch/CUDA 编译占满内存。构建目录独立为 `build/gaussian`，不向原 ROS CMake 增加 CUDA 依赖。`image_io.cpp` 将系统 OpenCV 的路径字符串留在其自身 ABI 内，兼容 pip LibTorch 的旧字符串 ABI。

## 运行

运行一个新轮次（数据、标定仍按原流程选择）：

```bash
bash scripts/run_wuhu_round.sh my_gaussian_1 --mode gaussian \
  --dataset /path/to/ros2bag --calibration /path/to/calibration.json
```

`--duration`、`--start-offset`、`--no-rtk` 等原选项仍可使用。高斯模式需要相机与扫描存档，不能搭配 `--no-save-map`。原生完整 PCD、地图导出、轨迹和评估按原流程保留。高斯建图失败时轮次失败且保留已完成的原生地图与轨迹，不将失败伪装为成功。

从已有结果生成高斯地图，不重跑估计器：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 tools/livo/gaussian_map.py results/<已有轮次>
```

默认从轮次清单读取原始录制，从运行的 `parameters.txt` 读取实际标定路径和相机偏移。必要时指定 `--dataset`、`--calibration`。输出默认在 `Log/fusion/<run>/gaussian/`；重复实验使用新的 `--out`，避免覆盖已有产品。

调节建图质量及资源使用：

```bash
/usr/bin/python3 tools/livo/gaussian_map.py results/<轮次> \
  --out results/<轮次>/gaussian_high \
  --width 960 --interval 0.5 --max-depth 120 \
  --submap-frames 40 --iterations 40 --refine-passes 20
```

默认宽 640、每 0.5 秒一帧、每 40 帧一个子图、每个子图最多 600000 个高斯，15 轮末尾精修，激光最远 120 m。深度损失权重默认 0.1、尺度标准差上限 0.5 m（`--depth-weight` / `--max-scale`）。子图逐个训练释放显存，可覆盖长录制；总输出大小随场景增长。可用 `--start`、`--duration` 指定已有轮次内的时间窗口，单位秒、相对首个存档扫描。`--prepare-only` 仅准备固定相机位姿、RGB、深度和初始化点，`--keep-inputs` 保留这些中间文件。

## 数据与算法边界

1. 使用原始 HEVC 的彩色图像，按标定内参缩放并去畸变；使用实际 `effective.img_time_offset_sec`。在原生存档位姿间插值，禁止跨缺失数据外推。
2. 相机姿态链为 `T_native_imu * T_imu_rear * T_rear_camera`，再左乘最终 `C(t_image)`。扫描点复用 `map_export.py` 的逐点变换：原生扫描末位姿、LiDAR 外参、点时间 `C(t_point)`。相机与高斯统一处于最终地图坐标系。
3. 使用真实 LiDAR 深度及 Z-buffer，遮挡表面不取前景颜色。迁入原 Gaussian-LIC 的 SH 高斯、可微 CUDA 渲染、alpha 增量扩展、稀疏 Adam、RGB/SSIM/深度约束。相机位姿不进入可训练参数，不回传估计器。默认 `--position-lr 0` 锁定已校正 LiDAR 初始化的高斯中心，优化尺度、方向、透明度和 SH 颜色；显式设置正学习率才允许高斯中心移动，始终不改变相机位姿或原有地图。
4. 每八帧留出一帧，只用于渲染评估，不加入训练相机或初始化点。`initial` 是所有训练帧 LiDAR/RGB 初始化的未训练高斯模型；`trained` 是训练完成的模型。PSNR 同时记录全图值和 LiDAR 有效像素九像素邻域内的值，SSIM 为全图，深度 MAE 只计算双方有深度的位置，并单独记录覆盖率及 30 m 内深度 MAE/中位数，避免把扩展远景范围带来的深度变化误当成近场退化。
5. 这是最终轨迹完成后的建图阶段，未声称在线实时。没有接入另一套里程计、相机位姿优化、学习深度补全或 TensorRT/SPNet 权重。相机视场外、超距、天空和动态物体缺少稳定的 LiDAR 几何支持；子图之间也未新增全局外观优化。高斯效果不代表独立验证了全场景厘米级定位指标。

## 产品

- `point_cloud.ply`：合并的标准 SH 高斯参数，可用支持 3D Gaussian Splatting 的查看器打开；普通点云查看器不能显示其完整外观。
- `gaussian_centers.pcd`：完整高斯中心与 DC 颜色，便于普通点云工具查看。原有 `map_*.pcd` 和 `map/global_map*.pcd` 是保留的原生几何产品。
- `submap_*/trained_*.png`、`initial_*.png`、`reference_*.png`：留出视角的训练后、初始化及真实图像。
- `submap_*/metrics.csv`、`map_info.json`：每视角质量数据、实际选项与数据来源。
- `camera_poses.json`：仅供高斯建图使用的最终相机位姿和匹配时间，不修改原有轨迹。

Studio 结果中的“高斯地图”显示 PLY/PCD/元信息链接及留出视角渲染预览。

## 浏览器查看与 Linux 黑屏

推荐使用专用启动命令：

```bash
bash scripts/view_gaussian.sh results/xingye_13_gaussian_3
```

参数可以换成其他轮次目录、具体 `point_cloud.ply` 或 `.ssproj` 工程。目录内有 `.ssproj` 时优先恢复其显示方向和初始视角；直接打开本项目 PLY 时，根据 `camera_poses.json` 自动选取车辆视角，并将 Z 向上的地图调整为浏览器导航使用的 Y 向上。这只是显示变换。

首次使用自动在 `build/gaussian/viewer` 安装固定版本 Playwright，并准备其 Chromium；需要 Node.js >=20.19、npm 和网络。本机已配置完成。后续复用该专用浏览器进程，打开其他地图只需再次运行同一命令并更换参数。也可在专用窗口使用“文件 → 打开”恢复 `.ssproj`、“文件 → 导入”选择高斯 PLY。保存为 `.ssproj` 可保留初始视角。

本机已对比实际有界面的 Chromium 启动方式：普通启动能访问 `navigator.gpu`，但 `requestAdapter()` 返回 `null`；专用配置返回 NVIDIA/Lovelace，53.8 万高斯可渲染并支持鼠标导航。当前环境的默认 Vulkan 清单目录缺少 NVIDIA 条目，因此启动器在自己的运行目录生成 NVIDIA ICD 清单，为该浏览器进程配置 Vulkan/WebGPU。普通 Chrome 窗口不会继承这些启动参数，单独在普通窗口再次打开网页可能仍然黑屏。对照记录在 `results/xingye_13_gaussian_3/browser_gpu_comparison.json`。

网格也是 GPU 渲染内容，因此“网格也没有”应先检查浏览器图形链路；PLY 加载完成不代表 WebGPU 渲染设备初始化成功。排查其他浏览器时检查 `chrome://settings/system` 的图形加速、`chrome://gpu` 的 WebGPU/驱动信息，以及控制台中 `navigator.gpu.requestAdapter()` 是否成功。仅检查是否存在 `navigator.gpu` 不足以确认支持。

查看操作：左键拖动旋转、右键拖动平移、滚轮缩放、双击感兴趣的路面或墙面设置观察中心。首次查看用已保存的车辆视角；整体框选后的远距离鸟瞰视角通常看不到与训练视角相同的纹理细节。

此启动器使用独立配置目录，并且只调用本地文件导入接口。网络用于获取 SuperSplat 页面和首次安装查看依赖，不调用地图发布接口。系统 ROS、建图环境及普通浏览器设置不变。

第三方来源及保留的许可见 `gaussian/third_party/gaussian_lic/UPSTREAM.md` 和各源文件、许可文件。

## 本次实际验证

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
