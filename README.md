# FAST-LIVO2

## MY-LIVO：芜湖矿山 Truck 29（ROS 2 Humble）

本分支已适配 Ubuntu 22.04 / ROS 2 Humble 和当前 MCAP 数据。运行入口、话题、标定和 RViz2 配置均以本节为准；下方内容保留为上游 FAST-LIVO2 参考说明。

### 数据接口与处理边界

- 激光雷达：`/front_lidar`，`sensor_msgs/msg/PointCloud2`。数据字段为 `x/y/z/intensity/ring/timestamp`；消息头是首点/扫描起始时刻，`timestamp` 是从首点起算的 `float32` 相对秒。输入适配层直接使用 RTF 中每点的 10 us 偏移做去畸变；只有读取旧包且字段不合法时才按点序退化估算。
- IMU：`/imu_data`，`sensor_msgs/msg/Imu`，坐标系 `imu_link_rfu`（x 右、y 前、z 上）。当前转换器已输出 raw 角速度 `rad/s` 和包含完整姿态投影重力响应的 raw 比力 `m/s²`；输入适配层不再换轴、重复转换角速度或补重力。标准四元数只用于初始化矿山坐标朝向，不作为 LIO 观测融合。
- 组合导航：`/imu_data/odometry`，`nav_msgs/msg/Odometry`，父系 `ins_local_enu`、子系 `vehicle_rear_axle_rfu`。其中 x/y 严格为 `st_point_3d.x/y`，z 严格为 `f_pos_alt`。默认启用 `rear_axle_to_imu`，输入层按标定杆臂将参考轨迹和 LiDAR 外参同步转换到物理 IMU 点。位置只用于初始化完成时的静止均值锚定和 RViz2 同步参考轨迹，不进入 IMU 传播或 IEKF 更新。
- RTK 状态：`/imu_data/ins_status`，仅当 `ins_pos_mode` 精确为 `4` 时，后端才允许把同一时段的组合导航位置插值到关键帧时刻并生成稀疏位置因子。
- 相机压缩流：`/midrange_camera/ffmpeg`，HEVC；启动文件用 `image_transport` 解码为 `/midrange_camera/image_raw`（实测 `1920x1080 bgr8`）。

主要输出为 `/cloud_registered`、`/aft_mapped_to_init`、`/path`、`/imu_reference_odom` 和 `/imu_reference_path`，坐标系均为 `mine`。RViz2 中 LIVO 与 IMU/RTK 参考轨迹按当前 LIVO 时间戳同步推进。

当前后端从完整 LIO 更新中选取关键帧，使用持久化 GTSAM iSAM2 联合相邻
里程计边、经多分辨率 NDT 与一致性验证的鲁棒回环边，以及状态 4 的稀疏鲁棒
CGI-610 位置因子。输出包括 `/backend/keyframe_path_raw`、
`/backend/keyframe_path_optimized`、`/backend/odometry_optimized`、
`/backend/verified_loops` 和 `/backend/global_map_optimized`。后端以 `odom` 表示
连续前端轨迹、以 `mine` 表示全局轨迹，并发布 `mine -> odom`；任何修正都不会
回写 IEKF。全局地图由独立线程按 60 m 瓦片增量维护，退出时保存精确重建的
`Log/backend/optimized_global_map.pcd`。

### 编译

不要进入 Conda 环境。依次执行：

后端 Pose Graph 使用项目内 `3rdparty/gtsam` 的 GTSAM 4.2.2 iSAM2。构建脚本
会在首次编译时自动生成与 ROS/PCL 相同的 system-Eigen 版本，输出到
`3rdparty/gtsam/install-system-eigen`，不会覆盖已有 GTSAM build/install。

```bash
cd /home/project/MY-LIVO2.0
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
bash scripts/build_ros2_humble.sh
source install/setup.bash
```

launch 默认直接读取源码目录中的
`/home/project/MY-LIVO2.0/config/wuhu_truck29.yaml`。以后修改并保存该 YAML
只需重启 launch，无需重新编译；也可用 `config_file:=/绝对路径/file.yaml`
指定另一份配置。

### 建图与 RViz2

原始 MCAP 没有消息索引，且每个感知包还包含多个本算法不用的大流量雷达话题。首次运行前先生成一个按时间严格归并、带索引且只保留前雷达、IMU、组合导航 Odometry 和可选相机话题的运行包（原始包不会被修改）：

```bash
cd /home/project/MY-LIVO2.0
./scripts/prepare_wuhu_bag.py
```

默认输入为 `/home/project/data/haibo/huaining/03/ros2bag`，默认输出为相邻的
`ros2bag_my_livo`。已存在时脚本会拒绝覆盖；只需要 LIO 数据的副本时可另行
指定输出并加 `--lio-only`。

终端 1：

```bash
cd /home/project/MY-LIVO2.0
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_livo wuhu_truck29.launch.py use_camera:=false use_rviz:=true
```

以上命令是 LiDAR-IMU 模式。完整 LIVO 使用 `use_camera:=true`。RViz2 配置文件为 `rviz_cfg/wuhu_truck29.rviz`。

程序已内置当前数据的 LiDAR 时间契约：消息头是扫描起始时刻、逐点
`timestamp` 是相对秒，不再提供 `lidar_header_is_scan_end` 开关。组合导航位置
属于后轴，因此 `rear_axle_to_imu:=true`。实测三个 MCAP 共使用 34 个稀疏
`ring` ID、最大为 126，因此 `scan_line` 按 ID 上界配置为 128，而不是误填 34。

终端 2 播放完整 LIVO 输入：

```bash
cd /home/project/MY-LIVO2.0
./scripts/play_all_bags.sh --with-camera
```

仅播放 LIO 输入：

```bash
./scripts/play_all_bags.sh --lio-only --start-offset 0
```

例如从整段数据第 45 秒开始、以 0.5 倍速播放：

```bash
./scripts/play_all_bags.sh --start-offset 45 --rate 0.5
```

脚本优先以单个 `ros2 bag play` 进程读取上述索引运行包，从而避免分包进程启动抖动、跨话题乱序和大点云丢帧。`--start-offset` 是相对于整段 90 秒数据的偏移；从中段开始可用于定位和可视化检查，但新建地图时仍应从包含静止初始化的开头播放，否则任何纯惯性 LIO 都无法得到可靠的初始重力和偏置。可用 `--dry-run` 检查命令；`--all-topics` 只建议诊断时使用。

当前数据的 rosbag2 发布端为 RELIABLE QoS，Truck 29 配置已匹配为可靠订阅，并扩大离线大点云缓存。若以后直接连接只提供 BEST_EFFORT 的实时传感器驱动，应把 `mine.reliable_input_qos` 改为 `false`。

### 标定

统一标定文件为 `config/perception_truck29.json`，其中包含：

- 原始前雷达→后轴、前雷达→中距相机，以及由杆臂自动计算的前雷达/中距相机→物理 IMU 齐次变换；
- 用户提供的中距相机和鱼眼相机内参；
- `equation_10m` 至 `equation_100m`、最大距离/间隔和焦距参数。

物理 IMU 外参使用以下关系自动生成：

```text
T_imu_lidar  = T_imu_rear_axle * T_rear_axle_lidar
T_imu_camera = T_imu_rear_axle * T_rear_axle_lidar * inverse(T_camera_lidar)
```

重新计算并写回时执行：

```bash
./scripts/generate_mine_calibration.py config/perception_truck29.json
```

距离分段方程已完整保存在相机标定项下，但它不是 OpenCV 的标准五参数畸变模型，且给定 `distcoeff` 全为零；在没有明确方程输入/输出定义的情况下不把它强行套入图像像素校正，以免改变原算法投影逻辑。当前 VIO 使用用户提供的针孔内参和零标准畸变系数。

### LIO 验证结果

当前三段感知包分别包含 `2999/3000/3000` 帧 IMU 和同数量、同纳秒时间戳的组合导航 Odometry。旧数据格式下得到的精度数值不再适用于本次 raw RFU IMU，完成新格式全程运行后应重新生成指标；评估脚本现在直接读取 `/imu_data/odometry`，不再从 IMU covariance 中解析伪装的位姿字段。

需要复核当前轨迹时执行：

```bash
./scripts/evaluate_wuhu_lio.py --rear-axle-to-imu
```

评估参数必须与 launch 中的 `rear_axle_to_imu` 保持一致；关闭该 launch
开关时，评估脚本也不要传入 `--rear-axle-to-imu`。

完整 Loop + RTK + 全局地图验收命令为：

```bash
./scripts/validate_backend_keyframes.py \
  --trajectory Log/result/wuhu_truck29.txt
./scripts/validate_loop_candidates.py
./scripts/validate_loop_registration.py
./scripts/validate_loop_verification.py
./scripts/validate_loop_pose_graph.py
./scripts/validate_rtk_fusion.py
./scripts/validate_optimized_global_map.py
```

反斜杠续行后必须立即跟下一行参数；不要把
`Log/result/wuhu_truck29.txt` 拆成两个 shell 命令。多分辨率 NDT 和瓦片地图均在
独立后台线程运行；进程退出时会先排空回环队列，再完成最终全图重建和日志关闭，
因此必须让 mapping 节点正常退出后再运行验证器。`validate_odometry_pose_graph.py`
仅用于关闭 Loop/RTK 的消融模式；联合模式本来就应当改变 raw 轨迹，不能再用
“optimized/raw 重合”作为通过条件。

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

### 📢 News

- 🔓 **2025-01-23**: Code released!  
- 🎉 **2024-10-01**: Accepted by **T-RO '24**!  
- 🚀 **2024-07-02**: Conditionally accepted.

### 📬 Contact

For further inquiries or assistance, please contact [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk).

## 1. Introduction

FAST-LIVO2 is an efficient and accurate LiDAR-inertial-visual fusion localization and mapping system, demonstrating significant potential for real-time 3D reconstruction and onboard robotic localization in severely degraded environments.

**Developer**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)

<div align="center">
    <img src="pics/Framework.png" width = 100% >
</div>

### 1.1 Related video

Our accompanying video is now available on [**Bilibili**](https://www.bilibili.com/video/BV1Ezxge7EEi) and [**YouTube**](https://youtu.be/6dF2DzgbtlY).

### 1.2 Related paper

[FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)  

[FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)  

[FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

[FAST-Calib: LiDAR-Camera Extrinsic Calibration in One Second](https://www.arxiv.org/pdf/2507.17210)

### 1.3 Our hard-synchronized equipment

We open-source our handheld device, including CAD files, synchronization scheme, STM32 source code, wiring instructions, and sensor ROS driver. Access these resources at this repository: [**LIV_handhold**](https://github.com/xuankuzcr/LIV_handhold).

### 1.4 Our associate dataset: FAST-LIVO2-Dataset
Our associate dataset [**FAST-LIVO2-Dataset**](https://connecthkuhk-my.sharepoint.com/:f:/g/personal/zhengcr_connect_hku_hk/ErdFNQtjMxZOorYKDTtK4ugBkogXfq1OfDm90GECouuIQA?e=KngY9Z) used for evaluation is also available online.

### 1.5 Our LiDAR-camera calibration method
The [**FAST-Calib**](https://github.com/hku-mars/FAST-Calib) toolkit is recommended. Its output extrinsic parameters can be directly filled into the YAML file. 

## 2. Prerequisited

### 2.1 Ubuntu and ROS

Ubuntu 18.04~20.04.  [ROS Installation](http://wiki.ros.org/ROS/Installation).

### 2.2 PCL && Eigen && OpenCV

PCL>=1.8, Follow [PCL Installation](https://pointclouds.org/). 

Eigen>=3.3.4, Follow [Eigen Installation](https://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV>=4.2, Follow [Opencv Installation](http://opencv.org/).

### 2.3 Sophus

Sophus Installation for the non-templated/double-only version.

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 2.4 Vikit

Vikit contains camera models, some math and interpolation functions that we need. Vikit is a catkin project, therefore, download it into your catkin workspace source folder.

```bash
# Different from the one used in fast-livo1
cd catkin_ws/src
git clone https://github.com/xuankuzcr/rpg_vikit.git 
```

## 3. Build

Clone the repository and catkin_make:

```
cd ~/catkin_ws/src
git clone https://github.com/hku-mars/FAST-LIVO2
cd ../
catkin_make
source ~/catkin_ws/devel/setup.bash
```

## 4. Run our examples

Download FAST-LIVO2-Dataset from [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA) Section IV.

```
roslaunch fast_livo mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```


## 5. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact me at <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk> to discuss an alternative license.
