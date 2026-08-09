# FAST-LIVO2

## MY-LIVO：芜湖矿山 Truck 29（ROS 2 Humble）

本分支已适配 Ubuntu 22.04 / ROS 2 Humble 和当前 MCAP 数据。运行入口、话题、标定和 RViz2 配置均以本节为准；下方内容保留为上游 FAST-LIVO2 参考说明。

### 数据接口与处理边界

- 激光雷达：`/front_lidar`，`sensor_msgs/msg/PointCloud2`。数据字段为 `x/y/z/intensity/ring/timestamp`；包内 `timestamp` 是低精度整秒值。该雷达是约 124° 视场、10 Hz 的固态扫描器，输入适配层按消息中的采样顺序重建 0–100 ms 帧内相对时间，只用于原算法的去畸变。
- IMU：`/imu_data`，`sensor_msgs/msg/Imu`。按照 `clip_converter.cpp` 的打包约定解析 pitch、roll、heading、局部平面坐标和高度。陀螺仪由 deg/s 转为 rad/s；对已去重力的融合加速度完成车辆系到标定 IMU 系的轴变换并恢复机体系重力后，仍由原 FAST-LIVO2 IMU 模型融合。
- `/imu_data` 中的 RTK/INS 位置不进入后续状态传播或观测更新。它只用于：在原算法 IMU 初始化完成的准确时刻，对此前局部位姿求均值并设置一次 `mine` 初始世界位姿；以及发布同步参考轨迹供 RViz2 对照。
- 相机压缩流：`/midrange_camera/ffmpeg`，HEVC；启动文件用 `image_transport` 解码为 `/midrange_camera/image_raw`（实测 `1920x1080 bgr8`）。

主要输出为 `/cloud_registered`、`/aft_mapped_to_init`、`/path`、`/imu_reference_odom` 和 `/imu_reference_path`，坐标系均为 `mine`。RViz2 中 LIVO 与 IMU/RTK 参考轨迹按当前 LIVO 时间戳同步推进。

### 编译

不要进入 Conda 环境。依次执行：

```bash
cd /home/project/MY-LIVO
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
export PATH=/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 建图与 RViz2

原始 MCAP 没有消息索引，且每个感知包还包含多个本算法不用的大流量雷达话题。首次运行前先生成一个按时间严格归并、带索引且只保留所需话题的运行包（原始包不会被修改）：

```bash
cd /home/project/MY-LIVO
./scripts/prepare_wuhu_bag.py
```

默认输出为 `/home/project/data/haibo/wuhu_livo/ros2bag_my_livo`，已存在时脚本会拒绝覆盖。只需要 LIO 数据的副本时可另行指定输出并加 `--lio-only`。

终端 1：

```bash
cd /home/project/MY-LIVO
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONHOME PYTHONPATH
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_livo wuhu_truck29.launch.py use_camera:=false use_rviz:=true
```

仅验证 LiDAR-IMU 时，可改为 `use_camera:=false`。RViz2 配置文件为 `rviz_cfg/wuhu_truck29.rviz`。

终端 2 播放完整 LIVO 输入：

```bash
cd /home/project/MY-LIVO
./scripts/play_all_bags.sh --start-offset 0
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

- LiDAR→IMU、LiDAR→中距相机，以及自动计算的中距相机→IMU 齐次变换；
- 用户提供的中距相机和鱼眼相机内参；
- `equation_10m` 至 `equation_100m`、最大距离/间隔和焦距参数。

相机→IMU 使用以下关系自动生成：

```text
T_imu_camera = T_imu_lidar * inverse(T_camera_lidar)
```

重新计算并写回时执行：

```bash
./scripts/generate_mine_calibration.py config/perception_truck29.json
```

距离分段方程已完整保存在相机标定项下，但它不是 OpenCV 的标准五参数畸变模型，且给定 `distcoeff` 全为零；在没有明确方程输入/输出定义的情况下不把它强行套入图像像素校正，以免改变原算法投影逻辑。当前 VIO 使用用户提供的针孔内参和零标准畸变系数。

### LIO 验证结果

ROS 2 Release 构建已通过。索引运行包包含 900 帧前雷达和 8999 帧 IMU；1 倍速全程测试得到 876 个初始化后 LIO 位姿，无时间回退和零有效特征帧，首帧矿山位姿使用初始化结束前 234 帧静止 INS 均值。相对包内 RTK/INS 参考，矿山坐标系位置 RMSE 为 1.79 m、P95 为 3.91 m、末端误差为 5.23 m；姿态 RMSE 为 2.37°、末端误差为 5.34°。RTK 位置只参与首帧锚定和参考显示，没有进入滤波传播或更新。

需要复核当前轨迹时执行：

```bash
./scripts/evaluate_wuhu_lio.py
```

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
