# MY-LIVO 后端坐标系与时间规范

本文档冻结后端接入时使用的位姿、点云和时间语义。除非后续阶段明确更新，所有
Keyframe、Pose Graph、Loop、RTK 和 Global Map 代码都必须遵守本规范。

## 1. 变换方向

统一记号 `T_A_B` 表示把 B 坐标系中的点转换到 A 坐标系：

```text
p_A = T_A_B * p_B
T_A_C = T_A_B * T_B_C
```

后端 `Pose3d` 的 `rotation` 和 `translation` 满足：

```text
p_parent = rotation * p_child + translation
```

不得使用变量名 `pose` 隐藏变换方向；跨模块接口必须使用 `T_A_B` 形式。

## 2. 当前实际坐标系

### body

FAST-LIVO2 的 `_state.rot_end`、`_state.pos_end` 表示估计器 body（IMU 中心）
到当前估计器世界的变换。Truck 29 默认
`reference_frame_conversion.enabled=true`，多雷达点先进入后轴 RFU，再通过
`extR/extT` 移到物理 IMU 中心。因此本项目后端中的 `body` 是物理 IMU 中心，
轴向为 RFU（x 右、y 前、z 上）。

若关闭该配置，`body` 退化为 FAST-LIVO2 配置的估计器 body；同一次运行中不得
切换。

### frontend odom

当前源码历史上把 FAST-LIVO2 世界坐标命名为 `mine`。标准 RFU 数据完成 IMU
初始化时，`initializeMineFrame()` 使用此前 CGI-610 Odometry 样本对 `_state`
做一次世界系左变换。因此它在数值上从矿区 ENU 初值开始，但后续只由
FAST-LIVO2 连续积分和雷达更新，语义上仍是可能漂移的前端 odom。

第一阶段定义：

```text
T_odom_body := (_state.rot_end, _state.pos_end)
```

为了不破坏已经验证的 LIO 输出，本阶段现有 `/aft_mapped_to_init`、`/path`、
`/cloud_registered` 和 TF 仍使用 frame id `mine`。新增原始关键帧调试话题也
暂用参数 `backend.frontend_frame_id=mine`。Pose Graph 使用独立的优化变量和
话题，而不是突然修改前端状态或体素地图。Odometry-only 验收期间优化话题仍
配置为 frame id `mine`，保证它与原始路径处于同一棵 TF 树；只有 Loop/RTK
提供全局修正以后才新增严格的 `map -> odom`。

### map

在 Odometry-only PGO 阶段，初始化为：

```text
T_map_body(k) = T_odom_body(k)
```

加入 Loop/RTK 后，`T_map_body(k)` 是每个关键帧独立的优化变量。实时修正只由
最新已优化关键帧计算：

```text
T_map_odom = T_map_body(k) * inverse(T_odom_body(k))
```

历史轨迹和全局地图必须逐关键帧使用 `T_map_body(k)` 重建，不能套一个固定的
`T_map_odom` 重算全部历史。

## 3. 点云规范

FAST-LIVO2 的 `feats_down_body` 名称容易误解：在 Truck 29 多雷达模式中，它
仍位于合成虚拟雷达/后轴局部系。进入 KeyframeManager 前必须执行：

```text
p_body = extR * p_virtual_lidar + extT
```

关键帧只保存 `pcl::PointXYZI` 的 `cloud_body`，不保存已经乘过旧 world/map
位姿的点云。当前后端另做一次可配置体素降采样，默认叶长 0.5 m。原 FAST-LIVO2
点云、VIO 数据和体素地图均不被修改。

调试话题 `/backend/keyframe_cloud_raw` 为方便 RViz 显示，会临时使用该关键帧的
`T_odom_body` 把存储的 `cloud_body` 转到前端 frame；这不改变内部存储语义。

## 4. 协方差顺序

FAST-LIVO2 状态协方差的前六维为：

```text
[rotation, position]
```

后端 `Matrix6d` 统一采用 Miao/g2o 风格：

```text
[position, rotation]
```

Keyframe 接入层负责交换对应块和交叉项。后端不得再次交换。

## 5. 时间规范

所有传感器和后端内部时间均使用 ROS 消息 `header.stamp` 对应的 Unix 秒：

```text
double timestamp = sec + nanosec * 1e-9
```

关键帧时间严格使用：

```text
LidarMeasures.last_lio_update_time
```

它是当前完整 LIO 更新使用的传感器时间，不是 callback 到达时间，也不是
`node->now()`。KeyframeManager 强制观测时间严格递增，违反时抛出逻辑错误，
防止错误时间进入后续图优化。

CGI-610 后续接入同样使用 Odometry/DiagnosticArray 的 `header.stamp`。状态与
位姿只在有时限的缓冲区中关联；禁止让旧的 `ins_pos_mode=4` 无限延续。

## 6. 当前阶段输出

```text
/backend/keyframe_path_raw   nav_msgs/Path
/backend/keyframe_cloud_raw  sensor_msgs/PointCloud2（仅最新关键帧）
/backend/keyframe_path_optimized nav_msgs/Path
/backend/odometry_optimized  nav_msgs/Odometry
Log/backend/keyframes.csv    原始关键帧元数据与协方差标准差
Log/backend/pose_graph.csv   图规模、求解状态、耗时及 raw/optimized 位姿
```

本阶段只有相邻里程计边，没有回环、RTK factor、全局地图或 `map -> odom`
修正，因而应满足：

```text
现有 FAST-LIVO2 轨迹和点云数值不变
关键帧 id 从 0 连续递增
关键帧 timestamp 严格递增
T_map_body 初始值等于 T_odom_body
图节点数等于关键帧数
里程计边数等于关键帧数减一
优化轨迹在数值误差内等于原始关键帧轨迹
```
