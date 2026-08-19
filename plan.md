# FAST-LIVO2 + Loop Backend + CGI-610 RTK 高性能全局 SLAM 改造任务书

## 实施状态与实际代码校准（2026-08-19）

本节是结合 `/home/project/MY-LIVO2.0` 当前源码形成的实施记录；当它与后文的
初始任务拆分或进度要求冲突时，以本节和实际代码为准。仍坚持“一次完成一个可
独立验收的大模块”，但允许把强相关的小 Task 合并，减少反复编译和接口返工。

当前进度（本节覆盖后文仍保留的初始阶段性措辞）：

```text
Task 01--05 baseline/frame/keyframe/odom/candidate       已完成
Task 06--08 NDT registration/verification/robust Loop    已完成
Task 09--10 optimized map + mine->odom                   已完成
Task 11--18 CGI-610 status/solution/time/buffer/interp   已完成
Task 19--25 selector/GPSFactor/covariance/gate/robust    已完成
Task 26--28 LIO + Loop + RTK 联合 iSAM2                  已完成
Task 31 RTK 丢失/恢复                                    已完成单元验收
Task 32/40 后台 NDT、地图线程、增量 iSAM2                已完成
Task 41 瓦片化增量在线地图 + 最终精确重建                已完成
Task 42--48 日志/invariant/高频降采样/离群值测试          已完成
Task 14 NavSatFix ENU 转换                               当前数据链无需启用
Task 29/30/49 RTK yaw                                   按计划保留为可选二期
```

已落地文件与接口：

```text
docs/frame_convention.md
include/backend/keyframe.h
include/backend/keyframe_manager.h
src/backend/keyframe_manager.cpp
tests/keyframe_manager_test.cpp
scripts/validate_backend_keyframes.py
include/backend/pose_graph_optimizer.h
src/backend/pose_graph_optimizer.cpp
tests/pose_graph_optimizer_test.cpp
scripts/validate_odometry_pose_graph.py
scripts/build_gtsam_system_eigen.sh
docs/odometry_pose_graph.md
include/backend/loop_candidate_detector.h
src/backend/loop_candidate_detector.cpp
tests/loop_candidate_detector_test.cpp
scripts/validate_loop_candidates.py
docs/loop_candidate_detection.md
include/backend/loop_registration.h
src/backend/loop_registration.cpp
tests/loop_registration_test.cpp
scripts/validate_loop_registration.py
docs/loop_registration.md
include/backend/loop_verifier.h
src/backend/loop_verifier.cpp
tests/loop_verifier_test.cpp
scripts/validate_loop_verification.py
include/backend/pose_graph_optimizer.h
src/backend/pose_graph_optimizer.cpp
scripts/validate_loop_pose_graph.py
include/backend/rtk_observation_buffer.h
src/backend/rtk_observation_buffer.cpp
tests/rtk_observation_buffer_test.cpp
scripts/validate_rtk_fusion.py
include/backend/optimized_global_map.h
src/backend/optimized_global_map.cpp
tests/optimized_global_map_test.cpp
scripts/validate_optimized_global_map.py

/backend/keyframe_path_raw
/backend/keyframe_cloud_raw
/backend/keyframe_path_optimized
/backend/odometry_optimized
/backend/loop_candidates
/backend/loop_registrations
/backend/verified_loops
/backend/global_map_optimized
Log/backend/keyframes.csv
Log/backend/pose_graph.csv
Log/backend/loop_detection.csv
Log/backend/loop_candidates.csv
Log/backend/loop_registrations.csv
Log/backend/loop_registration_levels.csv
Log/backend/loop_verification.csv
Log/backend/loop_factors.csv
Log/backend/rtk_status.csv
Log/backend/rtk_solutions.csv
Log/backend/rtk_queries.csv
Log/backend/rtk_decisions.csv
Log/backend/rtk_factors.csv
Log/backend/optimized_trajectory.csv
Log/backend/global_map_updates.csv
Log/backend/optimized_global_map.pcd
```

最终联合实测（12 个连续 Perception MCAP，启动偏移 10 s）：447 个关键帧、31
次 NDT 配准、27 个通过验证并加入图的鲁棒回环因子、83 个由 35,000 条 CGI-610
solution 生成的稀疏 RTK 位置因子。RTK 残差中位数由 0.0536 m 降至 0.0242 m，
优化轨迹相对 raw LIO 最大修正 3.4231 m/2.9166 deg，相邻里程计边最大变化仅
0.0148 m/0.1073 deg。最终 3,616,075 个关键帧点经 0.75 m 体素生成 110,608
点全局 PCD，`mine -> odom` 复合误差为 `1.28e-13 m`。所有离线验证器通过。

长时性能补充实测：全局地图使用 60 m 瓦片并在独立线程构建；40 秒真实数据中
8 个 RTK 因子经修正阈值和 10 KF 去抖后，在线地图只执行初始全建、一次增量瓦片
更新和一次图修正全建，退出时再做最终精确全建。增量更新 20 个新关键帧/19 个
瓦片耗时 12.9 ms。普通 odometry 节点只提取最新 GTSAM 位姿，完整 Path 每 10
个关键帧发布，避免随任务时长形成 O(N^2) 的历史轨迹复制与网络发送。

实际项目校准：

1. 当前 `_state` 在初始化时已被 CGI-610 矿区位姿做一次世界系左变换。为保持
   原前端/相机接口不变，旧 `/path`、`/aft_mapped_to_init` 仍使用 `mine`；新后端
   内部明确把该连续局部轨迹记为 `odom`，全局优化输出使用 `mine`，并发布严格的
   `mine -> odom`。该变换和优化位姿从不回写 IEKF。
2. Truck 29 多雷达模式的 `feats_down_body` 实际仍是合成虚拟雷达/后轴局部系。
   Keyframe 接入前显式执行 `p_body = extR * p + extT`，内部只保存物理 IMU
   body 点云。
3. 当前 `/imu_data/odometry` 已给出 `ins_local_enu` 下的笛卡尔位置，所以当前
   数据链不需要重复执行 WGS84→ECEF→ENU。后文 Task 14 改为“仅在未来直接
   使用 `/imu_data/navsat_fix` 时启用的可选适配”。
4. `ins_pos_mode` 位于 `/imu_data/ins_status`；`prepare_wuhu_bag.py` 和
   `play_all_bags.sh` 均已保留/播放该话题。只有精确解析为 4 且 keyframe 时刻
   具有合法 status 与 solution bracket 时才生成 RTK observation。
5. Lightning-LM 只借鉴/移植 Miao、NDT 回环流程和参数经验；不接入它的 LIO
   前端。其 RTK PGO 代码大部分仍是注释/TODO，不能作为现成 RTK 融合模块。
6. 当前只在 `handleLIO()` 的完整 LIO 更新后产生关键帧，不修改相机、VIO、
   FAST-LIVO2 IEKF 或原体素地图。
7. 2026-08-18 实测关键帧共 447 个、时长 347.299 s，关键帧与原始 LIO
   轨迹最大位置差 `7.91e-7 m`，Task 03 验收完成。
8. 用户已在 `3rdparty/gtsam` 提供 GTSAM 4.2.2。原始构建使用 bundled Eigen
   3.3.7，与前端/PCL 的 system Eigen 3.4.0 不一致；工程因此增加隔离的
   `build-system-eigen` / `install-system-eigen` 构建，且关闭会与当前 OpenCV
   `libtbb.so.2` 冲突的 GTSAM TBB。Task 04 数值后端已迁移为持久化 GTSAM
   iSAM2，每个关键帧只提交新节点与新边，不重建整图。
9. 最终图包含相邻里程计 `BetweenFactor<Pose3>`、经验证的鲁棒 Loop
   `BetweenFactor<Pose3>` 和稀疏鲁棒 `GPSFactor`。首节点紧先验保留 CGI 初始化
   给出的 ENU gauge；RTK 仅约束 position 且使用有限协方差，不融合 CGI 姿态。
10. 2026-08-18 原 Ceres 版本完成 446 节点 / 445 里程计边 rosbag 验收，
    optimized/raw 最大位置差 `1.22e-12 m`、角度差 `1.71e-6 deg`，求解耗时
    median `1.887 ms`、max `4.457 ms`。两条轨迹重合是纯里程计图的理论正确
    结果：相邻边和初值来自同一条 raw LIO 轨迹，尚无 Loop/RTK 冲突约束可使
    轨迹发生全局修正。该结果同时验证了图约束方向和旧实现链路；随后按用户要求
    迁移到 GTSAM iSAM2，需重新进行同等级 rosbag 验收。
11. GTSAM iSAM2 版本完成 425 节点 / 424 里程计边实测，optimized/raw 最大
    位置差 `1.59e-12 m`、角度差 `1.71e-6 deg`，求解耗时 median `0.723 ms`、
    max `1.737 ms`，图结构、输入和数值链路通过。日志中的超大
    `variablesReeliminated` 不是实际计算量，而是 GTSAM 4.2.2 空 `update()`
    未初始化该标量；现已改用 detailed variable status 计数，并给验证脚本增加
    严格上界；后续联合实测已确认统计正常。该段仅作为迁移历史记录。
12. 2026-08-19 修复后的 Task 04/05 联合实测通过：425 节点、424 里程计边，
    iSAM2 `reeliminated_total=1272`、单关键帧最多 3；19 次候选搜索从 881 个
    空间近邻中经排序/NMS 选出 30 对候选，初值最大位置误差 `5.18e-14 m`。
    Task 06 随后接入后台单线程多分辨率 NDT，使用候选局部系历史子地图，输出逐级
    convergence/probability/fitness/overlap；当前版本已在其后增加 Task 07 验证，
    只有通过质量与邻域一致性判定的结果才进入 GTSAM。

## 0. 总目标

现有系统已经完成 FAST-LIVO2 数据适配并能够稳定运行和建图。

当前 FAST-LIVO2 只负责连续局部里程计和局部建图。现在需要在**尽量不侵入 FAST-LIVO2 前端内部算法**的前提下，增加：

1. Keyframe 管理；
2. Odometry Pose Graph；
3. Loop Candidate Detection；
4. Loop Registration；
5. Loop Pose Graph Optimization；
6. Optimized Global Map；
7. `map -> odom` 全局修正；
8. CGI-610 GNSS/INS 数据适配；
9. RTK 时间同步；
10. RTK 绝对位置约束；
11. Loop + RTK 联合优化；
12. 大范围矿山地图高性能管理。

所有功能必须按下面阶段逐步实现。

**禁止进行不必要的测试**

**禁止一次性重构整个 FAST-LIVO2。**

**禁止一次性加入 Loop + RTK + Global Map 后再统一调试。**

---

# 1. 总体架构

目标架构：

```text
LiDAR ─────┐
Camera ────┼──── FAST-LIVO2
IMU ───────┘         │
                     │ T_odom_body(t)
                     ▼
               KeyframeManager
                     │
             ┌───────┴────────┐
             │                │
             ▼                ▼
      OdometryFactor      LoopDetector
                              │
                              ▼
                       LoopRegistration
                              │
                              ▼
                          LoopFactor
             │                │
             └───────┬────────┘
                     │
                     │
CGI-610 GNSS/INS     │
      │              │
      ▼              │
RtkSolutionBuffer    │
      │              │
InsStatusBuffer      │
      │              │
      ▼              │
RtkMeasurementManager
      │
      │ 只在 keyframe timestamp
      │ 生成 RTK observation
      ▼
RTK Position Factor
      │
      └──────────┬─────────────┘
                 ▼
          BackendOptimizer
             / miao
                 │
         ┌───────┴────────┐
         ▼                ▼
     T_map_odom      Optimized KF Poses
                          │
                          ▼
                    GlobalMapBuilder
```

核心原则：

```text
FAST-LIVO2
负责：
局部连续性
高频 odometry
实时前端

Backend
负责：
长期一致性
回环
绝对位置
全局地图
```

第一版禁止 backend correction 直接修改 FAST-LIVO2 内部 ESIKF state 或 voxel map。

---

# 2. 坐标系规范

统一采用：

```text
map / ENU
    │
    │ T_map_odom
    ▼
odom
    │
    │ T_odom_body
    ▼
body/base_link
    ├── lidar
    ├── camera
    └── gnss/ins
```

必须在项目中创建：

```text
docs/frame_convention.md
```

统一规定：

```text
T_A_B
```

表示：

```text
把 B frame 中的坐标转换到 A frame
```

FAST-LIVO2：

```text
T_odom_body(t)
```

后端节点：

```text
X_k = T_map_body(k)
```

实时全局位姿：

```text
T_map_body(t)
=
T_map_odom
*
T_odom_body(t)
```

当前 correction：

```text
T_map_odom
=
T_map_body(k)
*
inverse(T_odom_body(k))
```

注意：

`T_map_odom` 只用于表示当前实时 odom 到 map 的 correction。

禁止使用一个固定 `T_map_odom` 去重新计算全部历史优化轨迹。

历史地图必须使用：

```text
optimized_pose[k]
```

逐关键帧重建。

---

# 3. Keyframe 数据结构

第一版建议定义：

```cpp
struct Keyframe {
    uint64_t id;
    double timestamp;

    SE3 T_odom_body;

    PointCloud::Ptr cloud_local;

    Matrix6d odom_cov;

    SE3 T_map_body;
};
```

其中：

```text
timestamp
```

必须统一为秒单位 `double` 或内部统一纳秒整数，不允许不同模块各自混用时间单位。

点云必须存储：

```text
body/local/lidar frame
```

而不是已经变换到旧 world/map frame 的点云。

---

# 4. RTK/INS 输入定义

CGI-610 状态来自：

```text
/imu_data/ins_status
```

状态信息形如：

```yaml
status:
  - name: CGI-610 INS/RTK
    message: navigation solution available
    values:
      - key: ins_pos_mode
        value: '4'
```

项目中明确规定：

```text
ins_pos_mode == 4
```

是 RTK/GNSS 观测**具备进入融合链路资格的必要条件**。

实现时不得依赖：

```text
values[0]
```

这种数组固定位置。

必须按 key 查找：

```cpp
key == "ins_pos_mode"
```

然后安全转换为整数。

建议接口：

```cpp
std::optional<int> ParseInsPosMode(
    const diagnostic_msgs::msg::DiagnosticArray& msg);
```

需要考虑：

```text
key 不存在
value 为空
非数字
存在多个 status item
消息过期
```

这些情况均不得崩溃。

---

# 5. RTK 融合的核心原则

这是整个 RTK 部分最重要的设计规定。

## 错误设计

禁止：

```text
CGI-610 100 Hz
    ↓
每收到一条消息
    ↓
add RTK factor
    ↓
100 Hz absolute factors
```

也禁止：

```text
RTK timestamp
    ↓
插值 SLAM pose
    ↓
随便关联某一个附近 keyframe
```

---

# 6. 正确设计：Keyframe-Driven RTK Fusion

整个 factor graph 的状态变量是：

```text
X0
X1
X2
...
Xk
```

所以 RTK 约束必须围绕：

```text
keyframe timestamp
```

构造。

对于关键帧：

```text
Kk
timestamp = tk
```

执行：

```text
tk
 ↓
查询 RTK buffer
 ↓
找到包围 tk 的两个有效 RTK solution
 ↓
把 RTK observation 插值到 tk
 ↓
得到 z_rtk(tk)
 ↓
最多创建一个 RTK factor
```

即：

```text
RTK message frequency
≠
RTK factor frequency
```

而是：

```text
RTK factor frequency
<=
selected keyframe frequency
```

CGI-610 的高频输出只是：

```text
continuous observation stream
```

不是：

```text
100 Hz independent graph constraints
```

---

# 7. RTK 数据缓冲区

实现：

```cpp
class RtkMeasurementManager;
```

内部至少维护：

```cpp
std::deque<RtkSolution> rtk_buffer_;
std::deque<InsStatusSample> status_buffer_;
```

或者等价的环形缓冲区。

建议：

```cpp
struct RtkSolution {
    double timestamp;

    Eigen::Vector3d position_enu;

    Eigen::Quaterniond orientation_enu_body;

    Eigen::Vector3d velocity;

    std::optional<Eigen::Matrix3d> position_cov;

    bool valid;
};
```

状态：

```cpp
struct InsStatusSample {
    double timestamp;
    int ins_pos_mode;
};
```

所有 callback 只负责：

```text
解析
检查
写 buffer
```

不得：

```text
callback 内执行 PGO
callback 内创建 graph vertex
callback 内添加 RTK edge
callback 内重建地图
```

---

# 8. RTK 时间必须使用传感器时间

优先使用：

```text
header.stamp.sec
header.stamp.nanosec
```

转换：

```cpp
double t =
    sec +
    nanosec * 1e-9;
```

禁止默认使用：

```text
ROS callback arrival time
system_clock::now()
```

作为测量发生时间。

必须记录：

```text
rtk_measurement_timestamp
ros_receive_timestamp
```

方便以后分析 latency。

---

# 9. ins_pos_mode 与 RTK solution 的关联

对于任意 RTK solution：

```text
z_i at t_i
```

必须能够获得：

```text
mode(t_i)
```

只有：

```text
mode(t_i) == 4
```

才能将该 RTK solution 标记为：

```text
fusion_eligible = true
```

如果状态话题与 solution 话题不是完全同时间戳：

允许：

```text
nearest status
```

或：

```text
last valid status before measurement
```

但必须设置：

```yaml
max_status_age_sec:
```

超过最大允许年龄：

```text
status_unknown
→ 不参与融合
```

绝不能让一个很旧的：

```text
ins_pos_mode = 4
```

永久污染后续数据。

---

# 10. RTK position 必须插值到 keyframe timestamp

假设当前关键帧时间：

```text
tk
```

在 RTK buffer 中找到：

```text
ta <= tk <= tb
```

对应：

```text
pa
pb
```

要求：

```text
mode(ta) == 4
mode(tb) == 4
```

并且：

```text
tb - ta <= max_rtk_interp_gap
```

否则：

```text
本关键帧不添加 RTK factor
```

定义：

```text
alpha =
(tk - ta) / (tb - ta)
```

位置插值：

```text
p_rtk(tk)
=
(1-alpha) * pa
+
alpha * pb
```

这里的 `p` 必须已经处于：

```text
ENU / map compatible Cartesian frame
```

禁止直接对：

```text
latitude
longitude
```

做线性插值以后直接当 xyz 使用。

---

# 11. 不插值 FAST-LIVO2 pose 来制造 unary factor

禁止以下实现：

```text
RTK time = tg

Xk at tk
Xk+1 at tk+1

先算：
X(tg) = interpolate(Xk, Xk+1)

然后把这个 measurement
强行挂到 Xk
```

因为这样 measurement 实际上依赖：

```text
Xk
和
Xk+1
```

却被错误表达成：

```text
unary(Xk)
```

第一版保持简单和严格：

```text
Graph node time = keyframe time
RTK observation time = keyframe time
```

即：

```text
measurement interpolation
而不是
state interpolation
```

---

# 12. 特殊时间情况

允许三个情况。

### Case A：完全匹配

```text
|tk - trtk| < timestamp_epsilon
```

直接：

```text
p(tk) = p_rtk
```

不需要插值。

### Case B：存在有效 bracket

```text
ta < tk < tb
```

执行线性 position interpolation。

### Case C：没有 bracket

例如：

```text
RTK loss
status != 4
buffer gap 太大
数据还未到达
```

则：

```text
skip RTK factor
```

第一版不要通过长时间 extrapolation 猜 RTK。

---

# 13. 姿态的时间同步

虽然第一版：

```text
enable_rtk_orientation_factor = false
```

仍然必须把姿态正确记录和同步。

如果之后需要得到 keyframe 时刻的 CGI-610 orientation：

```text
qa
qb
```

使用：

```cpp
qk = qa.slerp(alpha, qb);
```

必须处理 quaternion sign continuity。

不得：

```text
roll/pitch/yaw 分别普通线性插值
```

尤其 yaw 在：

```text
+179°
-179°
```

附近会产生错误。

---

# 14. 第一版 RTK 只加入 POSITION FACTOR

第一版：

```yaml
enable_rtk_position_factor: true
enable_rtk_yaw_factor: false
enable_rtk_orientation_factor: false
```

不要把 CGI-610 pose 直接做成：

```text
hard SE3 prior
```

CGI-610 的 position 和 orientation 都是有误差的测量。

RTK 的作用不是：

```text
告诉优化器“这里就是绝对真值”
```

而是：

```text
提供一个具有有限 covariance 的 absolute observation
```

---

# 15. RTK position factor

对于关键帧：

```text
Xk = T_E_B
```

RTK measurement：

```text
z_k = p_E_G
```

预测 GNSS measurement：

```text
h(Xk)
```

位置残差：

```text
r_k =
h(Xk) - z_k
```

代价：

```text
E_rtk
=
r_k^T
Sigma_rtk^-1
r_k
```

绝对禁止：

```text
Sigma -> 0
```

或者：

```text
information -> extremely large
```

来“强制贴 RTK”。

---

# 16. RTK 位置本身存在厘米级甚至更大误差

设计时明确：

```text
RTK != Ground Truth
```

RTK position 有：

```text
horizontal error
vertical error
multipath
solution transition
短时间 bias
时间同步误差
```

因此配置文件增加：

```yaml
rtk:
  enable: false

  required_ins_pos_mode: 4

  sigma_xy: ...
  sigma_z: ...

  min_sigma_xy: ...
  min_sigma_z: ...

  robust_kernel: cauchy
  robust_delta: ...

  innovation_gate_chi2: ...

  max_interp_gap_sec: ...
  max_status_age_sec: ...

  factor_min_dt_sec: ...
  factor_max_dt_sec: ...
  factor_min_distance_m: ...
```

所有数值必须可配置。

禁止把经验参数硬编码在 C++。

---

# 17. covariance 使用原则

如果 CGI solution 能提供可靠 position covariance：

```text
Sigma_device
```

优先使用。

但增加下限：

```text
sigma_x >= sigma_floor_x
sigma_y >= sigma_floor_y
sigma_z >= sigma_floor_z
```

防止设备偶发报告异常小 covariance，造成过度信任。

即：

```text
sigma_used =
max(sigma_reported, sigma_floor)
```

如果当前数据源没有可靠 covariance：

使用：

```yaml
sigma_xy
sigma_z
```

作为可调参数。

第一版宁可：

```text
稍微低估 RTK 权重
```

也不要：

```text
过度相信 RTK
```

---

# 18. 不把高频 GNSS/INS solution 当作独立因子

这是本次修改的第二个核心要求。

即使 CGI-610 输出：

```text
50 Hz
100 Hz
```

也不允许：

```text
factor_count = GNSS_message_count
```

正确关系：

```text
GNSS/INS samples
       │
       ▼
time buffer
       │
       ▼
interpolation
       │
       ▼
keyframe observation
       │
       ▼
factor selector
       │
       ▼
RTK factor
```

满足：

```text
RTK factor count
<=
keyframe count
```

---

# 19. 每个 keyframe 最多一个 RTK factor

增加明确 invariant：

```cpp
assert(num_rtk_factor_for_keyframe <= 1);
```

Backend 必须能够通过：

```text
keyframe_id
```

判断这个节点是否已经存在 RTK factor。

禁止：

```text
同一个 Xk
被 20 条相邻 RTK solution
反复添加 absolute edge
```

---

# 20. 进一步做 RTK Factor Selection

即使每个 KF 最多一个 RTK factor，大规模矿山仍可能有很多关键帧。

增加：

```cpp
class RtkFactorSelector;
```

只有满足：

```text
ins_pos_mode == 4
AND
interpolation_valid
AND
timestamp_valid
AND
innovation_valid
AND
factor_spacing_valid
```

才建立 factor。

Factor spacing 建议采用：

```text
距离门限
+
时间上限
```

例如逻辑：

```cpp
bool spatial_trigger =
    distance_from_last_rtk_factor >= min_distance;

bool max_time_trigger =
    dt_from_last_rtk_factor >= max_dt;

bool min_time_ok =
    dt_from_last_rtk_factor >= min_dt;

add_factor =
    min_time_ok &&
    (spatial_trigger || max_time_trigger);
```

这样既避免：

```text
静止时疯狂重复添加绝对位置约束
```

又避免：

```text
低速行驶很长时间完全没有 RTK anchor
```

这些阈值全部放 YAML。

不要在第一版假定某个固定值适合所有矿区和车速。

---

# 21. 高性能要求

RTK 高频 callback 的复杂度必须极低。

允许：

```text
parse
validate
push buffer
purge old samples
```

禁止：

```text
registration
optimization
map rebuilding
global nearest-neighbor search
```

RTK factor 只在：

```text
new keyframe event
```

上生成。

---

# 22. RTK buffer 查询性能

因为时间戳正常情况下单调递增，可以使用：

```text
deque + moving cursor
```

而不是每次从头遍历所有历史 RTK。

目标：

```text
amortized O(1)
```

或：

```text
O(log N)
```

查询。

当：

```text
RTK timestamp << oldest needed keyframe timestamp
```

时及时清理历史数据。

禁止让 RTK buffer 随矿山运行数小时无限增长。

---

# 23. RTK factor 不触发一次独立全图优化

禁止：

```text
RTK msg callback
→ add factor
→ optimize graph
```

因为这种设计会把计算频率绑定到 CGI-610 高频输出。

正确设计：

```text
New Keyframe
      ↓
Construct odometry edge
      ↓
Maybe construct one RTK factor
      ↓
Maybe construct loop edge
      ↓
Backend update
```

也就是说：

```text
Optimization cadence
≈
backend/keyframe cadence
```

而不是：

```text
GNSS message cadence
```

---

# 24. RTK innovation gating

RTK 不能因为：

```text
ins_pos_mode == 4
```

就无条件加入图。

`ins_pos_mode == 4` 是：

```text
必要条件
```

不是：

```text
充分条件
```

计算预测位置：

```text
p_pred
```

RTK：

```text
p_rtk
```

innovation：

```text
v =
p_pred - p_rtk
```

如果当前 covariance 可用，优先使用 Mahalanobis distance：

```text
d² =
v^T
S^-1
v
```

其中：

```text
S
```

由 RTK measurement covariance 与合理的状态预测 uncertainty 构成。

超过配置 threshold：

```text
reject factor
```

必须统计：

```text
rtk_candidate_count
rtk_accepted_count
rtk_rejected_status_count
rtk_rejected_time_count
rtk_rejected_gap_count
rtk_rejected_innovation_count
```

---

# 25. RTK factor 必须使用 robust kernel

即使通过 innovation gate：

RTK edge 仍应该使用：

```text
Huber
或
Cauchy
```

等 robust loss。

第一版优先选择当前 backend/miao 最容易稳定实现和调试的 robust kernel。

不要同时实现多个复杂动态权重算法。

目标首先是：

```text
正确
可解释
稳定
```

再优化参数。

---

# 26. RTK orientation 的处理

CGI-610 orientation 不是绝对真值。

已知：

```text
旋转角存在一定误差
```

因此第一阶段：

```yaml
enable_rtk_yaw_factor: false
enable_rtk_roll_pitch_factor: false
```

但是必须：

```text
完整记录 orientation
完成坐标转换
完成 timestamp interpolation
输出 debug
```

为后续实验做准备。

---

# 27. 第二阶段如需融合 orientation

先只增加：

```text
yaw factor
```

不要直接增加完整：

```text
SE(3) RTK pose factor
```

例如：

```text
r_yaw =
wrap(
yaw(Xk) -
yaw_rtk(k)
)
```

代价：

```text
E_yaw =
r_yaw² / sigma_yaw²
```

增加：

```yaml
enable_rtk_yaw_factor: false
sigma_yaw_deg: ...
yaw_robust_delta: ...
```

必须默认关闭。

只有在：

```text
RTK position-only
+
Loop
```

完全稳定以后才能打开。

---

# 28. RTK yaw 同样需要 keyframe-time interpolation

如果启用 yaw：

RTK quaternion：

```text
qa at ta
qb at tb
```

计算：

```text
qk =
SLERP(qa, qb, alpha)
```

然后在统一 ENU frame 下获得 yaw。

禁止：

```text
nearest orientation without timestamp test
```

也禁止：

```text
直接对 359° 和 1° 求算术平均
```

---

# 29. 纯 SLAM 后端开发阶段

下面严格按顺序开发。

---

## Task 01：冻结 FAST-LIVO2 baseline

禁止修改核心算法。

保存：

```text
baseline trajectory
baseline pointcloud
CPU
memory
frequency
```

准备固定测试 bag。

至少：

```text
开阔长距离
大闭环
多回环
矿区坡道
重复道路
RTK FIX/Lost/FIX
```

---

## Task 02：Frame Convention Audit（已完成）

分析：

```text
FAST-LIVO2 pose direction
LiDAR extrinsic
body frame
world/odom frame
timestamp
pointcloud frame
```

生成：

```text
docs/frame_convention.md
```

必须通过人工检查后继续。

---

## Task 03：KeyframeManager（已完成）

实现：

```text
translation trigger
rotation trigger
time trigger
```

保存：

```text
id
timestamp
T_odom_body
cloud_local
```

不做 graph。

测试：

```text
关键帧顺序正确
timestamp 单调
pose 正确
cloud frame 正确
```

---

## Task 04：Odometry-only Pose Graph（GTSAM iSAM2 数值链路已实测，诊断待复跑）

仅添加：

```text
Xk ↔ Xk+1
```

第一阶段可增加：

```text
Xk ↔ Xk+2
```

但建议先把最基本链式图验证正确。

定义：

```text
Zij =
inverse(T_odom_i)
*
T_odom_j
```

保持统一 SE(3) convention。

测试：

```text
optimized trajectory
≈
raw FAST-LIVO2 keyframe trajectory
```

这里不通过：

```text
禁止继续 Loop
禁止继续 RTK
```

当前实现校准（2026-08-18）：

```text
solver: GTSAM 4.2.2 iSAM2 (incremental)
node: one SE(3) pose per selected keyframe
gauge: tight PriorFactor<Pose3> on keyframe 0
factor: exactly one X(k-1) <-> X(k) odometry factor
initial value: previous optimized pose composed with raw relative motion
raw pose: immutable Keyframe::T_odom_body
update: one new-factor update plus configurable empty refinement updates
diagnostics: relinearized/reeliminated variable counts per keyframe
failure policy: throw with exact keyframe ID; never continue an invalid graph
```

验收脚本：

```bash
./scripts/validate_odometry_pose_graph.py
```

必须验证：

```text
nodes == keyframes
odometry_factors == keyframes - 1
all solution_usable == true
optimized/raw trajectory position delta <= 5e-5 m
optimized/raw trajectory angle delta <= 0.01 deg
pose graph raw inputs exactly match KeyframeManager output
```

---

## Task 05：Loop Candidate Detector

只检测，不加 graph。

输出：

```text
current KF
candidate KF
distance
initial relative transform
```

保存 debug 文件。

当前实现校准（2026-08-18）：

```text
check cadence: every 20 keyframes after enough history exists
history exclusion: >= 50 keyframes and >= 30 seconds
spatial gate: optimized-pose XY <= 20 m and |height| <= 5 m
ranking: nearest XY distance first
candidate NMS: historical IDs separated by >= 20 keyframes
bounded output: <= 3 candidates per checked keyframe
initial transform: inverse(T_map_candidate) * T_map_current
graph mutation: none
debug: loop_detection.csv + loop_candidates.csv + RViz yellow lines
```

验收脚本：

```bash
./scripts/validate_loop_candidates.py
```

---

## Task 06：Loop Registration

实现多分辨率 NDT 等几何 registration。

输出：

```text
Tij
score
convergence
overlap
```

仍禁止修改 graph。

当前实现校准（2026-08-19）：

```text
target frame: historical candidate body frame
target submap: candidate +/- 40 keyframes, stride 4
source: current keyframe body cloud
initial: detector T_candidate_current_initial
NDT levels: 10 m -> 5 m -> 2 m -> 1 m
voxel leaf: max(0.5 m, resolution * 0.25)
execution: bounded FIFO background worker; frontend never runs NDT
metrics: convergence, iterations, probability, fitness, overlap, inlier RMSE
debug: summary CSV + per-level CSV + RViz diagnostic markers
graph mutation: none
```

验收脚本：

```bash
./scripts/validate_loop_registration.py
```

---

## Task 07：Loop Verification

至少实现：

```text
convergence
score
overlap
translation sanity
rotation sanity
neighbor consistency
```

目标优先：

```text
high precision
```

而不是追求极高 recall。

---

## Task 08：Loop Pose Graph

加入：

```text
Odometry edges
+
Loop edges
```

Loop edge 使用 robust kernel。

测试：

```text
闭环误差下降
不存在错误地图折叠
```

---

## Task 09：Optimized Global Map

禁止继续累积旧 world-frame cloud。

地图构建：

```text
for each keyframe:

P_map =
T_map_body_optimized
*
T_body_lidar
*
P_lidar
```

---

## Task 10：map -> odom correction

实现实时：

```text
T_map_odom
```

FAST-LIVO2 继续工作在：

```text
odom frame
```

禁止 backend correction 修改 FAST-LIVO2 ESIKF。

---

# 30. RTK 数据链开发阶段

---

## Task 11：InsStatusAdapter

订阅：

```text
/imu_data/ins_status
```

解析：

```text
ins_pos_mode
```

建立测试：

```text
value='4' → 4
value='0' → 0
missing → invalid
bad string → invalid
```

此阶段禁止进入 PGO。

---

## Task 12：RtkSolutionAdapter

解析 CGI-610 position/orientation/velocity 等实际使用的话题。

统一输出：

```cpp
RtkSolution
```

此阶段只记录，不融合。

---

## Task 13：统一 timestamp

对：

```text
FAST-LIVO2 keyframe
RTK solution
INS status
```

统一时间基准。

打印：

```text
timestamp
receive time
delta
```

制作 debug CSV。

---

## Task 14：ENU conversion（当前 Odometry 数据链跳过，NavSatFix 接入时启用）

实现：

```text
WGS84
→
ECEF
→
ENU
```

并编写 unit test。

若未来直接使用 NavSatFix，后端不能直接使用：

```text
lat/lon
```

作为 Cartesian graph position。

---

## Task 15：RtkMeasurementBuffer

实现：

```text
RtkSolutionBuffer
InsStatusBuffer
```

要求：

```text
有序
有上限
可清理
线程安全
低开销
```

此阶段依然：

```yaml
enable_rtk_factor: false
```

---

## Task 16：ins_pos_mode eligibility

对每个 RTK sample 计算：

```text
fusion_eligible
```

要求：

```text
ins_pos_mode == 4
```

才 eligible。

记录：

```text
total
mode4
non-mode4
unknown-status
```

---

## Task 17：RTK position interpolation

实现：

```cpp
std::optional<RtkObservation>
GetObservationAt(double keyframe_timestamp);
```

行为：

```text
输入：
tk

输出：
p_ENU(tk)
orientation(tk)
covariance(tk)
status validity
time diagnostics
```

position：

```text
linear interpolation
```

orientation：

```text
SLERP
```

但 orientation 暂不进入 graph。

---

## Task 18：Interpolation Unit Tests

必须覆盖：

### exact timestamp

```text
tk == ta
```

返回：

```text
pa
```

### midpoint

```text
ta=0
tb=1

pa=(0,0,0)
pb=(10,0,0)

tk=0.5
```

要求：

```text
p=(5,0,0)
```

### invalid mode

如果：

```text
mode(ta)=4
mode(tb)!=4
```

默认：

```text
reject interpolation
```

不要跨越 RTK 状态失效区间制造观测。

### large time gap

如果：

```text
tb-ta > max_interp_gap
```

返回：

```text
nullopt
```

### no bracket

返回：

```text
nullopt
```

---

# 31. 防止高频 RTK 过度约束的核心测试

---

## Task 19：FactorSelector

实现：

```cpp
bool ShouldAddRtkFactor(
    const Keyframe& kf,
    const RtkObservation& obs);
```

必须确保：

```text
一个 keyframe
最多一个 RTK factor
```

---

## Task 20：100 Hz RTK 压力测试

构造：

```text
RTK = 100 Hz
Keyframe = 2 Hz
运行 60 秒
```

RTK messages：

```text
约 6000
```

Keyframes：

```text
约 120
```

要求：

```text
rtk_factor_count <= 120
```

绝不能：

```text
≈6000
```

---

## Task 21：RTK frequency invariance test

使用完全相同轨迹。

Case A：

```text
原始高频 CGI solution
```

Case B：

```text
对 CGI observation stream 大幅降采样
但保证 keyframe timestamp 附近仍可正常插值
```

如果 factor selection 和 interpolation 正确：

```text
Case A
与
Case B
```

的最终 graph 不应该因为输入消息频率变化而产生巨大差异。

这个测试专门用于发现：

```text
错误地按 RTK message frequency 加 factor
```

的问题。

---

# 32. RTK Position Factor

---

## Task 22：实现 position unary factor

输入：

```text
Xk
z_rtk(tk)
Sigma_rtk
```

输出 residual：

```text
3D position residual
```

第一版：

```yaml
enable_rtk_factor: false
```

编写人工 Jacobian / numerical Jacobian comparison test。

如果 optimizer API 允许，必须测试 Jacobian。

---

## Task 23：RTK covariance

支持：

```text
configured covariance
```

以及未来：

```text
device covariance
```

必须实现 minimum covariance floor。

禁止 hard position constraint。

---

## Task 24：RTK innovation gating

先只输出：

```text
innovation xyz
innovation norm
Mahalanobis distance
accepted/rejected
```

不要立刻调很多参数。

先统计 residual distribution。

---

## Task 25：RTK robust kernel

给 position factor 增加：

```text
Cauchy / Huber
```

并允许 YAML 配置。

---

# 33. RTK Position-only Offline PGO

---

## Task 26

开启：

```text
Odometry
+
RTK position
```

关闭：

```text
Loop
RTK orientation
```

即：

```text
B vs D
```

测试：

```text
长期 drift
RTK residual
trajectory continuity
Z
yaw
```

重点检查：

RTK 不是 ground truth，因此优化结果：

```text
不应该逐点完全贴住 RTK
```

如果发现：

```text
optimized trajectory ≈ 每一个 RTK sample
```

应优先怀疑：

```text
RTK information 过大
factor 数过多
covariance 太小
```

而不是认为融合效果特别好。

---

# 34. Loop-only 测试

---

## Task 27

开启：

```text
Odometry
+
Loop
```

关闭：

```text
RTK
```

确认纯 SLAM 后端独立工作正确。

---

# 35. Loop + RTK Joint Optimization

---

## Task 28

最终：

```text
Odometry
+
Loop
+
RTK position
```

全部开启。

三类约束职责：

```text
Odometry
→ 局部连续性

Loop
→ 全局相对一致性

RTK
→ 绝对位置和长期漂移约束
```

如果 Loop 与 RTK 存在冲突：

禁止第一反应：

```text
提高 RTK 权重
```

应该输出：

```text
loop residual
rtk residual
odom residual
```

定位冲突来源。

---

# 36. RTK orientation 第二阶段

只有 Task 28 完整稳定以后才能开始。

---

## Task 29：Yaw observation debug

先不参与优化。

输出：

```text
FAST-LIVO2 yaw
CGI yaw
difference
timestamp
speed
ins_pos_mode
```

分析误差分布。

---

## Task 30：可选 weak yaw factor

如果确认确有必要：

```yaml
enable_rtk_yaw_factor: true
```

增加独立：

```text
1DoF yaw factor
```

而不是直接增加 6DoF pose factor。

yaw factor 必须：

```text
独立 covariance
独立 robust kernel
独立 innovation gate
```

RTK position covariance 与 orientation covariance 禁止混为一个经验权重。

---

# 37. RTK 丢失和恢复

---

## Task 31

测试：

```text
ins_pos_mode = 4
        ↓
mode != 4
        ↓
数百米 FAST-LIVO2 + Loop
        ↓
ins_pos_mode = 4 recovered
```

要求：

RTK 丢失期间：

```text
0 RTK factors
```

恢复以后：

```text
只有新的 keyframe
才能重新产生 factor
```

禁止：

```text
把丢失期间积压的旧 RTK solution
一次性全部添加
```

---

# 38. 在线多线程架构

---

## Task 32

建议线程划分：

```text
Thread A
FAST-LIVO2 frontend

Thread B
RTK/status callbacks + buffer

Thread C
Keyframe/backend

Thread D
Loop registration

Thread E
Global map/tile update
```

具体线程数可以根据当前系统结构调整。

关键原则：

RTK 高频消息绝不能阻塞：

```text
LiDAR processing
Camera processing
IMU processing
```

---

# 39. Backend Queue

Frontend 向 backend 传：

```text
Keyframe
```

而不是：

```text
每一帧 LiDAR
每一帧 image
每一条 RTK
```

因此系统计算复杂度主要跟：

```text
keyframe 数
```

而不是：

```text
sensor message 数
```

增长。

---

# 40. Incremental optimization

优先继续采用增量 PGO 结构。

禁止每产生一条 RTK observation 就：

```text
重新构造整张 graph
```

Backend update 的基本单位是：

```text
new keyframe
+
new edges
```

---

# 41. Global Map 性能

地图不能每次优化以后：

```text
把所有关键帧全部重新生成一遍
```

作为在线唯一方案。

第一阶段允许完整重建用于验证。

大规模版本改成：

```text
keyframe storage
+
submap
+
tile/chunk
+
dirty region update
```

例如：

```text
50m × 50m
或
100m × 100m
```

具体尺寸配置化。

---

# 42. 必须实现的日志

每个 keyframe 输出：

```text
kf_id
kf_timestamp

T_odom_body

rtk_available
rtk_mode

rtk_t_before
rtk_t_after
rtk_interp_alpha
rtk_interp_gap

rtk_position_x
rtk_position_y
rtk_position_z

rtk_factor_selected

rtk_reject_reason

rtk_residual_x
rtk_residual_y
rtk_residual_z

rtk_mahalanobis

loop_count

optimization_time_ms
```

---

# 43. RTK reject reason 必须枚举化

例如：

```cpp
enum class RtkRejectReason {
    NONE,

    NO_DATA,
    NO_STATUS,
    MODE_NOT_FIXED,
    NO_BRACKET,
    INTERPOLATION_GAP_TOO_LARGE,
    TIMESTAMP_INVALID,

    FACTOR_TOO_DENSE,

    INNOVATION_TOO_LARGE
};
```

这样以后调试不能只看到：

```text
RTK rejected
```

而必须知道：

```text
为什么 rejected
```

---

# 44. 重要 invariant

在 debug build 中加入断言或统计。

必须始终满足：

```text
keyframe timestamps monotonic
```

```text
RTK factor timestamp
==
对应 keyframe timestamp
```

逻辑上必须完全一致。

必须满足：

```text
RTK factors
<=
keyframes
```

必须满足：

```text
一个 keyframe
最多一个 RTK position factor
```

必须满足：

```text
ins_pos_mode != 4
→
不得创建 RTK factor
```

必须满足：

```text
invalid interpolation
→
不得创建 RTK factor
```

---

# 45. Ablation

最终运行：

```text
A:
FAST-LIVO2

B:
FAST-LIVO2
+
PGO

C:
FAST-LIVO2
+
PGO
+
Loop

D:
FAST-LIVO2
+
PGO
+
RTK position

E:
FAST-LIVO2
+
PGO
+
Loop
+
RTK position
```

RTK yaw 如果以后实现：

单独增加：

```text
F:
FAST-LIVO2
+
PGO
+
Loop
+
RTK position
+
RTK yaw
```

不要把 yaw factor 混入第一次 RTK 实验。

---

# 46. Benchmark 指标

至少输出：

```text
ATE
RPE@10m
RPE@50m
RPE@100m

loop precision
loop recall

RTK candidate count
RTK accepted count
RTK rejected count

RTK factor / keyframe ratio

RTK residual xyz
RTK residual norm

RTK innovation histogram

odom edge chi2
loop edge chi2
RTK edge chi2

keyframe count
loop count
RTK factor count

PGO runtime
Loop registration runtime

CPU
memory
```

---

# 47. 一个非常重要的新性能指标

必须记录：

```text
RTK message count
```

和：

```text
RTK factor count
```

定义：

```text
rtk_factor_reduction_ratio
=
rtk_factor_count
/
rtk_message_count
```

高频 CGI-610 情况下，该值应该明显小于 1。

这不是“丢失 RTK 信息”，而是因为：

```text
原始消息用于时间插值
graph factor 对应 graph state
```

两者职责不同。

---

# 48. RTK noise injection test

为了验证系统没有把 RTK 当真值，增加 synthetic test。

给 RTK position 人工加入：

```text
几厘米随机扰动
```

例如统一测试框架支持：

```yaml
debug:
  rtk_position_noise_std: ...
```

要求：

优化轨迹应该：

```text
平滑响应
```

而不是：

```text
每一个关键帧跟着 RTK jitter 抖动
```

如果后者发生：

优先检查：

```text
RTK factor 太密
RTK covariance 太小
robust kernel 参数
odom weight
```

---

# 49. RTK orientation noise test

以后启用 yaw factor 前：

人工加入小角度扰动。

检查：

```text
trajectory 是否被频繁扭曲
```

如果 yaw factor 开启后导致：

```text
局部轨迹抖动
频繁方向修正
Loop 与 RTK fighting
```

则关闭 yaw factor，并重新估计：

```text
sigma_yaw
```

不要简单继续增大 position 权重。

---

# 50. Codex 工作纪律

每个 Task：

```text
一个独立 commit / PR
```

Codex 每次必须先：

1. 阅读现有相关源码；
2. 输出它理解的数据流；
3. 明确 frame convention；
4. 明确 timestamp convention；
5. 给出计划修改的文件；
6. 实现；
7. 编译；
8. 跑 unit test；
9. 跑指定 offline bag；
10. 输出测试结果。

如果某阶段测试失败：

```text
停止
```

禁止继续增加新模块掩盖已有错误。

---

# 51. Codex 禁止事项

禁止：

```text
为了快速跑通而修改 FAST-LIVO2 ESIKF
```

禁止：

```text
直接把 lightning-lm LoopClosing 整个复制进项目
```

禁止：

```text
每个 GNSS/INS message 添加 RTK edge
```

禁止：

```text
把 RTK pose 当 ground truth
```

禁止：

```text
ins_pos_mode != 4 时融合
```

禁止：

```text
没有有效 bracket 时长时间外推 RTK
```

禁止：

```text
为了 RTK unary factor
把 SLAM state 插值到 RTK timestamp 后再随便关联 KF
```

禁止：

```text
同一个 keyframe 添加多个 RTK position factor
```

禁止：

```text
RTK callback 内执行全图优化
```

禁止：

```text
第一版直接使用强 6DoF CGI pose prior
```

---

# 52. 最终开发顺序

严格按照：

```text
FAST-LIVO2 baseline
        ↓
Frame convention
        ↓
KeyframeManager
        ↓
Odometry-only PGO
        ↓
Loop candidate detection
        ↓
Loop registration
        ↓
Loop verification
        ↓
Loop PGO
        ↓
Optimized global map
        ↓
map→odom correction
        ↓
InsStatusAdapter
        ↓
RtkSolutionAdapter
        ↓
timestamp validation
        ↓
ENU conversion
        ↓
RTK/status buffer
        ↓
ins_pos_mode == 4 eligibility
        ↓
RTK observation interpolation
to keyframe timestamp
        ↓
RTK factor selector
        ↓
RTK position unary factor
        ↓
covariance
        ↓
innovation gate
        ↓
robust kernel
        ↓
Odometry + RTK
        ↓
Odometry + Loop
        ↓
Odometry + Loop + RTK
        ↓
RTK drop/recovery
        ↓
online multithreading
        ↓
tile/chunk global map
        ↓
optional RTK yaw experiment
```

---

# 53. 最终原则

整个系统必须遵守三个时间尺度：

```text
高频：
FAST-LIVO2 odometry

中频：
CGI-610 GNSS/INS observation buffer

低频：
keyframe / backend factor graph
```

RTK 高频输出的价值在于：

```text
能够为 keyframe timestamp
提供质量更好的时间对齐 observation
```

而不是：

```text
制造大量重复 absolute factors
```

因此最终 RTK 链路必须严格设计成：

```text
CGI-610 high-rate solutions
        ↓
timestamp ordered buffer
        ↓
ins_pos_mode == 4
        ↓
interpolate measurement
at keyframe time
        ↓
quality / innovation check
        ↓
factor spacing check
        ↓
at most ONE RTK position factor
per selected keyframe
        ↓
robust weighted PGO
```

RTK 是：

```text
有噪声的绝对观测
```

不是：

```text
绝对真值
```

FAST-LIVO2、Loop 和 RTK 的最终职责必须保持清晰：

```text
FAST-LIVO2
→ 高频局部连续性

Loop
→ 长期相对几何一致性

RTK position
→ 有噪声的绝对地理位置约束

RTK yaw
→ 可选弱方向约束
```

只有这样才能得到：

```text
可调试
可解释
稳定
高性能
适合大尺度矿山
```

的完整 SLAM 系统。

这里最关键的实现变化其实可以浓缩成一句：**以后 graph 不再“消费 RTK 消息”，而是 keyframe 在创建时向 RTK 时间缓冲区“查询自己这个时刻的 RTK 观测”。** 这样即使 CGI-610 是 100 Hz，后端也不会被 100 Hz absolute factor 拖慢或重复计权，而较高的 RTK 输出频率反而主要用于降低关键帧时间对齐误差。

[1]: https://github.com/hku-mars/fast-livo2 "GitHub - hku-mars/FAST-LIVO2: FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry · GitHub"
