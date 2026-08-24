# 全局后端数据流与运行约束

## 数据流

FAST-LIVO2 输出连续前端状态；关键帧冻结为 `T_odom_body`。local GTSAM 只接收
相邻里程计与通过验证的回环 `BetweenFactor<Pose3>`，输出 `T_slam_body`。
RTK 只进入独立的低频全局层，产生 `T_global_body=C(s)A0T_slam_body`。
`C(s)` 使用基于当前 `T_global` 残差的距离自适应径向弹性刚度。

CGI-610 的 status/solution callback 只写入有界缓冲区。状态必须精确为 4，且
关键帧时间必须有合法 bracket；稀疏位置观测更新有界修正场，而不是 local 图。

退化后约 1 Hz 速度保护仅小幅修正后续传播速度；位置、姿态和历史 local 图不回写。
重定位锁存后的关键帧保留用于恢复，但不进入可信全局 Path/PCD。

## 坐标系

- 旧前端话题保持 `mine`，以免改变 FAST-LIVO2/VIO 接口。
- `/path` 只是前端显示缓存；后端不读取该 `nav_msgs::Path`。
- local GTSAM 使用不可变 `T_odom_body` 的相对运动，不使用其绝对轨迹作结果。
- 回环/全局层分别使用 `T_slam_body`/`T_global_body`。
- 后端结果从不回写 IEKF 或前端 voxel map。

## 大地图策略

在线地图使用 60 m 可配置瓦片。无显著历史位姿变化时，仅变换新关键帧并重滤波
受影响瓦片；Loop 或超过配置阈值的全局修正执行全量精确重建。地图请求只保留
最新 snapshot，全部构建在独立线程完成。退出时
停止该线程，再按最终图位姿精确重建并保存压缩 PCD。

普通 odometry 节点只从 iSAM2 提取最新位姿；Loop 约束才刷新 local 历史位姿。
完整 Path 低频发布，optimized odometry 和 `mine -> odom` 每个关键帧发布。

## 关键输出

- ROS：`/backend/keyframe_path_raw`、`/backend/keyframe_path_optimized`、
  `/backend/odometry_optimized`、`/backend/verified_loops`、
  `/backend/global_map_optimized`。
- 文件：`Log/backend/optimized_trajectory.csv`、
  `Log/backend/optimized_global_map.pcd` 以及 `Log/backend/*.csv` 诊断。

联合模式结束后依次运行 `validate_backend_keyframes.py`、全部 `validate_loop_*`
脚本、`validate_rtk_fusion.py` 和 `validate_optimized_global_map.py`。纯里程计的
`validate_rtk_fusion.py` 会审计 raw/local/global 三层的完整 6DOF 来源。
