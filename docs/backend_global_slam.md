# 全局后端数据流与运行约束

## 数据流

FAST-LIVO2 仍只负责连续 `T_odom_body`。完整 LIO 更新产生关键帧后，后端依次
执行相邻里程计因子、keyframe-time RTK 查询/选择、优化结果发布与低频回环候选
检测。多分辨率 NDT 在有界后台队列运行；只有通过单次质量门限和邻域一致性检查
的结果才作为鲁棒 `BetweenFactor<Pose3>` 加入持久化 GTSAM iSAM2。

CGI-610 的 100 Hz status/solution callback 只写入 20 s 有界缓冲区，不执行图
优化。每个关键帧最多查询并添加一个 RTK `GPSFactor`，且必须同时满足：

- `ins_pos_mode` 可解析且精确为 4；
- keyframe 时间被两条有效 solution 包围，不外推且不跨越状态失效区；
- 通过时间/位移稀疏选择和 Mahalanobis innovation gate；
- 位置协方差带下限，并使用 Cauchy/Huber 鲁棒核。

CGI 姿态目前只参与插值诊断，不进入图；这是有意保留的二期功能。

## 坐标系

- 旧前端话题保持 `mine`，以免改变 FAST-LIVO2/VIO 接口。
- 后端把 immutable raw LIO 数值视作 `T_odom_body`。
- GTSAM 优化结果是 `T_mine_body`，并发布
  `T_mine_odom = T_mine_body * inverse(T_odom_body)`。
- 后端结果从不回写 IEKF 或前端 voxel map。

## 大地图策略

在线地图使用 60 m 可配置瓦片。无显著历史位姿变化时，仅变换新关键帧并重滤波
受影响瓦片；Loop 或超过配置阈值的图修正执行全量精确重建。RTK 图修正有最小
关键帧间隔去抖，地图请求只保留最新 snapshot，全部构建在独立线程完成。退出时
停止该线程，再按最终图位姿精确重建并保存压缩 PCD。

普通 odometry 节点只从 iSAM2 提取最新位姿；Loop/RTK 约束才刷新历史位姿。
完整 Path 低频发布，optimized odometry 和 `mine -> odom` 每个关键帧发布。

## 关键输出

- ROS：`/backend/keyframe_path_raw`、`/backend/keyframe_path_optimized`、
  `/backend/odometry_optimized`、`/backend/verified_loops`、
  `/backend/global_map_optimized`。
- 文件：`Log/backend/optimized_trajectory.csv`、
  `Log/backend/optimized_global_map.pcd` 以及 `Log/backend/*.csv` 诊断。

联合模式结束后依次运行 `validate_backend_keyframes.py`、全部 `validate_loop_*`
脚本、`validate_rtk_fusion.py` 和 `validate_optimized_global_map.py`。纯里程计的
`validate_odometry_pose_graph.py` 不适用于联合模式，因为全局约束本来就应改变
raw 轨迹。
