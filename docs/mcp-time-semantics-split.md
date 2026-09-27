# 时间语义分离架构：标定环节定参，SLAM 通用化（MCP 会话 2026-09-27）

用户原则：外参与时间戳全部在标定环节确定，SLAM 只作通用建图工具；不同矿车数据均保持高精度。

## 设计

标定文件 time_offsets 两个 lidar 字段，语义分离：

| 字段 | 语义 | 消费方 |
| --- | --- | --- |
| lidar_frontend | 扫描↔IMU 硬件对齐（如 -0.125） | LIVMapper 前端：IMU 同步/去畸变/图像配对 |
| lidar | 扫描↔INS 对齐（axle 阶段解出，含 INS 链路延迟） | 融合：INS 参考关联 |

实现（全部通用逻辑，无车辆特判）：
- LIVMapper：lidar_frontend 存在时前端走硬件基，差值 (lidar - lidar_frontend) 经构造参数
  传入 RtkFusion，平移 rtk.time_offset_sec / attitude_time_offset_sec；
- RtkFusion 在 parameters.txt 首行写出生效值，评估脚本 grep -m1 自动一致；
- livo_calib：DEFAULT_TIME_OFFSETS 增加 lidar_frontend=-0.125（import 播种，各阶段深拷贝
  自动传承），set 增加 --lidar-frontend-offset。

真值链条：lidar_frontend/camera 是硬件族属性（同型雷达/相机各车一致），由 T4 实验族标定
（truck29: -0.125/-0.115）；lidar 由 axle 阶段按车数据驱动（truck29 -0.1459，truck05 -0.0328，
两车 INS 链路延迟差 92ms 正是 v017 axle 发现的偏移）。

## 验证（xy max, cm）

| 场景 | 分离前 | 分离后 |
| --- | --- | --- |
| truck29 v063 LIVO 120s | 6.93 (v062) | **5.92**（生效 INS 偏移 0.0041 已在 parameters.txt 确认） |
| truck05 v021 W2 LIO | 6.0 (v019) | **5.81**，native d3max 降至 32 cm（前端 92ms 同步修正的直接收益） |
| truck05 v021 W2 LIVO | 10.4/27.7 (v019 刀锋) | 9.13（native 1.33 m，仍属刀锋带，复测中） |

truck05 相机偏移同步取硬件族值 -0.115（v021；原 0 值是光流门限未过的未标定态）。

## 结论对"通用性"的意义

- SLAM 配置 yaml 中不再需要任何车辆特定时间值（yaml 仅默认，标定文件全量覆盖）；
- 新车标定 SOP：import（播种硬件族值）→ axle（数据驱动解 INS 对齐 τ）→ ground/driven →
  camtime（可标则实测，否则族值）→ camera-lidar/camloop；产物喂给通用 SLAM 即可。
