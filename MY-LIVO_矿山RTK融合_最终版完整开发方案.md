# MY-LIVO 矿山场景 RTK 融合：最终问题分析与完整开发方案

> 日期：2026-09-10  
> 适用工程：用户提供的 `MY-LIVO（复件）`  
> 核心目标：在不破坏 FAST-LIVO2 原生局部几何精度的前提下，引入矿山 RTK 的厘米级全局位置能力，并科学处理逐历元不确定性、慢变位置偏差、RTK 失锁/假固定、姿态信息相关性、时间同步、多 LiDAR 外参和长时间运行问题。

---

## 0. 最终结论

对现有项目重新复核后，最终生产路线应明确为：

> **FAST-LIVO2 继续作为高频局部运动和扫描内几何的权威估计器；RTK 后端不再重新构造一套 raw-IMU + 简化 LiDAR 的局部导航平滑器，而改为“局部轨迹保持型全局校正图（Global Correction Graph）”。**

最终系统只让 RTK 修正 FAST-LIVO2 的低频全局漂移，而不允许 RTK 或第二套后端破坏 FAST-LIVO2 已经建立好的短时相对位姿和扫描内去畸变。

最终生产后端包含：

1. FAST-LIVO2 原生局部 Pose 之间的 `BetweenFactor<Pose3>`；
2. RTK 天线位置/参考点位置因子，并使用逐历元 covariance；
3. GNSS 完整性门控、innovation/NIS 门控、Huber 鲁棒核；
4. RTK 恢复后的 soft relock，禁止瞬时硬拉；
5. 低频、均值回归的 Gauss–Markov RTK 位置偏置状态；
6. 稀疏、严格门控的长程 LiDAR 几何约束，用于重复经过区域或强几何区域；
7. 仅在存在**独立双天线 GNSS heading 或其他独立姿态源**时加入 heading 因子；当前 CGI INS attitude 若与原始 IMU 同源，不作为连续独立因子重复加入；
8. 原生 FAST-LIVO2 已去畸变点云 + 平滑全局 correction 生成最终地图。

最终数据流：

```text
4×LiDAR + IMU
      │
      ▼
+---------------------------+
|        FAST-LIVO2         |
| ESIKF + voxel-map geometry|
| native local trajectory   |
| native undistorted scan   |
+---------------------------+
      │
      │ local pose / quality
      ▼
+----------------------------------------------+
|          GlobalCorrectionFusion              |
|                                              |
| Pose3 correction knots                       |
| LIO relative factors                         |
| RTK position + covariance + robust/gating    |
| Gauss-Markov RTK position bias               |
| sparse long-range geometry                   |
| independent GNSS heading (only when valid)   |
+----------------------------------------------+
      │
      ▼
C(t) = X_global(t) · X_lio(t)^(-1)
      │
      ▼
Native FAST-LIVO2 undistorted point cloud
      + smooth C(t)
      │
      ▼
Global mine trajectory / map
```

这一架构是本文唯一推荐的生产架构。现有 `rtk_fusion.cpp` 中的 raw-IMU navigation smoother 应保留为实验对照分支，不再承担最终在线建图主链路。

---

# 1. 对现有工程的代码事实复核

本次结论以当前工程真实实现为基础，而不是按通用 FAST-LIVO2+RTK 方案推测。

## 1.1 当前后端实际是第二套导航平滑器

当前 `rtk_fusion.cpp` 已经联合优化：

\[
X_k=(R_k,p_k),\qquad v_k,\qquad b^g_k,\qquad b^a_k
\]

并加入：

- `gtsam::ImuFactor` 原始 IMU 预积分；
- IMU bias 随机游走；
- `gtsam::GPSFactorArm`；
- 自定义 `ScanPlaneBatchFactor`；
- local / overlap / revisit 几何；
- 后端重新生成扫描期间连续轨迹并重新投影原始点。

因此当前系统不是简单的“FAST-LIVO2 + RTK 位置因子”，而是：

```text
FAST-LIVO2 局部导航
      +
第二套 IMU/LiDAR/RTK 导航平滑
```

这是理解当前地图变厚问题的关键。

## 1.2 当前工程已经正确避免 RTK 直接反馈 FAST-LIVO2 前端

`src/rtk_fusion.cpp` 中明确禁止 `frontend_position_aiding=true`。

这一设计必须保留。

RTK 在矿山中会经历：

- 临时失锁；
- FIX/FLOAT 切换；
- 边坡/大型设备附近多路径；
- 假固定；
- 短时间厘米到分米级系统偏移。

如果直接覆盖 FAST-LIVO2 ESIKF 状态，会污染局部 voxel map，且历史地图无法一致回退。

最终原则固定为：

```text
FAST-LIVO2：局部相对运动
RTK 后端：全局低频校正
```

## 1.3 当前 RTK 因子频率与关键帧频率已经分离

配置中：

```yaml
keyframe_interval_sec: 1.0
keyframe_min_interval_sec: 0.5
position_factor_interval_sec: 0.5
```

代码也注明连续 INS outputs 存在时间相关性，不能因为关键帧增多而无限重复加入 RTK/INS 位置。

这一思想正确，应继续保留。

## 1.4 当前配置已经明确 CGI IMU 与 INS attitude 相关

`wuhu_truck29.yaml` 中已有注释：

```text
CGI IMU and INS attitude are correlated: no duplicate attitude/IMU factors.
```

因此，如果 `/imu_data/odometry` 的 orientation 是同一 CGI 组合导航由当前 IMU 融合产生的姿态，则不能把它再当成独立高斯观测加入图。

## 1.5 当前坐标系是 RFU，而辅助姿态查看工具存在错误标注

工程明确使用：

```text
X = right
Y = forward
Z = up
```

即 RFU。

但 `scripts/rtk_orientation_viewer.py` 标注为：

```text
X = 前
Y = 左
Z = 上
```

这是明确的工具级错误，应修正。姿态、heading、杆臂、外参调试不能建立在错误坐标轴解释上。

---

# 2. 已有实验已经证明的主要问题

当前最重要的证据不是某个 factor cost，而是已经完成的局部地图消融。

同一约 20.33 m、68 帧窗口：

| 方案 | 地面厚度 |
|---|---:|
| 原生 FAST-LIVO2 | 13.59 cm |
| 当前实际融合 | 35.28 cm |
| 原生去畸变 + 融合帧末位姿 | 36.66 cm |
| 融合去畸变 + 原生帧末位姿 | 13.97 cm |

这个实验直接说明：

> **当前地图的主要恶化来源是后端修改后的跨帧帧末位姿关系，而不是扫描内去畸变。**

第 17 轮进一步显示：

- 三维 RTK P95：约 12.61 cm；
- RTK 最大残差：约 16.30 cm；
- 后端单帧耗时 P95：约 186.36 ms；
- 同一局部窗口地面仍约 34.12 cm；
- 立面约 29.13 cm；
- 54 个固定截面中仍有 38 个比原生更宽。

因此必须明确：

> **RTK residual 变小、几何 factor residual 下降、优化收敛或者运行速度提升，都不能证明地图已经正确。**

另一个诊断中，低频关键帧旋转修正插值后，非关键帧与原生旋转差异达到：

- median：0.119°；
- P95：0.308°；
- max：0.583°。

这说明第二套后端正在明显改变 FAST-LIVO2 原本的帧间姿态结构。

对于 60 m 目标，0.3° 相对姿态误差对应约：

\[
60\times0.3^\circ\times\pi/180\approx31.4\text{ cm}
\]

与当前几十厘米级重影量级高度一致。

---

# 3. 当前算法的核心不足和错误之处

## 3.1 结构性错误：第二套后端在重新做局部 LIO

FAST-LIVO2 前端已经利用：

- 高频 IMU；
- ESIKF；
- 当前扫描与增量 voxel map 的直接点面约束；
- 完整局部地图上下文；
- 连续扫描内运动补偿。

当前后端又利用：

- raw IMU；
- 稀疏状态；
- 简化 scan-to-plane 几何；
- RTK；

重新估计局部轨迹。

问题不在于这些因子“数学错误”，而在于这套后端的局部几何信息结构明显弱于 FAST-LIVO2 前端，却拥有重新修改全轨迹的自由度。

矿区又存在大量：

- 平坦或缓坡路面；
- 长边坡；
- 大面积弱纹理/弱立面区域；
- yaw 与横向方向可观性不足。

结果就是第二套平滑器可以在 cost 下降的同时改变相邻扫描姿态，最终把地图变厚。

### 最终改进

生产后端不再优化 raw IMU velocity/bias，也不再用相邻 scan-plane factor 重建局部运动。

改为：

\[
\boxed{\text{FAST-LIVO2 local motion} + \text{low-frequency global correction}}
\]

---

## 3.2 统计问题：组合导航位置与原始 IMU并非独立

当前外部位置来自 RTK/INS 组合导航，而 FAST-LIVO2 与现有后端都使用相同原始 IMU。

因此外部 INS position 已经包含部分 IMU 信息。

如果把：

```text
FAST-LIVO2 / raw IMU
```

和：

```text
CGI INS pose
```

全部当作独立高斯观测叠加，理论上会过度计数信息。

### 最终改进

数据源优先级固定为：

1. **RTK 天线相位中心原始位置解**；
2. 对应 hAcc/vAcc 或完整 3×3 covariance；
3. solution mode / satellites / DOP / correction age / ambiguity ratio；
4. 独立双天线 heading；
5. vendor INS rear-axle pose 只作为无法取得 raw RTK 时的兼容输入。

生产全局图不再使用 raw IMU factor，因此即使暂时仍用 INS position，相关性风险也显著降低。

---

## 3.3 RTK covariance 当前语义混乱

当前代码存在：

```cpp
sample.covariance += floor;
sample.covariance *= covariance_scale;
```

但配置注释又把 3 cm / 5 cm 描述为“消息 covariance 全零时使用的 nominal standard deviation”。

这混淆了三种不同概念：

- 接收机报告的测量 covariance；
- 没有 covariance 时的 fallback；
- 对未建模误差附加的 model covariance。

### 最终改进

必须明确拆为：

\[
\Sigma_{reported},\qquad
\Sigma_{fallback},\qquad
\Sigma_{model}.
\]

有效 reported covariance 时：

\[
\boxed{
\Sigma_{RTK}
=
\kappa\Sigma_{reported}
+
\Sigma_{model}
+
\Sigma_{lever}
+
\Sigma_{time}
}
\]

无 reported covariance 时：

\[
\boxed{
\Sigma_{RTK}
=
\Sigma_{fallback}
+
\Sigma_{lever}
+
\Sigma_{time}
}
\]

这样参数才具有明确物理意义。

---

## 3.4 RTK 只有质量状态过滤，没有完整 innovation 统计门控

当前 mode/satellite/recovery/covariance/Huber 是必要基础，但不能解决矿区：

```text
RTK 仍为 FIX
但位置发生持续偏移
```

的情况。

### 最终改进

增加两层完整性检查：

**第一层：GNSS 自身完整性**

- FIX/FLOAT；
- satellites；
- hAcc/vAcc；
- DOP；
- correction age；
- ambiguity ratio；
- RTK 连续稳定时间；
- RTK 速度/位置增量物理合理性。

**第二层：估计 innovation**

\[
\nu=z-h(\hat X)
\]

\[
S=HPH^T+R
\]

\[
NIS=\nu^TS^{-1}\nu.
\]

三维位置 99% 卡方阈值：

\[
\chi^2_{3,0.99}\approx11.345.
\]

最终使用：

```text
GNSS integrity
→ NIS/absolute gate
→ robust kernel
→ factor graph
```

三层不能互相替代。

---

## 3.5 RTK 恢复后存在硬拉风险

长时间 RTK loss 后，FAST-LIVO2 可能已有一定全局漂移。

如果 RTK 一恢复就用 3 cm covariance 直接加入，会形成很强的瞬时约束，使 correction 出现拐折甚至地图断层。

### 最终改进：Soft Relock

RTK 恢复后采用 2 s 左右的软恢复：

\[
\Sigma_{eff}(t)=s(t)\Sigma_{RTK}
\]

其中：

```text
刚恢复：s = 10
稳定后：s → 1
```

并同时要求：

- 连续 FIX；
- innovation 方向稳定；
- 无明显 false fix；
- correction 的平移/旋转速率不超过限值。

禁止一次 RTK 点直接把全局轨迹硬拉回去。

---

## 3.6 RTK 慢变误差没有显式建模

矿区的 RTK 几厘米误差并不一定是独立白噪声。边坡、矿卡车体、大型机械、金属构筑物和卫星几何变化均可能产生秒到几十秒的相关误差。

若只把这些误差当作每历元独立噪声，优化器会试图让全局轨迹逐点跟随 RTK 抖动。

### 最终改进：Gauss–Markov 位置偏置

加入低频三维 RTK bias：

\[
b_j=[b_E,b_N,b_U]^T.
\]

RTK 因子变为：

\[
\boxed{
z_k=p(X_k)+R(X_k)l+b_j+n_k
}
\]

bias 采用一阶均值回归过程：

\[
b_{j+1}=\phi b_j+w_j,
\]

\[
\phi=e^{-\Delta t/\tau}.
\]

设稳态 covariance 为 \(\Sigma_\infty\)，则过程噪声：

\[
\boxed{
Q=(1-\phi^2)\Sigma_\infty
}
\]

而不是任意设置随机游走 sigma。

生产参数建议：

```yaml
gnss_position_bias:
  enabled: true
  node_interval_sec: 2.0
  correlation_time_sec: 30.0
  stationary_sigma_xy_m: 0.03
  stationary_sigma_z_m: 0.05
  initial_sigma_xy_m: 0.03
  initial_sigma_z_m: 0.05
```

这个 bias 只能吸收**低频、小幅、相关**的位置误差；通过均值回归和小稳态方差限制，禁止它无限漂移或跟踪单帧 false fix。

---

## 3.7 “0.1°姿态误差”不能简单等价为“必须加 RTK 姿态因子”

姿态误差对远距离点云非常敏感：

\[
e\approx d\delta\theta.
\]

0.1° 对应：

| 距离 | 横向误差量级 |
|---:|---:|
| 20 m | 3.49 cm |
| 40 m | 6.98 cm |
| 60 m | 10.47 cm |
| 100 m | 17.45 cm |

但必须区分：

- **恒定姿态偏置**：主要造成整张地图刚性旋转；
- **帧间姿态变化误差**：造成同一物体重影和地图变厚。

更关键的是，当前 INS attitude 与当前 IMU 可能同源。

### 最终改进

最终软件支持两种模式：

```yaml
gnss_heading:
  source: correlated_ins | dual_antenna_gnss
```

若为 `correlated_ins`：

```yaml
enabled: false
```

连续姿态只用于：

- 初始化 yaw；
- 可视化；
- 时间同步诊断；
- 外参诊断；
- residual monitoring。

若为 `dual_antenna_gnss`：

加入独立 yaw factor：

\[
r_\psi=wrap(\psi(X_k)-\psi_{GNSS}).
\]

推荐：

```yaml
gnss_heading:
  enabled: true
  source: dual_antenna_gnss
  factor_interval_sec: 0.5
  sigma_floor_deg: 0.15
  maximum_sigma_deg: 0.50
  huber_delta: 2.5
  absolute_gate_deg: 1.0
```

默认只加 heading，不加 full RPY。矿区地面和重力已能很好约束 roll/pitch；真正长期弱约束通常是 yaw。

---

# 4. 最终生产架构的状态和因子

## 4.1 全局 Pose 节点

定义 FAST-LIVO2 local frame 为 \(L\)，矿区全局 ENU/mine frame 为 \(M\)，body/物理 IMU frame 为 \(B\)。

FAST-LIVO2 原生 pose：

\[
\bar X_k=T^L_B(t_k).
\]

全局图优化变量：

\[
X_k=T^M_B(t_k)\in SE(3).
\]

RTK bias：

\[
b_j\in\mathbb R^3.
\]

如果存在独立双天线 GNSS heading，不需要额外姿态状态，直接作用于 \(X_k\)。

最终变量：

\[
\boxed{
\mathcal X=\{X_0,\ldots,X_N,b_0,\ldots,b_M\}
}
\]

生产全局图中**不再优化**：

\[
v_k,b^g_k,b^a_k.
\]

也不再加入 raw IMU factor。

这是避免二次重建局部 LIO 和相关信息重复计入的关键。

---

## 4.2 FAST-LIVO2 相对位姿保持因子

相邻 knot 的原生相对运动：

\[
Z^{LIO}_{k,k+1}
=
\bar X_k^{-1}\bar X_{k+1}.
\]

残差：

\[
\boxed{
r_k^{LIO}
=
Log\left[
(Z^{LIO}_{k,k+1})^{-1}
X_k^{-1}X_{k+1}
\right]
}
\]

GTSAM 使用：

```cpp
gtsam::BetweenFactor<gtsam::Pose3>
```

含义是：

> 在短时间/短距离范围内，FAST-LIVO2 原生相对运动是主要局部几何信息；RTK 只允许以低频、连续方式修正累计全局漂移。

---

## 4.3 LIO BetweenFactor covariance

不建议把每个 FAST-LIVO2 pose 的 marginal covariance 直接作为相邻 Between covariance，因为相邻状态高度相关。

最终采用“基础噪声 + 前端退化尺度”模型：

\[
\Sigma_{L,k}=\alpha_k\Sigma_{L,0}.
\]

基础值建议：

```yaml
lio_between:
  sigma_xy_m: 0.03
  sigma_z_m: 0.02
  sigma_roll_deg: 0.06
  sigma_pitch_deg: 0.06
  sigma_yaw_deg: 0.10
```

节点间隔建议固定为 0.5 s。

退化尺度：

\[
1\le\alpha_k\le25.
\]

FAST-LIVO2 新增 `LioQuality` 输出：

```text
stamp
num_effective_lidar_points
point_plane_rms
hessian_min_eigenvalue
hessian_condition_number
visual_inlier_count
eskf_pose_uncertainty_proxy
```

推荐规则：

```text
正常几何：alpha = 1
轻度退化：alpha = 2~4
明显退化：alpha = 5~10
严重退化：alpha = 10~25
```

原则：只允许“退化时放松 LIO”，不允许在未知情况下把 LIO covariance 调到极小来锁死图。

---

## 4.4 RTK 位置因子

### 最优输入：RTK 天线位置

测量：

\[
z_k^{ant}.
\]

预测：

\[
\hat z_k
=p(X_k)+R(X_k)l_{B\rightarrow ant}+b_j.
\]

残差：

\[
\boxed{
r_k^{RTK}=
\hat z_k-z_k^{ant}
}
\]

由于同时存在 bias，最终建议实现自定义：

```cpp
class RtkPositionBiasFactor
  : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>
```

构造参数：

```cpp
poseKey
biasKey
measurement_enu
lever_arm_body
noise_model
```

### 若只有 vendor rear-axle position

预测改成：

\[
\hat z_k
=p(X_k)+R(X_k)l_{B\rightarrow rear}+b_j.
\]

但 covariance 必须额外包含 vendor 从 antenna 转 rear 时姿态误差引入的杆臂项。

---

## 4.5 Gauss–Markov Bias 因子

自定义：

```cpp
class GnssBiasGaussMarkovFactor
  : public gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Point3>
```

残差：

\[
\boxed{
r_j^b=b_{j+1}-\phi b_j}
\]

其中：

\[
\phi=e^{-\Delta t/\tau}.
\]

过程 covariance：

\[
Q_j=(1-\phi^2)\Sigma_\infty.
\]

第一个 bias：

\[
b_0\sim N(0,\Sigma_\infty).
\]

这样 bias 的统计解释完整：它是零均值、有有限稳态方差、有相关时间的 colored GNSS error。

---

## 4.6 稀疏长程 LiDAR 几何因子

生产图不再对所有相邻扫描构建 `ScanPlaneBatchFactor`。

只在以下情况建立长程几何边：

- 重复经过同一区域；
- 两 submap 有足够重叠；
- RTK 粗定位支持候选搜索；
- 点云配准有明确几何可观性；
- 不属于大面积动态矿卡/机械区域。

候选约束来源可为：

- submap-to-submap point-to-plane ICP；
- VGICP；
- 当前已有 plane correspondence 机制重构为 submap 级约束。

长程边必须满足：

```text
minimum overlap
fitness threshold
maximum residual
rotation/translation consistency
Hessian eigenvalue gate
robust kernel
```

其作用是：

> 在 RTK 只有厘米级绝对噪声时，利用真正重复观测的静态几何进一步稳定相对姿态与跨趟一致性，而不是替代 FAST-LIVO2 的短时 local odometry。

---

## 4.7 独立 GNSS Heading 因子

只有 `source == dual_antenna_gnss` 时建立。

建议实现：

```cpp
class GnssHeadingFactor
  : public gtsam::NoiseModelFactor1<gtsam::Pose3>
```

残差：

\[
r_\psi=wrap(\psi_{pred}-\psi_{meas}).
\]

必须统一 heading 语义：

- 全局 frame：ENU；
- body frame：RFU；
- +Y 是车辆前向；
- heading 必须明确是 North-clockwise 还是 ENU yaw counter-clockwise。

禁止直接把厂商 heading 数字当作 ROS yaw 使用。

---

# 5. 最终目标函数

最终生产图的优化问题：

\[
\boxed{
\begin{aligned}
\min_{\{X_k\},\{b_j\}}
&\sum_k
\|r_k^{LIO}\|_{\Sigma_{L,k}^{-1}}^2
\\
&+\sum_m
\rho_P\left(
\|r_m^{RTK}\|_{\Sigma_{RTK,m}^{-1}}^2
\right)
\\
&+\sum_j
\|b_{j+1}-\phi_j b_j\|_{Q_j^{-1}}^2
\\
&+\sum_q
\rho_G\left(
\|r_q^{long-range}\|_{\Sigma_{G,q}^{-1}}^2
\right)
\\
&+\sum_h
\rho_H\left(
\frac{r_h^{heading}}{\sigma_{\psi,h}}
\right)^2.
\end{aligned}
}
\]

最后一项只有独立 heading 存在时启用。

生产图明确不包含：

```text
raw ImuFactor
Velocity state
IMU bias state
相邻扫描高密度 ScanPlaneBatchFactor
同源 INS RPY factor
```

---

# 6. RTK covariance 的完整最终模型

## 6.1 数据接口要求

RTK 驱动最终必须向融合层提供：

```cpp
struct RtkMeasurement {
    double stamp;
    Eigen::Vector3d position_enu;
    Eigen::Matrix3d covariance_enu;
    bool covariance_valid;

    int solution_mode;
    int satellites;
    double h_acc;
    double v_acc;
    double hdop;
    double vdop;
    double correction_age;
    double ambiguity_ratio;

    ReferencePoint reference_point; // ANTENNA or REAR_AXLE
};
```

优先使用天线相位中心位置。

---

## 6.2 Reported covariance

接收机若给完整 covariance，先进行：

1. 对称化：

\[
\Sigma\leftarrow(\Sigma+\Sigma^T)/2;
\]

2. finite 检查；
3. PSD 检查；
4. 特征值下限：防止接收机报不现实的小值；
5. 最大 uncertainty gate。

不要简单无条件 `+= nominal_floor`。

---

## 6.3 Fallback covariance

当 covariance 缺失或全零时：

```yaml
fallback_sigma_xy_m: 0.03
fallback_sigma_z_m: 0.05
```

这只是当前硬件信息不足时的保守工程模型，不应在文档中称为“RTK真实1σ精度”。

---

## 6.4 Model covariance

用于容纳：

- 未完全建模的多路径；
- 坐标转换误差；
- 天线相位中心变化；
- 车辆结构/安装误差；
- vendor filtering 未知误差。

建议：

```yaml
model_sigma_xy_m: 0.015
model_sigma_z_m: 0.025
```

最终值应由 whitened residual/NIS 标定。

---

## 6.5 杆臂姿态误差 covariance

若测量参考点经过姿态进行杆臂转换：

\[
\delta p\approx-R[l]_\times\delta\theta.
\]

因此：

\[
\boxed{
\Sigma_{lever}
=
R[l]_\times\Sigma_\theta[l]_\times^TR^T
}
\]

当前工程 antenna→rear 杆臂长度约 8.14 m。若姿态误差约 0.1°，位置误差量级约 1.4 cm，已经与厘米级 RTK 误差同量级。

这也是为什么生产数据接口应优先使用 antenna position，而不是 vendor rear-axle position。

---

## 6.6 时间误差 covariance

若 RTK 时间戳 1σ 不确定性为 \(\sigma_t\)，车辆速度为 \(v\)：

\[
\Sigma_{time,p}\approx vv^T\sigma_t^2.
\]

如果还使用 heading：

\[
\sigma_{time,\psi}\approx|\omega_z|\sigma_t.
\]

例如：

- 10 m/s；
- 5 ms；

对应 5 cm 位置误差。

因此厘米级融合必须把时间同步当作测量模型的一部分，而不是单纯“消息能对上时间”即可。

---

# 7. RTK 完整性、异常值和恢复逻辑

## 7.1 GNSS 自身质量 Gate

生产默认：

```yaml
quality:
  accepted_modes: [4]
  minimum_satellites: 12
  maximum_hacc_m: 0.15
  maximum_vacc_m: 0.25
  maximum_correction_age_sec: 2.0
  status_max_age_sec: 0.10
  recovery_stable_interval_sec: 0.50
```

如果驱动能提供 DOP/ratio，应加入：

```yaml
maximum_hdop: 2.0
maximum_vdop: 3.0
minimum_ambiguity_ratio: <按接收机定义标定>
```

不能硬编码一个跨厂商 ambiguity ratio 门槛。

---

## 7.2 Temporal integrity

在进入图之前检查：

\[
\Delta p_{rtk},\quad
\Delta v_{rtk},\quad
\Delta a_{rtk}
\]

与车辆物理边界是否一致。

矿卡可设置保守上限，例如：

```yaml
temporal_gate:
  max_speed_mps: 20.0
  max_accel_mps2: 4.0
  max_jump_m: 1.0
```

实际值按车辆工况调整。

---

## 7.3 NIS Gate

如果能够取得当前 global node 的 marginal covariance \(P_k\)，严格使用：

\[
S=HPH^T+R.
\]

三维位置：

```yaml
nis_gate_3d: 11.345
```

若计算开销过高，可按固定频率（例如 2 Hz RTK factor 时每个 factor 都算）获取 marginal covariance，而不是对每个 LiDAR scan 都求。

---

## 7.4 Absolute innovation Gate

即使 NIS 通过，也增加绝对上限防止 covariance 本身异常：

```yaml
absolute_gate_m: 0.50
```

正常连续 FIX 过程中大于 0.5 m 的点原则上拒绝。

RTK 长时失锁恢复则进入 soft relock，而不是按 normal gate 直接拒绝或直接强加入。

---

## 7.5 Robust Kernel

通过上述 gate 后使用 Huber：

```yaml
huber_delta: 2.8
```

注意它作用于白化残差。

职责关系：

```text
covariance = 正常测量可信度
integrity gate = 是否允许进入图
NIS = 预测与观测统计一致性
Huber = 仍然存在的重尾误差
bias = 低频时相关误差
```

任何一个都不能替代另外几个。

---

## 7.6 Soft Relock

定义 RTK 状态机：

```text
UNAVAILABLE
  ↓ 连续FIX
RECOVERING
  ↓ 稳定时间 + innovation合理
LOCKED
  ↓ 失效
UNAVAILABLE
```

`RECOVERING` 中 covariance scale：

\[
s(t)=1+(s_0-1)e^{-t/\tau_r}
\]

建议：

```yaml
relock:
  initial_covariance_scale: 10.0
  time_constant_sec: 0.7
  minimum_duration_sec: 2.0
  max_correction_translation_rate_mps: 0.30
  max_correction_rotation_rate_degps: 0.50
```

这保证 RTK 恢复后全局 correction 平滑回归，而不是地图瞬时折断。

---

# 8. 姿态信息的最终处理

## 8.1 当前 CGI INS attitude

如果它使用的就是当前 CGI IMU，则：

```text
不连续融合
```

保留用途：

- 初始化全局 yaw；
- 检查 IMU/vehicle 固定旋转；
- 检查时间偏差；
- 监测长期 yaw divergence；
- 数据可视化。

## 8.2 独立双天线 GNSS heading

若硬件确认是独立载波相位基线航向：

加入 heading factor。

测量标准差使用逐历元 heading std；没有时以 0.15° 为保守下限。

附加质量条件：

- heading fixed；
- baseline 解有效；
- 双天线状态连续；
- baseline length 与标定一致；
- heading innovation < 1°；
- Huber 2.5。

## 8.3 不建议三维动态姿态 bias

禁止直接新增：

\[
b^R=[b_r,b_p,b_y].
\]

因为它会与：

- IMU gyro bias；
- IMU→vehicle 安装角；
- 多 LiDAR 旋转外参；
- 时间偏差；

发生强耦合。

如果独立 GNSS heading 长期残差明确表现为稳定缓慢偏移，优先重新标定固定安装角。只有确定硬件 heading 本身存在慢变系统偏差时，才允许在独立实验分支加入 1D yaw bias；不纳入默认生产图。

---

# 9. 时间同步：最终必须达到的标准

## 9.1 精度目标

矿山厘米级地图要求建议：

\[
\boxed{|\Delta t|<3\sim5\text{ ms}}
\]

原因：

10 m/s 车辆在 5 ms 内移动：

\[
0.05m.
\]

20°/s 转弯角速度在 5 ms 内旋转：

\[
0.1^\circ.
\]

这已经等于你给出的姿态误差量级。

## 9.2 时间标定方法

必须分别验证：

```text
LiDAR ↔ IMU
RTK position ↔ LIO motion
GNSS heading ↔ LIO yaw-rate（若有独立heading）
```

不要只在单段数据上搜索一个能减小 residual 的 offset 后直接固化。

要求：

- 至少 3 个独立矿区数据段；
- 包含直线、转弯、加减速；
- 求出的 offset 方向和数值具有重复性；
- 改 offset 后独立数据也改善。

## 9.3 在线诊断

记录：

```text
rtk residual vs speed
rtk lateral residual vs yaw-rate
heading residual vs yaw-rate
```

若 residual 与速度显著相关，优先怀疑时间偏差；不要先调 covariance 或 bias。

---

# 10. 杆臂、参考点和旋转外参

## 10.1 统一约定

项目统一：

```text
Global: ENU
Body: RFU
+X: right
+Y: forward
+Z: up
```

所有外参名称统一成：

```text
T_body_lidar_i
lever_body_to_gnss_antenna
lever_body_to_rear_axle
R_body_to_vehicle
```

禁止继续使用方向含义不清的 `extrinsic_T`。

## 10.2 杆臂单元测试

必须建立测试：

\[
p_A^M=p_B^M+R_{MB}l_{BA}.
\]

人工构造：

- yaw=0°；
- yaw=90°；
- roll/pitch 非零；

验证 antenna/rear 点在全局坐标中方向正确。

## 10.3 多 LiDAR 旋转外参

矿山远距离点云对旋转外参极敏感。

例如：

\[
0.05^\circ\ @\ 60m\approx5.2cm.
\]

因此必须做：

```text
单 LiDAR map
任意双 LiDAR map
四 LiDAR map
```

并按距离分层统计厚度。

如果：

```text
单 LiDAR 薄，四 LiDAR 厚
```

优先修多 LiDAR rotation extrinsic，而不是调 RTK。

---

# 11. 全局 Correction 的定义和使用

优化节点：

\[
X_k=T^M_B(t_k).
\]

对应 FAST-LIVO2 local pose：

\[
\bar X_k=T^L_B(t_k).
\]

定义 correction：

\[
\boxed{
C_k=X_k\bar X_k^{-1}
}
\]

任意时间：

\[
X(t)=C(t)\bar X(t).
\]

相邻 correction knot 之间使用 SE(3) 插值：

\[
C(t)=C_kExp\left[
\alpha Log(C_k^{-1}C_{k+1})
\right]
\]

\[
\alpha=\frac{t-t_k}{t_{k+1}-t_k}.
\]

这样 FAST-LIVO2 高频转向、颠簸、俯仰全部保留；RTK 只修改低频全局 envelope。

---

# 12. Correction Knot 频率的最终配置

生产默认：

```yaml
global_correction:
  knot_interval_sec: 0.50
```

即约 2 Hz。

理由：

- RTK 本身作为低频全局约束；
- FAST-LIVO2 高频运动已经保留；
- correction 只需要描述全局 drift，而不是车辆真实高频运动；
- 更高 knot 频率会增加自由度，反而更容易追踪 RTK 厘米级噪声。

节点触发同时允许运动条件：

```yaml
min_translation_m: 1.0
min_rotation_deg: 1.0
```

最终策略是“时间或运动触发”，但平均 correction rate 控制在约 2 Hz。

---

# 13. RTK factor 频率

最终固定：

```yaml
rtk_position:
  factor_interval_sec: 0.50
```

即约 2 Hz。

即使 RTK/INS 输出 10 Hz、20 Hz、100 Hz，也不把每条消息都作为独立 factor。

如果未来取得真正的 raw RTK antenna solution，并能证明历元间统计相关性较弱，可再重新标定 factor rate，但当前项目不应该通过提高 RTK factor 频率来“增强融合”。

---

# 14. 最终地图生成方式

这是生产结果正确性的硬约束。

## 14.1 使用 FAST-LIVO2 原生已去畸变扫描

FAST-LIVO2 输出 `feats_undistort` 后，最终地图点：

\[
p_M=X_{scan,end}\,T_{BL}\,p_L^{undistorted}.
\]

其中：

\[
X_{scan,end}=C(t_{end})\bar X(t_{end}).
\]

禁止再使用第二套 raw-IMU smoother 对扫描内原始点重新 deskew 作为生产地图默认路径。

## 14.2 若必须做到逐点全局 correction

若测试证明长扫描或快速运动时单一 scan-end correction 仍造成可测误差，则：

\[
p_M(t_p)
=
C(t_p)\bar X(t_p)T_{BL}p_L.
\]

这里仍然使用：

```text
FAST-LIVO2 原生连续 trajectory
+
低频 global correction
```

而不是重新估计局部轨迹。

---

# 15. 初始化与全局坐标对齐

## 15.1 Mine/ENU 原点

使用高质量连续 RTK FIX 建立全局平移原点。

## 15.2 Global yaw

优先级：

1. 独立双天线 GNSS heading；
2. 足够直线运动下由 RTK displacement 与 LIO displacement 对齐估计 yaw；
3. 同源 INS attitude 仅用于初始 guess，不作为后续重复约束。

不能用单个位置点估 yaw。

## 15.3 静止判定

当前初始化应增加真正 stationary gate：

```text
vehicle speed < threshold
gyro norm/variance < threshold
acc norm near g
acc variance < threshold
连续满足一定时间
```

推荐：

```yaml
initialization:
  stationary_duration_sec: 2.0
  max_speed_mps: 0.10
  max_gyro_norm_degps: 0.50
```

具体阈值由静止数据标定。

---

# 16. 在线计算架构

现有历史全量 refresh 不能继续作为小时级矿区在线方案。

最终线程职责：

```text
FAST-LIVO2 thread
  └─ 高频 local pose / undistorted scan

Global correction thread ~2 Hz
  ├─ 建立 LIO between
  ├─ RTK factor
  ├─ bias factor
  ├─ optional heading
  └─ iSAM2 update

Long-range geometry worker
  └─ 候选 submap registration / loop/revisit

Diagnostics ~1 Hz
  └─ residual / NIS / timing / performance

Map exporter
  └─ native undistorted scan + C(t)
```

禁止：

```text
每个新节点遍历全部历史 scan
每次 pose 微调全历史重新 deskew
每次更新重建所有 plane correspondence
```

---

# 17. 长程几何约束的最终实现

## 17.1 Submap 构建

每个 correction knot 不必建立 submap。

建议每 5~10 m 或约 2~5 s 建一个局部 submap。

使用原生 FAST-LIVO2 corrected poses 聚合。

## 17.2 候选搜索

利用：

- corrected global position；
- RTK 粗位置；
- 时间间隔；

找候选。

避免相邻 submap 重复构建高密度约束。

## 17.3 几何验证

至少满足：

```text
minimum age: 15 s
minimum overlap: 40%
fitness gate
translation consistency
rotation consistency
minimum Hessian eigenvalue
condition number threshold
```

矿区重复道路可能外观相似，因此 RTK coarse gating 非常重要。

## 17.4 信息矩阵

从注册 Jacobian：

\[
\Lambda=J^TWJ
\]

获得相对可观性。

若 yaw 或横向对应小特征值非常低，则不要给该自由度虚假高权重。

可以对特征值进行上下限裁剪后构造 factor covariance。

---

# 18. FAST-LIVO2 前端需要新增的输出

不修改其估计器数学结构，只增加可用于后端判断的诊断量。

建议新增：

```cpp
struct LioQuality {
    double stamp;
    int effective_lidar_points;
    double point_plane_rms;
    double hessian_min_eigenvalue;
    double hessian_condition_number;
    int visual_inliers;
    bool lidar_degenerate;
    bool visual_degenerate;
};
```

这用于调整：

\[
\alpha_k
\]

而不是让后端再次重复点面优化。

---

# 19. 最终代码重构方案

## 19.1 新增 `GlobalCorrectionFusion`

建议新增：

```text
include/global_correction_fusion.h
src/global_correction_fusion.cpp
include/global_correction_factors.h
```

当前 `RtkFusion` 改名或配置成：

```text
raw_navigation_smoother_experimental
```

保留用于对照，不再由默认 launch 启用。

---

## 19.2 `global_correction_fusion.h`

核心结构：

```cpp
struct LioPoseSample {
    double stamp;
    gtsam::Pose3 local_pose;
    LioQuality quality;
};

struct RtkPositionObservation {
    double stamp;
    Eigen::Vector3d position_enu;
    Eigen::Matrix3d reported_covariance;
    bool covariance_valid;

    int mode;
    int satellites;
    double h_acc;
    double v_acc;
    double hdop;
    double vdop;
    double correction_age;
    double ambiguity_ratio;

    enum class ReferencePoint { ANTENNA, REAR_AXLE } reference_point;
};

struct GnssHeadingObservation {
    double stamp;
    double heading_rad;
    double sigma_rad;
    bool independent;
    bool fixed;
};

struct CorrectionNode {
    size_t index;
    double stamp;
    gtsam::Pose3 local_pose;
    gtsam::Pose3 global_pose;
    bool has_rtk;
    bool has_heading;
};
```

---

## 19.3 `global_correction_factors.h`

实现：

```text
RtkPositionBiasFactor
GnssBiasGaussMarkovFactor
GnssHeadingFactor
```

`BetweenFactor<Pose3>` 和 `PriorFactor` 直接使用 GTSAM。

长程几何最终输出为相对 Pose3 测量后，也优先使用标准 `BetweenFactor<Pose3>`，而不是在全局图里长期保存大量点面 residual。

---

## 19.4 `LIVMapper.cpp`

只做以下接口修改：

- 继续输出原生 FAST-LIVO2 pose；
- 继续输出 `feats_undistort`；
- 新增 `LioQuality`；
- 将 local pose 和 cloud 提交给 `GlobalCorrectionFusion`；
- 不让 RTK 调用前端状态 update；
- 不在 Mapper 内堆积图优化业务逻辑。

---

## 19.5 `rtk_fusion.cpp`

现有代码：

- 保留；
- 重命名配置模式；
- 默认关闭；
- 用于科研对比。

禁止继续在该类中叠加生产新功能，防止两种后端架构混合。

---

## 19.6 `evaluate_wuhu_lio.py`

扩展输出：

```text
rtk_raw_residual.csv
rtk_whitened_residual.csv
rtk_nis.csv
gnss_bias.csv
correction.csv
heading_residual.csv
lio_quality.csv
long_range_factor_stats.csv
performance.csv
```

地图指标继续固定同一批 18 个完整窗口、54 个相同返回点截面，不能每次改 ROI。

---

## 19.7 `rtk_orientation_viewer.py`

修正为 RFU：

```text
X = 右
Y = 前
Z = 上
```

同时显式显示：

```text
ROS/ENU yaw
GNSS heading
转换后的 vehicle forward direction
```

增加 0°、90°、180°、270° 四个 unit test。

---

# 20. 最终参数模板

下面是推荐生产配置的起始标定值。它们是工程参数初值，不代表传感器厂家真实统计量；上线前必须通过本文第 24 节的统计与地图指标重新标定。

```yaml
rtk:
  backend_mode: "global_correction"
  frontend_position_aiding: false

  frames:
    global: "mine_enu"
    local: "lio_local"
    body: "imu_rfu"

  global_correction:
    knot_interval_sec: 0.50
    min_translation_m: 1.0
    min_rotation_deg: 1.0

    lio_between:
      sigma_xy_m: 0.03
      sigma_z_m: 0.02
      sigma_roll_deg: 0.06
      sigma_pitch_deg: 0.06
      sigma_yaw_deg: 0.10
      adaptive_quality: true
      alpha_min: 1.0
      alpha_max: 25.0

  position:
    reference_point: "antenna_preferred"
    factor_interval_sec: 0.50

    fallback_sigma_xy_m: 0.03
    fallback_sigma_z_m: 0.05

    model_sigma_xy_m: 0.015
    model_sigma_z_m: 0.025

    covariance_scale: 1.0
    min_sigma_xy_m: 0.015
    min_sigma_z_m: 0.030
    maximum_sigma_xy_m: 0.15
    maximum_sigma_z_m: 0.25

    nis_gate_3d: 11.345
    absolute_gate_m: 0.50
    huber_delta: 2.8

  quality:
    accepted_modes: [4]
    minimum_satellites: 12
    status_max_age_sec: 0.10
    interpolation_max_gap_sec: 0.05
    recovery_stable_interval_sec: 0.50
    maximum_hacc_m: 0.15
    maximum_vacc_m: 0.25
    maximum_correction_age_sec: 2.0

  temporal_integrity:
    max_speed_mps: 20.0
    max_accel_mps2: 4.0
    max_jump_m: 1.0

  relock:
    enabled: true
    initial_covariance_scale: 10.0
    time_constant_sec: 0.7
    minimum_duration_sec: 2.0
    max_correction_translation_rate_mps: 0.30
    max_correction_rotation_rate_degps: 0.50

  position_bias:
    enabled: true
    node_interval_sec: 2.0
    correlation_time_sec: 30.0
    stationary_sigma_xy_m: 0.03
    stationary_sigma_z_m: 0.05
    initial_sigma_xy_m: 0.03
    initial_sigma_z_m: 0.05

  heading:
    # correlated_ins: 不加入连续因子
    # dual_antenna_gnss: 加入独立 heading 因子
    source: "correlated_ins"
    enabled: false
    factor_interval_sec: 0.50
    sigma_floor_deg: 0.15
    maximum_sigma_deg: 0.50
    absolute_gate_deg: 1.0
    huber_delta: 2.5

  timing:
    fixed_offset_sec: 0.0
    target_absolute_error_sec: 0.005

  long_range_geometry:
    enabled: true
    submap_interval_m: 7.5
    minimum_age_sec: 15.0
    search_radius_m: 10.0
    minimum_overlap_ratio: 0.40
    observability_gate: true
    robust: true

  map:
    use_native_fast_livo_undistorted_scan: true
    use_smooth_global_correction: true
    continuous_point_correction: true
```

### 参数优先级

调参顺序必须固定：

```text
数据源/参考点
→ 坐标系
→ 时间同步
→ 外参
→ RTK covariance
→ LIO relative covariance
→ integrity/NIS
→ bias correlation/variance
→ long-range geometry
```

严禁反过来通过不断调 robust delta 或减小 sigma 遮盖上游问题。

---

# 21. 详细开发任务清单

这里不是“不同版本”，而是**实现最终系统所必须完成的依赖顺序**。最终上线前全部硬性项都必须完成。

## 21.1 数据合同与坐标系

完成：

- 明确 `/imu_data/odometry` position 是 antenna 还是 rear axle；
- 明确它是否经过 INS attitude 杆臂补偿；
- 明确 orientation 的传感器来源；
- 获取 raw RTK antenna topic；
- 获取 covariance/hAcc/vAcc；
- 获取 correction age / DOP / ratio（硬件支持则必须接入）；
- 修复 RFU viewer；
- 写杆臂 unit tests；
- 写 heading conversion unit tests。

验收：任何一个位置测量都可以回答：

```text
它在哪个 frame？
参考点在哪里？
时间戳对应哪个物理时刻？
covariance 的单位和统计语义是什么？
是否经过 INS/滤波？
```

---

## 21.2 GlobalCorrectionFusion 核心图

完成：

- Pose3 correction node；
- FAST-LIVO2 BetweenFactor；
- RtkPositionBiasFactor；
- GaussMarkov bias factor；
- iSAM2；
- correction interpolation；
- 原生 undistorted map export。

这一步建立最终生产主链路。

---

## 21.3 RTK 概率模型与完整性

完成：

- reported/fallback/model covariance 分离；
- lever covariance；
- time covariance；
- covariance PSD 修复；
- GNSS quality gate；
- temporal integrity；
- full NIS；
- Huber；
- soft relock 状态机。

---

## 21.4 FAST-LIVO2 Quality 输出

完成：

- 点面残差 RMS；
- 有效点数；
- Hessian condition；
- 最小特征值；
- 视觉 inlier；
- 退化标志；

并转换成 \(\alpha_k\) 调整 LIO Between covariance。

---

## 21.5 时间与外参标定

完成：

- LiDAR↔IMU 时间；
- GNSS↔LIO 时间；
- 多 LiDAR rotation extrinsic；
- IMU↔GNSS lever；
- vehicle/IMU fixed rotation；
- 独立 heading baseline 与 frame（若有）。

---

## 21.6 长程几何

完成：

- submap manager；
- RTK/correction spatial candidate search；
- submap registration；
- Hessian observability；
- false loop rejection；
- BetweenFactor 输出；
- dirty-set 更新，不全历史 refresh。

---

## 21.7 独立 GNSS heading

仅在硬件确认为独立双天线时启用生产配置。

完成：

- heading frame conversion；
- heading std；
- status gate；
- time alignment；
- yaw factor；
- residual/NIS 监控。

如果没有独立双天线，该模块仍保留代码支持，但生产配置必须保持关闭。

---

## 21.8 故障恢复与长时间运行

必须完成故障注入：

```text
启动无 RTK
RTK loss 10 s
RTK loss 30 s
RTK loss 60 s
FIX/FLOAT 往复
0.5 m false fix
1 m false fix
2 m false fix
covariance 突变
status 延迟
correction age 过期
时间戳人为偏移
```

要求：

- FAST-LIVO2 local map 不受坏 RTK 破坏；
- global correction 在 loss 时冻结/平滑外推；
- false fix 被 gate；
- recover 时软收敛；
- 不出现地图折线或局部撕裂。

---

# 22. 关键 C++ 实现接口建议

## 22.1 RtkPositionBiasFactor

```cpp
class RtkPositionBiasFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>
{
public:
  RtkPositionBiasFactor(
      gtsam::Key pose_key,
      gtsam::Key bias_key,
      const gtsam::Point3& measurement,
      const gtsam::Point3& lever_arm_body,
      const gtsam::SharedNoiseModel& model);

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      const gtsam::Point3& bias,
      boost::optional<gtsam::Matrix&> H_pose = boost::none,
      boost::optional<gtsam::Matrix&> H_bias = boost::none) const override;
};
```

预测：

```cpp
const gtsam::Point3 predicted =
    pose.translation() + pose.rotation().rotate(lever_arm_body_) + bias;
return predicted - measurement_;
```

Jacobian 必须写解析形式并用 numericalDerivative 单元测试。

---

## 22.2 GnssBiasGaussMarkovFactor

```cpp
class GnssBiasGaussMarkovFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Point3>
{
  double phi_;
};
```

残差：

```cpp
return bias_next - phi_ * bias_prev;
```

noise covariance：

```cpp
Q = (1.0 - phi * phi) * Sigma_stationary;
```

---

## 22.3 GnssHeadingFactor

内部禁止使用没有 wrap 的欧拉角直接相减。

至少：

```cpp
double error = normalizeAngle(predicted_heading - measured_heading);
```

并验证 RFU body +Y 前向对应的 heading 计算。

---

# 23. 日志和诊断必须成为算法的一部分

每个 RTK factor 记录：

```text
stamp
solution mode
satellites
reported sigma
model sigma
lever sigma
time sigma
final sigma
raw residual x/y/z
whitened residual
NIS
accepted/rejected
reject reason
robust weight
bias x/y/z
relock state
```

每个 LIO Between factor 记录：

```text
stamp_a/stamp_b
relative translation
relative rotation
alpha
final sigma
front-end quality
```

每个 correction node：

```text
C translation
C rotation
correction velocity
correction angular velocity
```

每个 long-range factor：

```text
source/target
age
RTK distance
registration fitness
overlap
Hessian eigenvalues
condition number
accepted/rejected
```

没有这些日志，后续任何“调权重”都缺乏可解释性。

---

# 24. 最终验收指标

## 24.1 RTK 一致性

目标建议：

```text
3D RTK residual P95 <= 0.10~0.15 m
连续 FIX 下无明显长尾
NIS 分布不过度偏大/偏小
```

但必须注明：

> 因为 RTK 本身进入了优化，这不是独立绝对定位精度。

绝对精度必须用独立控制点、全站仪、独立高等级 GNSS 或 surveyed landmarks 验证。

---

## 24.2 局部地图精细度

现有 18 个完整窗口、54 个固定截面继续作为永久 regression set。

计算：

```text
native thickness
fused thickness
ratio
```

按：

```text
ground
wall
20~40 m
40~60 m
>60 m
```

分组。

生产建议 gate：

```text
median(fused/native thickness ratio) <= 1.10
P90(fused/native thickness ratio) <= 1.25
```

且不能出现当前 34~36 cm 对 13~14 cm 的系统性恶化。

---

## 24.3 Correction 平滑性

记录：

\[
C(t)=X(t)\bar X(t)^{-1}.
\]

正常状态：

```text
correction 应为低频、连续、平滑
```

禁止：

```text
每个RTK点对应一次明显折线
RTK恢复瞬时跳变
高频跟踪厘米级RTK噪声
```

---

## 24.4 Bias 合理性

GNSS bias 应：

- 均值长期接近零；
- 水平典型量级为厘米；
- 垂向略大；
- 变化时间尺度为秒到几十秒；
- 不追踪单历元跳变；
- 不持续增长。

若 bias 接近 0.2~1 m 且长时间不回归，首先判断模型/坐标/时间/杆臂错误，而不是增大 bias covariance。

---

## 24.5 姿态/远距离地图

如果没有独立 heading，评估 FAST-LIVO2 相对姿态稳定性和多 LiDAR 外参。

如果有独立 dual-antenna heading：

```text
heading residual P95
>40 m map thickness
>60 m map thickness
repeated-pass orientation consistency
```

必须同时改善，不能只看 heading residual。

---

## 24.6 性能

矿区在线运行建议：

```text
global graph update P95 < 50 ms @ 2 Hz
no queue backlog growth
memory bounded
1 h continuous run without historical refresh explosion
```

长程几何 worker 可以异步于局部里程计，但不能阻塞 FAST-LIVO2 实时线程。

---

# 25. 开发过程中固定的实验矩阵

每次提交至少运行：

```text
A. Native FAST-LIVO2
B. Current raw-navigation smoother（实验对照）
C. Final GlobalCorrectionFusion
```

C 的所有算法模块均属于最终生产设计，测试时可以通过配置做消融，但不另立“简化版本”。

必须做以下消融：

```text
C - bias
C - long-range geometry
C - adaptive LIO covariance
C - NIS
C - soft relock
C + independent heading（有硬件时）
```

目的不是选择是否实现，而是确认每个模块确实解决对应问题，避免模块之间互相掩盖错误。

---

# 26. 调试决策树

```text
地图变厚
│
├─ Native FAST-LIVO2 短窗口本身已经厚？
│    ├─ 是 → 查 LiDAR/IMU 时间、多LiDAR外参、FAST-LIVO2前端、数据质量
│    └─ 否
│
├─ Native薄，施加 GlobalCorrection 后厚？
│    ├─ 是 → 查 LIO Between权重、RTK covariance、correction 高频抖动
│    └─ 否
│
├─ 单LiDAR薄，四LiDAR厚？
│    ├─ 是 → 查多LiDAR rotation extrinsic
│    └─ 否
│
├─ 厚度随目标距离近似线性增大？
│    ├─ 是 → 姿态、旋转外参、时间优先
│    └─ 否
│
├─ RTK residual 与车速相关？
│    ├─ 是 → GNSS-LIO时间偏差
│    └─ 否
│
├─ 横向 residual 与 yaw-rate 相关？
│    ├─ 是 → 时间/杆臂/heading frame
│    └─ 否
│
├─ RTK residual 连续多秒同方向偏移？
│    ├─ 是 → 检查多路径；再看Gauss-Markov bias
│    └─ 否
│
├─ bias持续增大？
│    ├─ 是 → 坐标/时间/杆臂/global alignment错误
│    └─ 否
│
└─ 单个RTK isolated jump？
     └─ integrity/NIS/Huber处理，绝不能用bias吸收
```

---

# 27. 后续开发必须禁止的做法

1. **禁止**将 RTK position 直接 overwrite 到 FAST-LIVO2 ESIKF。
2. **禁止**继续把当前 raw-IMU + 相邻 scan-plane smoother 作为默认生产后端。
3. **禁止**同一 CGI IMU 已进入 LIO 后，又把由同一 IMU 产生的 INS attitude 当独立高精度因子重复加入。
4. **禁止**因为 RTK 标称几厘米，就给所有历元固定 0.02/0.03 m covariance。
5. **禁止**把 Huber 当成 covariance 或 quality gate 的替代。
6. **禁止**用 dynamic bias 吸收 false fix、时间错位或杆臂错误。
7. **禁止**把降低 factor residual 当作地图变好的证据。
8. **禁止**仅通过增加 point-to-plane factor 数量修 yaw。
9. **禁止**用地面 normal equality 人为制造不存在的航向可观性。
10. **禁止**RTK factor rate 随 LIO 节点频率同步增加。
11. **禁止**在时间 offset 未跨多段数据复现前修改全局配置。
12. **禁止**在 RFU/FLU/ENU/heading 定义未统一时调姿态权重。
13. **禁止**每次图更新全历史重新 deskew / 重建所有几何关联。
14. **禁止**通过不断减小 sigma 获得“看起来更贴 RTK”的结果。
15. **禁止**用被融合的 RTK residual 宣称独立厘米级绝对定位精度。
16. **禁止**让 GNSS bias 每个 RTK 历元一个自由状态。
17. **禁止**使用无限随机游走 bias；生产必须使用有稳态方差的均值回归模型。
18. **禁止**仅根据 0.1° 厂家姿态指标直接开启 full RPY factor。

---

# 28. 推荐代码分支和上线结构

```text
main
├── backend/global_correction          # 最终生产后端
└── backend/raw_navigation_experimental # 当前后端保留对照
```

配置：

```yaml
rtk:
  backend_mode: "global_correction"
```

允许：

```text
global_correction
raw_navigation_smoother_experimental
disabled
```

生产 launch 固定使用 `global_correction`。

---

# 29. 每个开发任务的 Definition of Done

## 数据接口

完成条件：

- raw RTK reference point 明确；
- covariance 非零且单位明确；
- quality 字段可记录；
- 所有 frame 文档化；
- 时间戳语义文档化。

## Global correction graph

完成条件：

- 可以跑完整 bag；
- correction 连续；
- 使用 native undistorted map；
- 不需要历史全量 deskew；
- 不产生 backlog。

## Covariance / NIS

完成条件：

- 每历元 final covariance 可追溯到每个组成项；
- NIS 可输出；
- reject 原因明确；
- 不靠全局单一 scale 掩盖问题。

## Bias

完成条件：

- 有限稳态；
- 不漂移；
- 不吸收 false fix；
- ACF 和 whitened residual 统计改善；
- 地图不恶化。

## Long-range geometry

完成条件：

- 只生成稀疏边；
- false loop 有明确拒绝；
- 可观性不足不强加 6DOF 高权重；
- 对重复通过区域地图厚度有正收益。

## Heading

完成条件：

- 数据源确认为独立；
- heading frame unit test 全通过；
- 时间同步验证；
- >40 m/60 m 地图收益可测；
- 近距离地图不恶化。

## 长时间运行

完成条件：

- ≥1 h；
- queue 不持续增长；
- 内存有界；
- iSAM2 P95 满足实时要求；
- RTK loss/recovery 不破坏 local map。

---

# 30. 最终误差预算建议

为了让调参有明确目标，可按 60 m 感知距离建立误差预算。

假设希望远距离跨帧地图误差控制在约 15 cm 内：

若全局位置误差约 5 cm，则姿态相对误差预算：

\[
\delta\theta
\lesssim
\frac{\sqrt{0.15^2-0.05^2}}{60}
\approx0.135^\circ.
\]

因此：

- 0.1°级独立 heading 是有价值的；
- 0.3°级帧间姿态变化明显不可接受；
- 多 LiDAR 外参 0.05° 都可能贡献约 5 cm；
- 5 ms 时间误差在快速转弯时也可贡献约 0.1°。

这说明最终精度不是通过“RTK位置更强”单独获得的，而是：

```text
局部LIO相对姿态
+ 多LiDAR旋转外参
+ 时间同步
+ RTK全局位置
+ 独立heading（若有）
```

共同决定。

---

# 31. 针对当前项目的最终优先级

按实际影响排序：

### P0：立即修复并作为架构基础

1. 停止生产后端二次重建局部 LIO；
2. 新建 GlobalCorrectionFusion；
3. 最终地图使用 FAST-LIVO2 native undistorted scan；
4. 明确 RTK reference point/covariance 数据合同；
5. 修正 RFU/heading 工具；
6. 建立固定 54 截面 regression。

### P1：决定厘米级融合是否真正可靠

7. 完整 RTK covariance；
8. NIS + integrity + Huber；
9. soft relock；
10. Gauss–Markov RTK position bias；
11. 时间同步至 3~5 ms；
12. 多 LiDAR rotation extrinsic 精修。

### P2：提高弱几何和重复路径的一致性

13. LIO quality 自适应 Between covariance；
14. 稀疏 long-range submap factors；
15. 独立 dual-antenna heading（硬件存在时）。

这里的 P0/P1/P2 表示工程依赖和优先级，不是三个不同算法版本；生产上线时应完成所有适用于当前硬件的项目。

---

# 32. 最终判断：当前算法到底哪里错、该怎样走

当前项目中，RTK 融合的大方向没有错：

- RTK 不进 FAST-LIVO2 前端；
- 有杆臂；
- 有质量 gate；
- 有鲁棒核；
- RTK factor 有限频；
- 有失锁处理；

这些都值得保留。

真正的问题是：

> **后端承担了它不应该承担的“重新生成局部运动”职责。**

已有消融已经显示，地图从约 13~14 cm 恶化到 34~36 cm 的主要原因是后端修改后的跨帧位姿，而不是扫描内去畸变本身。

因此最终算法应把职责重新划分：

```text
FAST-LIVO2
  = 高频局部运动、扫描内运动、短时相对姿态、局部几何

GlobalCorrectionFusion
  = 低频全局漂移、RTK概率约束、慢变GNSS误差、重锁、跨趟稀疏几何

Independent GNSS heading
  = 仅在真正独立时提供绝对yaw
```

对于你给出的矿山 RTK 特性：

- 位置存在几厘米误差：不能简单强拉，需要逐历元 covariance + colored bias；
- 姿态可能有约 0.1°误差：首先判断它是不是独立 heading；若是同源 INS attitude，不应重复融合；
- 远距离地图要求高：时间同步和多 LiDAR 旋转外参与 RTK 本身同等重要；
- RTK 失锁/恢复不可避免：必须通过状态机 + soft relock，而不是硬切 factor；
- 矿区局部几何可能退化：应该通过 FAST-LIVO2 quality 和稀疏长程几何改善，而不是用第二套弱局部 LIO 替代原前端。

最终工程目标应从：

```text
“让融合轨迹更靠近RTK”
```

转变成：

```text
“让FAST-LIVO2的局部几何基本不变，
同时把长期漂移平滑、统计一致地约束到RTK全球坐标系。”
```

这才是该矿山项目后续所有开发、调参、验收和故障分析应遵循的统一原则。

---

# 33. 最终上线检查表

上线前必须逐项打勾：

- [ ] FAST-LIVO2 native map regression 已冻结；
- [ ] 18 窗口 / 54 截面可一键自动评估；
- [ ] 生产 backend 已切换到 `global_correction`；
- [ ] 当前 raw navigation smoother 仅作为 experimental；
- [ ] RTK position reference point 已确认；
- [ ] RTK antenna lever 已验证；
- [ ] ENU/RFU/frame convention 已统一；
- [ ] orientation viewer 已修复；
- [ ] RTK reported covariance 能正确读取；
- [ ] fallback/model/lever/time covariance 已拆分；
- [ ] covariance PSD 检查已实现；
- [ ] GNSS quality gate 已实现；
- [ ] temporal integrity 已实现；
- [ ] NIS gate 已实现；
- [ ] Huber 已实现；
- [ ] soft relock 已实现；
- [ ] Gauss–Markov position bias 已实现；
- [ ] bias 有限稳态且日志可检查；
- [ ] FAST-LIVO2 quality 输出已实现；
- [ ] adaptive LIO Between covariance 已实现；
- [ ] LiDAR↔IMU 时间已复核；
- [ ] GNSS↔LIO 时间达到约 3~5 ms 目标；
- [ ] 多 LiDAR rotation extrinsic 已通过单/双/四 LiDAR 对照；
- [ ] long-range geometry 有严格 observability gate；
- [ ] 若启用 heading，已确认其真正独立于当前 IMU；
- [ ] heading RFU/ENU 定义和时间已单测；
- [ ] RTK loss 10/30/60 s 故障注入已通过；
- [ ] false fix 0.5/1/2 m 故障注入已通过；
- [ ] 1 h 连续运行无 backlog 增长；
- [ ] 地图局部厚度没有系统性劣化；
- [ ] RTK consistency、NIS、bias、correction 均有自动报告；
- [ ] 独立绝对精度使用外部控制点评估，而非使用被融合的 RTK 自证。

---

## 结语

这个项目真正需要的不是“再向图里加入更多传感器因子”，而是把信息来源的职责和统计关系理顺。

最值得坚持的一条设计原则是：

\[
\boxed{
\text{不要用厘米级 RTK 的全局优势，去交换 FAST-LIVO2 已经拥有的局部几何优势。}
}
\]

最终系统应让两者各自做最擅长的事情：FAST-LIVO2 保持局部连续和高频相对精度；RTK 提供低频绝对坐标和长期漂移约束；Gauss–Markov bias、完整性门控、soft relock 和稀疏长程几何负责处理中间那些现实世界中不可避免的非理想误差。
