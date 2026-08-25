# FAST-LIVO2（纯LIO模式）与RTK融合思路
## ——面向 Codex Agent 的工程实现与渐进改造指南

> **文档用途**  
> 本文档用于指导 Codex Agent 在现有 FAST-LIVO2 工程基础上，逐步实现“纯 LIO 局部几何 + RTK 低频全局约束”的融合后端。  
> 目标不是让 FAST-LIVO2 的每个局部 Pose 逐点贴合 RTK，而是在尽量保留纯 LIO 局部地图质量的前提下，使全局轨迹长期稳定地处于 RTK 轨迹邻域内，并将全局偏差控制在工程要求范围内。
>
> **最重要的工程原则**  
> 本文档给出的是算法主线、数学意图和推荐实现结构。**实际编写代码时，必须首先阅读并理解当前 FAST-LIVO2 项目的真实代码逻辑、数据流、线程模型、坐标系、状态定义、时间戳策略、地图更新方式和已有优化模块，再根据实际情况进行适度、科学的实现调整，也可以根据实际算法需要来调整目前已有的回环优化功能。**  
> 允许改变类名、文件组织、接口形式、控制点参数化方式、优化频率、缓存结构以及部分数学实现细节，但必须保持本文档定义的主线思想不变：
>
> 1. FAST-LIVO2 的纯 LIO 局部估计继续负责局部几何；
> 2. RTK 不直接高频修改 FAST-LIVO2 内部 LIO 状态；
> 3. RTK 主要作用于独立的低频连续全局修正场；
> 4. LIO–RTK 距离越小，RTK 约束越弱；距离越大，RTK 约束连续增强；
> 5. 修正必须空间连续、逐渐释放，不允许出现“突然被 RTK 拉正”的局部跳变；
> 6. RTK 健康状态与 LIO–RTK 残差大小必须分开处理；
> 7. 第一版优先实现简单、可解释、可测量的核心方案，再逐步加入高级功能。

## 0.1 正式算法术语与物理解释

算法主模型统一称为 **Distance-Adaptive Radial Elastic Constraint**
（距离自适应径向弹性约束），等价势能表述为 **Radial Adaptive
Elastic Potential**。对时间对应的全局状态与健康 RTK 观测定义

\[
e_i=p_i^G-p_i^{RTK},\qquad d_i=\|e_i\|,
\]

\[
\boxed{F_i=-k_0q_i\alpha(d_i)e_i},\qquad \alpha'(d)\geq0.
\]

因此 \(\|F_i\|=k_0q_i\alpha(d_i)d_i\)：距离和刚度同时增强恢复力；
拉回后两者自然减小，但不强制 \(d=0\)。实现链路必须理解为

```text
距离 d → 弹性刚度 α(d) → 径向恢复力 → C(s) 低频连续释放
```

弹性约束连接的是 `RTK点 ↔ global corrected状态 T^G`，不是
`RTK点 ↔ 原始T^L`。`T^L/T_slam` 提供局部运动强先验而非硬锁死，目标是
保护局部结构“不被RTK噪声乱动”，而不是保护轨迹“绝对不动”。

“RTK Magnetic Field”仅保留为空间直觉和 RViz2 名称；球壳、颜色不得反向
定义优化权重。正常漂移由弹性势场处理；不可由有界连续 \(C(s)\) 吸收的
非刚性失效仍必须冻结、隔离并分段重定位。

---

# 1. 项目问题定义

当前工作场景为矿山，FAST-LIVO2 仅考虑 LIO 模式。

已有现象：

- 纯 LIO 模式下，局部地图点云质量很好，短距离内几乎没有明显重影；
- 融入 RTK 后，尤其在起步前约 30 m 范围内，优化后的点云出现约 20 cm 量级的明显重影；
- RTK 的绝对位置在大尺度上可靠，可以有效抑制 LIO 长期漂移；
- RTK 在局部范围仍存在厘米级位置误差、零点几度姿态误差（会带来几十厘米的重影）、时间同步误差或其他局部波动；
- 如果固定提高 RTK 权重，局部地图会被 RTK 带偏；
- 如果固定降低 RTK 权重，则全局 LIO 漂移无法得到有效限制。

因此，本问题不能继续通过简单调节“RTK 固定权重”解决。

核心目标是：

> **近距离范围内主要相信 LIO；远距离偏离时逐渐增强 RTK 恢复力；当 LIO 被拉回 RTK 邻域后，RTK 约束自动减弱，让 LIO 重新恢复局部自由度。**

工程目标可暂定为：

- 局部点云质量尽可能接近纯 LIO；
- 不因 RTK 小尺度波动产生新的明显重影；
- 全局轨迹长期跟随 RTK 的低频趋势；
- 正常 RTK 条件下，位置尽量维持在 RTK 轨迹约 0.3 m 邻域以内；
- 不要求 LIO 每个 Pose 与 RTK 点完全重合；
- 修正必须连续，不出现明显位置跳变、姿态跳变或地图折线。

---

# 2. 总体算法架构

推荐最终架构：

\[
\boxed{
T_i^G = C(s_i)\,A_0\,T_i^L
}
\]

其中：

- \(T_i^L\)：FAST-LIVO2 原始纯 LIO 输出；
- \(A_0\)：LIO 局部坐标系到 RTK/全局坐标系的一次性刚性对齐；
- \(C(s)\)：沿累计里程 \(s\) 变化的低频连续修正场；
- \(T_i^G\)：最终用于发布全局轨迹和重建全局地图的 Pose。

职责必须清楚分离：

### FAST-LIVO2 / \(T^L\)

负责：

- 高频运动估计；
- LiDAR–IMU 局部配准；
- 局部地图几何；
- 短时间相对精度。

原则：

> **不要因为 RTK 误差而破坏 FAST-LIVO2 内部已经表现良好的局部 scan matching。**

### 全局刚体对齐 / \(A_0\)

负责：

- 将 LIO local frame 对齐到 RTK global frame；
- 吸收初始固定 translation / yaw 等坐标系差异。

### 连续修正场 / \(C(s)\)

负责：

- 抵消 LIO 的长期漂移；
- 将 RTK 的绝对约束低频、平滑地施加到最终轨迹；
- 避免 RTK 逐帧直接改变 LIO Pose；
- 保持局部轨迹形状尽可能接近原始 LIO。

---

# 3. 为什么不要直接把 RTK 强约束到 FAST-LIVO2 每个 Pose

禁止优先采用如下思路：

```text
FAST-LIVO2 Pose_i
    ↓
RTK factor
    ↓
直接修改 Pose_i
    ↓
该修改后的 Pose 又进入后续局部地图 / scan matching
```

这种结构容易造成：

1. RTK 局部厘米级误差直接传递到局部地图；
2. RTK 姿态误差被几何放大；
3. 当前帧被 RTK 推偏后，又成为后续 scan-to-map 的参考；
4. 小误差可能通过地图递归积累；
5. 最终产生原本纯 LIO 没有的重影。

推荐改成：

```text
FAST-LIVO2
    ↓
原始纯 LIO 轨迹 T^L
    ↓
独立全局后端
    ↓
A0 + C(s)
    ↓
最终全局轨迹 T^G
```

RTK 只影响 \(C(s)\)，而不是直接高频干预 FAST-LIVO2 内部 LIO 估计。

---

# 4. 首先检查 20 cm 重影是否来自 yaw，而不仅仅是 RTK 位置

RTK 姿态误差不能忽略。

如果 yaw 误差为：

\[
0.3^\circ
\]

在 30 m 空间尺度上，其横向误差约为：

\[
30\tan(0.3^\circ) \approx 0.157m
\]

约 15.7 cm。

如果 yaw 误差为：

\[
0.4^\circ
\]

则约为：

\[
30\tan(0.4^\circ) \approx 0.21m
\]

已经与观测到的约 20 cm 点云重影处于同一数量级。

因此代码改造前必须优先排查：

- RTK heading/yaw 是否被高频加入优化；
- RTK yaw 的真实统计噪声；
- 起步阶段 yaw 初始化方式；
- RTK 天线 lever-arm；
- 时间同步误差；
- LIO 与 RTK 的 frame 定义；
- yaw 角符号、ENU/NED、轴顺序、度/弧度问题。

第一版算法**不推荐直接使用高频 RTK yaw factor**。

---

# 5. 距离自适应径向弹性约束

用户希望实现的物理直觉是：

> 在对应 RTK 轨迹点附近存在一个球形磁场。  
> LIO 越靠近 RTK，磁场越弱；  
> LIO 越偏离 RTK，磁场越强；  
> 当轨迹被拉回来以后，磁场强度随距离减小而自然下降。

对于第 \(i\) 个 LIO 状态：

\[
p_i^G
\]

为全局修正后的 LIO 位置。

通过**时间对应/时间插值**获得同一时刻的 RTK 位置：

\[
p_i^{RTK}
\]

定义：

\[
e_i = p_i^G-p_i^{RTK}
\]

以及距离：

\[
\boxed{
d_i=\|e_i\|
}
\]

径向恢复方向：

\[
-\frac{e_i}{d_i}
\]

弹性刚度由一个连续函数：

\[
\boxed{
\alpha_p(d)
}
\]

决定。

要求：

\[
\alpha_p'(d)\ge 0
\]

即距离越大，RTK 约束越强。

---

# 6. 球形磁场只用于解释与RViz2

不要优先实现：

```text
0–10 cm   -> weight A
10–20 cm  -> weight B
20–25 cm  -> weight C
25–30 cm  -> weight D
```

因为这样会使：

```text
19.99 cm -> 20.01 cm
```

时权重发生阶跃。

正确理解应该是：

> **球壳用于解释；数学实现采用连续、无级、光滑变化的径向函数。**

概念上可以仍然划分为：

1. 软核区；
2. 轻牵引区；
3. 强化区；
4. 强束缚区。

但数学上所有区域连续连接。

---

# 7. 推荐的连续径向增益函数

推荐使用五次 smoothstep：

\[
S(x)=
\begin{cases}
0,&x\le 0\\
6x^5-15x^4+10x^3,&0<x<1\\
1,&x\ge 1
\end{cases}
\]

推荐：

\[
\boxed{
\alpha_p(d)=
\alpha_0+
(\alpha_1-\alpha_0)
S\left(
\frac{d-d_0}{d_1-d_0}
\right)
+
(\alpha_2-\alpha_1)
S\left(
\frac{d-d_1}{d_2-d_1}
\right)
}
\]

第一版距离参数可以作为初始值：

```text
d0 ≈ 0.10 m
d1 ≈ 0.20 m
d2 ≈ 0.27 ~ 0.28 m
```

含义：

```text
0.00 ────── 0.10 ───────── 0.20 ─────── 0.27/0.28 ── 0.30 m
      极弱 RTK        缓慢增强       明显增强          强束缚
```

这些数值只是**初始工程参数**。

最终必须从真实 bag 数据统计得到，而不是长期固定使用经验值。

---

# 8. 不要把 α 简单理解为“RTK参与百分比”

如果实现：

\[
\Lambda_i^{eff}=
\alpha_i\Lambda_i^{RTK}
\]

则：

\[
\Sigma_i^{eff}
=
\frac{\Sigma_i^{RTK}}{\alpha_i}
\]

标准差满足：

\[
\boxed{
\sigma_i^{eff}
=
\frac{\sigma_i^{RTK}}{\sqrt{\alpha_i}}
}
\]

因此：

```text
alpha = 0.05
```

不应该被口语化解释成：

```text
RTK 只参与 5%
```

更推荐在代码和配置里直接使用：

```text
effective_rtk_sigma(d)
```

或者至少明确：

```text
alpha(d) = information scaling
```

不要让调参人员误解其物理意义。

---

# 9. 第一版建议的弹性刚度

假设 RTK 正常时：

```text
sigma_rtk ≈ 0.02 ~ 0.04 m
```

可考虑：

### 0 ~ 0.10 m：软核区

让 RTK 等效 sigma 很大，例如：

```text
sigma_eff ≈ 0.30 ~ 0.50 m
```

此时仅保留非常轻的全局方向感。

### 0.10 ~ 0.20 m：轻牵引区

逐渐降低：

```text
sigma_eff ≈ 0.12 ~ 0.20 m
```

仍主要相信 LIO。

### 0.20 ~ 0.27 m：强化区

RTK 信息快速但连续增强。

### 0.27 ~ 0.30 m：强束缚区

逐渐恢复到：

```text
RTK nominal covariance
```

或接近 nominal information。

关键行为必须满足：

\[
d\downarrow
\Rightarrow
\alpha(d)\downarrow
\]

即轨迹被拉近以后自动“松手”。

---

# 10. 严格的径向弹性势能定义

如果直接定义：

\[
J(d)=\frac12\alpha(d)d^2
\]

由于 \(\alpha\) 与 \(d\) 有关，其梯度会多出：

\[
\frac12\alpha'(d)d^2
\]

因此如果希望实际恢复力严格满足：

\[
\boxed{
F(d)=k_p\alpha(d)d
}
\]

则推荐先定义力，再积分得到势能：

\[
\boxed{
\rho_p(d)
=
k_p
\int_0^d
\alpha_p(r)r\,dr
}
\]

从而：

\[
\boxed{
\rho_p'(d)
=
k_p\alpha_p(d)d
}
\]

论文或最终算法文档推荐使用这个严格定义。

---

# 11. 第一版工程实现采用外层固定刚度

为了降低实现复杂度，第一版不要求立刻实现复杂自定义势能。

推荐 IRLS / outer-loop 风格：

```text
1. 使用当前 C(s) 计算 global LIO trajectory
2. 根据 timestamp 获取对应 RTK
3. 计算 d_i
4. alpha_i = alpha(d_i)
5. 本轮 nonlinear optimization 中固定 alpha_i
6. 优化 C(s)
7. 下一轮重新计算 d_i / alpha_i
```

即：

\[
\boxed{
\Lambda_i^{eff}
=
q_i^{RTK}\,
\alpha(d_i)\,
\Lambda_{i,nom}^{RTK}
}
\]

注意：

> **不要在一次 Gauss-Newton/LM 内层 iteration 中随着变量变化不断重新切权重。**

第一版外层更新更容易控制、调试和定位问题。

---

# 12. RTK健康度 q 与弹性距离 d 必须完全分离

定义：

\[
q_i^{RTK}\in[0,1]
\]

只由 RTK 自身状态决定，例如：

- FIX / FLOAT / INVALID；
- 组合导航状态；
- covariance；
- 时间同步状态；
- 数据延迟；
- 丢包；
- 明确跳变；
- 驱动层质量字段；
- 是否满足项目既定 RTK 可用条件。

严禁使用：

```text
LIO–RTK 残差大
    ↓
认为 RTK 不可信
    ↓
降低 RTK 权重
```

因为本项目已经明确采用如下前提：

> **RTK健康时，大尺度残差主要视为 LIO 漂移。**

因此：

\[
d\uparrow
\Rightarrow
\alpha(d)\uparrow
\]

不能被传统“大残差降权”鲁棒核抵消。

---

# 13. 连续低频修正场 C(s)

建议第一版：

\[
\xi(s)=
[
\delta x(s),
\delta y(s),
\delta z(s),
\delta\psi(s)
]^T
\]

只做：

\[
\boxed{x,y,z,yaw}
\]

4DOF 修正。

再由：

\[
C(s)=\exp(\xi(s))
\]

构造修正变换。

第一版不建议动态修改：

```text
roll
pitch
```

原因：

- IMU + LIO 对重力方向通常已有很强短时约束；
- 高频修改 roll/pitch 更容易破坏点云局部结构；
- RTK / INS roll/pitch 如果确实存在低频价值，可以后续通过 bag 再验证。

---

# 14. C(s) 推荐使用稀疏 B-spline 或等价平滑参数化

推荐第一版：

```text
cubic B-spline
```

沿累计里程 \(s\) 建立控制点。

初始控制点间距可设：

```text
10 ~ 20 m
```

但注意：

> **控制点间距不是子地图长度，也不是实际形变尺度。**

即使：

```text
10 m 一个 knot
```

通过相邻 knot 平滑正则，实际修正仍可以在：

```text
30 ~ 50 m
```

甚至更长尺度缓慢释放。

因此不要重新把问题退化成：

```text
30m / 50m 一个刚性子地图
```

---

# 15. 修正场平滑正则

第一版优先只使用两组正则：

\[
\boxed{
J_{smooth}
=
\lambda_1
\sum_k
\|\xi_{k+1}-\xi_k\|_{W_1}^2
+
\lambda_2
\sum_k
\|\xi_{k+1}-2\xi_k+\xi_{k-1}\|_{W_2}^2
}
\]

第一项：

```text
限制单位距离内修正变化速度
```

第二项：

```text
限制修正变化速度本身突然变化
```

其目标是：

> RTK 可以让 100 m 轨迹逐渐弯回来几十厘米，但不能让 5~10 m 局部轨迹突然发生明显几何变形。

第一版不要同时堆入过多 5m/10m/20m local-rigidity factor。

如果真实测试仍发现局部地图被扭曲，再考虑逐步增加显式 local rigidity。

---

# 16. 必须解决 A0 与 C(s) 的 Gauge Freedom

模型：

\[
T_i^G=C(s_i)A_0T_i^L
\]

存在潜在自由度重叠：

- \(A_0\) 可以整体平移；
- 所有 \(C(s)\) 也可以整体反向平移；
- 最终 \(T^G\) 不变。

因此必须固定：

\[
\boxed{
C(s_0)=I
}
\]

等价于：

```text
xi_0 = 0
```

对第一个 correction knot 加强 prior。

这样：

```text
A0    -> 负责一次性初始 global alignment
C(s)  -> 负责之后的长期漂移修正
```

避免优化病态和变量之间互相“抢解释权”。

---

# 17. 0.3 m 外层安全磁场

为了接近工程上：

```text
global error <= ~0.3 m
```

的目标，可增加连续外层 barrier-like soft potential。

推荐从：

```text
d_b ≈ 0.26 ~ 0.27 m
```

开始增强。

例如：

\[
J_{bound}(d)
=
\lambda_b
\tau^2
\operatorname{softplus}
\left(
\frac{d-d_b}{\tau}
\right)^2
\]

其中：

```text
tau ≈ 0.01 ~ 0.02 m
```

此项仍然是连续软约束。

不要错误宣称：

```text
它可以数学上绝对保证 d < 0.300000 m
```

更准确的表述是：

> 接近 0.3 m 时，代价快速连续增加，使优化结果在正常 RTK 条件下极难继续向外漂移。

---

# 18. 数值求解必须符合“慢慢拉回”的物理意图

虽然磁场满足：

\[
d\downarrow
\Rightarrow
\alpha(d)\downarrow
\]

但非线性优化器的一步可能仍然过大。

因此推荐：

- LM / trust-region；
- 外层更新磁场权重；
- 内层一轮优化固定权重；
- 对 correction knot 单次更新设置合理上限；
- 检查 damping；
- 必要时限制单次：
  - translation correction；
  - yaw correction。

目标：

> 实际优化过程也表现为连续渐进回拉，而不仅仅是理论势场连续。

---

# 19. 第一版姿态策略

第一版建议：

\[
\boxed{
\text{不要直接加入瞬时 RTK yaw factor}
}
\]

但是：

\[
C(s)
\]

仍保留 yaw 自由度。

让 RTK **位置轨迹的长基线几何方向**约束低频 yaw。

直观上：

- RTK 单个 heading 可能有 0.x° 波动；
- 多个 RTK 高精度位置点在长距离上形成的轨迹方向，可能比单个瞬时 heading 更稳定。

因此推荐优先：

```text
RTK position
    ↓
global translation + low-frequency yaw
```

第二阶段，如果证明全局 yaw 仍不足，再加入 RTK heading。

---

# 20. 如果后续加入 RTK yaw 磁场

必须独立定义：

\[
e_R=
\log(R_{RTK}^{-1}R_G)
\]

\[
\theta=\|e_R\|
\]

再定义：

\[
\alpha_R(\theta)
\]

不要把“米”和“度”混入同一残差半径。

软核范围不要拍脑袋固定为 0.15°。

应该根据真实 RTK heading 噪声：

\[
\sigma_{\psi,RTK}
\]

设置：

\[
\boxed{
\theta_0
\approx
1.5\sim2.0\,\sigma_{\psi,RTK}
}
\]

例如：

```text
sigma_yaw ≈ 0.3°
```

则软核可从：

```text
0.45 ~ 0.60°
```

附近开始考虑。

---

# 21. 第一版暂时不加入“预测磁场”

高级方案可以根据：

\[
\frac{d\bar d}{ds}
\]

判断 LIO 是否正在持续向外漂，从而提前增强 RTK。

但第一版不要加入。

原因：

- 增加滤波窗口；
- 增加预测尺度；
- 增加导数噪声问题；
- 增加调参耦合；
- 增加故障定位复杂度。

先验证：

```text
连续 C(s)
+
距离自适应磁场
+
平滑正则
```

能否实现目标。

只有出现：

```text
总是到 25~28 cm 才开始回收，提前量明显不足
```

时，再考虑预测磁场。

---

# 22. 时间同步是前置硬条件

绝对不要根据：

```text
空间最近 RTK 点
```

作为当前 LIO 的匹配 RTK。

必须根据 timestamp：

\[
p^{RTK}(t_i)
\]

做时间对应和必要插值。

例如车辆速度：

```text
5 m/s
```

仅：

```text
40 ms
```

时间偏差就会造成：

```text
0.20 m
```

的位置错配。

这已经足以制造当前关注的重影量级。

因此 Codex Agent 在实现优化前，应首先检查：

- ROS header.stamp；
- 传感器驱动时间源；
- system time / GNSS time；
- 是否存在 callback 延迟误当测量时间；
- RTK 插值；
- LIO Pose timestamp 定义；
- IMU/LiDAR 时间同步方式。

---

# 23. RTK天线 Lever-Arm 必须正确处理

RTK 测量通常位于 GNSS 天线相位中心。

FAST-LIVO2 Pose 可能表示：

- IMU frame；
- body frame；
- LiDAR frame；
- 或自定义状态 frame。

因此 RTK 预测不应简单写：

\[
p(T_i^G)-p_i^{RTK}
\]

正确形式应根据真实 TF / extrinsic 定义：

\[
\boxed{
p_i^{pred}
=
\operatorname{trans}
\left(
T_i^G T_B^A
\right)
}
\]

其中：

```text
T_B^A
```

为 body/IMU/LiDAR 参考点到 RTK 天线的 lever-arm。

尤其在：

- 转弯；
- 坡道；
- 车体姿态变化；

时，lever-arm 误差会显著影响 RTK 残差。

---

# 24. 起步阶段 A0 的 yaw 初始化必须谨慎

当前问题在起步前约 30 m 更明显，因此要重点检查：

```text
A0 的 yaw 是如何初始化的？
```

禁止用非常短的 RTK 基线直接估计稳定 yaw。

因为几厘米位置误差在几米基线上可能转换成很大的方向误差。

推荐：

### 阶段 1

初始化 translation。

### 阶段 2

积累足够运动基线，例如：

```text
20 ~ 50 m
```

根据真实轨迹条件自适应。

### 阶段 3

稳定估计 LIO local frame 到 RTK global frame 的 yaw。

### 阶段 4

固定/强约束 A0。

之后：

```text
长期小漂移交给 C(s)
```

而不是让 A0 和 C(s) 同时自由漂。

---

# 25. 第一版总目标函数

推荐保持简单：

\[
\boxed{
J=
J_{RTK-mag}
+
J_{smooth}
+
J_{bound}
+
J_{anchor}
}
\]

其中：

## RTK磁场

\[
J_{RTK-mag}
=
\sum_i
q_i^{RTK}
\rho_p(d_i)
\]

且：

\[
\rho_p'(d)
=
k_p\alpha_p(d)d
\]

## 修正场平滑

\[
J_{smooth}
=
\lambda_1
\sum_k
\|\xi_{k+1}-\xi_k\|^2
+
\lambda_2
\sum_k
\|\xi_{k+1}-2\xi_k+\xi_{k-1}\|^2
\]

## 外层安全势场

\[
J_{bound}
\]

接近工程上限时快速增强。

## Gauge Anchor

\[
J_{anchor}
=
\|\xi_0\|^2_{W_0}
\]

保证：

\[
C(s_0)\approx I
\]

---

# 26. 推荐工程运行频率

不要让全局后端与 FAST-LIVO2 高频状态估计使用相同频率。

一个可行初值：

```text
FAST-LIVO2：
    保持原有频率，例如 50~100+ Hz

LIO keyframe：
    每 1~2 m
    或每 0.5~1.0 s

RTK factor：
    1~5 Hz
    根据真实 RTK 频率和车辆速度调整

C(s) spline knot：
    每 10~20 m

global optimization：
    约 1~2 Hz
    或车辆每行驶 2~5 m 更新一次
```

必须根据：

- CPU；
- RTK频率；
- 车速；
- 矿山规模；
- 当前 FAST-LIVO2 线程负载；

进行实测调整。

不要为了照抄本文数字而破坏实时性。

---

# 27. 输出轨迹与地图策略

推荐同时保留两套 Pose：

```text
T_LIO_local
T_global_corrected
```

## FAST-LIVO2 内部地图

继续使用：

```text
T_LIO_local
```

避免 RTK 反馈污染 scan matching。

## 最终全局点云

离线或在线重建时：

```text
point_global
=
T_global_corrected
*
point_lidar
```

从而获得：

- 保留局部 LIO 几何；
- 又拥有全局 RTK 对齐的地图。

如果现有项目必须在线维护全局地图，则 Codex Agent 需要根据真实 map data structure 决定：

- 保存 keyframe point cloud 后按 corrected pose 重投影；
- 还是维护 local map + global display map 双层结构；
- 不要简单整体重复变换同一 voxel map 导致数值或重复融合问题。

---

# 28. Bag 数据标定磁场参数

最终：

```text
d0 / d1 / d2
```

不能长期使用拍脑袋参数。

推荐流程：

## Step 1：关闭 RTK 后端

运行纯 LIO。

## Step 2：只做一次 A0 对齐

获得：

\[
p_{LIO}^{aligned}(s)
\]

## Step 3：计算

\[
d(s)
=
\|
p_{LIO}^{aligned}(s)
-
p_{RTK}(s)
\|
\]

## Step 4：找纯 LIO 点云质量优秀区间

人工或通过地图指标选取：

```text
没有明显重影
局部配准稳定
环境几何条件正常
```

## Step 5：统计这些区间的 d

例如：

```text
P50
P90
P95
P99
```

如果发现优质 LIO 区域：

```text
P95(d) ≈ 0.11 m
```

那么：

```text
d0 ≈ 0.10 ~ 0.12 m
```

才有真实数据依据。

## Step 6：确定外边界

根据工程目标：

```text
d_max_target ≈ 0.30 m
```

设置：

```text
d2 ≈ 0.27 ~ 0.28 m
```

并让：

```text
d1 ≈ 0.18 ~ 0.22 m
```

作为渐强区域中点。

---

# 29. 必须做的消融实验

Codex Agent 完成基础版本后，应按顺序支持以下实验配置。

## Experiment A：Pure LIO

```text
FAST-LIVO2 only
```

作为局部地图质量基准。

## Experiment B：Fixed RTK Weight

复现当前方案。

用于证明固定权重的矛盾。

## Experiment C：Magnetic Weight Only

```text
固定/简单 global correction
+
alpha(d)
```

观察距离自适应权重本身的作用。

## Experiment D：Continuous C(s) Only

```text
固定 RTK weight
+
continuous correction field
```

观察低频形变场的作用。

## Experiment E：Full Core Version

```text
continuous C(s)
+
adaptive radial RTK field
+
smoothness
+
bound
```

这是第一版最终主方案。

## Experiment F：Optional RTK Yaw

仅在需要时加入。

## Experiment G：Optional Predictive Field

仅在核心方案确实出现提前量不足时加入。

---

# 30. 重点评价指标

不要只看 trajectory RMSE。

必须同时观察：

## 全局指标

- LIO–RTK 距离分布；
- P50 / P90 / P95 / P99；
- 最大误差；
- 100m / 500m / 1km 长距离漂移；
- yaw 长期误差。

## 局部指标

- 5m / 10m / 20m 相对位姿变化；
- 点云墙面厚度；
- 道路边缘重影；
- 静态结构重复边；
- local map consistency；
- 相邻 keyframe correction magnitude。

## 平滑性指标

\[
\|\xi'(s)\|
\]

\[
\|\xi''(s)\|
\]

以及：

```text
单次 correction update
```

是否出现尖峰。

---

# 31. RViz2：RTK“磁性球场”可视化

该功能可以实现，并建议作为调试模块加入。

目标：

> 在 RViz2 中直接看到当前 LIO 状态正在受到哪个 RTK 对应点的磁场影响，以及当前距离对应的磁场强度区域。

---

# 32. 当前激活 RTK 磁场中心

对当前 LIO 状态：

```text
t_lio
```

根据 timestamp 插值得到：

```text
p_rtk(t_lio)
```

该点即当前磁场中心。

RViz2 中至少显示：

1. 当前 RTK 对应点；
2. 当前 corrected LIO 位置；
3. 两者连线；
4. 当前距离 \(d\)；
5. 当前 \(\alpha(d)\)；
6. 当前 RTK health \(q\)；
7. 当前 effective sigma；
8. 当前所在磁场区域。

例如文字：

```text
RTK Magnetic Field
d = 0.184 m
alpha = 0.083
q_rtk = 1.00
sigma_eff = 0.104 m
zone = soft/transition
```

---

# 33. 连续磁场的 RViz2 渐变显示

RViz2 本身不会自动渲染一个数学意义上的连续三维标量场。

推荐通过：

```text
visualization_msgs/msg/Marker
visualization_msgs/msg/MarkerArray
```

近似显示。

### 推荐实现：多层同心半透明球壳

将：

```text
0 ~ d_visual_max
```

离散成例如：

```text
12 ~ 30
```

个仅用于**显示**的球壳。

注意：

> 这些显示球壳不等于算法使用离散权重。  
> 算法仍然使用连续无级 \(\alpha(d)\)。

每个显示半径：

\[
r_j
\]

根据：

\[
\alpha(r_j)
\]

映射颜色。

例如可以设计为：

```text
低磁场：
    蓝 / 青 / 绿色
        ↓
中等磁场：
    黄
        ↓
高磁场：
    橙 / 红
```

颜色必须平滑插值，形成渐变。

透明度建议较低，例如：

```text
alpha_visual ≈ 0.03 ~ 0.15
```

具体根据 RViz 实际效果调。

---

# 34. RViz2 同心球的实现注意事项

简单使用多个：

```text
Marker::SPHERE
```

可以快速实现。

每层：

```text
scale.x = scale.y = scale.z = 2 * radius
```

但嵌套透明实体球可能出现视觉混色过重。

因此优先级如下：

### 第一版

直接：

```text
Marker::SPHERE
```

低 alpha 快速验证。

### 如果视觉效果不理想

改为：

```text
LINE_LIST
```

绘制：

- 纬线；
- 经线；
- 若干大圆。

或者使用：

```text
TRIANGLE_LIST
```

生成薄球壳 mesh。

这样可以更清楚看到内部的 LIO 点。

---

# 35. 当前 LIO 位置在磁场中的显示

建议额外发布一个明显 Marker：

```text
current_corrected_lio_pose
```

例如小球。

再发布：

```text
LINE_STRIP / LINE_LIST
```

从：

```text
RTK magnetic center
```

连接到：

```text
current corrected LIO
```

这样可以非常直观地看到：

```text
当前 LIO 位于磁场的哪个半径
```

连线长度即：

\[
d
\]

---

# 36. 连续磁场颜色映射建议

颜色应直接由：

\[
\alpha(d)
\]

而不是由“分段编号”决定。

伪代码：

```cpp
float a = magneticGain(distance);

// 仅示意
Color c = interpolateGradient(a);
```

推荐渐变顺序：

```text
alpha ≈ 0
    蓝 / 青
alpha 中等
    绿 / 黄
alpha 较高
    橙
alpha ≈ 1
    红
```

不要硬切颜色。

---

# 37. 如果最终代码真的采用分段逻辑

虽然算法推荐连续无级变化，但如果实际工程中由于已有代码结构、性能或调试原因，Codex Agent 最终选择：

```text
分段函数
```

则必须做到：

1. 尽量保证段间连续；
2. 最好使用 piecewise smooth interpolation；
3. 避免纯 piecewise constant；
4. 在 RViz2 中把每个分段边界画出来。

用户特别要求：

> **在分段处的球壳表面设置一定透明度的浅白色。**

因此在：

```text
d0
d1
d2
...
```

边界处增加：

```text
浅白色半透明球壳
```

例如：

```text
RGB ≈ (0.9, 0.9, 0.9)
alpha ≈ 0.15 ~ 0.35
```

使用户能够直接判断当前 LIO 位于哪个区间。

注意：

> 这些白色球壳只用于 UI，不应影响算法本身。

---

# 38. RViz2 建议的 Topic

实际 topic 名称必须根据当前项目命名规范适度调整。

建议逻辑上至少有：

```text
/rtk_fusion/rtk_path
/rtk_fusion/lio_local_path
/rtk_fusion/lio_global_path

/rtk_fusion/magnetic_field
/rtk_fusion/magnetic_center
/rtk_fusion/magnetic_vector
/rtk_fusion/magnetic_text

/rtk_fusion/correction_knots
```

如果 RViz 性能出现问题，可以：

- 只显示当前 active field；
- 不显示整条 RTK 每个点的球场；
- 只在 debug 模式开启；
- 降低磁场 Marker 发布频率，例如 5~10 Hz。

---

# 39. 不要给 RTK 轨迹上的每一个点同时画完整磁场

实际矿山轨迹可能有成千上万个 RTK 点。

如果给每个 RTK 点画：

```text
20层球壳
```

RViz 会产生大量 Marker。

推荐：

\[
\boxed{
\text{只渲染当前 LIO 时间对应的 active RTK magnetic field}
}
\]

必要时可以额外显示：

```text
前后 1~2 个 RTK 点
```

用于观察插值。

这样用户看到的就是：

> 当前 LIO Pose 此刻实际正在触发的那一个磁场。

这完全符合调试目标。

---

# 40. 磁场 UI 与算法必须共用同一个 alpha(d) 函数

禁止：

```text
优化器里一套 alpha(d)
RViz 里重新手写另一套阈值
```

必须抽成公共接口，例如：

```cpp
MagneticFieldModel
{
    double gain(double distance);
    double effectiveSigma(double distance);
    Region debugRegion(double distance);
    Color debugColor(double distance);
}
```

或者等价设计。

从而保证：

```text
RViz 看见的磁场
==
优化器实际使用的磁场
```

避免调试误导。

---

# 41. 推荐代码模块划分

Codex Agent 在看完真实项目结构后，可以参考如下职责拆分。

## 1. RTKMeasurementBuffer

负责：

- 接收 RTK；
- 保存 timestamp；
- 插值；
- health；
- covariance；
- 坐标转换。

## 2. GlobalAlignment

负责：

```text
A0
```

的初始化和锁定。

## 3. MagneticFieldModel

负责：

- \(\alpha_p(d)\)；
- effective covariance；
- safety potential；
- debug color；
- 参数读取。

## 4. CorrectionSpline

负责：

- knot；
- \(C(s)\)；
- interpolation；
- 4DOF correction。

## 5. GlobalFusionOptimizer

负责：

- RTK factor；
- smooth factor；
- anchor；
- outer-loop weight update；
- GTSAM/iSAM2 或当前项目已有求解器。

## 6. GlobalTrajectoryPublisher

负责：

- corrected odometry；
- global path；
- corrected keyframe poses。

## 7. MagneticFieldVisualizer

负责：

- active RTK center；
- gradient sphere field；
- LIO point；
- error vector；
- text；
- correction knots。

具体类名可以完全不同。

关键是职责要尽量分离。

---

# 42. Codex Agent 修改代码时的工作顺序

必须按渐进方式实施。

---

## Phase 0：只读代码，不修改核心逻辑

先回答：

1. FAST-LIVO2 当前 Pose state 在哪里定义？
2. LIO odometry 在哪里发布？
3. point cloud map 在哪里更新？
4. RTK 当前如何进入系统？
5. 是否已有 GTSAM？
6. 是否已有后端优化线程？
7. 当前坐标系关系？
8. 当前时间同步逻辑？
9. RTK antenna lever-arm 是否存在？
10. RTK yaw 是否进入优化？
11. 起步 A0 / global yaw 如何确定？
12. 哪个 Pose 被用于 scan-to-map？

必须先形成代码级数据流图。

---

## Phase 1：增加观测与日志，不改变算法

增加：

```text
d(t)
RTK covariance
RTK health
yaw difference
time difference
lever-arm corrected residual
```

以及 CSV / rosbag 可分析输出。

先确认 20 cm 重影到底与：

- position；
- yaw；
- timestamp；
- lever-arm；

哪一个最相关。

---

## Phase 2：隔离 LIO 与 Global Pose

明确建立：

```text
T_L
T_G
```

两套 Pose。

此阶段仍可使用简单 global transform。

确保：

```text
FAST-LIVO2 scan-to-map 永远使用 T_L
```

---

## Phase 3：实现 A0

实现稳定 global alignment。

加入：

```text
C(s0)=I
```

或未来 spline 第一 knot anchor。

---

## Phase 4：实现最简单的 C(s)

先不加磁场。

验证：

- spline interpolation；
- global path；
- corrected map；
- knot 更新；
- 平滑正则。

---

## Phase 5：实现 alpha(d) 径向弹性刚度

加入：

```text
distance dependent RTK information
```

第一版 outer loop。

---

## Phase 6：加入 outer bound

接近：

```text
0.27 ~ 0.30 m
```

显著增强。

---

## Phase 7：加入 RViz2 Magnetic Field 调试 UI

必须保证使用优化器同一套：

```text
RadialElasticStiffnessModel / alpha(d)
```

---

## Phase 8：Bag 标定

重新确定：

```text
d0
d1
d2
lambda1
lambda2
```

---

## Phase 9：消融实验

验证核心方案。

---

## Phase 10：仅在必要时加入高级功能

例如：

- RTK yaw；
- predictive field；
- local rigidity；
- roll/pitch 低频修正；
- RTK bias state。

---

# 43. Codex Agent 禁止事项

除非真实代码强制要求，否则不要：

1. 一开始就重写 FAST-LIVO2 核心 ESIKF；
2. 直接把 RTK 高频塞入内部 scan-to-map state；
3. 同时加入十几种 factor；
4. 一开始就加入 RTK bias、预测磁场、姿态磁场、local rigidity；
5. 用固定 30m/50m 子地图代替连续修正；
6. 用“大残差降低 RTK 权重”的传统鲁棒核覆盖磁场逻辑；
7. 忽略 timestamp；
8. 忽略 RTK lever-arm；
9. 直接把 RTK heading 当绝对真值高频注入；
10. 在未理解真实代码数据流之前大量改文件；
11. 为了完全照抄本文公式而破坏已有稳定代码结构；
12. 为了方便实现而改变“LIO保局部、RTK管低频全局”的核心思想。

---

# 44. 实现时允许科学调整的内容

Codex Agent 可以根据真实项目适度改变：

- \(C(s)\) 是按 distance 还是 timestamp 参数化；
- B-spline / cubic interpolation / GP / piecewise Lie interpolation；
- knot 间距；
- RTK factor 稀疏策略；
- optimizer update frequency；
- GTSAM factor 具体实现；
- LM / GN / iSAM2；
- ROS2 topic；
- class name；
- threading；
- memory buffer；
- map reconstruction；
- 参数文件组织；
- RViz marker 实现方式；
- 渐变颜色；
- smoothstep 的具体平滑函数。

但需要在代码注释或开发日志中说明：

```text
为什么调整？
调整后是否仍满足主线意图？
```

---

# 45. 不允许被改变的主线思想

以下原则属于算法核心：

\[
\boxed{
\text{Pure LIO determines local geometry}
}
\]

\[
\boxed{
\text{RTK determines long-term global trend}
}
\]

\[
\boxed{
\text{RTK does not directly dominate every local LIO pose}
}
\]

\[
\boxed{
d\uparrow \Rightarrow RTK\ constraint\uparrow
}
\]

\[
\boxed{
d\downarrow \Rightarrow RTK\ constraint\downarrow
}
\]

\[
\boxed{
\text{Global correction must be spatially smooth}
}
\]

\[
\boxed{
\text{Healthy RTK + large residual = primarily suspect LIO drift}
}
\]

---

# 46. 第一版最小可行版本（MVP）

如果希望尽快得到结果，只实现：

1. RTK timestamp interpolation；
2. lever-arm；
3. A0；
4. \(T_L\) 与 \(T_G\) 分离；
5. 4DOF \(C(s)\)；
6. cubic spline；
7. first/second difference smoothness；
8. smoothstep \(\alpha(d)\)；
9. outer-loop information scaling；
10. anchor \(C(s_0)=I\)；
11. 0.27~0.30m soft bound；
12. RViz active magnetic field；
13. corrected global point cloud。

明确不做：

```text
RTK yaw
predictive field
RTK bias state
roll/pitch correction
multiple local-rigidity factors
```

先验证：

> **20 cm 局部重影是否显著减少，同时长距离全局漂移是否仍被控制。**

---

# 47. 成功判据

第一版成功不意味着：

```text
LIO轨迹完全贴着RTK
```

成功应该表现为：

### 局部

- 地图厚度接近纯 LIO；
- 不重新出现 RTK 引入的明显 20 cm 重影；
- correction 在短距离内非常平滑。

### 全局

- LIO 不长期逃离 RTK；
- 大尺度漂移受到明显限制；
- 正常 RTK 时大多数状态处于目标邻域；
- 接近 0.3 m 时能够被明显拉回。

### 行为

当：

```text
d 很小
```

在 RViz 中磁场显示弱。

当：

```text
d 增大
```

颜色逐渐进入强磁场区域。

轨迹被拉回后：

```text
d 下降
alpha 下降
```

磁场重新变弱。

---

# 48. 最终算法概念总结

最终系统应被理解成：

\[
\boxed{
\textbf{LIO 决定局部轨迹形状}
}
\]

\[
\boxed{
\textbf{RTK 轨迹形成全局径向吸引势场}
}
\]

\[
\boxed{
\textbf{LIO–RTK 距离决定吸引力大小}
}
\]

\[
\boxed{
\textbf{连续修正场决定该吸引力如何平滑传播到全局地图}
}
\]

核心不是：

```text
“RTK 权重到底应该调成多少？”
```

而是：

\[
\boxed{
\textbf{Distance-Adaptive Radial Elastic Constraint}
+
\textbf{Low-Frequency Continuous Correction Field}
+
\textbf{Local LIO Geometry Isolation}
}
\]

可暂称为：

> **Radial Adaptive Elastic Potential（径向自适应弹性势场）**

该名称用于描述算法思想，不应在未经充分文献检索前直接宣称为独有学术术语或论文创新。

---

# 49. 给 Codex Agent 的最终执行指令

在开始编码前：

> **先完整阅读当前 FAST-LIVO2 项目相关代码，梳理真实数据流，再设计最小侵入式修改方案。不要机械照抄本文中的类名、文件名、频率和参数。**

编码过程中：

> **可以根据现有代码结构科学调整实现细节，但必须保持“纯 LIO 保局部、RTK 管低频全局、距离越大约束越强、距离减小时自动松弛、全局修正连续”的主线思路不变。**

每完成一个 Phase：

1. 编译；
2. 运行基础测试；
3. 输出关键日志；
4. 对比 Pure LIO；
5. 确认没有引入新的局部退化；
6. 再进入下一阶段。

不要一次性完成所有功能后再调试。

优先追求：

```text
可解释
可回退
可消融
可调参
实时性稳定
局部地图质量稳定
```

而不是代码形式上的“复杂”和“完整”。

---

# 50. 推荐最终系统结构图

```text
                    ┌───────────────────────────┐
                    │       FAST-LIVO2 LIO      │
LiDAR + IMU ───────>│    local state / map      │
                    └─────────────┬─────────────┘
                                  │
                                  │ T_L
                                  ▼
                    ┌───────────────────────────┐
                    │    Global Alignment A0    │
                    └─────────────┬─────────────┘
                                  │
                                  ▼
RTK ── timestamp ──>┌───────────────────────────┐
      interpolation │  Continuous Correction    │
      lever-arm ───>│        Field C(s)         │
      health q ────>│                           │
                    │  Radial Magnetic RTK      │
                    │       alpha(d)            │
                    └─────────────┬─────────────┘
                                  │
                                  │ T_G
                    ┌─────────────┴─────────────┐
                    ▼                           ▼
          Global Corrected Path         Global Corrected Map
                    │
                    ▼
             RViz2 Debug UI
                    │
      ┌─────────────┼─────────────────┐
      ▼             ▼                 ▼
 RTK path       LIO global      Active magnetic
                                field + gradient
```

---

# 51. 最终提醒

该方案的目标不是否定 RTK，也不是在小残差时“抛弃 RTK”。

正确理解是：

> RTK 始终存在，但它在不同空间误差尺度下承担不同强度的恢复作用。

在 LIO 已经处于 RTK 附近时：

```text
RTK 很软
```

让 FAST-LIVO2 发挥自己的局部几何优势。

当 LIO 长期向外漂移时：

```text
RTK 持续变硬
```

逐渐将全局趋势拉回。

当轨迹重新靠近 RTK：

```text
RTK 自动变软
```

因此系统不会把最终地图变成 RTK 轨迹的逐点复制，而是形成：

> **局部像 LIO，全局像 RTK。**

这就是本次融合改造应始终保持不变的核心目标。

---

# 52. 当前 MY-LIVO2.0 工程审计与实施记录（2026-08-21）

本节是对前述通用指南的工程落地补充。若本节与前面的“推荐类名、推荐阶段数字”存在差异，以本节记录的真实代码和后续实测结果为准；第 45 节定义的算法主线保持不变。

## 52.1 Phase 0 真实代码数据流结论

1. **FAST-LIVO2 Pose state**

   - 高频 LIO 状态为 `LIVMapper::_state`，类型为 `StatesGroup`；
   - 位姿为 `_state.rot_end / _state.pos_end`；
   - `VoxelMapManager::StateEstimation()` 更新该状态。

2. **LIO odometry 与 path 发布**

   - `LIVMapper::publish_odometry()` 发布 `/aft_mapped_to_init`；
   - `LIVMapper::publish_path()` 发布 `/path`；
   - 两者均直接读取 `_state`，不读取后端优化 Pose。

3. **scan-to-map 与局部 voxel map**

   - `handleLIO()` 使用 `_state` 将当前点云变换到前端世界系；
   - `VoxelMapManager::StateEstimation()` 和 `UpdateVoxelMap()` 始终使用 `_state`；
   - `Keyframe::T_map_body` 从未写回 `_state`。

   因此已经确认：**当前 RTK 后端不会递归污染 FAST-LIVO2 的 IEKF 或局部 scan matching。**

4. **关键帧与全局地图**

   - `handleBackendKeyframe()` 将 `_state` 固化为不可变 `T_odom_body`；
   - body-frame keyframe cloud 同样不可变；
   - `OptimizedGlobalMap` 使用 `T_map_body` 的快照重投影关键帧点云；
   - 因而当前观察到的 RTK 融合重影发生在后端全局重建层，而不是 FAST-LIVO2 局部 voxel map 内。

5. **RTK 当前进入系统的方式**

   - `/ins_status` 解析 `ins_pos_mode`；
   - `/ins_odometry` 保存位置、姿态和 covariance；
   - `RtkObservationBuffer` 按关键帧 timestamp 对 RTK 做线性位置插值和 quaternion slerp；
   - 只有插值两端和区间内状态均满足 `ins_pos_mode == 4` 才返回观测；
   - `RtkFactorSelector` 再按时间/距离稀疏化；
   - 当前 `PoseGraphOptimizer::AddRtkPositionFactor()` 将每个被选中的 RTK 位置作为 GTSAM `GPSFactor`，直接连接到对应的 `T_map_body` Pose3 节点。

6. **GTSAM 与线程模型**

   - 工程使用本地 GTSAM 4.2.2 和 iSAM2；
   - odometry、loop、RTK 当前处于同一个 Pose3 因子图；
   - 关键帧和 RTK 图更新在关键帧调用链内执行；
   - loop NDT 注册与全局点云重建各有后台线程；
   - `Keyframe::T_map_body` 带互斥保护，前端 `_state` 与后端 Pose 分离。

7. **坐标系的真实情况**

   配置声明：

   ```text
   backend.frontend_frame_id = odom
   backend.map_frame_id      = mine
   ```

   但 IMU 初始化结束时，`initializeMineFrame()` 会将静止期 INS 的平均完整姿态和平移直接左乘进 `_state`：

   ```text
   _state.pos_end = R_INS * _state.pos_end + p_INS
   _state.rot_end = R_INS * _state.rot_end
   ```

   同时前端 `/path`、`/aft_mapped_to_init` 又使用 `mine` frame_id。也就是说，当前 `T_odom_body` 数值已经带有一次被写进前端状态的 INS 全局初始化，但后端命名仍把它当作 `odom`。这不是指南要求的独立 $A_0$，也是下一阶段必须消除的 frame semantic 混用。

8. **时间策略**

   - INS status、INS odometry 和 IMU 都减去同一个 `imu_time_offset`；
   - LIO keyframe 使用 `LidarMeasures.last_lio_update_time`；
   - RTK 查询使用 keyframe timestamp，并要求有效 bracket、最大插值间隔和最大端点距离；
   - 没有使用空间最近 RTK 点。

9. **lever-arm**

   - CGI-610 Odometry translation 的工程约定为后轴参考点；
   - 当 `rear_axle_to_imu_enabled` 时，代码使用姿态旋转 `-imu_to_rear_axle`，将 RTK 位置移动到 LIO/IMU 参考点；
   - 同一组转换还用于将 LiDAR estimator body 从后轴移至 IMU；
   - 公式链路在代码中是成对的，但此前日志没有同时保留“驱动原始后轴位置”和“修正后 IMU 位置”，无法从 bag 结果独立审计。

10. **RTK yaw 与初始 yaw**

    - 当前 GTSAM 图没有瞬时 RTK yaw factor；
    - 但 RTK/INS 完整姿态被 `initializeMineFrame()` 用于一次性旋转 `_state`；
    - 该姿态来自静止期 quaternion 均值，而不是 20~50 m 运动长基线估计；
    - 因此“没有 yaw factor”不等于“RTK yaw 没有影响当前全局地图”。

11. **20 cm 重影的直接结构原因**

    当前 RTK 不会破坏前端 scan matching，但每个 `GPSFactor` 会与极紧的首 Pose prior、odometry factors 和 loop factors共同重新分配误差，并直接改变用于全局点云重投影的 `T_map_body`。因此局部相邻 keyframe 可被同一图中的稀疏绝对约束产生不同幅度/姿态的修正，从而在 `/backend/global_map_optimized` 中形成重影。

    现阶段还不能只凭重影断言误差完全来自 RTK position。初始姿态、timestamp、lever-arm、固定 frame 偏差以及图更新的姿态分量均可能产生同数量级影响，必须先用 Phase 1 日志量化。

## 52.2 对后续阶段的工程化调整

依据真实代码，后续 `T_L` 应细分为：

```text
T_LIO_raw       = FAST-LIVO2 不可变局部里程计 Pose
T_SLAM_local    = 仅 odometry + loop 的局部 SLAM 后端 Pose
T_global        = C(s) * A0 * T_SLAM_local
```

调整理由：

- loop closure 属于 SLAM 内部长期一致性，不应与 RTK 地理配准变量混在同一解释层；
- RTK 不能再直接连接 `T_SLAM_local` 的每个 Pose 节点；
- FAST-LIVO2 scan-to-map 继续使用 `T_LIO_raw`；
- 全局地图使用 `T_global` 重投影不可变 keyframe cloud；
- 现有 `OptimizedGlobalMap` 的不可变点云重建机制可以复用；
- 现有 `PoseGraphOptimizer` 在移除 RTK Pose factor 后可以继续承担 odometry + loop 局部图职责。

因此，Phase 2/3 实施时不会机械地再建一套重复的 loop 图，而会将现有图收敛为 local SLAM graph，并在它外部建立独立 `A0 + C(s)` georeference 层。

## 52.3 Phase 1 已实现：只观测、不改变算法

本阶段新增 `RtkFusionDiagnostics`，它不持有优化变量，也没有修改 Pose 的接口。现有 RTK `GPSFactor` 行为暂时原样保留，便于建立 legacy baseline。

新增输出：

```text
Log/backend/rtk_initial_alignment.csv
Log/backend/rtk_diagnostics.csv
```

`rtk_initial_alignment.csv` 记录：

- 初始化 timestamp 与静止样本窗口；
- 样本数量；
- 当前被写入 `_state` 的完整刚体变换；
- roll / pitch / yaw；
- 静止位置 RMS、最大离散；
- 姿态最大离散。

`rtk_diagnostics.csv` 每个 keyframe 恰好一行，记录：

- RTK query 是否成功、拒绝原因、独立 health 值与两端 `ins_pos_mode`；
- lower/upper timestamp、两端时间差、插值 gap 与 alpha；
- 驱动原始后轴 RTK 位置；
- 旋转后的 lever-arm correction；
- 修正到 IMU/body 的 RTK 位置；
- 接收机原始 covariance 对角项；
- 后端实际使用的 effective sigma；
- 不可变纯 LIO Pose；
- 当前 legacy graph 在本次 RTK 更新前后的 Pose；
- raw/corrected RTK 位置残差；
- LIO/graph 与 RTK yaw 差；
- 当前 RTK factor 造成的即时 translation/yaw 图更新。

同时扩展 `rtk_solutions.csv` 与 `rtk_queries.csv`，保留 raw RTK、lever-arm、reported covariance、有效状态和 health；旧字段不删除，原验证脚本仍兼容。

新增验证命令：

```bash
./scripts/validate_rtk_diagnostics.py
```

它验证：

- diagnostic/keyframe 一一对应；
- timestamp bracket 与插值量数学一致；
- 状态 4 和 health 独立可追踪；
- `raw RTK + lever correction == corrected RTK`；
- CSV 中所有位置残差与源 Pose 可重算一致；
- reported covariance 与 effective sigma 没有混为一谈；
- 输出初始化、时间、位置、yaw、lever-arm 和 legacy graph 即时更新的统计量。

## 52.4 当前阶段状态与进入下一阶段的门槛

```text
Phase 0/1  已完成：数据流审计、RTK 观测与日志基线
Phase 2    已完成：T_odom / T_slam / T_global 隔离，RTK 退出 local 图
Phase 3a   已完成：弹性修正可行性监测与快速发散锁存
Phase 3b   已验证：有界低频连续 C(s)，KF288 后安全冻结
Phase 3c   已验证：退化感知的低频 RTK 速度保护
Phase 3d   已验证：4DOF 门控正确拒绝约 8% 尺度不一致段
Phase 3e   已完成：失效段隔离，防止待重定位点云污染可信全局地图
Phase 3f   已验证：基于 T_global 残差的距离自适应径向弹性刚度
Phase 3g   已验证：结构性失败锁存前端局部段重建请求
Phase 3h   已验证：保持连续状态的纯 LIO voxel 子图受控重建
Phase 3i   已验证：几何失效诊断与单次自动重启熔断
Phase 3j   已验证：杆臂修正 receiver_twist 稳定接管速度保护
Phase 3k   已验证：一次动态恢复将末端误差从约 106 m 降至约 21 m
Phase 3l   已实现：1 Hz 稳定窗口 4DOF 拟合并重锚定恢复后子段
Phase 3m   已实现：隔离恢复期的限加速度 RTK 速度跟踪
Phase 3n   已实现：速度跟踪与稳定 4DOF 重锚定的显式握手
Phase 3o   已实现：近软远强的因果径向弹性恢复
Phase 3p   已实现：LIO局部先融合、空间连续变形的全局地图
Phase 3q   已实现：各向异性弹性场与LIO几何可观测性审计
Phase 3r   已实现：可观测自适应牵引与无跳变恢复
Phase 3s   已实现：隔离段径向位置外环与即时弹性恢复
Phase 3t   已实现：近期关键帧窗口重建局部 voxel 子图
```

## 52.5 Phase 1 实测结果与日志一致性修复

实际 bag 诊断结果：

```text
keyframes / diagnostics : 376 / 376
RTK available           : 362
factor accepted          : 56
selector spacing         : 306
no bracket               : 14
endpoint p95 / max       : 11.088 / 11.239 ms
interpolation gap p95    : 20.000 ms
LIO-RTK position p50     : 0.7428 m
LIO-RTK position p95/max : 11.3052 / 13.6895 m
LIO-RTK yaw p95          : 1.2510 deg
legacy immediate update  : p95 0.0949 m, max 0.2372 m
```

这组结果证明时间 bracket 和状态门控基本按设计工作，同时也定量证明了“局部不应跟随每个 RTK 点、全局又必须消除长期漂移”的矛盾：单个 legacy RTK factor 已可将图瞬时改动 0.1~0.24 m，而长程 LIO/RTK 分歧可达 13.69 m。因此 Phase 2 不能使用一个固定 RTK 权重继续调参。

首次运行 `validate_rtk_fusion.py` 时的失败不是“每关键帧没有查询 RTK”，而是文件生命周期不一致：

```text
keyframes.csv             : 376 rows
rtk_decisions.csv         : 376 rows
rtk_diagnostics.csv       : 376 rows
rtk_queries.csv           : 359 rows，最后一行不完整
optimized_trajectory.csv  : 1155 rows，为上一轮残留快照
```

原因是 `rtk_queries.csv` 原先依赖 `ofstream` 缓冲区，而 `optimized_trajectory.csv` 只在 `PoseGraphOptimizer` 析构时写出。当 ROS 进程未走完正常析构链，或验证器在进程结束前读取时，就会把日志未落盘误报为算法节拍错误。

已实施：

- `rtk_queries.csv` 在每次关键帧查询后立即 flush；
- `optimized_trajectory.csv` 启动时清空，每次新增关键帧追加并 flush；
- Loop/RTK 改变历史位姿后，用 `.tmp + atomic rename` 发布新的完整轨迹快照；
- 验证器会报出每个文件的确切行数，并拒绝把未完整行或跨运行日志当作算法结果；
- 高频 status/solution 在进程中断时允许丢弃唯一的不完整尾行，但关键帧级 query/decision/trajectory 仍严格要求一一对应；
- 与 `RtkObservationBuffer` 的真实契约保持一致：接收机在同一 timestamp 重发更新观测是合法的，只禁止 timestamp 倒退；
- 取消默认的“全局修正不得超过 5 m”硬阈值，因为这会把合法的长程漂移修正误判为图崩溃；仍输出绝对修正量和相邻关键帧修正增量供局部变形审计。

新增回归测试在 logger/optimizer 对象仍存活时直接打开文件，验证 query 已落盘、trajectory 已包含所有关键帧，并验证 Loop 全局改写不会暴露截断文件。当前干净构建和 8/8 测试全部通过。

## 52.6 Phase 2 已实现：Local / Global 位姿隔离

- `T_odom_body` 不可变；`T_slam_body` 仅由 odometry + loop 图更新；`T_global_body` 归独立全局层所有。
- RTK 不再向 local GTSAM 图添加 `GPSFactor`；状态 4 且时间有效的稀疏观测进入全局层缓存，大残差不再被 innovation gate 拒绝。
- 本阶段固定 `A0=I, C(s)=I`，用于单独验证拆层不改变局部几何；下一阶段在该层内实现低频连续 `C(s)`。
- RViz 新增 `/backend/keyframe_path_local`、`/backend/keyframe_path_optimized` 和白色完整当前帧 `/backend/current_frame_global`。
- 验证：重跑 bag 后执行 `./scripts/validate_rtk_fusion.py`。

## 52.7 Phase 3a：修正可行性分流

- 以 20 m 空间基线计算所需修正梯度，不将大残差当成 RTK 失效。
- 默认 `>0.02 m/m` 标记 degraded；连续两次 `>0.15 m/m` 锁存 `relocalization_required`。
- 锁存后禁止弹性场吸收该类轨迹断裂；后续由分段/重定位分支处理。
- 输出 `Log/backend/correction_feasibility.csv`，由 `validate_rtk_fusion.py` 重算校验。

## 52.8 Phase 3b：有界连续修正场

- 采用路径中心 4DOF 位移+yaw、15 m knot、40 m 空间低通和 C1 三次插值。
- 显式限制平面/高程/yaw 空间导数与单次更新；`C(s0)=I`。
- `relocalization_required` 后冻结该段修正场，RTK 始终不进入 local 图。
- 输出 `Log/backend/correction_field.csv`，实测后再开发分段重定位。

## 52.9 Phase 3c：前端速度发散保护

- 实测 KF281 后 LIO/RTK 速度比从约 1 增至 3.5，不能由后端形变吸收。
- 仅在 degraded/relocalization 且速度持续不一致时，关键帧级有界修正速度；不直接改位置、姿态和历史 local 图。
- 实测 2 s 速度节拍无法收敛，故新增独立约 1 Hz 速度观察与按 `dt` 缩放的一阶增益；位置观测仍为 2 s/5 m 稀疏节拍。
- 黄色 local path 以固定 A0 发布到 global frame，避免动态 TF 造成整线假移动。
- 输出 `Log/backend/rtk_velocity_guard.csv`，实测稳定后再执行分段重定位。

## 52.10 Phase 3d：分段重定位就绪门控

- 仅估计无尺度 `xyz+yaw`；保留 LIO 的 roll/pitch 与重力方向。
- 速度后验误差、30 m 基线、路程比和刚性拟合连续合格后才输出 ready；当前仅影子判定，不切换地图段。
- 当前 `scale_mismatch` 是局部段非刚性失真，不放宽门槛强行切段。
- 输出 `Log/backend/rigid_relocalization.csv`，由 `validate_rtk_fusion.py` 审计。

## 52.11 位姿来源固化

- `/path` 仅是前端显示缓存；local 图只使用不可变 `T_odom` 相对运动。
- 已移除 local GTSAM 的 RTK factor 接口；回环、全局层显式读取 `T_slam`。
- 验证器按完整 6DOF 审计 `keyframe→raw graph→local global input`。

## 52.12 Phase 3e：失效段隔离

- 从首次 `relocalization_required` 关键帧起隔离地图尾段，但保留全部恢复证据。
- 可信全局 Path/PCD 排除隔离帧；RViz 单独显示待重定位轨迹。
- `map_eligible` 和地图点数由验证脚本交叉审计。

## 52.13 Phase 3f：距离自适应径向弹性势场

- 以当前 `T_global` 对同刻 RTK 的距离计算五次平滑 `alpha(d)`，不是用原始LIO残差。
- 径向步长为 `空间低通增益 × alpha(d) × (-e)`，仍受梯度/步长上限约束。
- 日志记录距离、刚度和 `alpha(d)d`，后续 RViz Magnetic Field 共用同一函数。

## 52.14 Phase 3g：局部段重建请求

- 4DOF 门控连续三次结构性失败且跨越至少 15 m 后，锁存 `frontend_restart_required`。
- 不放宽尺度/RMS 门槛，不把隔离尾段刚性拼回可信地图；RViz 显式告警。
- 本阶段只建立安全请求链路，下一阶段再受控重建前端局部子图。

## 52.15 Phase 3h：受控前端局部子图重建

- 仅纯 LIO 启用；保持位姿、速度、重力和 IMU 零偏连续，只为位姿协方差设置温和下限。
- 在触发帧删除旧 voxel 子图并用当前去畸变帧重建；随后清零并重启刚性恢复门控。
- 新段仍处于隔离区，需下一阶段通过新段门控后才能重新进入可信全局地图。

## 52.16 Phase 3i：几何失效诊断与重启熔断

- 4DOF 仍是唯一允许的刚性拼接模型；附加 SE(3)/Sim(3) 仅诊断尺度、姿态、弱可观或非刚性失效，绝不写回轨迹。
- 默认最多自动重建一次；若新段仍失败，则保持隔离并抑制后续重建，避免反复清图。
- `frontend_restart_supervisor.csv` 记录执行/抑制决定，安装角只有在充分激励、可观测且跨数据集一致后才允许标定。

## 52.17 Phase 3j：接收机速度软保护

- 状态 4 时优先使用 CGI-610 车体系 twist；用 INS 姿态角速度完成后轴到 IMU 的速度杠杆臂修正后旋转到 mine/world，不可用或与独立位移速度矛盾时才退回位置差分。
- 接收机速度使用短时低通，前端干预仍约 1 Hz 且受最大加速度限制；不直接修改位置、姿态或 local 图，并在介入时抬升速度协方差下限。

## 52.18 Phase 3k：动态失效恢复

- 若状态 4 `receiver_twist` 的后验速度误差连续 3 次超过 2 m/s，只执行一次传播速度重置与 voxel 子图重建；位姿、姿态、重力、偏置和 local 图不变。
- 新段从下一关键帧连续承接旧全局段，不以单点 RTK 瞬移；待稳定 4DOF 门控后再建立新的全局刚性段。

## 52.19 Phase 3l：稳定子段重锚定

- 动态重启后的瞬态不作为永久锚点；用独立约 1 Hz 状态 4 位置/速度窗口筛选速度、尺度和 4DOF 拟合。
- 连续两次通过后，仅从当前确认关键帧开始应用刚性变换；拟合窗口只作证据，历史隔离段不可回写。

## 52.20 Phase 3m：隔离恢复期速度稳定

- 实测动态重启后速度误差仍可回升至约 4 m/s，导致 4DOF 稳定窗口反复清空；不能靠放宽几何门限解决。
- 仅在已重启且隔离的恢复段内增强约 1 Hz 速度收敛和限加速度；不融合 RTK 位置到前端，健康段参数不变。
- 验证前必须停止 mapping 进程，脚本拒绝读取仍在写入的日志。

## 52.21 Phase 3n：恢复状态机握手

- 短时速度正常不再提前退出恢复跟踪；隔离段持续约 1 Hz 限加速度跟踪，直至稳定 4DOF 窗口完成重锚定。
- 重锚定成功后显式释放速度跟踪；验证脚本检查该生命周期，防止再次出现“先恢复、后发散”。

## 52.22 Phase 3o：局部几何保护

- 生产牵引使用单调 `|F|∝alpha(d)d`：内层无牵引，过渡带五次平滑增刚，超出外边界后满刚度。
- 4DOF 窗口有效时使用鲁棒刚性目标；非刚性漂移时降级为指向同刻 RTK 点的径向目标，仍受低频采样、C² 插值和距离相关梯度约束。
- 接收机姿态不进入生产修正；速度辅助必须先通过位移差分一致性检查。

## 52.23 Phase 3p：局部优先的全局地图

- 每个连续可信段的点云先在不可变 `T_slam` 系去重融合，再由本段邻近修正节点变形到 global 系；严禁跨隔离/重锚定断点插值。
- 隔离段仍不进入可信 PCD；RViz Path 在隔离/重锚定边界显式断开，不再用折线连接不连续段。
- 恢复段以 30 m 滑窗、最多 25% 瞬时离群的严格门控重新进入可信地图，最终 0.3 m/0.5° 指标仍以日志验收。

## 52.24 Phase 3q：轴向弹性与退化观测

- XY、Z、yaw 使用独立软区，避免单轴大误差打开其他轴的 RTK 牵引；远离后仍单调增刚。
- 恢复验收使用五次插值区间峰值梯度，不再误用 knot 端点零导数。
- 退化依据加权点面信息矩阵的平移/旋转 Schur 谱与速度证据；特征数量仅作辅助。本阶段先记录审计，实测后再定门限，禁止凭经验直接增强前端 RTK。

## 52.25 Phase 3r：退化自适应与连续恢复

- XY/Z 在 `0.15 m` 内无牵引，XY `0.50 m`、Z `0.60 m` 外满牵引，yaw 为 `0.25°/1.0°`；可观测变弱时约束连续增至 3 倍，但 C² 梯度上限不放宽。
- 状态 4 速度与位移差分一致时，才允许约 1 Hz 的限加速度高程偏置修正和一次预防性局部段重建。
- 4DOF 拟合只判定恢复是否可行；新段严格继承上一节点变换，以 `C(s)=I` 起步再因果地拉回，禁止用拟合结果整段瞬移。
- 强速度跟踪仅存在于隔离恢复期，弹性段通过 `0.3 m/0.5°` 窗口验收后立即释放。

## 52.26 Phase 3s：隔离段径向位置外环

- `v_LIO≈v_RTK` 只能阻止新的速度漂移，不能消除已存位置偏差；隔离期因此使用 `v_target=v_RTK-k(d)e` 径向闭合。
- `0.3 m` 内闭合速度为零，平面/高程最大为 `0.60/0.35 m/s`，只经约 1 Hz 限加速度通道修正传播速度。
- 新隔离段从起点即启用零增量 C² 弹性场；4DOF 门控只决定何时可进入地图验收，不再冻结前 30 m 恢复轨迹。

## 52.27 Phase 3t：有基线的局部子图重建

- 单帧重建会在退化路段丢失几何基线；改为用最近 `20 m`、最多 `20` 个不可变 `T_odom` 关键帧重建 voxel 图。
- 重建仍不使用 RTK 位姿，不回写历史轨迹；CSV 记录历史帧数、路径和点数。
