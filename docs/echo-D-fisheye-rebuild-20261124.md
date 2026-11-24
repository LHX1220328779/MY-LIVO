> 最新RGB状态：共用强度掩码的617万点/98.7MB尝试因地面空洞已回退。当前彩色图恢复为 `8a35515` / `11a0d14` 的独立同帧RGB，113530886点/1.816GB；强度不变。最新覆盖核查见 [覆盖恢复报告](rgb-11a0d14-coverage-restore-20261124.md)。

# D 阶段回波恢复与鱼眼全程重跑（2026-11-24）

用户选择高斯开发前 `8a35515` / `11a0d14` 的回波处理，并指定强度默认 0.1 m。实现提交 `735266992e4e05368284612d70f96acfe1e85133` 已上传到 `origin/1/continue-prior-orca-work`。

## 实现

- 保留距离优先比例 1.5、余量 2 m、判定格 0.5 m、25 m 看穿检查、修复后的地面细节补点；所有扫描参与选点，未恢复有问题的停车整帧删除。
- 原生强度分块与标准整图默认 0.1 m，Studio 旧默认值迁移。强度始终读取并核验 `map_rows.bin`；显式距离仅追加裁剪，关闭筛选/零体素/无掩码与不同来源参数明确拒绝。
- 高斯优先使用原生掩码，有现成 PCD 也不会绕过它；缺少掩码的 PCD 必须具备完整 D 来源和一致点数。移除额外远距补点、legacy 几何与原始逐视图回退；旧远距或未核验种子须重新初始化。新来源标记 `native_accepted_rows_D_v1`。
- RGB 保持独立同帧取色与 60 m / 3 cm 最优原始着色观测选择。原始扫描/位姿存档保留。标准导出错误传播到轮次失败状态。

## 实际运行

运行 `xingye13_fisheye_native_filtered_full_20261124`，68 个兴业13原始分包；`v015_camselect` 鱼眼厂家初值，与此前未采纳时间修正的 v016 实际参数相同。正式激活标定仍为原版本，没有采用失败精修。

- 播放479382条消息，30600个鱼眼视频消息；重复/倒序丢弃均0。
- 建图10176帧，1017.5s，约2904.7m。
- 最终化、标准导出及轨迹/汇总评估均成功，轮次状态done，导出退出码0。

| 强度处理阶段 | 点/回波数 |
|---|---:|
| 原始归档 | 483481472 |
| 45m内（标准导出计数） | 440580310 |
| 距离优先/看穿/补点（C++计数） | 126530264 |
| 0.1m分块保留行 | 25287180 |
| 0.1m标准整图 | 17111311 |

新强度PCD为273781170字节，较旧全量观察图4.406亿点减少约96.1%。RGB为113530886点、1816494366字节，使用271310738个独立着色观测，全10176帧参与。

已核对两份PCD全部XYZ有限、点数与二进制长度一致；原生掩码置位行数25287180与导出保留数相同。另以真实4帧验证掩码优先于全量PCD，显式25m裁剪保留掩码，旧远距种子和补点请求拒绝。

## 查看与证据

本机实际文件：

- `/home/lhx/docker/MY-LIVO/results/xingye13_fisheye_pointcloud/global_map.pcd`
- `/home/lhx/docker/MY-LIVO/results/xingye13_fisheye_pointcloud/global_map_rgb.pcd`
- 同目录 `README.md`、`calibration.json`、`map_info.json`、`validation.json`、`source_provenance.json`。

工作区快捷入口 `results/xingye13_fisheye_pointcloud/` 指向新轮次的 `Log/fusion/1795482900583373302/map/`。两处旧观察强度/RGB及旧标准强度共5份PCD已清理；旧扫描、位姿、评估和高斯历史结果保留。主目录PCD为实际普通文件（同文件系统硬链接），无需访问符号链接或容器路径。

- [运行与代码版本](../results/xingye13_fisheye_native_filtered_full_20261124/round.json)
- [导出日志](../results/xingye13_fisheye_native_filtered_full_20261124/map_export.log)
- [全程评估](../results/xingye13_fisheye_native_filtered_full_20261124/evaluation/metrics.json)
- [完整性记录](../results/xingye13_fisheye_pointcloud/validation.json)
- [定向真实扫描验证](../results/echo_D_native_check_20261124/native_surface_report.json)
- [旧PCD清理记录](../results/echo_D_native_check_20261124/old_fisheye_pcd_cleanup.json)

融合后相对RTK三维偏差中位/P95/最大为0.97/2.34/4.92cm，距离加权路程100%≤10cm；这是融合所用RTK的参考一致性。原生终点三维偏差约21.35m、起步跨侧仍存在。此次没有调整轨迹约束，也未自动验收灯柱平面/顶部误差与地面厚度，不能将减点或RTK指标作为独立几何精度达标证据。
