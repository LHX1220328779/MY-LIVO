# 自动标定实跑：空车身过滤数组与校验 JSON 解析修复

日期：2026-10-27（容器 `+08:00`）· 分支 `mcp/vehicle-body-box-visual-filter` · 关联任务：以 `zongyang/truck02/v009_ground` 为父版本启动自动标定流水线，出现问题时继续改进。

## 1. 背景与现象

车身过滤前置、全雷达彩色拼接与任意父版本自动化标定功能提交为 `274908a` 后，启动以 `zongyang/truck02/v009_ground` 为父版本的 LiDAR 自动标定流水线。历史失败任务 `1a17b7c890f` 的 LIO 建图步骤退出码为 3，日志重复出现：

```text
Mapping failed: parameter_value_from failed for parameter 'multi_lidar.body_filter.front_lidar': No parameter value set
```

同时，Studio 的 `POST /api/calibrations/validate` 虽然底层校验工具返回 `ok=true`，接口却把格式化后的 JSON 文本当成错误字符串放进 `errors`，影响后续自动化步骤对校验结果的判断与页面展示。

## 2. 根因

### 2.1 ROS 2 无法推断空数组参数类型

`tools/livo/body_filter.py::to_yaml_overrides()` 会为每个 LiDAR 话题输出过滤矩形数组。当前车辆标定文件中 `front_lidar` 没有配置矩形，因此输出 `front_lidar: []`。`tools/livo/effective_config.py` 原样把该空数组合并进生成的 `config.yaml`，ROS 2 读取参数时无法从空 YAML 序列推断元素类型，导致 mapper 进程初始化失败。

这不是标定算法问题，也不是父版本选择问题；故障发生在 `run_wuhu_round.sh` 生成有效配置并启动 `laser_mapping` 节点之前。

### 2.2 Studio 只解析单行 JSON

`tools/studio/app/common.py::run_tool()` 原来只从工具 stdout 的倒序单行中寻找完整 JSON。`livo_calib.py validate` 输出的是跨多行的格式化 JSON，因此 `payload` 为 `None`；`tools/studio/app/calibration.py::validate()` 随后把完整文本截断后写入 `errors`。底层校验结果本身是正确的。

## 3. 修复

### 3.1 生成有效配置时省略空过滤数组

在 `tools/livo/effective_config.py` 遍历 `body_filter` 覆盖项时：

- `minimum_z` 等标量正常写入；
- 非 `minimum_z` 的空列表不再写成 `[]`，而是从生成的 YAML 参数中删除，并记录日志 `override multi_lidar.body_filter.<topic>: disabled (no rectangles)`；
- 非空矩形数组保持原样写入；
- `multi_lidar.precheck.body_box` 与父版本 `calibration_file` 覆盖逻辑不变。

这样 `front_lidar` 对应的是“没有配置过滤矩形”，而不是“存在一个类型未知的空数组参数”。Mapper 对未声明话题沿用默认行为，其余三个 LiDAR 的车辆专属矩形继续生效。

### 3.2 解析多行 JSON 并返回结构化校验结果

- `tools/studio/app/common.py::run_tool()` 在倒序单行解析失败后，再尝试从 stdout 的首个 `{` 到最后一个 `}` 解析整体 JSON；
- `tools/studio/app/calibration.py::validate()` 在拿到字典 payload 时补齐 `ok/errors/warnings` 默认值并返回该字典；
- 未返回 JSON 的工具仍沿用原来的错误文本包装逻辑，兼容性不变。

## 4. 真实迭代测试

### 4.1 生成配置

在 `calibration` 容器内使用 `v009_ground/calibration.json` 执行 `scripts/run_wuhu_round.sh <tmp_round> --write-config-only`。输出日志包含：

```text
override multi_lidar.body_filter.front_lidar: disabled (no rectangles)
override multi_lidar.body_filter: from calibration .../v009_ground/calibration.json
override multi_lidar.precheck.body_box: [-2.8, 2.8, -4.5, 9.5]
```

解析生成的 `results/<tmp_round>/config.yaml`：

- `laser_mapping.ros__parameters.multi_lidar.body_filter` 的键为 `minimum_z / front_right_lidar / back_lidar / front_left_lidar`；
- `front_lidar` 参数不存在；
- 三个非空数组长度分别为 `8 / 4 / 8`，均为 4 的倍数；
- `precheck.body_box` 保持 `[-2.8, 2.8, -4.5, 9.5]`；
- 没有空列表参数。

临时轮次已删除，没有留下测试地图。

### 4.2 Studio 校验接口

触发 `studio.app.*` 热重载后，`POST /api/calibrations/validate` 对 `zongyang/truck02/v009_ground` 返回：

```json
{"ok": true, "errors": [], "warnings": ["time_offsets.camera is 0.0 and unverified (run the camera-time stage)"], "id": "zongyang/truck02/v009_ground"}
```

`warnings` 中的相机时间偏移未验证是非阻断提示，不影响 LiDAR 流水线启动。

### 4.3 流水线预检

在容器内直接调用 `pipeline_steps('lidar', body, dummy_mapping)`：

- 数据集 `/home/project/data/haibo/zongyang/02/20261026/ros2bag` 的 `lio_ready=true`，时长 `239.98 s`，`datasets.require()` 通过；
- 父版本校验 `ok=true`、`errors=[]`；
- 生成 `17` 个步骤，`info.parent` 为 `zongyang/truck02/v009_ground`，`body_filter_source_id` 为 `zongyang/truck02/v001_import`；
- 第一步为「前置：将当前车辆 body_filter 写入父版本标定文件」，证明任意父版本 + 标定前过滤的组合已经生效。

### 4.4 自动标定实跑

启动任务 `1a17bdd7087`，参数要点：

- `kind=lidar`；
- `id/parent_id=zongyang/truck02/v009_ground`，`from_factory=false`；
- `body_filter_source_id=zongyang/truck02/v001_import`；
- 数据集 `/home/project/data/haibo/zongyang/02/20261026/ros2bag`；
- `use_rtk=true`、`mode=fast`、`activate=true`，最终校验通过后自动激活结果版本。

前置注入返回 `body_filter_injected=false`，因为 `v009_ground` 已经包含 Studio 保存的车辆专属 `body_filter`。运动分析选择 `48 s` 快速标定窗口；`axle`、`ground`、第一次 `48 s` LIO 建图、`driven` 与新版本校验均完成。短窗口的 `driven` 阶段出现 `quality: warn`（`front_right_lidar` 平移修正略超 15 cm 门限、`back_lidar` 垂直平面匹配较少），按用户接受的欠约束策略继续，不作为阻断错误。

关键验证点是第 8 步与第 11 步两轮 `240 s` LIO 建图：生成配置日志均显示 `override multi_lidar.body_filter.front_lidar: disabled (no rectangles)`，两轮 mapper 都正常完成，日志中不再出现 `No parameter value set`。随后 RTK 关联时间两轮割线定版把 `time_offsets.rtk_position` 从 `0.0250 s` 调整为 `0.0669 s`；`gcons` 对 `back_lidar` 应用 `+0.196/-0.075 deg` 的俯仰/横滚修正与 `-1.38 cm` 的 z 修正。

### 4.5 最终结果

- 任务 `1a17bdd7087` 最终 `status=done`、`returncode=0`，17/17 步骤全部为 `done`；
- 结果版本 `zongyang/truck02/v015_gcons` 已按 `activate=true` 自动激活，`config/carlibration/registry.json` 中 `zongyang/truck02` 的 active 指向该版本；
- 最终校验 `ok=true`、`errors=[]`，仅有相机时间偏移未验证的非阻断 warning；
- 最终版本继承 `body_filter`（`enabled=true`、`minimum_z=-3.0`、`source=Studio 车身点云可视化过滤`），车辆专属过滤随子版本传递到最终结果；
- 清理步骤删除 6 个临时地图/扫描文件，共 3.24 GB，保留轨迹与评估结果。

## 5. 影响范围

- 只改变空 `body_filter` 数组在 ROS 参数 YAML 中的表达方式，以及 Studio 对底层结构化输出的解析；
- 不修改标定阈值、优化目标、采样策略或长数据门限，因此不影响正常长数据标定精度；
- 车辆已有非空过滤矩形、后轴车身框、`precheck` 与 `minimum_z` 的行为均保持不变；
- 对完全没有 `body_filter` 的旧标定文件仍由 `normalize()` 回退到历史默认值，不产生空数组。

## 6. 相关文档

- 功能主记录：`docs/mcp-body-filter-multilidar-parent.md`
- 车身过滤实现：`docs/mcp-vehicle-body-filter-implementation.md`
- 短数据门限设计：`docs/mcp-short-bag-calibration-thresholds.md`
- Studio 工作流：`docs/mcp-studio-v2-workflow.md`
