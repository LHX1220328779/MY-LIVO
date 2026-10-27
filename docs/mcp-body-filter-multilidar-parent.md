# 车身过滤前置、全雷达彩色拼接与任意父版本自动化标定

## 1. 需求

1. 车身点云过滤必须在自动化标定流程之前生效，避免车身残留点影响自动标定。
2. 车身过滤页把全部 LiDAR 按当前父版本标定文件拼接到同一坐标系，每路颜色不同。
3. 自动化流水线不得强制只能以出厂导入版本为父版本，必须允许当前车辆任意版本作为父版本。

## 2. 审计结论

- `tools/studio/app/calibration.py` 原 `pipeline_steps()` 在 `kind == "lidar"` 且 `from_factory` 为真时执行 `factory_root(parent)`，把请求版本强制回退到谱系根；前端流水线 POST 始终发送 `id: v.active`，没有把单步执行的父版本传给流水线。
- 离线精修脚本虽然会从 `--calibration` 读取 `body_filter`，但自动化流水线若选了没有 `body_filter` 的旧版本/出厂版本，会退回 truck29 历史默认框，不能保证车辆专属过滤在标定前生效。
- 标定文件的 `lidar[0].transform_matrix_lidar_{1,2,3,6}` 分别对应 `front_right_lidar / back_lidar / front_left_lidar / front_lidar`，语义为 `lidar_to_body_rear_axle`，可用于把四路点云拼到后轴坐标系。

## 3. 实现

### 3.1 自动化标定前置车身过滤

- 移除 `pipeline_steps()` 中基于 `from_factory` 的强制出厂回退；`from_factory` 默认改为 `false` 且不再影响父版本。
- 流水线第一步固定为「前置：将当前车辆 body_filter 写入父版本标定文件」。它从当前激活版本（或请求中的 `body_filter_source_id`）读取 `body_filter`，规范化后写入所选父版本 `calibration.json`；若父版本已有 Studio 可视化保存的 `body_filter`，则保留父版本自己的值。写入前生成一次 `.pre_body_filter.bak` 备份，步骤幂等。
- 后续 `livo_calib.py` 阶段通过 `copy.deepcopy(parent_calib)` 继承 `body_filter`；离线脚本的 `rear_axle_mask` 与流水线内映射轮次的 `tools/livo/effective_config.py` 都从子版本标定文件读取车辆专属过滤参数，因此过滤发生在任何标定计算之前。

### 3.2 任意父版本

- `_parent()` 优先使用 `parent_id`，其次 `id`。
- 标定页新增父版本下拉框，选项为当前车辆全部版本；选择结果同时用于单步执行、自动化流水线和车身过滤拼接外参，并持久化到 `state.calibration.parent_id`。版本详情「作为父版本」按钮也写入同一状态。
- `A.pipeline()` 发送 `id: parent`、`parent_id: parent`、`from_factory: false`、`body_filter_source_id: v.active`；后端保留请求父版本，不再替换为出厂根版本。
- 流水线 job 的 `meta.info` 增加 `body_filter_source_id`。

### 3.3 全雷达彩色拼接

- `tools/livo/body_filter.py` 增加 `LIDAR_MATRIX_KEYS`、`lidar_transform_matrix()`、`transform_points()`。
- `GET /api/body-filter` 增加 `parent_id` 参数；`tools/studio/app/body_filter.py` 返回 `multi_scan`：遍历 `ds.lidar_topics`，对每个话题采样同一时刻点云，按父版本外参变换到后轴坐标系，分配蓝/绿/橙/紫等颜色，并把该话题过滤矩形四角变换为多边形。
- 返回 `points_local`（未变换、全精度 LiDAR 系点），前端 `bfRecomputeMulti()` 用它和当前 `body_filter` 精确重算过滤点，保证与后端 `lidar_filter_mask` 一致。
- 页面新增「多雷达拼接俯视（后轴坐标系）」画布、颜色图例（点击可切换单话题视图）、父版本选择器和父版本标定文件只读框。红点＝当前参数会过滤的点，彩色线框＝各 LiDAR 过滤矩形，灰色虚线＝后轴车身框。

## 4. 真实迭代测试

### 4.1 后端

- `pipeline_steps('lidar', body, dummy_mapping)`：请求 `parent_id/id = zongyang/truck02/v007_gcons` 且故意保留 `from_factory=True`，`info.parent` 仍为该版本；第一步为前置注入；`body_filter_source_id` 显式指定时为 `v007_gcons`，未指定时为当前激活 `v001_import`；静态阶段命令未把出厂根作为 `--id` 父版本。
- 注入命令在临时父标定文件上执行：无 `body_filter` 时成功写入并生成备份；再次执行返回 `body_filter_injected: false`；父版本已有 Studio 保存的 `minimum_z=-1.01` 时保留该值。
- `copy.deepcopy(parent_calib)` 子版本继承 `body_filter` 通过；四个 LiDAR 话题变换矩阵与手工 `P @ R.T + t` 一致；`tools/calib/livo_calib.py validate --file` 对当前激活标定文件通过。
- `POST /api/body-filter` 保存测试返回 200，随后用备份精确还原，`source` 与 `updated` 不变。

### 4.2 线上 API

- 重启 8765 Studio 后，`GET /api/body-filter?...&parent_id=zongyang/truck02/v007_gcons` 返回 HTTP 200。
- `multi_scan.series` 包含 4 路：`back_lidar #0071e3`、`front_left_lidar #34c759`、`front_lidar #ff9f0a`、`front_right_lidar #af52de`；四路 `transform_identity` 均为 false。
- `max_points=12000` 时拼接原始 `240421` 点、抽样 `12000` 点、过滤 `4063` 点；各话题过滤数 `2193 / 861 / 0 / 1009`，与前端用 `points_local` 重算完全一致。

### 4.3 Chrome CDP 真实页面

- `#body` 页面 `S.body.multi_scan.series.length === 4`，`bf-all` 画布 `1259×428`，非白像素 `68916`，图例 4 个按钮且颜色互不相同，父版本下拉框 9 个版本。
- 切换父版本到 `v007_gcons` 后，捕获的 `/api/body-filter` 请求包含 `parent_id=zongyang%2Ftruck02%2Fv007_gcons`，响应 `parent_calibration_file` 正确。
- 点击图例 `front_right_lidar` 后单话题视图切换到该雷达。
- 关闭启用开关后 `multi_scan.filtered_count` 从 `4063` 变为 `0`，恢复后精确回到 `4063`，页面计数同步更新。
- 标定页父版本下拉框选择 `v009_ground` 后调用 LiDAR 流水线，捕获 POST 为 `id/parent_id = zongyang/truck02/v009_ground`、`from_factory: false`、`body_filter_source_id = zongyang/truck02/v001_import`，证明自动化流水线使用任意选定父版本。
- 控制台无 JavaScript 异常。

## 5. 部署与状态

- `tools/studio/server.py` 的 GET 参数发生变化，已重启用户侧 8765 Studio 进程；`studio.app.*` 的修改由每次请求自动热重载。
- 测试期间状态 `parent_id` 已重置为空（默认使用当前激活版本或其父版本），`from_factory` 置为 `false`。当前激活版本 `zongyang/truck02/v001_import` 的 `body_filter` 保持用户保存值不变。
- 后续修改 `server.py` 路由仍需重启 Studio；修改 `studio.app.*` 不需要重启。

## 6. 本仓库相关文件

- `tools/studio/app/calibration.py`：流水线父版本与前置注入步骤
- `tools/studio/app/body_filter.py`：多雷达采样与拼接
- `tools/livo/body_filter.py`：外参与变换辅助函数
- `tools/studio/web/app.js`、`tools/studio/web/style.css`：前端页面与画布
- `tools/studio/server.py`：`parent_id` 查询参数传递
- `docs/mcp-studio-v2-workflow.md`：Studio 工作流补充
- `docs/mcp-studio-body-filter-route-404-analysis.md`：此前 404 诊断

## 7. 2026-10-27：自动标定实跑的问题修复

以 `zongyang/truck02/v009_ground` 为父版本启动自动标定后，发现两个实跑问题：`front_lidar` 的空过滤矩形数组被写入 ROS 参数 YAML，导致 mapper 报 `No parameter value set`；Studio 校验接口把多行 JSON 文本误当成 `errors`。已分别修复为“生成 YAML 时省略空数组参数”和“解析整体 JSON 并返回结构化结果”。任务 `1a17bdd7087` 的 17 个步骤全部完成，结果版本 `zongyang/truck02/v015_gcons` 已自动激活。生成配置、校验 API、流水线预检与完整实跑验证见 `docs/mcp-auto-calibration-empty-body-filter-fix.md`。
