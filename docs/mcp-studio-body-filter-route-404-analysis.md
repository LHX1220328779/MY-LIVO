# Studio 车身过滤页 `unknown path /api/body-filter` 诊断与验证

## 1. 问题现象

用户在已打开的 Studio「车身过滤」页看到底部 toast：`unknown path /api/body-filter`。页面能够渲染，但采样点云、保存参数等请求失败。

本仓库分支 `mcp/vehicle-body-box-visual-filter` 的提交 `c76c8e7` 已包含 `tools/studio/server.py` 中的 GET / POST 路由；相关前端代码在 `tools/studio/web/app.js`、`tools/studio/web/index.html`、`tools/studio/web/style.css`，后端采样与保存在 `tools/studio/app/body_filter.py`，标定文件归一化在 `tools/livo/body_filter.py`。完整的实现与迭代测试记录见开发工作区的 `docs/mcp-vehicle-body-filter-implementation.md`。

## 2. 根因

用户实际访问的 8765 端口对应 `calibration` 容器内的长期运行进程：

```text
/usr/bin/python3 tools/studio/server.py --port 8765
```

旧进程启动时间为 `2026-10-26 16:48:29`，早于 2026-10-27 的 body-filter 提交。`tools/studio/server.py` 的 `reload_app_modules()` 只热重载 `studio.app.*` 模块；`server.py` 自身及 `HTTPServer` 已注册的路由表不会热更新。因此即使磁盘上的 `server.py` 已经包含新路由，`/api/system` 也动态显示当前 git 为 `c76c8e7`，旧进程仍返回：

```json
{"error": "unknown path /api/body-filter"}
```

证据链：

- `curl http://127.0.0.1:8765/api/system` 返回 `git: c76c8e7 ...`、`pid: 231269`（容器内 PID）。
- `curl http://127.0.0.1:8765/api/body-filter?...` 返回 HTTP 404 与上述 error。
- 磁盘源文件 `tools/studio/server.py` 第 196、263 行分别存在 GET / POST 的 `/api/body-filter` 分支。
- 临时测试端口 8799 用同一份源文件启动的进程，GET / POST 正常。
- 旧进程的父进程是 VS Code Server 终端 shell，无 supervisor 自动重启；进程启动时间确认为补丁前。

结论：不是前端缓存、不是数据集问题、也不是标定文件格式问题，而是**用户侧 Studio 进程未加载新 `server.py`**。

补充：`/api/reload` 并不是已注册的 HTTP 路由；`reload_app_modules()` 在 `_dispatch()` 中对每个请求自动调用。恢复磁盘上的标定文件后，下一次 GET 就会读到新内容，无需也无法通过 `/api/reload` 触发。

## 3. 修复操作

在 `calibration` 容器内停止旧 PID，并用补丁后的源文件重新启动 8765 端口服务：

```bash
docker exec calibration bash -lc 'kill 231269'
docker exec -d calibration bash -lc 'cd /home/project/MY-LIVO && exec /usr/bin/python3 tools/studio/server.py --port 8765'
```

新进程启动时间 `2026-10-27 11:11:45`，容器内 PID `1302619`、宿主机可见 PID `3327040`。重启后：

- `GET /api/system` 仍返回 `branch: mcp/vehicle-body-box-visual-filter`、`git: c76c8e7`。
- `GET /api/body-filter?...` 返回 200，解析到 `zongyang/truck02/v007_gcons`，`duration_s=239.98`，`front_right_lidar` 原始 `74766` 点、抽样 `1000` 点、过滤 `342` 点。
- 临时测试端口 8799 已关闭，不再有第二个 Studio 实例。

后续在 `tools/studio/web/index.html` 的 `<head>` 中加入 `<link rel="icon" href="data:,">`，消除浏览器对 `/favicon.ico` 的 404 请求；该静态文件每次请求都从磁盘读取，不需要重启进程。

## 4. 全功能真实迭代验证

### 4.1 静态与进程检查

- `python3 -m py_compile tools/studio/server.py tools/studio/app/*.py` 通过。
- `node --check tools/studio/web/app.js` 通过。
- `server.py` 两个 `/api/body-filter` 路由标记存在；`index.html` 有 `data-page="body"`；`app.js` 有 `bf-load`；`style.css` 有 `bf-canvas`。
- 线上 `/static/app.js`、`/static/style.css`、`/` 均返回包含上述标记的新内容。
- 8765 仅有一个 `server.py --port 8765` 进程，8799 无监听。

### 4.2 后端 API 测试（真实 ros2bag）

使用 `/home/project/data/haibo/zongyang/02/20261026/ros2bag`、`zongyang/truck02`：

| 项目 | 结果 |
| --- | --- |
| 四个话题 t=0 | `back_lidar` 72090 点 / 过滤 1066；`front_left_lidar` 78502 / 440；`front_lidar` 15063 / 0；`front_right_lidar` 74766 / 521 |
| t=60 s | `front_right_lidar` 85794 点 / 过滤 425，`t_s=60.0` |
| `max_points=100` | 后端下限钳制为 1000 抽样点 |
| 省略 dataset | HTTP 200，回退到 state 中已选数据源，原始 74766 点 |
| 非法 dataset 路径 | HTTP 403，`path outside allowed roots` |
| POST 临时改 `minimum_z=-2.9` | 保存成功；随后 GET 读到 -2.9；再 POST 原值恢复 -3.0 |
| YAML 覆盖预览 | `body_filter.minimum_z` 与标定文件一致，包含四个 LiDAR 的扁平矩形与 `precheck.body_box` |

### 4.3 前端 UI 测试（Chrome DevTools Protocol 驱动真实页面）

通过 headless Chrome + CDP 打开 `http://127.0.0.1:8765/#body`，派发真实 DOM / Pointer 事件并读取页面状态：

- 页面 boot 后 `S.page === "body"`，`S.body.id === "zongyang/truck02/v007_gcons"`，默认 `back_lidar` 原始 72090 点、抽样 12000 点、过滤 8653 点。
- 车辆选择器切到 `wuhu/truck29/v118_camloop` 再切回 `zongyang/truck02/v007_gcons`，均正确重新加载。
- LiDAR 话题选择器遍历四个话题，矩形数量分别为 1 / 2 / 0 / 2，过滤点数随话题变化。
- 采样时刻切到 60 s，`t_s=60`，原始 85794 点、过滤 3247 点；再切回 0 s。
- 启用开关关闭后 `filtered_count` 从 8653 变为 0；重新开启恢复。
- `minimum_z` 数值输入由 -3.0 改为 -2.8，前端重算过滤点数；XZ 画布 pointer 拖拽把 `minimum_z` 改为 0.23，证明拖拽链路有效；随后恢复。
- 矩形数值输入把第一个 `x_min` 从 -1.5 改为 -1.2；「新增」按钮使矩形数 1→2，「删除」按钮恢复 2→1。
- XY 画布在空白处拖拽绘制新矩形，矩形数 1→2；随后删除恢复。
- 后轴框数值输入把 `x_min` 从 -3.2 改为 -2.8；后轴画布拖拽重画使 `rear_axle_box` 发生变化；随后恢复。
- 「重新加载」按钮重新 GET 成功，恢复默认 `back_lidar`、原始 72090 点、`minimum_z=-3.0`。
- 「保存到标定文件」按钮 POST 成功，页面状态显示 `enabled=false`、`source="Studio 车身点云可视化过滤"`、`updated` 刷新；YAML 预览包含 `minimum_z` 与四个 LiDAR 键。
- 保存后已从备份恢复精确原始标定文件，最终 GET 验证 `enabled=true`、`minimum_z=-3.0`、原 source、原 `updated=2026-10-27T10:40:37+08:00`、`front_right_lidar` 两个矩形。
- 控制台仅有 `/favicon.ico` 的 404 网络日志；加入内联 data favicon 后消除。无 JavaScript 异常。
- 生成截图 `/tmp/body_page_cdp.png`（临时验证产物，不在仓库中）。

### 4.4 最终状态

- 用户侧 8765 Studio 进程已用补丁后的 `server.py` 重启，GET / POST `/api/body-filter` 均 200。
- `config/carlibration/zongyang/truck02/v007_gcons/calibration.json` 的 `body_filter` 保持用户此前保存的值，测试过程中的临时修改已全部还原。
- 分支仍为 `mcp/vehicle-body-box-visual-filter`；本次仅新增 `index.html` 的 favicon 链接与本诊断文档，未改动标定算法。

## 5. 后续操作建议

1. 在用户浏览器中对 Studio 页面执行一次强制刷新（Ctrl+F5），确保拿到带 data favicon 的 `index.html`。
2. 后续若再修改 `tools/studio/server.py` 的路由，务必重启 Studio 进程；`studio.app.*` 的修改可由 `reload_app_modules()` 自动热重载。
3. 若容器 `calibration` 重启，由于当前未配置 supervisor，需要手动重新执行第 3 节的启动命令。

## 6. 本仓库相关文件

- 服务端路由与热重载边界：`tools/studio/server.py`
- 车身过滤 API：`tools/studio/app/body_filter.py`
- 前端页面与交互：`tools/studio/web/index.html`、`tools/studio/web/app.js`、`tools/studio/web/style.css`
- 标定文件 schema 与默认回退：`tools/livo/body_filter.py`
- Studio 既有工作流：`docs/mcp-studio-v2-workflow.md`
