# 代码审计报告 —— 三维激光惯性 SLAM 系统 V1.0 切片

> 审计时间：2026-10-26。对象：`system/backend/main.py`、`system/backend/static/index.html`、`docker/` 流水线脚本。原则：区分"真实实现 / 部分实现 / 未实现 / Bug"。

## 1. FAST-LIO2 调用链真实性审计（红线项）

**结论：开始重建真实执行 FAST-LIO2，无伪装。** 证据链：

1. `POST /api/jobs` → `main.py:_run_pipeline()` → `subprocess.Popen(["docker","run","--rm","--net=host","-v","/data2/gxw/slam/ws:/root/ws",...,"fastlio:noetic","bash","/scripts/run_fastlio_kitti_verify.sh"])`。
2. 容器内：`roscore` → `roslaunch fast_lio mapping_kitti.launch`（加载 `config/kitti.yaml`）→ `rosbag play`。节点为 `fast_lio/fastlio_mapping`（hku-mars 原版，commit `7cc4175`）。
3. 输出 `/Odometry`（nav_msgs）由 FAST-LIO2 iEKF 产生；地图 PCD 由其 ikd-tree 增量累积（SIGINT 后保存）。
4. **不存在参考轨迹参与建图的路径**：0001/0033 属 KITTI raw 序列，本机从未下载过任何官方 odometry 真值文件，评估参考（OXTS GPS/INS）只存在于转换器内部的独立 CSV，不进入建图容器流程。
5. 佐证：运行期间 `fastlio_mapping` CPU 1.0~1.8 核（真做几何计算）；0033 发散事故（oxts 陀螺仪 bug）恰恰说明跑的是真实滤波器——伪装系统不会"发散"。

## 2. 后端接口审计（main.py，261 行）

| 接口 | 状态 | 说明 |
|---|---|---|
| GET /api/datasets | ✅ 真实 | 硬编码 2 个数据集 + 文件系统检查 has_map/has_traj |
| POST /api/jobs | ✅ 真实 | 单任务互斥锁；后台线程跑容器流水线 |
| GET /api/jobs(/{id}) | ✅ 真实 | 但**日志只留尾部 15 行**，且**内存态，重启即失**（无持久化） |
| GET /api/map/{tag} | ✅ 真实 | PCD 抽样（3050万→152万点）+ 缓存 |
| GET /api/traj/{tag} /stats | ✅ 真实 | 从 CSV/numpy 实算，非静态值 |
| 停止任务 | ❌ 缺失 | 无 stop API；`docker run` 无 `--name`，外部也无法停 |
| 任务历史/实验管理 | ❌ 缺失 | 无数据库，无法对比历史实验 |
| 数据集校验 | ❌ 缺失 | 无帧数/IMU 频率/时间覆盖检查 API |
| 评估 (ATE/RPE) | ❌ 缺失 | 无参考轨迹、无对齐、无指标 |
| 系统监控 | ❌ 缺失 | 无 CPU/内存/容器状态 API |
| 算法配置 | ❌ 缺失 | 参数写死在 launch/yaml，无 API |

## 3. 前端审计（index.html，单页）

| 项 | 状态 | 说明 |
|---|---|---|
| WebGL 三维渲染 | ✅ 真实 | 手写 mat4/着色器，高度着色 + 轨迹，环绕相机 |
| **地图点数显示** | 🐛 **Bug** | `meta.points_served?.toLocaleString ?? st.map_points.toLocaleString()` 缺调用括号——`?.` 后接 `??` 时若 meta 存在会把**函数引用**当字符串渲染，HUD 显示成 `function toLocaleString() {...}` |
| **FPS** | 🐛 **Bug** | 有采样循环 `fps=frames*2`，但从未写入 DOM，`<span id="fps">` 永远显示 `-` |
| 数据集管理页 | ❌ 缺失 | 仅一个下拉框 |
| 算法配置/评估/实验/报告/日志页 | ❌ 缺失 | — |

## 4. 数据流水线审计（docker/*.sh/.py）

| 项 | 状态 | 说明 |
|---|---|---|
| kitti_to_bag.py | ✅ 真实已验证 | ring/time 重建、100Hz oxts、合成角速度（本机独有发现） |
| 逐帧耗时采集 | 🐛 **Bug→已定位** | 验证脚本 grep `"time: [0-9.]+"` 匹配数为 0——FAST-LIO 实际格式是 `[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ... ave total: %0.6f`（laserMapping.cpp:1009，"time: " 后跟字面量 IMU）；**且源码本就输出逐帧 CSV**：`Log/fast_lio_time_log.csv`（总耗时/点数/ikd-tree 增删搜/预处理，已实测存在于 ws/src/FAST_LIO/Log/，0033 运行留下 63 帧真实数据）+ `Log/mat_out.txt`（每帧位姿阵）+ `pos_log.txt`。此前系统完全没接入 |
| 陈旧产物风险 | 🐛 | 脚本不清理旧 `PCD/scans.pcd`；若本次运行失败，runner 仍可能把上一次的 PCD 当成本次结果归档 |
| 位姿/真值数据 | ✅ 可用 | 里程计 bag（739 帧带时间戳）在 /data/kitti/；oxts 100Hz 可作 GPS/INS 参考轨迹（非官方 odometry 真值，评估口径须注明） |

## 5. 功能差距清单（对照需求 8 模块）

| # | 模块 | 现状 | 差距 |
|---|---|---|---|
| 1 | 数据集管理 | ◐ 列表+切换 | 校验 API（帧数/IMU 频率/时间戳/ring-time/标定验证）、按路径导入 |
| 2 | 建图与任务管理 | ◐ 真实但简陋 | 停止任务、全量日志落盘、逐帧位姿/耗时/地图点数服务、运行元数据入库 |
| 3 | 三维可视化 | ◐ 渲染有效 | 逐帧回放/进度、视角切换、着色切换、点大小/抽稀、坐标轴、PCD 导出、修复 2 个 Bug |
| 4 | 轨迹评估 | ❌ | OXTS 参考轨迹生成、时间关联、Umeyama 对齐、ATE/RPE、对比图 |
| 5 | 算法配置与实验管理 | ❌ | 参数 API+校验+预设、SQLite 实验库、历史查询、双实验对比 |
| 6 | 实验报告 | ❌ | 汇总 JSON/CSV 导出、HTML 报告（PDF 需装 reportlab，列为可选） |
| 7 | 闭环 | ❌ **按设计保留 V2.0，不实现不标记** | STD + 位姿图优化；前置：0020/0018 重访实测 |
| 8 | 监控与日志 | ❌ | CPU/内存/容器实时状态、日志检索、错误原因 |

## 6. 分阶段改造计划（按优先级）

- **P0 修 Bug + 真实性加固（先行）**：前端点数/FPS 两处 Bug；流水线预清理旧产物；接入 `Log/` 真实计时与位姿数据；任务加 `--name` 与 stop API。
- **P1 数据与评估（软著核心自研）**：SQLite 实验库；数据集校验 API；流水线导出带时间戳轨迹；OXTS 参考轨迹生成器；Umeyama 对齐 + ATE/RPE（自研实现，标注"GPS/INS 参考"口径）+ 单元测试（合成轨迹已知变换 → 指标可复现）。
- **P2 多页软件**：侧边栏 SPA 九个页面（总览/数据集/任务/工作台/配置/评估/实验/报告/日志），全部接真实 API，未实现功能一律显示"规划中"，不放空按钮。
- **P3 报告与验收**：JSON/CSV/HTML 报告导出；自动化自检脚本 + 真实端到端运行证据存 `docs/VERIFICATION.md`。
- **V2.0 闭环**：仅设计文档，明确标注未实现。

逐项完成标准：每个 API 有 curl/测试证据，每个页面有真实数据截图或响应样例，指标可由命令行复算。
