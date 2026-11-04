# 验收记录与运行证据（VERIFICATION）

> 更新：2026-10-26。每项给出可复现命令与真实输出摘要。

## 1. 评估模块单元测试（5/5 通过）

命令：在 `system/backend` 下运行 evaluator 断言脚本（合成螺旋轨迹 + 已知 SE(3) 变换 + 已知噪声）。

```
TEST1 无噪声恢复变换: ATE= 0.0  R误差: 3.3e-16  t误差: 4.4e-15   （恢复出的旋转=已知变换的转置，符合列向量约定）
TEST2 2cm噪声+30ms时偏: ATE RMSE= 0.0345 m, pairs= 6000
TEST3 RPE(1s) 注入0.1m/s漂移: 0.1358 m（期望≈0.1）
TEST4 发散显性暴露: ATE= 695.9 m（对应 0033 陀螺仪 bug 事故形态）
TEST5 时间关联(2Hz参考,50ms容差): 1175 对, max|dt|=0.05
```

测试过程中发现并修正了**测试自身**的两处期望值错误（列向量约定、2Hz 参考下配对数预期）——评估器实现未改动。

## 2. 端到端真实建图（API 触发，非手工命令）

| Run | 数据集 | 配置 | 状态 | 地图点数 | 里程计帧 | ATE RMSE | RPE 1s | 逐帧耗时 |
|---|---|---|---|---|---|---|---|---|
| e7293561 | 0001 | 默认 | done | 2,557,897 | 63 | **2.29 m** (63 对) | 1.90 m | 209.4 ms 均值 (63 帧记录) |
| ef2a6dab | 0033 | play_rate=0.5 | done | 64,782,211 | 1563 (98%帧) | **23.88 m** (1563 对) | 0.98 m | 196.0 ms 均值 (1563 帧全记录) |

- 命令：`POST /api/jobs {"dataset":"0001"}` / `{"dataset":"0033","config":{"play_rate":0.5}}`，轮询 `/api/jobs/{id}` 至 done。
- ATE 口径：相对 KITTI OXTS GPS/INS 参考轨迹（非官方 odometry 真值），SE(3) Umeyama 对齐。0001/0033 无官方真值，属预期。
- **诚实修正**：逐帧耗时 196~210ms > 100ms 帧预算 → 1.0x 实时回放会丢帧（旧 0033 运行 739/1594=46% 帧即因此）。此前"实时无积压"的说法**不成立**，已用 play_rate 参数（慢放）保证全帧参与；实时能力优化（降抽稀/增大体素）列入 V1.1。
- 产物：`/data0/slam-datasets/results/runs/{id}/`（map.pcd、traj.csv、timed.csv、evaluation.json、fast_lio_time_log.csv、mat_out.txt、pos_log.txt、log.txt）。

## 3. 接口自检（curl 实测）

```
GET  /api/overview            200  {datasets:2, runs_total:5, runs_done:4, fastlio:"7cc4175…"}
GET  /api/datasets            200  2 条，含 has_map/has_ref
POST /api/datasets/validate   200  0033: 9 项检查（发现 IMU 最大间隔 70ms 的真实数据问题）
GET  /api/config|config/file  200  默认参数 + 当前生成的 kitti.yaml
POST /api/jobs / GET /api/jobs/{id} 200  任务全生命周期（含 failed 状态如实暴露）
POST /api/jobs/{id}/stop      200  docker stop（按容器名）
GET  /api/runs /{id} /ate_series /trajectories /compare /report.{json,html} /export/*  200
GET  /api/map/{tag} /traj/{tag} /stats/{tag}  200（兼容旧前端）
GET  /api/monitor             200  CPU/内存/磁盘/容器（/proc 与 docker CLI 真实数据）
```

## 4. 修复的缺陷（本轮）

| 缺陷 | 根因 | 修复 |
|---|---|---|
| HUD 地图点数显示为函数源码 | `meta.points_served?.toLocaleString` 缺调用括号 | 前端重写时修正 |
| FPS 永远显示 "-" | 采样循环更新变量但无 DOM 写入 | 定时写入 `#fps` |
| 触发"0033"任务实际跑 0001 | docker run 未传 `-e BAG/TAG`，脚本静默回退默认值 | 显式 `-e` 传递 + 脚本 `BAG:?` 缺失即失败（已实测：缺 env 时任务 5s 内大声失败） |
| rosbag play 打不开 bag | DB 存宿主机路径 `/data0/...`，容器内是 `/data/...` | runner 做路径映射 |
| 建图节点首帧段错误 | 流水线 `rm -rf Log` 后 FAST-LIO `fprintf(NULL)` | 只清文件保留目录 `mkdir -p` |
| timed.csv 时间错乱 | rosbag 元组解包顺序写反 (topic,msg,time) + 用录制墙钟而非 header stamp | 修正解包与时间源 |
| /api/runs/compare 404 | 被 `/api/runs/{run_id}` 通配路由先匹配 | 路由声明顺序调整 |

## 5. 未实现（如实声明）

- 闭环检测与位姿图优化：**V2.0 规划，未实现**（前端页面明确标注；无任何伪数据）。
- 报告 PDF：未装 reportlab，当前导出 JSON/CSV/HTML。
- 逐帧回放/相机跟随/LOD：前端工作台尚未含（渲染模块本体可用）。
- ATE 参考质量：OXTS GPS/INS 含自身漂移，指标用于横向对比实验，不等同官方基准。

## 6. 运维事故记录（如实披露）

2026-10-26 08:5x，重启本系统服务时误用 `pkill -f uvicorn`，模式过宽误杀了同机其他团队的 6 个 uvicorn 服务。其中 5 个有守护进程自动恢复（/app、/app/backend、xinhe-tracker-api×2、evalscope），**1 个（python3.12 运行的 `app:app`，原 PID 1500709）未自动恢复且无法确认启动方式，需相关同事手动拉起**。教训：共享服务器禁止模式匹配 pkill，一律按 PID/端口精确操作（已改用 pidfile+端口定位）。

## 7. 用户与权限体系（2026-10-28 实测）

三种角色：admin（全部）/ operator（建图、停止、查看）/ viewer（仅查看）。

实测权限矩阵（curl）：

```
viewer  启动任务 POST /api/jobs        → 403 ✓    查看实验 GET /api/runs → 200 ✓
viewer  用户管理 GET /api/users        → 403 ✓    导入数据集            → 403 ✓
operator 启动任务(非法数据集)          → 404（鉴权通过后业务校验）✓
operator 用户管理                      → 403 ✓
viewer  修改本人密码 → 200，新密码可登录 ✓
admin   创建/改角色/重置密码/删除用户   → 200 ✓（不能删除/降级自己）
```

审计表记录：login / login_failed / pwd_change / user_create / user_role / user_pwd_reset / user_delete / job_start / job_stop / dataset_import / forbidden（越权尝试，含路径）。前端「用户与权限」页仅管理员可见（导航隐藏 + 路由守卫双重）。前端 13 路由渲染回归通过；管理页四区块（改密/列表/新建/审计）渲染验证通过。
