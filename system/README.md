# 三维激光惯性 SLAM 建图与轨迹评估系统 V1.0（垂直切片）

> 目标形态：在国产一体机（鲲鹏 920 + 昇腾 310P）上，通过网页完成「选数据集 → 一键重建 → 三维查看地图与轨迹 → 查看质量指标」。算法前端复用 FAST-LIO2（GPL-2.0，容器隔离），数据适配 / 任务管理 / 评估 / 可视化为本项目自研（软著申报边界）。

## 当前能力（V1.0 切片）

| 模块 | 状态 | 说明 |
|---|---|---|
| 数据适配 | ✅ 已验证 | `docker/kitti_to_bag.py`：KITTI bin+oxts → rosbag，ring/time 重建、100Hz IMU、**合成角速度**（修复 KITTI oxts 无陀螺仪问题，kitti2bag 亦有此 bug） |
| 激光惯性里程计 | ✅ 已验证 | FAST-LIO2 于 `fastlio:noetic` 容器，鲲鹏实测 1~2 核实时 |
| 三维地图重建 | ✅ 已验证 | 0033 全序列 3050 万点 PCD 已产出 |
| 轨迹质量统计 | ✅ 本切片含 | 轨迹长度/均速/高度范围/里程计帧数（ATE/RPE 对接 KITTI 真值 → V1.1） |
| 三维交互可视化 | ✅ 本切片含 | 零依赖手写 WebGL：高度着色点云 + 轨迹 + 环绕相机（左键旋转/滚轮缩放/右键平移） |
| 建图任务管理 | ✅ 本切片含 | 网页一键触发容器流水线，实时日志，完成后自动入库 |
| 闭环分析 | 规划 V2.0 | STD（hku-mars）地点识别 + 位姿图优化，用 KITTI Odometry 05/06/09 验证 |

## 目录

```
system/
├── backend/main.py          # FastAPI：数据集/任务/地图/轨迹 API + 静态页
├── backend/static/index.html # 零依赖 WebGL 前端（手写渲染，无 three.js）
└── tools/                   # 数据工具（PCD 抽样等，V1.1 扩充）
```

## 启动

```bash
cd /data2/gxw/slam/system/backend
nohup uvicorn main:app --host 0.0.0.0 --port 8000 >> /tmp/slam_system.log 2>&1 &
# 浏览器访问 http://<本机IP>:8000
```

首次访问会自动把已建图数据集的 PCD 抽样为网页格式（0033 约 1 分钟）；对未建图数据集点击「开始重建」即调用容器流水线（0001 约 1 分钟，0033 约 4 分钟）。

## API

| 方法/路径 | 功能 |
|---|---|
| GET /api/datasets | 数据集列表（是否已建图） |
| POST /api/jobs {dataset} | 启动重建任务（同时仅允许一个） |
| GET /api/jobs/{id} | 任务状态 + 日志尾部 |
| GET /api/map/{tag} | 抽样点云二进制（float32 x,y,z,intensity）+ X-Meta 元数据 |
| GET /api/traj/{tag} | 轨迹 JSON + 长度统计 |
| GET /api/stats/{tag} | 质量指标汇总 |

## 数据/产物位置

- 数据集 bag：`/data0/slam-datasets/kitti/*.bag`
- 建图产物（按数据集归档）：`/data0/slam-datasets/results/{tag}/{map.pcd, traj.csv, map_web.bin}`
- 工作区/镜像：`/data2/gxw/slam/{ws, docker}`（复用调研阶段成果）

## V1.1 计划

1. ATE/RPE 评估模块：对接 KITTI Odometry 真值（0001/0033 无官方 odometry 真值，选 00/05/06/07/09 等训练序列），坐标系转换 + 刚体对齐 + evo 同源指标实现。
2. 数据集自动导入：解析 KITTI raw 目录结构，自动计算外参并生成配置。
3. 渲染升级：LOD/八叉树分块（支持 3000 万点不抽样浏览）、相机跟随视角、逐帧回放。
4. 实验管理：任务历史入库（SQLite）、参数记录、多次实验对比与报告导出。

## V2.0 计划（闭环）

STD（Stable Triangle Descriptor, hku-mars, GPL-2.0）做地点识别候选 → 几何验证 → 位姿图优化（GTSAM）→ 校正前后轨迹/地图对比视图。验证序列：KITTI Odometry 06（`2011_09_30_drive_0020`）与 09（`2011_09_30_drive_0033`，回环在序列结尾）。

## 许可边界（软著申报须知）

- FAST-LIO2 / STD：GPL-2.0——以容器/进程方式隔离调用并保留版权声明；自研部分（数据适配、任务管理、评估、可视化、合成角速度方法）为独立成果。
- KITTI：CC BY-NC-SA 3.0——仅研究与演示用途，不得随商业产品分发。
