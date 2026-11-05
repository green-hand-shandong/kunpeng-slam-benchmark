# 多算法 SLAM 在鲲鹏 920 上的适配与实测基准

> **Kunpeng 920 (aarch64) × 经典 SLAM/LIO 横向对比**：FAST-LIO2、Point-LIO、KISS-ICP、Cartographer 在同一台鲲鹏 920 服务器、同一份 KITTI 数据、同一套评估口径下的完整适配记录与量化对比。所有结论均由本机实测得出，附可复现的 Dockerfile 与脚本。

**硬件**：华为鲲鹏 920（96 核 TaiShan V110，ARMv8.2 + NEON，1TB 内存）+ 昇腾 310P（NPU，本基准未用到）。
**数据**：KITTI Raw `2011_09_26_drive_0001`（108 帧，10Hz 64 线，车均速 ~10m/s）+ GPS/INS 参考轨迹。
**评估口径**：ATE（Umeyama SE(3) 对齐）/ RPE / 帧覆盖 / 地图规模，全部走同一 `evaluator.py`。

## 实测结果（kitti_0001 同口径横向对比）

| 算法 | 路线 | 许可 | 帧覆盖 | 3D 路径 | ATE RMSE | 逐帧耗时 | 结论 |
|---|---|---|---|---|---|---|---|
| **FAST-LIO2**（基线） | catkin 容器 | GPL-2.0 | 105/108 | 104.9m | **1.958m** | ~211ms | 车载首选，成熟稳定 |
| **Point-LIO** | 同族镜像复用 | GPL-2.0 | 106/108 | 103.5m | **0.084m** | — | **本组精度最优**；高频点级估计 |
| **KISS-ICP** | pip 纯 Python+numba | MIT | 108/108 | — | **0.607m** | **31ms（32Hz）** | 零 ROS 依赖、超实时 3 倍 |
| **LeGO-LOAM** | C++17 + GTSAM | BSD-3 | 27 关键帧 | 9.5m | **1.445m** | — | 纯几何（弱 IMU）正常水平 |
| **LIO-SAM** | GTSAM 4.1.1 源码编译 | BSD-3 | 26 关键帧 | 122.3m | **18.1m** | — | 跑通；合成 IMU 与其 9 轴高频假设差距大，调参后 19.8→18.1m |
| **Cartographer** | 源码构建 | Apache-2.0 | 67/108 | 24.4m | 24.7m | ~140ms | 车载高速场景结构性不适配（见下） |

**第二序列交叉验证（kitti_0033，1594 帧 / 1.7km 带回环）**：KISS-ICP ATE 33.0m（纯里程计无 IMU 无回环，漂移 ~2%，
符合算法定位）；同序列 FAST-LIO2 基线 4.83m。数据适配层（kitti_to_bag.py）在两条不同日期标定的序列上均可复用。

三条硬结论：

1. **鲲鹏 920 可以跑通全部主流 LIO 路线**（C++ catkin 栈、纯 Python 栈），ARM 平台不构成任何阻碍——所有适配坑都是算法自身的工程问题，x86 上同样存在。
2. **Point-LIO 精度最佳（0.084m），KISS-ICP 工程性最佳（pip 即用、31ms/帧）**，FAST-LIO2 成熟稳定，LeGO-LOAM 轻量可用（1.445m），LIO-SAM 跑通待真 IMU。车载梯队：Point-LIO / FAST-LIO2 / KISS-ICP。
3. **Cartographer 的 3D 前端对"高速起始 + 10Hz 点云"车载场景结构性不适配**：轨迹空间塌缩 4 倍（真值 112.9m 它只输出 24.4m），6 轮"一次一个变量"的修复实验（点云时间约定/匹配权重/截远点/累积帧/gravity 常数/odom 输入）全部无效——塌缩对配置不敏感，是 `PoseExtrapolator` 位姿差分速度估计被"起始钉死"历史拖住的正反馈。**它适合低速/室内场景**（其官方用例即背包/室内），适配工作已全部完成并固化，随时可接入平台。


## 算法全景：按输入模态界定

平台按**输入模态**组织算法，每个模态一个可插拔的算法矩阵（同模态内同数据同口径横向对比）：

| 模态 | 典型场景 | 平台状态 |
|---|---|---|
| **A. 2D 激光（±IMU/轮式）** | 室内机器人、扫地机 | Cartographer 2D 模式同镜像换配置即可；SLAM Toolbox/Gmapping/Hector 备选 |
| **B. 3D 激光 + IMU（LIO）** | 车载、无人机、手持 | **当前主线**：Point-LIO / KISS-ICP / FAST-LIO2 / LeGO-LOAM / LIO-SAM 已全部实测（见上表）；FASTER-LIO 排期 |
| **C. 视觉 / 视觉-惯性** | 室内、无激光、无人机 | **VINS-Fusion 已构建跑通**（双目模式，KITTI 0033；IMU 融合待 EuRoC 级数据）；ORB-SLAM3 待立项 |
| **D. 激光-视觉-惯导融合** | 高精度建图、研究 | **R3LIVE 构建成功 + 运行时验证**（合成 bag 全链路消费；FAST-LIVO2 未开源） |
| **E. 学习类（GPU/NPU）** | 语义、动态物体剔除 | **SuperPoint 特征前端在昇腾 310P 上推理成功（1.6ms/帧，torch_npu）**；语义 SLAM 外挂排期 |

详细选型矩阵与逐算法适配档案见 `system/docs/ALGO_PLAN.md`。

## 目录结构

```
├── README.md                    # 本文件
├── 00~07_*.md                   # 调研/实战文档（见下表）
├── docker/
│   ├── Dockerfile.fastlio       # fastlio:noetic 镜像（FAST-LIO2 基座，Point-LIO 复用）
│   ├── Dockerfile.carto         # carto:noetic 镜像（Cartographer 1.0.0，含全部 aarch64 修复）
│   ├── kitti_to_bag.py          # KITTI bin/oxts → rosbag（补 ring/time 字段、100Hz IMU、合成角速度）
│   ├── run_fastlio_kitti*.sh    # FAST-LIO2 一键运行/验证
│   ├── run_pointlio_kitti.sh    # Point-LIO 一键运行/验证
│   ├── run_carto_smoke.sh       # Cartographer 冒烟测试
│   ├── point_lio_config/        # Point-LIO 的 KITTI 配置与 launch
│   └── render_map.py            # 地图渲染出图
├── ws2/cfg/                     # Cartographer 配置（kitti.lua + URDF + 6 轮修复实验的 R1~R4 变体）
├── system/                      # 《多传感器激光惯性定位建图与三维轨迹评估软件》
│   ├── backend/                 # FastAPI 后端 + evaluator.py（ATE/RPE 评估器）
│   ├── static/                  # 零依赖 WebGL 三维查看器
│   └── docs/ALGO_PLAN.md        # 多算法选型矩阵、适配记录、实测结果全档案
└── figures/                     # 地图/轨迹渲染图
```

### 调研与实战文档

| 文件 | 内容 |
|---|---|
| 00_进度与计划 | 工作总账与迭代记录 |
| 01_经典数据集 / 02_经典SLAM算法 | 数据集与算法选型矩阵（含算力评级） |
| 03_算力需求与国产芯片 | 算力瓶颈拆解、国产 CPU/NPU 选型 |
| 04_开源生态与容器支持 | 各 SLAM 项目的 ARM/鲲鹏支持现状实测 |
| 05_贡献路线 | arm64 CI / 多架构 Dockerfile / 上游 PR 计划 |
| 06_跑通记录 | KITTI + FAST-LIO2 完整复现命令与踩坑记录 |
| 07_系统建设方案 | 软件系统架构与软著边界 |
| system/docs/ALGO_PLAN | **多算法适配全过程档案**（选型矩阵、6 轮修复实验、对比表） |

## 复现（鲲鹏/任何 aarch64 主机）

```bash
# 0. 数据：KITTI Raw 0001（S3 直链 + aria2），转换脚本自动补 ring/time/IMU
# 1. 镜像
docker build -f docker/Dockerfile.fastlio -t fastlio:noetic .
docker build -f docker/Dockerfile.carto   -t carto:noetic   .
# 2. bin+oxts → bag（容器内执行）
docker run --rm -v /data0/slam-datasets:/data -v $PWD/docker:/scripts \
  fastlio:noetic bash -c "source /opt/ros/noetic/setup.bash && \
  python3 /scripts/kitti_to_bag.py <sync_dir> <oxts_dir> /data/kitti/kitti_0001.bag"
# 3. FAST-LIO2 / Point-LIO（容器内一键：roscore+算法+录制+分析）
docker run --rm --net=host -v $PWD/ws:/root/ws -v /data0/slam-datasets:/data \
  -v $PWD/docker:/scripts fastlio:noetic bash /scripts/run_fastlio_kitti_verify.sh
# 4. KISS-ICP（零 ROS）
docker run --rm -v /data0/slam-datasets:/data python:3.11-slim bash -c \
  "pip install kiss-icp pyyaml && kiss_icp_pipeline <velodyne_data_dir> --config /tmp/kiss.yaml"
# 5. 评估：ATE/RPE 统一走 system/backend/evaluator.py（Umeyama 对齐，口径与表内一致）
```

Cartographer 复现注意事项（6 个运行期坑的解法都已固化在 `ws2/cfg/` 与 `run_carto_smoke.sh`）：
配置需对齐 1.0.0 键集（`use_nav_sat`、三个 `*_publish_period_sec`）；`tracking_frame` 需要 tf 树
（用 `calib_imu_to_velo` 取逆建 URDF/静态 TF）；offline 模式需 `points2:=/velodyne_points imu:=/kitti/oxts/imu`
remap，否则 expected sensor id 与 bag 话题不匹配导致**消息被静默丢弃**。

## 平台

`system/` 是配套的《多传感器激光惯性定位建图与三维轨迹评估软件 V1.0》（FastAPI + 零依赖 WebGL）：
数据体检 → 容器流水线（多算法）→ ATE/RPE 评估 → 三维可视化 → 实验库横向对比。`system/docs/` 有完整方案。

## 许可

- 本仓库的文档、脚本、评估器、平台代码：Apache-2.0
- 各算法遵守其上游许可（FAST-LIO2/Point-LIO: GPL-2.0；KISS-ICP: MIT；Cartographer: Apache-2.0）——本仓库不包含任何算法源码，仅含适配配置与构建脚本
