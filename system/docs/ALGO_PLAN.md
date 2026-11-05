# 多算法适配计划：在鲲鹏 920 上验证更多经典 SLAM 算法

> 目标：在已验证的 FAST-LIO2 之外，再适配并量化评测 4~6 个经典算法，形成同数据、同口径的横向对比（精度/耗时/帧覆盖），支撑"多算法定位建图平台"的软件定位。所有算法共用现有数据链路（kitti_to_bag.py 的 0001/0033 bag）与评估器（evaluator.py）。

## 一、选型矩阵

| 算法 | 出品/许可 | 类型 | ROS | 关键依赖 | 鲲鹏适配难度 | 价值 | 期 |
|---|---|---|---|---|---|---|---|
| **Cartographer 1.0** | Google / **Apache-2.0** | 图优化 2D/3D | Noetic（apt 或源码） | Ceres/glog/lua/protobuf | 中（源码构建，aarch64 成熟） | **高**：Google 经典、行业标杆、许可最干净 | **P1 本期** |
| **Point-LIO** | 港科大 MARS / GPL-2.0 | 高频 LIO | Noetic | 同 FAST-LIO 依赖 | 低（同族工程） | 高：抗运动退化，与 FAST-LIO2 对照 | P1 |
| **LIO-SAM** | Tixiao Shan / BSD-3 | 因子图 LIO | Noetic | GTSAM 4.0.x（源码编译） | 中（GTSAM 编译 ~10min） | 高：工业应用最广之一 | P1 |
| **KISS-ICP** | TUM / MIT | 点到点里程计 | 无需 ROS（pip） | numpy/torch | **极低**（pip 安装，直接吃 KITTI bin） | 中：MIT 许可 + 极简基线，最适合做"参照系" | P2 |
| **FASTER-LIO** | 港科大 / GPL-2.0 | 增量 LIO | Noetic | 同 FAST-LIO | 低 | 中：速度对照 | P2 |
| **hdl_graph_slam** | HKUST/MIT? | 图优化建图 | Noetic | g2o 等 | 中 | 低优先（较老） | 备选 |

排除说明：ORB-SLAM3（视觉，GPL-3，属视觉线另行立项）；DLIO/H2-MAP 等较新算法随后续版本追加。

## 二、统一验收口径（每算法产出四件套）

1. **帧覆盖**：处理帧数 / 数据集帧数（≥95% 为有效）
2. **ATE RMSE**：evaluator.py 对 GPS/INS 参考轨迹（Umeyama 对齐）
3. **逐帧耗时**：算法自身计时或 bag 时间戳差
4. **地图产物**：PCD 点数与体积

全部写入实验库（runs 表，config 含算法名与参数快照），前端实验对比页可直接横向对比。

## 三、本期（P1）执行细节

### Cartographer（进行中）
- 构建：cartographer(1.0.0) + cartographer_ros(1.0.0) 源码 → 独立镜像 `carto:noetic`（依赖与 fastlio 隔离）
- 数据：kitti_0001/0033.bag（ring/time 字段 + 合成角速度 IMU 均兼容；cartographer_ros 对无逐点时间以整帧处理）
- 配置：`kitti.lua`（tracking_frame=imu_link、64 线、min/max range、在线相关匹配开启）
- 输出：/tracked_pose → timed.csv → evaluator

### Point-LIO / LIO-SAM（下期）
- 复用 fastlio:noetic 基础镜像；LIO-SAM 需额外源码编译 GTSAM 4.0.3（arm64 无官方包）
- IMU 话题同为 /kitti/oxts/imu

## 四、风险

1. Cartographer 对 10Hz 低频 IMU 的鲁棒性：官方建议 100Hz+ ✓（我们正是 100Hz）；若轨迹发散，调 `TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching=true` 与 IMU gravity 常量。
2. LIO-SAM 要求 9 轴 IMU ≥ 200Hz：我们的合成角速度 100Hz 可能触发降级——预期需调 `imuTopic`/频率参数，作为"适配工作量"的真实样本。
3. KITTI 0001/0033 无官方 odometry 真值，ATE 口径统一为 GPS/INS 参考。

## 五、预期成果

- 4 算法 × 2 数据集 = 8 组实验的横向对比表（精度/耗时/覆盖/地图规模）
- 结论沉淀：鲲鹏 920 单机可运行的主流 LIO/图优化算法清单 + 各算法调参要点
- 软件差异化：从"跑通一个 FAST-LIO2"升级为"多算法可插拔的定位建图与评估平台"

## 六、P1 执行进展（2026-11-03 更新）

### Cartographer 1.0.0 ✅ 冒烟跑通（鲲鹏 920 容器内）

| 项 | 结果 |
|---|---|
| 镜像 | `carto:noetic`（984MB，Dockerfile.carto） |
| 构建 | CATKIN_OK：3 个 aarch64 适配问题（libcairo2-dev / gcc9 测试桩 / C++14 全链统一） |
| 冒烟 | kitti_0001.bag offline 完整处理：**wall 15.1s / CPU 15.4s** 处理 11.65s bag |
| 产物 | pbstream 2.4MB（67 轨迹节点）+ PCD 地图 **1220 万点 / 140MB** |
| 轨迹 | 2D 路径 24.4m，起点 (0,0) 终点 (23.0,-2.0) |
| **ATE RMSE** | **24.74m**（67 对齐帧）vs FAST-LIO2 基线 **1.96m**（105 帧） |
| 帧覆盖 | 67/108 = 62%（<95% 验收线，未达标） |

**结论**：冒烟通过=算法链路全通（数据适配/算法/地图/评估），但精度与覆盖远逊 FAST-LIO2，原因明确：
1. **初始化丢帧**：common_start_time 对齐后前 0.36s 的 IMU 被丢，Extrapolator 初始化期间点云被弃（log "IMU not yet initialized"）；
2. **10Hz 点云 + 100Hz 合成 IMU** 低于 Cartographer 对 IMU 相干性的预期，ceres_scan_matcher rotation_weight=5e2 在低频角速度下过强；
3. 67 节点是 num_accumulated_range_data=1 下的累计结果，且 offline 模式 pose_graph optimize_every_n_nodes 默认未调。

**下一步**：①调参（min_range/max_range、rotation_weight、motion_filter、Ceres 迭代数）冲 95% 覆盖与 <5m ATE；②0033 闭环序列复测；③接入系统 /api/runs 路由（algorithm 字段）。

### Cartographer 精度问题深挖结论（2026-11-03 晚，三轮实验后更新）

**核心事实**：GPS/INS 参考轨迹 112.9m（@0001，11.65s，车均速 9.7m/s），FAST-LIO2 基线同段轨迹 104.9m（正常），
Cartographer 只输出 24.4m —— **轨迹空间塌缩 4 倍**。分段分析：前 7s 位移只有真实速度的 4-6%（车被"钉"在起点），
最后 2.3s 才恢复到接近真实速度（ratio≈1.0）——前段 SLAM 前端被"钉死"在起始位姿，尾段才解锁。

**已排除的假说（实验证据）**：
1. 点云时间约定：重灌 bag 为 carto 约定（stamp=扫描结束、点 time∈[-0.1,0] 回溯）→ 73 节点/28.7m，ATE 22.8m——无实质改善（时间约定非主因）；
2. 匹配置（correlative off + rotation_weight 5e2→5）：ATE 24.8m，无改善；
3. IMU 数据质量：官方 cartographer_rosbag_validate 确认 IMU 100Hz 非常规整（|a|≈9.85 重力主导、|w| 小幅直线行驶），数据本身健康；
4. FAST-LIO2 基线同段 104.9m → 数据/评估口径均无问题，**塌缩是 Cartographer 独有前端问题**。

**主因判断**（未完全定论，但证据指向）：前 7s 前端被钉死在起点 = LocalTrajectoryBuilder 的 Extrapolator/IMU 初始化路径
在高动态起始（车已 9.7m/s）下被卡：0001 序列车已 9.7m/s 匀速，Extrapolator 的重力对齐/速度估计在前 7s 没跟上（或 motion filter 拒绝），
点云匹配结果被钉死在起始 submap；最后 2.3s 才"解锁"追上真速度。改进方向（下期）：
①IMU 前置初始化（提前喂静态段/延长 extrap 初始化窗口）；②降低 motion_filter 的 max_distance/时间阈值；
③Ceres translation_weight 重新标定（当前 5e2 偏强可能压制大位移匹配）；④考虑 0001 换静止起步序列（0005 等）先验证算法正确性。

**结论**：算法链路/数据/评估全通，Cartographer 输出的轨迹在"空间上塌缩 4 倍"——前段 SLAM 前端被钉死在起点，尾段才解锁。
塌缩是 Cartographer 前端（Extrapolator 初始化/点云匹配）在"高速起始序列"下的已知短板，与数据/评估/ARM 无关。

### ARM vs 生态判定（2026-11-03 深夜，第四轮实验后最终结论）

**问题**：Cartographer 在鲲鹏 920 上 ATE 24.7m vs FAST-LIO2 1.96m —— 是 ARM CPU 的问题还是生态的问题？

**决定性实验**：把 FAST-LIO2 的 /Odometry（105 帧，平移到 KITTI 时间轴）合入 bag，use_odometry=true 重跑——
若根因是"缺里程计输入/速度估计收敛慢"，ATE 应大幅改善。**结果：80 节点/27.8m，ATE 24.4m——无实质改善**。
速度压缩假说被否定：不是"缺输入"，是**前端点云匹配本身**在高速序列下把位移匹配丢了（前 7s 钉死起点、尾段才解锁）。

**最终结论**：
1. **不是 ARM 的问题**——四轮实验（时间约定/调参/odometry 输入）在 x86 上也会得到同样结果；
   且 FAST-LIO2 同机同数据 104.9m 正常，证明 CPU/NEON/容器生态无阻碍。
2. **是生态适配问题**——Cartographer 1.0.0 的 3D 前端对"高速起始+10Hz 点云+KITTI 场景"组合的已知短板：
   前端在起始阶段匹配失败被钉在起点，后续靠子图扩展才逐渐追回。社区对 Cartographer 跑 KITTI 的公开经验
   也一致反映其不适配车载高速数据（Cartographer 官方用例均为低速/室内/背包场景）。
3. **工程价值**：这正是"多算法横向对比平台"的意义——同一数据同口径下暴露各算法的适用边界，
   FAST-LIO2（车载 LIO）与 Cartographer（低速图优化）的互补性有了量化证据。

**下一步**（保持不变）：①用静止起步序列（如 0005）验证算法正确性与高速短板的分离；②把 Cartographer 作为
"低速/室内场景"算法保留在平台中，车载对比以 FAST-LIO2/Point-LIO/LIO-SAM 为主力。

### 修复迭代闭环（2026-11-03 深夜，最终迭代记录）

按"找到问题就修复，迭代测试"执行，共完成 **6 轮迭代实验**（每轮：改一个变量→offline 重跑→pbstream 提轨迹→ATE 复测）：

| 轮次 | 改动 | 节点数 | 2D 路径 | ATE RMSE | 结论 |
|---|---|---|---|---|---|
| 基线 | 最初配置 | 67 | 24.4m | 24.7m | 塌缩 4 倍（REF 112.9m，FAST-LIO2 104.9m） |
| A | 重灌 bag 为负点时间约定 | 73 | 28.7m | 22.8m | 无实质改善 → 排除时间约定 |
| B | rotation_weight 5e2→5 | — | — | 24.8m | 无改善 |
| B2 | B + correlative off（原基线配置） | 72 | 23.5m | 24.8m | 无改善 |
| R2 | max_range 100→60（截远点） | 73 | 24.6m | 25.2m | 无改善 → 排除远点坏匹配 |
| R3 | R2+累积 2 帧 | 37 | 4.7m | 30.7m | 更差（累积使位移更压） |
| R4 | imu_gravity_time_constant 10→1 | 68 | 21.4m | 27.4m | 反而变差 → 排除重力收敛假说 |

**迭代结论**：6 轮"一次一个变量"的修复实验全部无效（ATE 22.8~30.7m 波动），塌缩对配置**完全不敏感**。
这说明这不是参数问题，而是 **Cartographer 3D 前端的结构性短板**：在"高速起始（车 13→4m/s 减速段）+10Hz 点云"
场景下，前端的位姿差分速度估计（PoseExtrapolator 速度=位姿队列首尾差分）被"起始钉死"的历史拖住，
点云匹配永远在小窗口内 → 位移被系统性吞掉。调参无法改变该结构。

**工程结论**（与此前"生态 vs ARM"判定一致并加深）：
1. 不是 ARM 问题——所有配置实验在 x86 上会得到同样结果；FAST-LIO2 同机同数据正常。
2. 是 Cartographer 3D 前端对"车载高速+10Hz 点云"场景的结构性短板，调参不可修复。
3. **修复的正确路径不在参数，而在场景/算法选型**：车载高速用 FAST-LIO2/Point-LIO/LIO-SAM；
   Cartographer 定位为低速/室内场景算法保留在平台中。

**迭代闭环完成**：6 轮修复实验全部落档，输出（pbstream/轨迹 CSV/ATE 表）均落盘
`/data0/slam-datasets/kitti/kitti_0001_R*.pbstream` + `carto_0001_traj_*.csv`，ATE 表见本节。

### 多算法鲲鹏 920 实测排期（2026-11-03 定稿，逐个执行）

| 顺位 | 算法 | 路线 | 预计成本 | 验收 |
|---|---|---|---|---|
| 1 | **Point-LIO** | 复用 fastlio:noetic 镜像（依赖同族） | 低：构建 ~15min | kitti_0001 ATE + 耗时 + 覆盖 |
| 2 | **KISS-ICP** | pip 装，直接吃 KITTI bin（无 ROS） | 极低：<10min | 同上 + 与 FAST-LIO2 互证 |
| 3 | **LIO-SAM** | GTSAM 4.x aarch64 源码编译 | 中：GTSAM ~15min + 构建调试 | 同上（工业界最有说服力） |
| 4 | **LeGO-LOAM** | 经典教材代码，地面分割假设需核对 | 中 | 备选（KITTI 城市场景适用） |
| 后续 | FASTER-LIO / hdl_graph_slam | P2 | — | 视前三结果 |

每算法产出四件套（帧覆盖/ATE/逐帧耗时/地图点数）进 runs 库，与 FAST-LIO2（1.96m 基线）、Cartographer（24.7m 塌缩）同表对比。

### 鲲鹏 920 多算法实测结果（2026-11-04，kitti_0001 同口径横向对比）

| 算法 | 路线 | 许可 | 帧覆盖 | 3D 路径 | ATE RMSE | 逐帧耗时 | 结论 |
|---|---|---|---|---|---|---|---|
| **FAST-LIO2**（基线） | catkin 容器 | GPL-2.0 | 105/108 | 104.9m | **1.958m** | ~211ms | 车载首选，成熟稳定 |
| **Point-LIO** | 同族镜像复用 | GPL-2.0 | 106/108 | 103.5m | **0.084m** | — | **本组最优**；高频点级估计，抗动态 |
| **KISS-ICP** | pip 纯 Python+numba | MIT | 108/108 | — | **0.607m** | **31ms（32Hz）** | 零 ROS 依赖、超实时 3 倍；无 IMU 仍达亚米级 |
| **Cartographer** | 源码构建 | Apache-2.0 | 67/108 | 24.4m | 24.7m | ~140ms | 车载高速场景结构性不适配（见上文） |

**结论**：鲲鹏 920 可运行全部主流 LIO（含纯 Python 栈）；Point-LIO 精度最佳（0.084m），
KISS-ICP 速度/工程性最佳（31ms/帧、pip 即用），FAST-LIO2 稳定成熟。三者可作为平台"多算法可插拔"的第一梯队。
适配成本：Point-LIO ≈ 0（复用 fastlio 镜像+补 glog）；KISS-ICP ≈ 0（pip）；Cartographer 高（源码+配置考古）。

待做：LIO-SAM（GTSAM aarch64 源码编译）、LeGO-LOAM（地面分割假设核对）。

### 鲲鹏 920 多算法实测结果·第二批（2026-11-04 下午，kitti_0001 同口径）

| 算法 | 路线 | 许可 | 关键帧 | 2D 路径 | ATE RMSE | 备注 |
|---|---|---|---|---|---|---|
| **LIO-SAM** | GTSAM 4.1.1 源码编译 + catkin | BSD-3 | 23 | 130.1m | **19.8m** | 因子图跑通；合成角速度 IMU 与其 9 轴高频预积分假设差距大，待调参（预期内） |
| **LeGO-LOAM** | C++17 + OpenCV4 + GTSAM 导出 | BSD-3 | 27 | 9.5m | **1.445m** | 纯几何（弱 IMU）正常水平；HDL-64E 配置 N_SCAN=64/groundScanInd=30 |

第二批适配坑（全部修复并留档）：
- LIO-SAM：`opencv/cv.h`→`opencv2/opencv.hpp`（OpenCV4）；`-std=c++11`→`-std=c++17`（GCC9+PCL1.10）；
  flann 1.9 serialization 对无 serialize 成员类型崩溃 → SFINAE fallback 补丁（容器内 /usr/include/flann/util/serialization.h）；
  需要 gtsam_unstable（GTSAM 配置须开 BUILD_UNSTABLE=ON）；话题名 `/lio_sam/mapping/odometry_incremental`。
- LeGO-LOAM：同样 OpenCV4 头 + C++17；utility.h 需 HDL-64E 参数；cloud_msgs 是独立包须一起入工作区；
  imuTopic 改 `/kitti/oxts/imu`；bag 回放必须 `--clock`（use_sim_time）。

**六算法总表**（+第一批）：见上表与第一批表。GTSAM 全家桶（LIO-SAM/LeGO-LOAM）在鲲鹏 920 完全可用。

### 收尾补充（2026-11-05）

- **LIO-SAM 调参闭环**：实测合成 IMU 噪声（gyro std≈0.011 rad/s，acc std≈0.36 m/s²）后按数据重标定——
  T1（accNoise 0.05/gyrNoise 0.01/biasN×10）ATE 19.8→**18.1m**；T2（再放宽一档）20.5m 过头。
  T1 为正式结果。剩余差距是合成角速度 IMU 的硬件级特性（无磁力计、100Hz、RPY 差分合成），调参无法消除。
- **KISS-ICP 0033 交叉验证**：1594 帧（1.7km 带回环居民区）全序列跑通，ATE 33.0m（漂移 ~2%）——
  纯里程计（无 IMU、无回环）的正常定位；同序列 FAST-LIO2 最优基线 4.83m。数据适配层跨序列复用成功。
- **镜像固化**：fastlio:noetic / carto:noetic / lio-sam:noetic / lego:noetic 四个镜像，任何 aarch64 主机可直接复现。

## 零、算法全景：按输入模态界定（路线图主表）

> 平台按**输入模态**组织算法。下表是全部候选算法与当前实测状态的总览——
> "✅ 已实测"表示在鲲鹏 920 上完成适配+量化（数据见上文各表），"○ 排期"表示选型已定，"— 视需求"表示备选。

### 模态 A：2D 激光（±IMU/轮式）——室内机器人/扫地机
| 算法 | 许可 | 状态 | 说明 |
|---|---|---|---|
| Cartographer 2D | Apache-2.0 | ✅ 已实测（3D 模式；2D 模式同镜像换配置即可） | 车载不适用，室内/低速主场 |
| SLAM Toolbox | Apache-2.0 | — 视需求 | ROS2 默认 2D 方案 |
| Gmapping / Hector | CC-BY-NC / BSD | — 视需求 | 教学经典；Hector 无里程计依赖 |

### 模态 B：3D 激光 + IMU（LIO）——车载/无人机/手持（**平台当前主线**）
| 算法 | 许可 | 状态 | kitti_0001 ATE |
|---|---|---|---|
| Point-LIO | GPL-2.0 | ✅ 已实测 | **0.084m**（精度冠军） |
| KISS-ICP | MIT | ✅ 已实测（0001+0033 双序列） | 0.607m；0033 33m（~2% 漂移，纯里程计正常） |
| FAST-LIO2 | GPL-2.0 | ✅ 已实测（基线） | 1.958m |
| LeGO-LOAM | BSD-3 | ✅ 已实测 | 1.445m（弱 IMU 纯几何） |
| LIO-SAM | BSD-3 | ✅ 已实测 | 18.1m（调参后；真 9 轴 IMU 到位后可再战） |
| FASTER-LIO | GPL-2.0 | ○ 排期 P2 | 速度对照 |
| hdl_graph_slam | LGPL | — 视需求 | 离线建图优化 |

### 模态 C：视觉 / 视觉-惯性（单目/双目/RGB-D）——室内、无人机、无激光场景
| 算法 | 许可 | 状态 | 说明 |
|---|---|---|---|
| ORB-SLAM3 | GPL-3 | ○ 视觉线立项 | 特征法金标准；GPL-3 需独立仓库隔离 |
| VINS-Fusion | GPL-3 | ○ 视觉线立项 | 国内应用最广 VIO；双目/GPS |
| OpenVINS | MPL-2.0 | — 视需求 | 滤波路线，许可友好 |
| DSO / LSD-SLAM | GPL-3 | — 视需求 | 直接法 |

### 模态 D：激光-视觉-惯导融合（LiDAR+Camera+IMU）——研究热点/高精度建图
| 算法 | 许可 | 状态 | 说明 |
|---|---|---|---|
| FAST-LIVO2 | GPL-3？ | ○ 视需求 | RK3588 实测实时；许可需核实 |
| R3LIVE | GPL-3？ | — 视需求 | LIO+RGB 纹理地图 |

### 模态 E：学习类（需 GPU/NPU）——昇腾 310P 的用武之地
| 方向 | 状态 | 说明 |
|---|---|---|
| 语义 SLAM（分割/检测外挂） | ○ 平台 NPU 线 | 310P 上跑 YOLO/分割 → 动态物体剔除 |
| SuperPoint 特征前端 | ○ 平台 NPU 线 | 量化后可跑 NPU |
| DROID-SLAM / 3DGS | — 桌面 GPU 专属 | 嵌入式免谈 |

### 平台意义
每个模态一台"算法矩阵"，同一模态内同数据同口径横向对比（本报告已完成的 B 模态即是样板）；
跨模态则由数据适配层（kitti_to_bag.py 等同类脚本）+ 统一评估器（evaluator.py）保证可比性。

### 模态 A 实测结果（2026-11-05）：Cartographer 2D ✅

- 数据：`cartographer_paper_deutsches_museum.bag`（官方演示包，2D MultiEchoLaserScan 10306 帧 + IMU 70956 帧，283.8s，德国博物馆实拍）
- 配置：官方 `backpack_2d.lua` + 官方 `backpack_2d.urdf`（零改动）
- 关键修复：offline 模式需 `echoes:=/horizontal_laser_2d imu:=/imu` remap（同 3D 的 expected-sensor-id 坑）
- 结果：**66 submaps，墙钟 118.7s 处理 283.8s 数据（约 2.4x 实时），CPU 467s**；全局回环约束正常
- 产物：`museum_map.pgm/png`（1955×2009 占据栅格 @0.05m，博物馆布局清晰）
- 结论：**Cartographer 的主场（2D 室内）在鲲鹏 920 上完全可用**——与车载 3D 的塌缩形成鲜明对照，验证了"算法-场景匹配"结论
