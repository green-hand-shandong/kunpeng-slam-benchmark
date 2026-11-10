# 2. Related Work (Draft)

**SLAM algorithm families.** Filter-based LiDAR-inertial odometry (FAST-LIO2 [1], Point-LIO [2]) fuses IMU mechanics with iterated Kalman updates over raw points; factor-graph methods (LIO-SAM [3], LeGO-LOAM [4]) add loop closure via GTSAM; graph-based frameworks (Cartographer [5]) rely on submap-level local optimization; lightweight geometric systems (KISS-ICP [6]) dispense with IMU entirely; visual-inertial systems (VINS-Fusion [7]) and LiDAR-visual-inertial systems (R3LIVE [8]) broaden the modality space. Learning-assisted front-ends (SuperPoint [9], semantic segmentation [10]) supply data-driven features or masks.

**Platform-specific evaluation.** Prior SLAM evaluations target x86 workstations or embedded ARM boards (e.g., ORB-SLAM3 on Raspberry Pi 5 [11]). Server-class domestic ARM (Kunpeng 920, 96-core ARMv8.2) and heterogeneous NPU acceleration (Ascend 310P) have not been systematically evaluated for SLAM workloads; existing reports are single-algorithm or benchmark-layer only. Our work fills this gap with a multi-algorithm, multi-modality study on the full software stack.

**Semantic filtering in LiDAR SLAM.** Removing dynamic-object points via segmentation (Removert [12], ERASOR [13], semantic LIO variants) is a known accuracy/robustness lever in dynamic scenes. Existing studies report gains in urban driving; however, the cost side—discarding static vehicles that act as constraints—is rarely quantified. Our closed-loop experiment provides this quantification on a domestic heterogeneous platform.

# 3. Methodology (Draft)

## 3.1 Evaluation Platform

Kunpeng 920 (96 cores, ARMv8.2, 2.6 GHz) hosts all geometric back-ends. The Ascend 310P NPU (8 TOPS INT8 / 16 TFLOPS FP16) hosts learning front-ends through torch_npu 2.9.0 on CANN 9.0.1. Each algorithm is containerized (Docker, ROS Noetic, Ubuntu 20.04 aarch64) with pinned dependency versions; images and Dockerfiles are released.

## 3.2 Data Adaptation Layer

KITTI raw drives carry no ROS-native messages; our adaptation layer (`kitti_to_bag.py`) converts raw bin/oxts to `PointCloud2` (with per-point ring and relative-time fields synthesized from HDL-64E geometry) and 100 Hz IMU messages (oxts acceleration/gyro, orientation from Euler angles). The layer is algorithm-agnostic: all six LiDAR pipelines consume identical bags, eliminating data-side confounds. Known pitfalls are documented (e.g., oxts "IMU" is GPS-derived and unsuitable for tight VIO coupling—a finding that explains VINS-Fusion's divergent KITTI stereo+IMU mode while the EuRoC MH_01 mode with a real IMU achieves 0.228 m ATE).

## 3.3 Unified Metrics

Trajectories are timestamp-matched (0.05 s tolerance), Umeyama-aligned (SE(3)), and scored with ATE RMSE/mean/max and RPE at 1 s intervals. Every reported number is cross-checked by evo with agreement within 0.1%; both tools ship in the release. Per-frame cost is reported from algorithm-internal timers where available and externally measured wall-clock splits otherwise, with the source annotated per entry.

## 3.4 Semantic Closed-Loop Protocol

Masks are generated per frame (yolo26n-seg, conf 0.25, dynamic classes only), projected through the factory velodyne→camera calibration, and masked points are removed before FAST-LIO2 ingestion. The protocol evaluates the full pipeline (front-end cost + geometric impact), not the front-end in isolation.
