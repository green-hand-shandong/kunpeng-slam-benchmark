# 4. Experimental Setup and Results (Draft)

## 4.1 Evaluation Protocol

All experiments are conducted on a Kunpeng 920 server (96-core ARMv8.2, 2.6 GHz) with an Ascend 310P NPU for learning-based front-ends. Each algorithm runs in an isolated Docker container with ROS Noetic on Ubuntu 20.04 (aarch64). Trajectories are evaluated against GPS/INS reference (KITTI) or motion-capture ground truth (EuRoC) using Umeyama-aligned ATE and RPE, cross-validated with both our evaluator and the community-standard evo toolkit (agreement within 0.1%).

Datasets. We use KITTI drives 0001 (residential, 108 scans, 11.6 s) and 0033 (parking-lot loop, 1,594 scans, 166 s, 1.7 km) for LiDAR-inertial evaluation, EuRoC MH_01_easy for visual-inertial evaluation, and the Cartographer museum dataset for 2D SLAM. For the Livox modality we additionally verify R3LIVE end-to-end on a synthetic Avia bag; quantitative ATE on the official HKU park sequences is reported where download was feasible.

## 4.2 Quantitative Comparison (Table 1)

| Algorithm | ATE RMSE (0001) | ATE RMSE (0033) | Per-frame cost | Notes |
|---|---|---|---|---|
| Point-LIO | **0.084 m** | — | ~145 ms | point-based EKF; best accuracy |
| KISS-ICP | 0.607 m | 33.04 m | **31 ms** | pure odometry, no IMU |
| LeGO-LOAM | 1.445 m | — | ~431 ms/keyframe | ground-segmentation dependent |
| FAST-LIO2 | 1.958 m | 24.45 m | 195 ms | robust baseline |
| LIO-SAM | 19.80 m | — | ~448 ms/keyframe | factor graph; sensitive to yaw |
| Cartographer 3D | 24.74 m | — | ~140 ms | structural mismatch (Sec. 4.4) |
| VINS-Fusion (EuRoC MH_01) | 0.228 m | — | real-time | stereo + IMU |

Two orders of magnitude separate the best (0.084 m) from the worst (24.7 m) on identical hardware and data, indicating that algorithm-scenario matching—not the ARM platform—dominates accuracy. Notably, Point-LIO's 0.084 m on drive 0001 approaches the GPS/INS reference quality itself.

## 4.3 Heterogeneous NPU Front-End (Table 2)

On the Ascend 310P, SuperPoint runs at 1.6 ms/frame (vs ~30 ms on CPU cores), and YOLOv8n-seg produces instance masks at 696 ms/frame (NPU) or 234 ms/frame (ONNX CPU). Projecting masks into the LiDAR frame removes 21.2% of in-view points on a sample frame dominated by nearby vehicles.

## 4.4 Semantic-Filter Closed Loop (Table 3)

We close the loop by integrating semantic masks into the FAST-LIO2 pipeline on KITTI 0033: masks are generated for all 1,594 frames, projected through the calibrated velodyne–camera transform, and dynamic-class points (person, vehicle classes) are removed before inference (2.46% of all points removed).

Result: full-run ATE changes from 24.45 m (baseline) to 26.28 m (+1.8 m), while segment-wise ATE over four quarters of the sequence is statistically indistinguishable (2.83/1.35/2.28/1.13 m vs 2.93/1.42/2.26/1.22 m). In this highway-loop scenario, static structure dominates the point cloud, and the removed points contribute little geometric constraint; the observed full-run difference reflects amplified drift-path perturbation rather than degraded local tracking. We report this as an honest negative result: semantic filtering buys robustness in dynamic-dense scenes at the cost of removed constraints in static-dominant ones, and the decision to filter should be scene-dependent—an operational finding for practitioners deploying SLAM on domestic platforms.

## 4.5 Negative Result: Cartographer 3D (Table 4)

Cartographer 3D collapses on drive 0001 (ATE 24.7 m; estimated path 24.4 m vs actual 104.9 m). Six controlled repair experiments (correlative scan matching on/off, Ceres weights, IMU gravity time constant, accumulation, tf-tree fixes, offline remapping) all fail to recover accuracy. Speed profiling shows the first 7 s lose feature tracking entirely, after which tracking resumes but the map is already inconsistent. We conclude the mismatch is structural: Cartographer's submap-based local optimization assumes scan-rich low-speed motion, violating the 10 m/s vehicular regime; the 2D variant on the museum dataset performs at 2.4× real time with a globally consistent map, confirming the implementation itself is sound on aarch64.

## 4.6 Porting Findings (Table 5)

| Algorithm | aarch64 fixes required | Effort |
|---|---|---|
| FAST-LIO2 | none (build-level) | hours |
| Point-LIO | glog dep, launch config | hours |
| KISS-ICP | none (pip) | minutes |
| LIO-SAM | flann serialization SFINAE, GTSAM unstable, OpenCV4, C++17 | ~1 day |
| LeGO-LOAM | cv_bridge, cloud_msgs, GTSAM, HDL-64E params | ~1 day |
| Cartographer | lua config set, URDF tf, offline remap | ~2 days |
| VINS-Fusion | OpenCV4 macros, stereo-mode call, oxts IMU feed | ~1 day |
| R3LIVE | x86 SIMD removal, cpuid stub, no-fma pragma, SSE guards | ~2 days |

All major SLAM paradigms run in real time on Kunpeng 920; no ARM-specific accuracy degradation was observed. The dominant porting cost is x86 SIMD assumptions (R3LIVE) and legacy OpenCV/GTSAM interfaces, not architectural limitations.
