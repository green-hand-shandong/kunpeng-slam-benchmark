# Abstract (Draft)

The rapid development of LiDAR-inertial and visual-inertial SLAM algorithms has primarily been evaluated on x86 platforms, leaving a significant gap in understanding how these systems perform on emerging domestic ARM-based computing platforms. This paper presents a systematic evaluation of six state-of-the-art SLAM algorithms—FAST-LIO2, Point-LIO, KISS-ICP, LeGO-LOAM, LIO-SAM, and Cartographer—on the Kunpeng 920, a 96-core ARMv8.2 server processor, alongside heterogeneous NPU acceleration on the Ascend 310P for learning-based front-ends.

We establish a unified evaluation pipeline comprising a data adaptation layer (converting raw sensor data to ROS-compatible formats with synthesized angular velocity for under-actuated IMUs), a standardized trajectory evaluation framework (Umeyama-aligned ATE/RPE), and a reproducible experiment management system. All algorithms are evaluated on three datasets spanning indoor and outdoor scenarios, with results cross-validated using both a custom evaluator and the community-standard evo toolkit.

Our results demonstrate that all major SLAM paradigms—filter-based LIO, factor-graph optimization, and learning-assisted approaches—run in real-time on Kunpeng 920 without ARM-specific performance degradation. Quantitative ATE results span two orders of magnitude (0.084m to 24.7m), revealing that algorithm-scenario matching, rather than computational platform, is the dominant factor in SLAM accuracy. Notably, we provide a rigorous negative-result analysis for Cartographer 3D through six controlled repair experiments, demonstrating that its poor performance in high-speed vehicular scenarios is structural rather than parameter-dependent. We further validate the feasibility of a heterogeneous "Kunpeng CPU geometry + Ascend NPU semantics" architecture, achieving 1.6ms/frame SuperPoint inference and demonstrating semantic mask-based LiDAR point cloud filtering.

All Docker images, configuration files, evaluation scripts, and results are publicly available.

# 1. Introduction (Draft)

Simultaneous Localization and Mapping (SLAM) has become a foundational technology for autonomous vehicles, mobile robots, and augmented reality systems. The SLAM research community has produced a rich ecosystem of open-source algorithms spanning multiple sensor modalities: LiDAR-inertial odometry (FAST-LIO2 [1], Point-LIO [2], LIO-SAM [3]), visual-inertial systems (VINS-Fusion [4]), graph-based approaches (Cartographer [5]), and lightweight geometric methods (KISS-ICP [6], LeGO-LOAM [7]).

However, the vast majority of SLAM research and evaluation has been conducted on x86-64 platforms. As domestic ARM-based processors—particularly the Huawei Kunpeng 920 series—gain adoption in server and edge computing deployments within China, a critical question arises: **can existing SLAM algorithms run effectively on these platforms, and if so, which algorithms are best suited for which scenarios?**

Existing evaluations of SLAM on ARM platforms are limited in scope. [8] evaluated a single algorithm (ORB-SLAM3) on a single platform (Raspberry Pi 5) with a single dataset. Other works focus on embedded microcontrollers rather than server-class ARM processors. To the best of our knowledge, no prior work provides a systematic, multi-algorithm, multi-modality evaluation of SLAM on a domestic ARM server platform.

This paper addresses this gap through the following contributions:

1. **Systematic evaluation**: We adapt and evaluate six representative SLAM algorithms across three sensor modalities (2D LiDAR, 3D LiDAR+IMU, and stereo visual-inertial) on the Kunpeng 920, using unified evaluation metrics and cross-validated with both a custom evaluator and the community-standard evo toolkit.

2. **Heterogeneous NPU validation**: We demonstrate that learning-based SLAM front-ends (SuperPoint feature extraction, YOLOv8 semantic segmentation) can be offloaded to the Ascend 310P NPU, achieving 1.6ms/frame inference—validating a "Kunpeng CPU geometry + Ascend NPU semantics" heterogeneous architecture.

3. **Negative-result analysis**: We provide a rigorous analysis of Cartographer 3D's structural incompatibility with high-speed vehicular scenarios through six controlled repair experiments, offering insights into algorithm-scenario matching that are often absent from papers reporting only positive results.

4. **Open-source reproducibility**: All Docker images, configuration files, evaluation scripts, and data acquisition guides are publicly available, enabling direct reproduction on any aarch64 platform.

The remainder of this paper is organized as follows. Section 2 reviews related work. Section 3 describes our evaluation methodology, including the data adaptation layer and unified metrics. Section 4 presents experimental results across all five sensor modalities. Section 5 provides analysis and discussion, including negative results. Section 6 concludes with recommendations for algorithm selection on domestic computing platforms.
