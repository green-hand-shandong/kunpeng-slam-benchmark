#!/usr/bin/env python3
# 语义外挂 demo：YOLOv8n-seg 实例掩码 × 激光点云投影 → 动态/车辆点云滤除
# 数据：KITTI 0033 第 500 帧（激光 bin + 图像 PNG），外参 calib_velo_to_cam + calib_cam_to_cam
# 运行：在 yolo26_train 容器（ultralytics + torch_npu）
import sys, math
import numpy as np
import cv2
from ultralytics import YOLO

SEQ = '/data0/kitti/2011_09_30/2011_09_30_drive_0033_sync'
FRAME = 500

def read_kitti_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)  # x y z intensity

def read_calib():
    # velo->cam0: R|T from calib_velo_to_cam.txt
    def parse(fn, key):
        for line in open(fn):
            if line.startswith(key):
                return np.array(list(map(float, line.split(':')[1].split())))
        raise KeyError(key)
    base = '/data0/kitti/2011_09_30'
    Rv = parse(base + '/calib_velo_to_cam.txt', 'R:').reshape(3, 3)
    Tv = parse(base + '/calib_velo_to_cam.txt', 'T:').reshape(3, 1)
    # cam->cam0 rect from calib_cam_to_cam.txt (R0_1)
    R0 = parse(base + '/calib_cam_to_cam.txt', 'R0_1:').reshape(3, 3)
    # P_rect_00 from calib_cam_to_cam.txt
    P = parse(base + '/calib_cam_to_cam.txt', 'P_rect_00:').reshape(3, 4)
    return Rv, Tv, R0, P

def main():
    import io
    _buf = []
    Rv, Tv, R0, P = read_calib()
    # K = P[:, :3]（rect cam0 内参）
    K = P[:, :3]

    # 1. 语义分割（NPU）
    m = YOLO('yolov8n-seg.pt')
    img_path = f'{SEQ}/image_00/data/{FRAME:010d}.png'
    r = m.predict(img_path, conf=0.2, imgsz=1280, verbose=False)[0]
    n_det = len(r.boxes) if r.boxes else 0
    print(f'detections: {n_det}')
    if r.masks is None or n_det == 0:
        print('no masks this frame'); return

    # 合成动态物体掩码（car 类），0.5 分辨率对齐 1280 宽
    H, W = r.orig_shape
    mask_dyn = np.zeros((H, W), dtype=bool)
    for i, b in enumerate(r.boxes):
        if r.names[int(b.cls.cpu().item())] in ('car', 'person', 'truck', 'bus'):
            msk = r.masks.data[i].cpu().numpy()
            msk = cv2.resize(msk, (W, H)) > 0.5
            mask_dyn |= msk
    print(f'dynamic mask coverage: {mask_dyn.mean()*100:.1f}% of image')

    # 2. 激光点云 → 相机投影
    pts = read_kitti_bin(f'{SEQ}/velodyne_points/data/{FRAME:010d}.bin')
    xyz = pts[:, :3]
    # velo -> cam（R|T），再 rect（R0）
    pts_cam = (R0 @ (Rv @ xyz.T + Tv)).T
    # 前向、深度为正
    valid = pts_cam[:, 2] > 1.0
    pc = pts_cam[valid]
    # 投影：u = fx*x/z + cx, v = fy*y/z + cy（P_rect 第三列偏移=0 for cam0）
    uv = (K @ pc.T).T
    u = uv[:, 0] / uv[:, 2]
    v = uv[:, 1] / uv[:, 2]
    depth = pc[:, 2]
    in_img = (u >= 0) & (u < W) & (v >= 0) & (v < H) & (depth > 1) & (depth < 80)
    print(f'lidar points: {len(xyz)}, projected in-view: {in_img.sum()}')

    # 3. 掩码滤除
    idx = np.where(in_img)[0]
    u_i = u[idx].astype(int); v_i = v[idx].astype(int)
    dyn_flags = mask_dyn[v_i, u_i]
    n_dyn = dyn_flags.sum()
    print(f'points inside dynamic masks: {n_dyn} ({n_dyn/len(idx)*100:.1f}% of in-view)')
    # 深度一致性交叉验证：掩码内激光深度 vs 分割框隐含距离
    d_in = depth[idx][dyn_flags]
    if len(d_in):
        print(f'dynamic-point depth: median {np.median(d_in):.1f} m, range [{d_in.min():.1f}, {d_in.max():.1f}]')
    with open('/tmp/sem_demo_result.txt', 'w') as f:
        f.write(f'detections: {n_det}\n')
        f.write(f'dynamic mask coverage: {mask_dyn.mean()*100:.1f}%\n')
        f.write(f'lidar points: {len(xyz)}, in-view: {int(in_img.sum())}\n')
        f.write(f'dynamic points removable: {n_dyn} ({n_dyn/len(idx)*100:.1f}% of in-view)\n')
        if len(d_in):
            f.write(f'dynamic depth: median {np.median(d_in):.1f} m\n')

if __name__ == '__main__':
    main()
