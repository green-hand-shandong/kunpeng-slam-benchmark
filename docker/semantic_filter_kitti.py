#!/usr/bin/env python3
# P0-1 步骤B: 掩码 × 点云投影 → 滤除动态物体点 → 重打包 kitti_0033 bag
# 输入: KITTI 0033 sync (bin+png) + sem_masks/*.npz + calib
# 输出: /data0/slam-datasets/kitti/kitti_0033_semfiltered.bag (topic: /velodyne_points, /kitti/oxts/imu)
import os, sys, struct
import numpy as np
import cv2

SEQ = '/data0/slam-datasets/kitti/2011_09_30/2011_09_30_drive_0033_sync'
MASKS = '/data2/gxw/yolo26-310p-eval/sem_masks'
OUT_BAG = '/data0/slam-datasets/kitti/kitti_0033_semfiltered.bag'
CALIB = '/data0/slam-datasets/kitti/2011_09_30'

def parse_calib():
    def parse(fn, key):
        for line in open(fn):
            if line.startswith(key):
                return np.array(list(map(float, line.split(':')[1].split())))
        raise KeyError(key)
    Rv = parse(f'{CALIB}/calib_velo_to_cam.txt', 'R:').reshape(3, 3)
    Tv = parse(f'{CALIB}/calib_velo_to_cam.txt', 'T:').reshape(3, 1)
    R0 = parse(f'{CALIB}/calib_cam_to_cam.txt', 'R_rect_00:').reshape(3, 3)  # cam2
    P = parse(f'{CALIB}/calib_cam_to_cam.txt', 'P_rect_00:').reshape(3, 4)
    return Rv, Tv, R0, P

def main():
    Rv, Tv, R0, P = parse_calib()
    K = P[:, :3]
    # velo -> rect cam2: x_c = R0 @ (Rv @ x_v + Tv)
    R0_4 = np.eye(4); R0_4[:3, :3] = R0
    T = R0_4 @ np.vstack([np.hstack([Rv, Tv]), [0, 0, 0, 1]])
    R_c, t_c = T[:3, :3], T[:3, 3]

    bins = sorted(os.listdir(f'{SEQ}/velodyne_points/data'))
    import datetime
    base = datetime.datetime(2011, 9, 30, 0, 0, 0)
    stamps = []
    for l in open(f'{SEQ}/velodyne_points/timestamps.txt'):
        dt = datetime.datetime.strptime(l.strip()[:26], '%Y-%m-%d %H:%M:%S.%f')
        stamps.append((dt - base).total_seconds())
    assert len(stamps) == len(bins), (len(stamps), len(bins))

    stats = dict(total=0, removed=0)
    filtered_bins = f'{SEQ}/velodyne_points/data_semfiltered'
    os.makedirs(filtered_bins, exist_ok=True)
    for i, fn in enumerate(bins):
        outp = f'{filtered_bins}/{fn}'
        if os.path.exists(outp):
            continue
        pts = np.fromfile(f'{SEQ}/velodyne_points/data/{fn}', np.float32).reshape(-1, 4)
        xyz = pts[:, :3]
        # 投影到 cam2
        pc = (R_c @ xyz.T + t_c.reshape(3, 1)).T
        depth = pc[:, 2]
        valid = depth > 1.0
        uv = (K @ pc[valid].T).T
        uv = uv[:, :2] / uv[:, 2:3]
        H, W = 375, 1242
        inb = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H)
        remove_mask = np.zeros(len(pts), bool)
        mp = f'{MASKS}/{fn.replace(".bin", ".npz")}'
        if os.path.exists(mp):
            d = np.load(mp)
            ms = d['masks']
            if ms.shape[0] > 0:
                mh, mw = ms.shape[1:3]
                # 掩码是 letterbox 内容区: 原图(u,v) → 掩码(u-dx, v-dy)
                dxo = (1242 - mw) // 2
                dyo = (375 - mh) // 2
                idx_v = np.where(valid)[0]
                uvi_all = uv[inb].astype(int)
                uvi_all[:, 0] -= dxo
                uvi_all[:, 1] -= dyo
                inm = (uvi_all[:, 0] >= 0) & (uvi_all[:, 0] < mw) & (uvi_all[:, 1] >= 0) & (uvi_all[:, 1] < mh)
                pid_v = idx_v[inb][inm]
                uvi = uvi_all[inm]
                for m in ms:
                    mv = m[uvi[:, 1], uvi[:, 0]] > 0
                    remove_mask[pid_v[mv]] = True
        keep = ~remove_mask
        stats['total'] += len(pts)
        stats['removed'] += remove_mask.sum()
        pts[keep].tofile(outp)
        if (i + 1) % 200 == 0:
            print(f'{i+1}/{len(bins)} removed {stats["removed"]/max(stats["total"],1)*100:.1f}%', flush=True)
    print(f'FILTER DONE: removed {stats["removed"]}/{stats["total"]} = {stats["removed"]/stats["total"]*100:.1f}%', flush=True)

    # 重打包 bag (用 ROS 容器)
    print('bag 打包交给 run_fastlio_semfiltered.sh', flush=True)

if __name__ == '__main__':
    main()
