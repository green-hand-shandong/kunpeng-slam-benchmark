#!/usr/bin/env python3
"""KITTI raw drive -> ROS1 rosbag（供 FAST-LIO 使用）

  /velodyne_points : PointCloud2, 字段 x/y/z/intensity/ring(u16)/time(f32, 秒)
                     ring  由垂直角量化（HDL-64E: +2.3° ~ -24.9°, 64 线）
                     time  由方位角在整圈中的比例 * (1/scan_rate)
  /kitti/oxts/imu  : Imu, 加速度 af/al/au (m/s²), 角速度 wx/wy/wz (rad/s, 已实测验证),
                     姿态由 oxts 欧拉角(roll,pitch,yaw)转四元数

用法:
  source /opt/ros/noetic/setup.bash
  python3 kitti_to_bag.py <drive_sync_dir> <oxts_100hz_dir> <out.bag> [--scan-rate 10]
  例: python3 kitti_to_bag.py \
      /data0/slam-datasets/kitti/2011_09_26/2011_09_26_drive_0001_sync \
      /data0/slam-datasets/kitti/2011_09_26/2011_09_26_drive_0001_extract/oxts \
      /data0/slam-datasets/kitti/kitti_0001.bag
"""
import argparse
import glob
import math
import os
from datetime import datetime

import numpy as np
import rosbag
from sensor_msgs.msg import Imu, PointCloud2, PointField

V_MAX_DEG, V_MIN_DEG = 2.3, -24.9  # HDL-64E 垂直视场角范围


def parse_ts(line):
    """KITTI 时间戳(UTC, 纳精度) -> genpy.Time"""
    import genpy
    date, frac = line.strip().split('.')
    dt = datetime.strptime(date + '.' + frac[:6], '%Y-%m-%d %H:%M:%S.%f')
    secs = int((dt - datetime(1970, 1, 1)).total_seconds())
    return genpy.Time(secs, dt.microsecond * 1000)


def quat_from_euler(r, p, y):
    """R = Rz(yaw)Ry(pitch)Rx(roll) -> (x,y,z,w)"""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def velodyne_msg(bin_path, stamp, scan_rate):
    pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    v = np.degrees(np.arctan2(z, np.hypot(x, y)))
    ring = np.clip(np.round((V_MAX_DEG - v) / (V_MAX_DEG - V_MIN_DEG) * 63), 0, 63).astype(np.uint16)
    yaw = np.arctan2(y, x)
    t = (np.where(yaw < 0, yaw + 2 * np.pi, yaw) / (2 * np.pi)) / scan_rate

    rec = np.empty(len(pts), dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'),
                                    ('intensity', '<f4'), ('ring', '<u2'),
                                    ('pad', '<u2'), ('time', '<f4')])
    rec['x'], rec['y'], rec['z'] = x, y, z
    rec['intensity'] = pts[:, 3]
    rec['ring'], rec['time'] = ring, t.astype(np.float32)

    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = 'velodyne'
    msg.height, msg.width = 1, len(pts)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('intensity', 12, PointField.FLOAT32, 1),
        PointField('ring', 16, PointField.UINT16, 1),
        PointField('time', 20, PointField.FLOAT32, 1),
    ]
    msg.point_step = 24
    msg.row_step = 24 * len(pts)
    msg.is_dense = True
    msg.data = rec.tobytes()
    return msg


def imu_msg(values, stamp):
    f = values
    m = Imu()
    m.header.stamp = stamp
    m.header.frame_id = 'imu'
    qx, qy, qz, qw = quat_from_euler(f[3], f[4], f[5])
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = qx, qy, qz, qw
    # KITTI oxts 列实测结论（见 slam/06_跑通记录.md）:
    #   [3:6] roll/pitch/yaw (rad)  [14:17] af/al/au 车体系加速度 (m/s², au≈+9.8 ✓)
    #   [11:14] 是世界系 NED 加速度 an/ae/ad, 不是角速度! oxts 包里没有陀螺仪字段,
    #   角速度由 100Hz RPY 差分+平滑合成 (见 main 里的 gyro 序列, 通过 values[20:23] 传入)
    m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = f[14], f[15], f[16]
    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = f[20], f[21], f[22]
    for arr in (m.orientation_covariance, m.angular_velocity_covariance,
                m.linear_acceleration_covariance):
        arr[0] = arr[4] = arr[8] = 1e-2
    return m


def build_gyro(rows):
    """KITTI oxts 无陀螺仪字段, 由 100Hz RPY 差分合成角速度 (rad/s), 50ms 滑动平均去噪。
    车体系取 x前/y左/z上: wx=d(roll), wy=-d(pitch), wz=d(yaw)。rows: (N,30) oxts 数组。"""
    dt = 0.01
    roll, pitch, yaw = rows[:, 3], rows[:, 4], np.unwrap(rows[:, 5])

    def rate(x):
        k = np.ones(5) / 5.0
        return np.convolve(np.gradient(x, dt), k, mode='same')

    return np.stack([rate(roll), -rate(pitch), rate(yaw)], axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sync_dir', help='.../2011_09_26_drive_XXXX_sync')
    ap.add_argument('oxts_dir', help='100Hz oxts 目录（extract 包解出，含 data/ 与 timestamps.txt）')
    ap.add_argument('out_bag')
    ap.add_argument('--scan-rate', type=float, default=10.0)
    ap.add_argument('--velodyne-subdir', default='data', help='velodyne_points 子目录(data 或 data_semfiltered)')
    ap.add_argument('--topic-lidar', default='/velodyne_points')
    ap.add_argument('--topic-imu', default='/kitti/oxts/imu')
    args = ap.parse_args()

    bins = sorted(glob.glob(os.path.join(args.sync_dir, f'velodyne_points/{args.velodyne_subdir}/*.bin')))
    assert bins, 'no velodyne bins found'
    velo_ts = open(os.path.join(args.sync_dir, 'velodyne_points/timestamps.txt')).read().splitlines()
    assert len(bins) == len(velo_ts), f'{len(bins)} bins vs {len(velo_ts)} timestamps'

    ox_data = sorted(glob.glob(os.path.join(args.oxts_dir, 'data/*.txt')))
    assert ox_data, 'no oxts files found'
    ox_ts = open(os.path.join(args.oxts_dir, 'timestamps.txt')).read().splitlines()
    ox_rows = []
    for f in ox_data:
        for line in open(f):
            if line.strip():
                ox_rows.append([float(v) for v in line.split()])
    assert len(ox_rows) == len(ox_ts), f'{len(ox_rows)} oxts rows vs {len(ox_ts)} timestamps'
    ox_rows = np.array(ox_rows)
    gyro = build_gyro(ox_rows)

    # GPS/INS 参考轨迹（仅用于评估，不进入建图）：经纬度 -> 局部 ENU 平面近似
    lat0, lon0 = ox_rows[0, 0], ox_rows[0, 1]
    m_per_deg = 111319.49
    east = (ox_rows[:, 1] - lon0) * np.cos(np.radians(lat0)) * m_per_deg
    north = (ox_rows[:, 0] - lat0) * m_per_deg
    up = ox_rows[:, 2] - ox_rows[0, 2]
    ref_path = os.path.splitext(args.out_bag)[0] + '_ref.csv'
    with open(ref_path, 'w') as f:
        f.write('t,x,y,z\n')
        for i, ts in enumerate(ox_ts):
            tt = parse_ts(ts)
            t = tt.secs + tt.nsecs * 1e-9
            f.write(f'{t:.3f},{east[i]:.3f},{north[i]:.3f},{up[i]:.3f}\n')
    print('reference trajectory:', ref_path)

    with rosbag.Bag(args.out_bag, 'w') as bag:
        for i, ts in enumerate(ox_ts):
            stamp = parse_ts(ts)
            bag.write(args.topic_imu,
                      imu_msg(list(ox_rows[i]) + list(gyro[i]), stamp), stamp)
        for i, (b, ts) in enumerate(zip(bins, velo_ts)):
            stamp = parse_ts(ts)
            bag.write(args.topic_lidar, velodyne_msg(b, stamp, args.scan_rate), stamp)
            if i % 20 == 0:
                print(f'  velodyne {i + 1}/{len(bins)}')
    print('done:', args.out_bag, f'{os.path.getsize(args.out_bag) / 1e6:.1f} MB')


if __name__ == '__main__':
    main()
