#!/usr/bin/env python3
"""FAST-LIO 地图 PCD + 轨迹 CSV → 效果图（鸟瞰/侧视/轨迹叠加/局部细节）
用法: python3 render_map.py [pcd路径] [轨迹csv路径] [输出目录]
"""
import sys
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

pcd_path = sys.argv[1] if len(sys.argv) > 1 else '/data2/gxw/slam/ws/src/FAST_LIO/PCD/scans.pcd'
traj_csv = sys.argv[2] if len(sys.argv) > 2 else '/data0/slam-datasets/kitti/odom_0001_traj.csv'
out_dir = sys.argv[3] if len(sys.argv) > 3 else '/data2/gxw/slam/figures'
TITLE = sys.argv[4] if len(sys.argv) > 4 else 'FAST-LIO map, top view (KITTI 0001)'
os.makedirs(out_dir, exist_ok=True)


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            key = line.split()[0]
            if key in ('FIELDS', 'SIZE', 'TYPE', 'COUNT'):
                header[key] = line.split()[1:]
            elif key == 'POINTS':
                header['POINTS'] = int(line.split()[1])
            elif key == 'DATA':
                break
        dt = np.dtype([(n, t.lower() + s) for n, t, s in
                       zip(header['FIELDS'], header['TYPE'], header['SIZE'])])
        data = np.fromfile(f, dtype=dt, count=header['POINTS'])
    return data, header['POINTS']


pts, n_total = read_pcd(pcd_path)
x, y, z = pts['x'].astype(np.float64), pts['y'].astype(np.float64), pts['z'].astype(np.float64)
inten = pts['intensity'].astype(np.float64) if 'intensity' in pts.dtype.names else None
valid = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
x, y, z = x[valid], y[valid], z[valid]
if inten is not None:
    inten = inten[valid]

# 采样，控制绘图点数
rng = np.random.default_rng(0)
idx = rng.choice(len(x), size=min(500_000, len(x)), replace=False)
xs, ys, zs = x[idx], y[idx], z[idx]
i_s = inten[idx] if inten is not None else None

traj = np.genfromtxt(traj_csv, delimiter=',', names=True) if os.path.exists(traj_csv) else None

# ---------- 图1: 鸟瞰图（按高度着色, 深色底） ----------
fig, ax = plt.subplots(figsize=(11, 9), facecolor='#0d1117')
ax.set_facecolor('#0d1117')
sc = ax.scatter(xs, ys, c=zs, cmap='turbo', s=0.25, lw=0)
cb = fig.colorbar(sc, ax=ax, shrink=0.85, pad=0.02)
cb.set_label('height z (m)', color='w')
cb.ax.tick_params(colors='w')
ax.set_aspect('equal')
ax.set_title(f'{TITLE}  {len(x)/1e6:.1f}M pts',
             color='w', fontsize=12)
ax.set_xlabel('x (m)', color='w'); ax.set_ylabel('y (m)', color='w')
ax.tick_params(colors='w')
for s in ax.spines.values():
    s.set_color('#555')
fig.tight_layout()
f1 = os.path.join(out_dir, '1_birdseye_height.png')
fig.savefig(f1, dpi=140, facecolor=fig.get_facecolor())
plt.close(fig)

# ---------- 图2: 侧视图（x-z 剖面） ----------
fig, ax = plt.subplots(figsize=(12, 3.6), facecolor='#0d1117')
ax.set_facecolor('#0d1117')
sc = ax.scatter(xs, zs, c=ys, cmap='viridis', s=0.2, lw=0)
cb = fig.colorbar(sc, ax=ax, shrink=0.9, pad=0.01)
cb.set_label('y (m)', color='w')
cb.ax.tick_params(colors='w')
ax.set_title('Side view (x-z profile: ground / facades / tree tops)', color='w', fontsize=12)
ax.set_xlabel('x (m)', color='w'); ax.set_ylabel('z (m)', color='w')
ax.tick_params(colors='w')
for s in ax.spines.values():
    s.set_color('#555')
fig.tight_layout()
f2 = os.path.join(out_dir, '2_sideview.png')
fig.savefig(f2, dpi=140, facecolor=fig.get_facecolor())
plt.close(fig)

# ---------- 图3: 地图(灰) + 轨迹(红) ----------
fig, ax = plt.subplots(figsize=(11, 9))
ax.scatter(xs, ys, c='0.75', s=0.2, lw=0, label='point cloud map')
if traj is not None:
    ax.plot(traj['x'], traj['y'], '-', color='#d62728', lw=2.2, label='FAST-LIO trajectory')
    ax.plot(traj['x'][0], traj['y'][0], 'o', color='#2ca02c', ms=11,
            markeredgecolor='k', zorder=5, label='start')
    ax.plot(traj['x'][-1], traj['y'][-1], '*', color='#ff7f0e', ms=17,
            markeredgecolor='k', zorder=5, label='end')
ax.set_aspect('equal')
ax.set_title(f'{TITLE} + estimated trajectory')
ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
ax.legend(loc='best')
ax.grid(alpha=0.25)
fig.tight_layout()
f3 = os.path.join(out_dir, '3_map_with_trajectory.png')
fig.savefig(f3, dpi=140)
plt.close(fig)

# ---------- 图4: 局部放大（结构细节） ----------
cx, cy = np.median(xs), np.median(ys)
half = 45
m = (np.abs(xs - cx) < half) & (np.abs(ys - cy) < half)
fig, ax = plt.subplots(figsize=(9, 9), facecolor='#0d1117')
ax.set_facecolor('#0d1117')
ax.scatter(xs[m], ys[m], c=zs[m], cmap='turbo', s=0.6, lw=0)
if traj is not None:
    tm = (np.abs(traj['x'] - cx) < half * 1.2) & (np.abs(traj['y'] - cy) < half * 1.2)
    ax.plot(traj['x'][tm], traj['y'][tm], '-', color='w', lw=2, alpha=0.9)
ax.set_aspect('equal')
ax.set_title(f'Zoom-in ({cx:.0f},{cy:.0f} ±{half} m): facades / tree crowns / poles',
             color='w', fontsize=12)
ax.set_xlabel('x (m)', color='w'); ax.set_ylabel('y (m)', color='w')
ax.tick_params(colors='w')
for s in ax.spines.values():
    s.set_color('#555')
fig.tight_layout()
f4 = os.path.join(out_dir, '4_zoom_detail.png')
fig.savefig(f4, dpi=150, facecolor=fig.get_facecolor())
plt.close(fig)

for f in (f1, f2, f3, f4):
    print(f, os.path.getsize(f) // 1024, 'KB')
