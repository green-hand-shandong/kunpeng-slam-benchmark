#!/usr/bin/env python3
# P0-1 语义滤除闭环图: 轨迹对比 + 分段 ATE 柱状
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np, csv

plt.rcParams.update({'font.size': 11})

def load_csv(p):
    rows = list(csv.reader(open(p)))
    if rows[0][0] == 't': rows = rows[1:]
    return np.array([[float(x) for x in r[:4]] for r in rows])

gt = load_csv('/data0/slam-datasets/kitti/kitti_0033_ref.csv')
base = load_csv('/data0/slam-datasets/kitti/odom_0033_timed.csv')
semf = load_csv('/data0/slam-datasets/kitti/odom_0033_semfiltered_timed.csv')

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 4.2))

# 左: 轨迹 XY
ax1.plot(gt[:,1], gt[:,2], 'k-', lw=2, label='GPS/INS reference')
ax1.plot(base[:,1], base[:,2], 'b-', lw=1, alpha=0.8, label='FAST-LIO2 (baseline)')
ax1.plot(semf[:,1], semf[:,2], 'r--', lw=1, alpha=0.8, label='FAST-LIO2 + semantic filter')
ax1.set_xlabel('x [m]'); ax1.set_ylabel('y [m]')
ax1.set_title('KITTI 0033 trajectories')
ax1.legend(fontsize=9); ax1.grid(alpha=0.3); ax1.set_aspect('equal')

# 右: 分段 ATE
labels = ['Q1', 'Q2', 'Q3', 'Q4']
b = [2.83, 1.35, 2.28, 1.13]
s = [2.93, 1.42, 2.26, 1.22]
x = np.arange(4); w = 0.35
ax2.bar(x - w/2, b, w, label='Baseline', color='#4878CF')
ax2.bar(x + w/2, s, w, label='Semantic filter', color='#D65F5F')
ax2.set_xticks(x); ax2.set_xticklabels(labels)
ax2.set_ylabel('Segment ATE RMSE [m]')
ax2.set_title('Segment-wise ATE (full-run drift removed)')
ax2.legend(fontsize=9); ax2.grid(alpha=0.3, axis='y')
ax2.text(0.98, 0.95, 'Full-run ATE: 24.45 m vs 26.28 m\nRemoved: 2.46% points (dynamic classes)',
         transform=ax2.transAxes, ha='right', va='top', fontsize=8.5,
         bbox=dict(boxstyle='round', fc='lightyellow', alpha=0.9))

plt.tight_layout()
plt.savefig('/data2/gxw/slam/figures/fig_semantic_closedloop.png', dpi=150)
print('saved fig_semantic_closedloop.png')
