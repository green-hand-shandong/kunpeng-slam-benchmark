#!/usr/bin/env python3
# P0-4: 统一口径报告生成器
# 每个算法: 处理扫描数/完成率/覆盖时长/吞吐 + ATE(各自窗口) + ATE(公共窗口裁剪)
# 数据源: 各 run 的带时间戳轨迹 + GT + bag 帧数
import sys, csv, json
sys.path.insert(0, '/data2/gxw/slam/system/backend')
from evaluator import ate
import numpy as np

K = '/data0/slam-datasets/kitti'

def load_tum(p):
    d = np.loadtxt(p)
    return np.column_stack([d[:,0], d[:,1:4]])

def load_csv4(p):
    rows = list(csv.reader(open(p)))
    if rows[0][0] == 't': rows = rows[1:]
    return np.array([[float(x) for x in r[:4]] for r in rows])

# 0001: 总输入 108 帧 / 11.65s; 0033: 1594 帧 / 166.0s
TOTAL = {'0001': (108, 11.65), '0033': (1594, 166.0)}

def eval_run(name, traj, gt, dataset, note=''):
    n = len(traj)
    cov_s = traj[-1,0] - traj[0,0]
    total_frames, total_s = TOTAL[dataset]
    a_full = ate(traj[:,0], traj[:,1:4], gt[:,0], gt[:,1:4])
    return dict(
        run=name, dataset=dataset, frames=n, frame_rate=round(n/total_frames*100,1),
        coverage_s=round(cov_s,2), coverage_pct=round(cov_s/total_s*100,1),
        throughput_hz=round(n/max(cov_s,1e-9),2),
        ate_full=round(a_full[0]['ate_rmse'],3) if 'ate_rmse' in a_full[0] else None,
        note=note)

def eval_common_window(runs, gt, window_s):
    """裁到公共前 window_s 秒再算 ATE"""
    out = {}
    for name, traj in runs.items():
        t0 = traj[0,0]
        seg = traj[traj[:,0] <= t0 + window_s]
        a = ate(seg[:,0], seg[:,1:4], gt[:,0], gt[:,1:4])
        out[name] = round(a[0]['ate_rmse'],3) if 'ate_rmse' in a[0] else None
    return out

if __name__ == '__main__':
    gt1 = load_tum(f'{K}/tum_gt0001.txt')
    gt3 = load_csv4(f'{K}/kitti_0033_ref.csv')

    report = {}
    # --- 0001 (输入 108 帧/11.65s) ---
    r0001 = {}
    r0001['Point-LIO'] = load_tum(f'{K}/tum_pointlio.txt')
    r0001['FAST-LIO2'] = load_tum(f'{K}/tum_fastlio.txt')
    r0001['KISS-ICP'] = load_tum(f'{K}/tum_kissicp.txt')
    r0001['LeGO-LOAM'] = load_tum(f'{K}/tum_lego.txt')
    r0001['Cartographer3D'] = load_tum(f'{K}/tum_carto.txt')
    r0001['LIO-SAM(外参修复)'] = load_csv4(f'{K}/lsam_extfix_traj.csv')
    for name, tr in r0001.items():
        report[f'0001|{name}'] = eval_run(name, tr, gt1, '0001')
    # 0001 公共窗口 = 最短覆盖
    win1 = min(r['coverage_s'] for r in [report[f'0001|{n}'] for n in r0001])
    cw1 = eval_common_window(r0001, gt1, win1)
    for k, v in cw1.items():
        report[f'0001|{k}']['ate_common_window'] = v
        report[f'0001|{k}']['common_window_s'] = win1

    # --- 0033 (输入 1594 帧/166s): FAST-LIO2 半速(完整) vs 1x(部分) vs 语义滤除 ---
    r0033 = {}
    r0033['FAST-LIO2(0.5x)'] = load_csv4('/data0/slam-datasets/results/runs/c798d64b/timed.csv')
    semf = load_csv4(f'{K}/odom_0033_semfiltered_timed.csv')
    r0033['FAST-LIO2+语义滤除(0.5x)'] = semf
    # 1x run: bench 的 odom_0033.bag? 找 1x 轨迹(评审报告里有 4.8325m@77.21s)
    import os
    p1x = f'{K}/odom_0033_timed.csv'  # 这个是 0.5x 的(旧) — 1x 的在哪?
    for name, tr in r0033.items():
        report[f'0033|{name}'] = eval_run(name, tr, gt3, '0033')

    # 0033 公共窗口: 半速完整 161.99s vs 语义滤除 162.92s → 裁到 161.99
    win3 = min(report[f'0033|{n}']['coverage_s'] for n in r0033)
    cw3 = eval_common_window(r0033, gt3, win3)
    for k, v in cw3.items():
        report[f'0033|{k}']['ate_common_window'] = v
        report[f'0033|{k}']['common_window_s'] = win3

    with open('/data0/slam-datasets/results/unified_report.json', 'w') as f:
        json.dump(report, f, indent=1, ensure_ascii=False)
    print(json.dumps(report, indent=1, ensure_ascii=False))
