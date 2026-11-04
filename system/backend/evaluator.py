"""轨迹评估模块（自研，仅依赖 numpy）
口径：参考轨迹为 KITTI OXTS GPS/INS（raw 序列无官方 odometry 真值时使用），
指标含义为"估计轨迹相对 GPS/INS 参考的误差"，ATE 经 SE(3) Umeyama 对齐。
"""
import numpy as np


def load_track_csv(path):
    a = np.genfromtxt(path, delimiter=',', names=True)
    t = np.atleast_1d(a['t']).astype(float)
    p = np.stack([np.atleast_1d(a['x']), np.atleast_1d(a['y']), np.atleast_1d(a['z'])], 1)
    return t, p


def match_by_time(t_est, p_est, t_ref, p_ref, max_diff=0.05):
    """最近邻时间关联。返回 (t, p_est, p_ref, dt)，仅保留 |dt|<=max_diff 的配对。"""
    order = np.argsort(t_ref)
    t_ref, p_ref = t_ref[order], p_ref[order]
    idx = np.clip(np.searchsorted(t_ref, t_est), 1, len(t_ref) - 1)
    left = np.abs(t_ref[idx - 1] - t_est) < np.abs(t_ref[idx] - t_est)
    idx = np.where(left, idx - 1, idx)
    dt = np.abs(t_ref[idx] - t_est)
    m = dt <= max_diff
    return t_est[m], p_est[m], p_ref[idx[m]], dt[m]


def umeyama(src, dst):
    """求 R,t 使 dst ≈ R@src + t（闭式解，含反射修正）。src/dst: (N,3)"""
    assert len(src) >= 3
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (dst - mu_d).T @ (src - mu_s) / len(src)
    U, _, Vt = np.linalg.svd(H)
    S = np.eye(3)
    S[2, 2] = np.sign(np.linalg.det(U @ Vt)) or 1.0
    R = U @ S @ Vt
    return R, mu_d - R @ mu_s


def ate(est_t, est_p, ref_t, ref_p, max_diff=0.05):
    """绝对轨迹误差：对齐后逐点欧氏误差。返回 指标dict, (误差序列, 时间)"""
    t, pe, pr, _ = match_by_time(est_t, est_p, ref_t, ref_p, max_diff)
    if len(t) < 3:
        return {'error': 'too few matched pairs: %d' % len(t)}, None, None
    R, tr = umeyama(pe, pr)
    aligned = pe @ R.T + tr
    err = np.linalg.norm(aligned - pr, axis=1)
    metrics = {'ate_rmse': round(float(np.sqrt((err ** 2).mean())), 4),
               'ate_mean': round(float(err.mean()), 4),
               'ate_max': round(float(err.max()), 4),
               'n_pairs': int(len(err)),
               'alignment_R': R.round(5).tolist(), 'alignment_t': tr.round(4).tolist()}
    return metrics, (err.tolist(), t.tolist()), (R, tr)


def rpe(est_t, est_p, ref_t, ref_p, delta=1.0, tol=0.15, max_diff=0.05):
    """相对位姿误差（平移分量）：间隔≈delta 秒的位移向量差。
    估计轨迹先做整体 Umeyama 对齐到参考系。"""
    t, pe, pr, _ = match_by_time(est_t, est_p, ref_t, ref_p, max_diff)
    if len(t) < 10:
        return {'error': 'too few matched pairs: %d' % len(t)}
    R, tr = umeyama(pe, pr)
    al = pe @ R.T + tr
    j = np.searchsorted(t, t + delta)
    ok = (j < len(t)) & (np.abs(t[np.minimum(j, len(t) - 1)] - t - delta) <= tol)
    j = j[ok]
    i = np.arange(len(t))[ok]
    if len(i) < 5:
        return {'error': 'no pairs at delta=%.1fs' % delta}
    d_est = al[j] - al[i]
    d_ref = pr[j] - pr[i]
    err = np.linalg.norm(d_est - d_ref, axis=1)
    return {'rpe_rmse': round(float(np.sqrt((err ** 2).mean())), 4),
            'rpe_mean': round(float(err.mean()), 4),
            'rpe_max': round(float(err.max()), 4),
            'delta_s': delta, 'n_pairs': int(len(err))}


def evaluate(est_csv, ref_csv, deltas=(0.5, 1.0, 2.0), max_diff=0.05):
    """完整评估入口：返回指标 dict + ATE 误差序列（供前端画曲线）"""
    te, pe = load_track_csv(est_csv)
    tr, pr = load_track_csv(ref_csv)
    m, series, _ = ate(te, pe, tr, pr, max_diff)
    out = {'ate': m, 'rpe': [rpe(te, pe, tr, pr, d, max_diff=max_diff) for d in deltas]}
    if series:
        out['ate_series'] = {'err': [round(v, 4) for v in series[0]],
                             't': [round(v, 2) for v in series[1]]}
    return out
