"""三维激光惯性 SLAM 建图与轨迹评估系统 V2 后端
模块: 数据集管理(导入/校验) / 建图任务(容器化 FAST-LIO2, 可停止) / 算法配置 /
      实验库(SQLite) / 轨迹评估(ATE/RPE, evaluator.py) / 系统监控(monitor.py) / 报告导出
启动: uvicorn main:app --host 0.0.0.0 --port 8000
"""
import json
import os
import shutil
import sqlite3
import subprocess
import threading
import time
import uuid
from pathlib import Path

import numpy as np
from fastapi import FastAPI
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

import evaluator
import monitor

DATA = Path("/data0/slam-datasets")
RESULTS = DATA / "results"
RUNS = RESULTS / "runs"
WS = Path("/data2/gxw/slam/ws")
DOCKER_DIR = Path("/data2/gxw/slam/docker")
PCD_LIVE = WS / "src/FAST_LIO/PCD/scans.pcd"
LOG_DIR_LIVE = WS / "src/FAST_LIO/Log"
DB_PATH = RESULTS / "system.db"
FASTLIO_COMMIT = subprocess.run(["git", "-C", str(WS / "src/FAST_LIO"), "log", "--oneline", "-1"],
                                capture_output=True, text=True).stdout.strip()

DEFAULT_CONFIG = {
    "point_filter_num": {"value": 3, "min": 1, "max": 10, "type": "int", "desc": "点云抽稀：每 N 点取 1"},
    "filter_size_surf": {"value": 0.3, "min": 0.05, "max": 2.0, "type": "float", "desc": "特征点体素滤波 (m)"},
    "filter_size_map": {"value": 0.4, "min": 0.05, "max": 2.0, "type": "float", "desc": "地图体素滤波 (m)"},
    "max_iteration": {"value": 4, "min": 1, "max": 10, "type": "int", "desc": "iEKF 最大迭代次数"},
    "blind": {"value": 1.0, "min": 0.0, "max": 5.0, "type": "float", "desc": "近距离盲区剔除 (m)"},
    "acc_cov": {"value": 0.1, "min": 0.001, "max": 10.0, "type": "float", "desc": "加速度计噪声协方差"},
    "gyr_cov": {"value": 0.1, "min": 0.001, "max": 10.0, "type": "float", "desc": "陀螺仪噪声协方差"},
    "play_rate": {"value": 0.5, "min": 0.2, "max": 3.0, "type": "float",
                  "desc": "数据回放速率：默认 0.5 慢放保证全部帧参与建图（1.0 会丢帧，实测仅覆盖 47%）"},
}

app = FastAPI(title="三维激光惯性SLAM建图与轨迹评估系统 V2")
JOBS = {}
JOBS_LOCK = threading.Lock()

DB_LOCK = threading.Lock()


# ---------------- 认证 / 审计 / 安全 ----------------
import hashlib, secrets
from fastapi import Request, HTTPException
from fastapi.responses import PlainTextResponse

def _sha(s): return hashlib.sha256(("slam-salt|" + s).encode()).hexdigest()


def audit(user, action, detail=""):
    with DB_LOCK, db() as c:
        c.execute("INSERT INTO audit(ts,user,action,detail) VALUES(?,?,?,?)", (time.time(), user, action, detail))

def current_user(req: Request):
    sid = req.cookies.get("sid") or (req.headers.get("authorization") or "").replace("Bearer ", "")
    if not sid: return None
    with db() as c:
        r = c.execute("SELECT username, role FROM sessions WHERE token=?", (sid,)).fetchone()
    return dict(r) if r else None

ROLES = {"admin": "管理员", "operator": "操作员", "viewer": "只读用户"}

def require_role(request: Request, allowed):
    user = current_user(request)
    if not user:
        raise HTTPException(status_code=401, detail="未登录")
    if user["role"] not in allowed:
        audit(user["username"], "forbidden", request.url.path)
        raise HTTPException(status_code=403, detail="权限不足（需要 %s）" % "/".join(allowed))
    return user


@app.middleware("http")
async def security_and_auth(request: Request, call_next):
    resp = await call_next(request)
    resp.headers["X-Content-Type-Options"] = "nosniff"
    resp.headers["X-Frame-Options"] = "DENY"
    resp.headers["Referrer-Policy"] = "no-referrer"
    resp.headers["Content-Security-Policy"] = "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self' data:"
    return resp

@app.middleware("http")
async def auth_gate(request: Request, call_next):
    p = request.url.path
    if p.startswith("/api/") and p not in ("/api/auth/login",) and not p.startswith("/api/auth/"):
        user = current_user(request)
        if not user:
            return JSONResponse({"error": "未登录或会话过期"}, status_code=401)
        request.state.user = user["username"]
    return await call_next(request)

@app.post("/api/auth/login")
async def auth_login(request: Request):
    body = await request.json()
    u, p = str(body.get("username","")), str(body.get("password",""))
    with db() as c:
        r = c.execute("SELECT username, role FROM users WHERE username=? AND pwd_hash=?", (u, _sha(p))).fetchone()
    if not r:
        audit(u, "login_failed")
        return JSONResponse({"error": "用户名或密码错误"}, status_code=401)
    token = secrets.token_hex(24)
    with db() as c:
        c.execute("INSERT INTO sessions VALUES(?,?,?,?)", (token, u, r["role"], time.time()))
    audit(u, "login")
    resp = JSONResponse({"user": u, "role": r["role"]})
    resp.set_cookie("sid", token, httponly=True, samesite="strict")
    return resp

@app.post("/api/auth/logout")
async def auth_logout(request: Request):
    sid = request.cookies.get("sid")
    if sid:
        with db() as c: c.execute("DELETE FROM sessions WHERE token=?", (sid,))
    return {"ok": True}

@app.get("/api/auth/me")
def auth_me(request: Request):
    return current_user(request) or JSONResponse({"error": "anon"}, status_code=401)

@app.post("/api/auth/change_password")
async def change_password(request: Request):
    user = require_role(request, list(ROLES))
    body = await request.json()
    old, new = str(body.get("old", "")), str(body.get("new", ""))
    if len(new) < 6:
        return JSONResponse({"error": "新密码至少 6 位"}, status_code=400)
    with db() as c:
        r = c.execute("SELECT 1 FROM users WHERE username=? AND pwd_hash=?", (user["username"], _sha(old))).fetchone()
        if not r:
            return JSONResponse({"error": "原密码错误"}, status_code=400)
        c.execute("UPDATE users SET pwd_hash=? WHERE username=?", (_sha(new), user["username"]))
    audit(user["username"], "pwd_change")
    return {"ok": True}


@app.get("/api/users")
def list_users(request: Request):
    require_role(request, ["admin"])
    with db() as c:
        rows = [{"username": r["username"], "role": r["role"]} for r in c.execute("SELECT username, role FROM users ORDER BY username")]
    return rows


@app.post("/api/users")
async def create_user(request: Request):
    admin = require_role(request, ["admin"])
    body = await request.json()
    u, p, role = str(body.get("username", "")).strip(), str(body.get("password", "")), str(body.get("role", ""))
    if not u or not p or role not in ROLES:
        return JSONResponse({"error": "用户名/密码必填，角色须为 %s" % "/".join(ROLES)}, status_code=400)
    if len(p) < 6:
        return JSONResponse({"error": "密码至少 6 位"}, status_code=400)
    with db() as c:
        if c.execute("SELECT 1 FROM users WHERE username=?", (u,)).fetchone():
            return JSONResponse({"error": "用户已存在"}, status_code=400)
        c.execute("INSERT INTO users VALUES(?,?,?)", (u, _sha(p), role))
    audit(admin["username"], "user_create", u)
    return {"created": u, "role": role}


@app.post("/api/users/{username}/role")
async def set_role(request: Request, username: str):
    admin = require_role(request, ["admin"])
    body = await request.json()
    role = str(body.get("role", ""))
    if role not in ROLES:
        return JSONResponse({"error": "非法角色"}, status_code=400)
    with db() as c:
        if username == admin["username"] and role != "admin":
            return JSONResponse({"error": "不能降级自己的管理员角色"}, status_code=400)
        cur = c.execute("UPDATE users SET role=? WHERE username=?", (role, username)).rowcount
        if not cur:
            return JSONResponse({"error": "用户不存在"}, status_code=404)
    audit(admin["username"], "user_role", f"{username}->{role}")
    return {"ok": True, "username": username, "role": role}


@app.post("/api/users/{username}/reset_password")
async def reset_password(request: Request, username: str):
    admin = require_role(request, ["admin"])
    body = await request.json()
    new = str(body.get("password", ""))
    if len(new) < 6:
        return JSONResponse({"error": "密码至少 6 位"}, status_code=400)
    with db() as c:
        cur = c.execute("UPDATE users SET pwd_hash=? WHERE username=?", (_sha(new), username)).rowcount
        if not cur:
            return JSONResponse({"error": "用户不存在"}, status_code=404)
    audit(admin["username"], "user_pwd_reset", username)
    return {"ok": True}


@app.delete("/api/users/{username}")
def delete_user(request: Request, username: str):
    admin = require_role(request, ["admin"])
    if username == admin["username"]:
        return JSONResponse({"error": "不能删除当前登录用户"}, status_code=400)
    with db() as c:
        cur = c.execute("DELETE FROM users WHERE username=?", (username,)).rowcount
        c.execute("DELETE FROM sessions WHERE username=?", (username,))
        if not cur:
            return JSONResponse({"error": "用户不存在"}, status_code=404)
    audit(admin["username"], "user_delete", username)
    return {"ok": True}


@app.get("/api/audit")
def get_audit(request: Request, tail: int = 100):
    require_role(request, ["admin"])
    with db() as c:
        rows = [dict(x) for x in c.execute(
            "SELECT ts,user,action,detail FROM audit ORDER BY id DESC LIMIT ?", (tail,))]
    return rows

DB_LOCK = threading.Lock()


def db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    RESULTS.mkdir(parents=True, exist_ok=True)
    with DB_LOCK, db() as c:
        c.execute("CREATE TABLE IF NOT EXISTS users(username TEXT PRIMARY KEY, pwd_hash TEXT, role TEXT)")
        c.execute("DROP TABLE IF EXISTS sessions")
        c.execute("CREATE TABLE sessions(token TEXT PRIMARY KEY, username TEXT, role TEXT, created REAL)")
        c.execute("CREATE TABLE IF NOT EXISTS audit(id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL, user TEXT, action TEXT, detail TEXT)")
        if not c.execute("SELECT 1 FROM users").fetchone():
            c.execute("INSERT INTO users VALUES('admin', ?, 'admin')", (_sha("admin123"),))
    with DB_LOCK, db() as c:
        c.executescript("""
        CREATE TABLE IF NOT EXISTS datasets(
          tag TEXT PRIMARY KEY, name TEXT, bag TEXT, sync_dir TEXT, oxts_dir TEXT,
          frames INTEGER, duration REAL, bag_size INTEGER, ref_csv TEXT);
        CREATE TABLE IF NOT EXISTS runs(
          id TEXT PRIMARY KEY, dataset TEXT, status TEXT, config TEXT,
          started REAL, finished REAL, returncode INTEGER,
          log_path TEXT, map_pcd TEXT, traj_csv TEXT, metrics TEXT);
        """)
        for tag, name, bag, sync, oxts, frames, dur in [
            ("0001", "KITTI 2011_09_26_drive_0001 (campus, 10.8s)",
             DATA / "kitti/kitti_0001.bag", DATA / "kitti/2011_09_26/2011_09_26_drive_0001_sync",
             DATA / "kitti/2011_09_26/2011_09_26_drive_0001_extract/oxts", 108, 10.8),
            ("0033", "KITTI 2011_09_30_drive_0033 (residential, 2.6min)",
             DATA / "kitti/kitti_0033.bag", DATA / "kitti/2011_09_30/2011_09_30_drive_0033_sync",
             DATA / "kitti/2011_09_30/2011_09_30_drive_0033_extract/oxts", 1594, 159.4),
        ]:
            c.execute("INSERT OR IGNORE INTO datasets VALUES(?,?,?,?,?,?,?,?,?)",
                      (tag, name, str(bag), str(sync), str(oxts), frames, dur,
                       bag.stat().st_size if Path(bag).exists() else 0, ""))





init_db()

# ---------------- 数据集管理 ----------------

@app.get("/api/datasets")
def datasets():
    with db() as c:
        rows = [dict(r) for r in c.execute("SELECT * FROM datasets ORDER BY tag")]
    for r in rows:
        r["has_map"] = bool(r["map_pcd"] if "map_pcd" in r else False) or any(
            (RUNS / run_id / "map_web.bin").exists() for run_id in _run_ids_of(r["tag"]))
        r["has_traj"] = (DATA / f"kitti/odom_{r['tag']}_traj.csv").exists()
        r["has_ref"] = (DATA / f"kitti/kitti_{r['tag']}_ref.csv").exists()
    return rows


def _run_ids_of(tag):
    with db() as c:
        return [r["id"] for r in c.execute(
            "SELECT id FROM runs WHERE dataset=? AND status='done' ORDER BY started", (tag,))]


@app.post("/api/datasets/import")
def import_dataset(body: dict, request: Request):
    admin = require_role(request, ["admin"])
    """按路径导入本地 KITTI Raw 数据集（真实校验目录结构与文件数量）"""
    tag = str(body.get("tag", "")).strip()
    name = str(body.get("name") or tag).strip()
    sync = Path(str(body.get("sync_dir", "")).strip())
    oxts = Path(str(body.get("oxts_dir", "")).strip())
    bag = Path(str(body.get("bag", "")).strip())
    errors = []
    if not tag or not tag.replace("_", "").isalnum():
        errors.append("tag 必须为字母/数字/下划线")
    bins = sorted((sync / "velodyne_points/data").glob("*.bin")) if sync.exists() else []
    if not bins:
        errors.append(f"sync_dir 下无 velodyne_points/data/*.bin: {sync}")
    vts = (sync / "velodyne_points/timestamps.txt")
    vts_n = len(vts.read_text().splitlines()) if vts.exists() else 0
    if bins and vts_n != len(bins):
        errors.append(f"激光帧数 {len(bins)} 与时间戳行数 {vts_n} 不一致")
    oxts_n = len(list(oxts.glob("data/*.txt"))) if oxts.exists() else 0
    if not oxts_n:
        errors.append(f"oxts_dir 下无 data/*.txt: {oxts}")
    if not bag.exists():
        errors.append(f"rosbag 不存在: {bag}")
    if not (sync.parent / "calib_imu_to_velo.txt").exists():
        errors.append("缺少 calib_imu_to_velo.txt")
    if errors:
        return JSONResponse({"error": "导入校验失败", "errors": errors}, status_code=400)
    audit(admin["username"], "dataset_import", tag)
    with DB_LOCK, db() as c:
        c.execute("INSERT OR REPLACE INTO datasets VALUES(?,?,?,?,?,?,?,?)",
                  (tag, name, str(bag), str(sync), str(oxts), len(bins),
                   round(len(bins) / 10.0, 1), bag.stat().st_size))
    return {"imported": tag, "frames": len(bins), "oxts_packets": oxts_n,
            "duration_s": round(len(bins) / 10.0, 1)}


@app.post("/api/datasets/validate")
def validate_dataset(body: dict):
    """真实校验：帧数/时间戳一致性、oxts 频率与时间覆盖、bag/标定/参考轨迹存在性"""
    tag = str(body.get("dataset", ""))
    with db() as c:
        row = c.execute("SELECT * FROM datasets WHERE tag=?", (tag,)).fetchone()
    if not row:
        return JSONResponse({"error": "unknown dataset"}, status_code=404)
    sync, oxts, bag = Path(row["sync_dir"]), Path(row["oxts_dir"]), Path(row["bag"])
    rep = {"dataset": tag, "checks": []}

    def check(name, ok, detail):
        rep["checks"].append({"item": name, "ok": bool(ok), "detail": detail})

    bins = sorted((sync / "velodyne_points/data").glob("*.bin")) if sync.exists() else []
    vts = (sync / "velodyne_points/timestamps.txt")
    vts_n = len(vts.read_text().splitlines()) if vts.exists() else 0
    check("激光帧数", len(bins) > 0, f"{len(bins)} 帧, 时间戳 {vts_n} 行, 一致={len(bins) == vts_n}")
    if bins:
        sz = bins[0].stat().st_size
        check("单帧点云尺寸", sz % 16 == 0, f"{sz} bytes → {sz // 16} 点/帧 (x,y,z,intensity float32)")
    oxs = sorted(oxts.glob("data/*.txt")) if oxts.exists() else []
    ots = (oxts / "timestamps.txt")
    if oxs and ots.exists():
        times = [float(l.split('.')[-1][:6]) / 1e6 + 0 for l in ots.read_text().splitlines()]
        import datetime
        base = [datetime.datetime.strptime(l.strip().split('.')[0], "%Y-%m-%d %H:%M:%S")
                for l in ots.read_text().splitlines()]
        tt = np.array([b.timestamp() + f for b, f in zip(base, times)])
        rate = 1.0 / np.median(np.diff(tt)) if len(tt) > 2 else 0
        gaps = np.diff(tt)
        check("IMU 频率(100Hz 期望)", 90 <= rate <= 110, f"{rate:.1f} Hz, {len(oxs)} 包")
        check("IMU 时间连续性", float(gaps.max()) < 0.1,
              f"最大间隔 {gaps.max()*1000:.1f}ms (阈值 100ms, >100ms 会影响去畸变)")
        check("IMU 时长覆盖激光", tt[-1] - tt[0] >= vts_n / 10.0 - 0.5,
              f"IMU {tt[-1]-tt[0]:.1f}s vs 激光 {vts_n/10.0:.1f}s")
    else:
        check("oxts 数据", False, "目录缺失")
    check("rosbag", bag.exists(), f"{bag.name} {bag.stat().st_size/1e6:.0f}MB" if bag.exists() else "未生成")
    calib = sync.parent / "calib_imu_to_velo.txt"
    check("IMU-Velodyne 标定", calib.exists(), "calib_imu_to_velo.txt" if calib.exists() else "缺失")
    check("GPS/INS 参考轨迹", (DATA / f"kitti/kitti_{tag}_ref.csv").exists(),
          "kitti_%s_ref.csv (评估用)" % tag)
    check("角速度合成", True, "oxts 无陀螺仪字段, 由 100Hz RPY 差分合成 (见 AUDIT.md)")
    rep["passed"] = all(c["ok"] for c in rep["checks"])
    return rep


# ---------------- 算法配置 ----------------

@app.get("/api/config")
def get_config():
    return {"defaults": DEFAULT_CONFIG,
            "note": "value 为默认值；POST 校验后用于下一次建图任务"}


@app.get("/api/config/file")
def config_file():
    p = WS / "src/FAST_LIO/config/kitti.yaml"
    return Response(content=p.read_text() if p.exists() else "(尚未生成：首次运行任务时写入)",
                    media_type="text/plain")


@app.post("/api/config/validate")
def validate_config(body: dict):
    cfg, errs = {}, []
    for k, spec in DEFAULT_CONFIG.items():
        v = body.get(k, spec["value"])
        try:
            v = int(v) if spec["type"] == "int" else float(v)
        except (TypeError, ValueError):
            errs.append(f"{k}: 不是数值"); continue
        if not (spec["min"] <= v <= spec["max"]):
            errs.append(f"{k}: {v} 超出 [{spec['min']},{spec['max']}]"); continue
        cfg[k] = v
    return {"config": cfg, "errors": errs, "valid": not errs}


def write_config_files(cfg: dict):
    """把配置写入 FAST-LIO 的 kitti.yaml（launch 读取该文件）"""
    yaml_text = f"""common:
    lid_topic:  "/velodyne_points"
    imu_topic:  "/kitti/oxts/imu"
    time_sync_en: false
    time_offset_lidar_to_imu: 0.0
preprocess:
    lidar_type: 2
    scan_line: 64
    scan_rate: 10
    timestamp_unit: 0
    blind: {cfg['blind']}
mapping:
    acc_cov: {cfg['acc_cov']}
    gyr_cov: {cfg['gyr_cov']}
    b_acc_cov: 0.0001
    b_gyr_cov: 0.0001
    fov_degree:    180
    det_range:     100.0
    extrinsic_est_en:  false
    extrinsic_T: [ 0.810544, -0.307054, 0.802724]
    extrinsic_R: [  9.999976e-01, -7.854027e-04,  2.024406e-03,
                    7.553071e-04,  9.998898e-01,  1.482454e-02,
                   -2.035826e-03, -1.482298e-02,  9.998881e-01]
publish:
    path_en:  true
    scan_publish_en:  true
    dense_publish_en: true
    scan_bodyframe_pub_en: false
pcd_save:
    pcd_save_en: true
    interval: -1
"""
    (WS / "src/FAST_LIO/config/kitti.yaml").write_text(yaml_text)
    launch = f"""<launch>
  <arg name="rviz" default="false" />
  <rosparam command="load" file="$(find fast_lio)/config/kitti.yaml" />
  <param name="feature_extract_enable" type="bool" value="0"/>
  <param name="point_filter_num" type="int" value="{int(cfg['point_filter_num'])}"/>
  <param name="max_iteration" type="int" value="{int(cfg['max_iteration'])}" />
  <param name="filter_size_surf" type="double" value="{cfg['filter_size_surf']}" />
  <param name="filter_size_map" type="double" value="{cfg['filter_size_map']}" />
  <param name="cube_side_length" type="double" value="1000" />
  <param name="runtime_pos_log_enable" type="bool" value="1" />
  <node pkg="fast_lio" type="fastlio_mapping" name="laserMapping" output="screen" />
</launch>
"""
    (WS / "src/FAST_LIO/launch/mapping_kitti.launch").write_text(launch)


# ---------------- 建图任务 ----------------

def _db_insert_run(job_id, tag, cfg):
    with DB_LOCK, db() as c:
        c.execute("INSERT INTO runs(id,dataset,status,config,started) VALUES(?,?,?,?,?)",
                  (job_id, tag, "pending", json.dumps(cfg), time.time()))


def _db_update(job_id, **kw):
    sets = ",".join(f"{k}=?" for k in kw)
    with DB_LOCK, db() as c:
        c.execute(f"UPDATE runs SET {sets} WHERE id=?", (*kw.values(), job_id))


def _db_get_run(job_id):
    with db() as c:
        r = c.execute("SELECT * FROM runs WHERE id=?", (job_id,)).fetchone()
    return dict(r) if r else None


def _read_pcd_header_count(path: Path):
    with open(path, "rb") as f:
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            if line.startswith("POINTS"):
                return int(line.split()[1])
            if line.startswith("DATA"):
                return None


def _harvest_timing(run_dir: Path):
    """从 FAST-LIO 的 Log/ 提取真实逐帧耗时统计"""
    csv_path = LOG_DIR_LIVE / "fast_lio_time_log.csv"
    if not csv_path.exists():
        return None
    lines = csv_path.read_text().strip().splitlines()
    if len(lines) < 2:
        return None
    rows = [l.split(",") for l in lines[1:]]
    total = np.array([float(r[1]) for r in rows])
    pts = np.array([float(r[2]) for r in rows])
    pre = np.array([float(r[10]) if len(r) > 10 else 0 for r in rows])
    return {"frames_logged": len(rows),
            "total_ms_mean": round(float(total.mean() * 1000), 1),
            "total_ms_median": round(float(np.median(total) * 1000), 1),
            "total_ms_max": round(float(total.max() * 1000), 1),
            "scan_points_mean": round(float(pts.mean())),
            "preprocess_ms_mean": round(float(pre.mean() * 1000), 2),
            "time_log_csv": str(run_dir / "fast_lio_time_log.csv")}


def _run_pipeline(job_id: str, tag: str, cfg: dict):
    job = JOBS[job_id]
    run_dir = RUNS / job_id
    run_dir.mkdir(parents=True, exist_ok=True)
    log_path = run_dir / "log.txt"
    with db() as c:
        ds = c.execute("SELECT * FROM datasets WHERE tag=?", (tag,)).fetchone()
    env = dict(os.environ, SAVE_WAIT="30")
    write_config_files(cfg)
    started = time.time()
    # 宿主机 DATA 根目录映射为容器内 /data，bag 路径需做转换
    bag_in_container = "/data" + str(Path(ds["bag"]))[len(str(DATA)):]
    cmd = ["docker", "run", "--rm", "--name", f"slam-job-{job_id}", "--net=host",
           "-e", f"BAG={bag_in_container}", "-e", f"TAG={tag}",
           "-e", f"PLAY_RATE={cfg.get('play_rate', 1.0)}",
           "-v", f"{WS}:/root/ws", "-v", f"{DATA}:/data", "-v", f"{DOCKER_DIR}:/scripts",
           "fastlio:noetic", "bash", "/scripts/run_fastlio_kitti_verify.sh"]
    job.update(status="running", log_path=str(log_path))
    _db_update(job_id, status="running", log_path=str(log_path))
    proc = subprocess.Popen(cmd, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, bufsize=1)
    job["proc"] = proc
    tail = []
    with open(log_path, "w") as lf:
        for line in proc.stdout:
            lf.write(line)
            tail.append(line.rstrip())
            if len(tail) > 12:
                tail.pop(0)
            job["log"] = tail
    proc.wait()
    job["returncode"] = proc.returncode
    _db_update(job_id, finished=time.time(), returncode=proc.returncode)
    metrics = {"config": cfg, "fastlio": FASTLIO_COMMIT}
    ok = proc.returncode == 0
    if ok:
        # 陈旧保护：PCD 修改时间必须晚于本次任务启动
        if PCD_LIVE.exists() and PCD_LIVE.stat().st_mtime >= started:
            shutil.copy(PCD_LIVE, run_dir / "map.pcd")
            metrics["map_points"] = _read_pcd_header_count(run_dir / "map.pcd")
        else:
            ok = False
            metrics["error"] = "map.pcd 未更新（可能建图失败）"
    if ok:
        for name in ("traj", "timed"):
            src = DATA / f"kitti/odom_{tag}_{name}.csv"
            if src.exists():
                shutil.copy(src, run_dir / f"{name}.csv")
        for f in ("fast_lio_time_log.csv", "pos_log.txt", "mat_out.txt"):
            if (LOG_DIR_LIVE / f).exists():
                shutil.copy(LOG_DIR_LIVE / f, run_dir / f)
        metrics["timing"] = _harvest_timing(run_dir)
        decimate_to_cache(run_dir / "map.pcd", run_dir / "map_web.bin", run_dir / "map_web.meta.json")
        meta = json.loads((run_dir / "map_web.meta.json").read_text())
        metrics["map_served_points"] = meta["points_served"]
        # 轨迹评估（GPS/INS 参考口径）
        est_csv, ref_csv = run_dir / "timed.csv", Path(ds["oxts_dir"]).parent / f"kitti_{tag}_ref.csv"
        ref_csv = DATA / f"kitti/kitti_{tag}_ref.csv"
        if est_csv.exists() and ref_csv.exists():
            ev = evaluator.evaluate(est_csv, ref_csv)
            metrics["evaluation_reference"] = "KITTI OXTS GPS/INS (非官方 odometry 真值)"
            metrics["evaluation"] = ev
            (run_dir / "evaluation.json").write_text(json.dumps(ev, ensure_ascii=False, indent=1))
        else:
            metrics["evaluation"] = {"error": "缺少带时间戳轨迹或参考轨迹 CSV"}
    job["status"] = "done" if ok else "failed"
    _db_update(job_id, status=job["status"], map_pcd=str(run_dir / "map.pcd") if ok else None,
               traj_csv=str(run_dir / "traj.csv") if (run_dir / "traj.csv").exists() else None,
               metrics=json.dumps(metrics, ensure_ascii=False))


@app.post("/api/jobs")
def start_job(body: dict, request: Request):
    user = require_role(request, ["admin", "operator"])
    tag = str(body.get("dataset", ""))
    cfg_in = body.get("config") or {}
    v = validate_config(cfg_in)
    if not v["valid"]:
        return JSONResponse({"error": "配置校验失败", "errors": v["errors"]}, status_code=400)
    with db() as c:
        if not c.execute("SELECT 1 FROM datasets WHERE tag=?", (tag,)).fetchone():
            return JSONResponse({"error": "unknown dataset"}, status_code=404)
    with JOBS_LOCK:
        running = [j for j in JOBS.values() if j["status"] in ("running", "pending")]
        if running:
            return JSONResponse({"error": "已有任务在运行", "job": running[0]["id"]}, status_code=409)
        job_id = uuid.uuid4().hex[:8]
        JOBS[job_id] = {"id": job_id, "dataset": tag, "status": "pending",
                        "started": time.time(), "log": []}
    audit(user["username"], "job_start", f"{job_id}:{tag}")
    _db_insert_run(job_id, tag, v["config"])
    threading.Thread(target=_run_pipeline, args=(job_id, tag, v["config"]), daemon=True).start()
    return {"job": job_id, "dataset": tag, "config": v["config"]}


@app.get("/api/jobs")
def list_jobs():
    return [{k: v for k, v in j.items() if k != "proc"} for j in
            sorted(JOBS.values(), key=lambda x: -x["started"])]


@app.get("/api/jobs/{job_id}")
def job_status(job_id: str):
    if job_id in JOBS:
        j = {k: v for k, v in JOBS[job_id].items() if k != "proc"}
        if j["status"] == "running":
            j["elapsed"] = round(time.time() - j["started"], 1)
        return j
    r = _db_get_run(job_id)
    if not r:
        return JSONResponse({"error": "not found"}, status_code=404)
    return r


@app.post("/api/jobs/{job_id}/stop")
def stop_job(job_id: str, request: Request):
    user = require_role(request, ["admin", "operator"])
    if job_id not in JOBS or JOBS[job_id].get("proc") is None:
        return JSONResponse({"error": "job not running"}, status_code=400)
    subprocess.Popen(["docker", "stop", f"slam-job-{job_id}"],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    JOBS[job_id]["status"] = "stopping"
    audit(user["username"], "job_stop", job_id)
    return {"job": job_id, "status": "stopping"}


@app.get("/api/jobs/{job_id}/log")
def job_log(job_id: str, tail: int = 200):
    r = _db_get_run(job_id)
    path = Path(r["log_path"]) if r and r.get("log_path") else None
    if job_id in JOBS and JOBS[job_id].get("log_path"):
        path = Path(JOBS[job_id]["log_path"])
    if not path or not path.exists():
        return JSONResponse({"error": "no log"}, status_code=404)
    lines = path.read_text(errors="ignore").splitlines()
    flt = None
    return {"total_lines": len(lines), "lines": lines[-tail:]}


# ---------------- 实验库 / 评估 / 报告 ----------------

@app.get("/api/runs")
def runs(dataset: str = None):
    with db() as c:
        q = "SELECT id,dataset,status,started,finished,returncode,metrics FROM runs"
        args = ()
        if dataset:
            q += " WHERE dataset=?"
            args = (dataset,)
        rows = [dict(r) for r in c.execute(q + " ORDER BY started DESC", args)]
    for r in rows:
        m = json.loads(r.pop("metrics") or "{}")
        ev = m.get("evaluation", {})
        r["ate_rmse"] = ev.get("ate", {}).get("ate_rmse")
        r["ate_pairs"] = ev.get("ate", {}).get("n_pairs")
        r["play_rate"] = m.get("config", {}).get("play_rate")
        r["error"] = m.get("error")
        run_dir = RUNS / r["id"]
        tc = run_dir / "timed.csv"
        r["odom_rows"] = (sum(1 for _ in open(tc)) - 1) if tc.exists() else None
        r["duration_s"] = round(r["finished"] - r["started"], 1) if r["finished"] else None
        mp = run_dir / "map.pcd"
        r["map_pcd_size"] = mp.stat().st_size if mp.exists() else None
    return rows


@app.get("/api/runs/{run_id}/loops")
def run_loops(run_id: str, radius: float = 15.0, min_gap_s: float = 20.0, max_candidates: int = 15, source: str = "est"):
    """回环候选检测（几何重访法，真实计算）：
    在估计轨迹上寻找 时间间隔 > min_gap_s 且 空间距离 < radius 的位姿对。
    注意：这是候选生成，不含几何验证与位姿图优化（V2.x）。"""
    source = source if source in ("est", "ref") else "est"
    if source == "ref":
        r = _db_get_run(run_id)
        ref = DATA / f"kitti/kitti_{r['dataset']}_ref.csv" if r else None
        if not ref or not ref.exists():
            return JSONResponse({"error": "无参考轨迹"}, status_code=404)
        t, p = evaluator.load_track_csv(ref)
    else:
        est = RUNS / run_id / "timed.csv"
        if not est.exists():
            return JSONResponse({"error": "该实验无轨迹数据"}, status_code=404)
        t, p = evaluator.load_track_csv(est)
    n = len(t)
    if n < 10:
        return JSONResponse({"error": "轨迹点过少"}, status_code=400)
    d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=2)   # n<=2000, 内存可控
    gap = t[None, :] - t[:, None]
    mask = (gap > min_gap_s) & (d < radius)
    ii, jj = np.where(mask)
    order = np.argsort(d[ii, jj])
    cands, used_t = [], []
    for k in order:                       # 按距离从近到远，做时间上的一层去重
        ti, tj = float(t[ii[k]]), float(t[jj[k]])
        if any(abs(ti - a) < min_gap_s * 0.5 and abs(tj - b) < min_gap_s * 0.5 for a, b in used_t):
            continue
        used_t.append((ti, tj))
        cands.append({"t1": round(ti, 2), "t2": round(tj, 2),
                      "p1": [round(float(v), 2) for v in p[ii[k]]],
                      "p2": [round(float(v), 2) for v in p[jj[k]]],
                      "dist": round(float(d[ii[k], jj[k]]), 2),
                      "gap_s": round(tj - ti, 1)})
        if len(cands) >= max_candidates:
            break
    step = max(1, n // 2000)
    return {"run": run_id, "source": source, "radius_m": radius, "min_gap_s": min_gap_s,
            "candidates": cands,
            "trajectory": [[round(float(x), 2), round(float(y), 2)] for x, y in zip(p[::step, 0], p[::step, 1])],
            "note": "几何重访候选，未做几何验证与位姿图优化"}


@app.get("/api/runs/compare")
def compare_runs(a: str, b: str):
    ra, rb = _db_get_run(a), _db_get_run(b)
    if not ra or not rb:
        return JSONResponse({"error": "not found"}, status_code=404)
    ma = json.loads(ra["metrics"] or "{}")
    mb = json.loads(rb["metrics"] or "{}")

    def flat(m):
        ate = m.get("evaluation", {}).get("ate", {})
        tim = m.get("timing", {}) or {}
        return {"ate_rmse": ate.get("ate_rmse"), "ate_pairs": ate.get("n_pairs"),
                "map_points": m.get("map_points"),
                "total_ms_mean": tim.get("total_ms_mean"),
                "scan_points_mean": tim.get("scan_points_mean"),
                "play_rate": m.get("config", {}).get("play_rate"),
                "config": m.get("config")}
    fa, fb = flat(ma), flat(mb)
    diff = {k: (fa.get(k), fb.get(k)) for k in fa}
    return {"a": a, "b": b, "fields": diff}


@app.get("/api/runs/{run_id}")
def run_detail(run_id: str):
    r = _db_get_run(run_id)
    if not r:
        return JSONResponse({"error": "not found"}, status_code=404)
    r["metrics"] = json.loads(r.pop("metrics") or "{}")
    return r


@app.get("/api/runs/{run_id}/ate_series")
def run_ate_series(run_id: str):
    r = _db_get_run(run_id)
    p = RUNS / run_id / "evaluation.json"
    if not p.exists():
        return JSONResponse({"error": "no evaluation"}, status_code=404)
    ev = json.loads(p.read_text())
    return ev.get("ate_series") or {}


@app.get("/api/runs/{run_id}/trajectories")
def run_trajectories(run_id: str):
    """估计 vs 参考轨迹（二维俯视对比用），真实文件读取"""
    rdir = RUNS / run_id
    r = _db_get_run(run_id)
    if not r or not r.get("dataset"):
        return JSONResponse({"error": "not found"}, status_code=404)
    with db() as c:
        ds = c.execute("SELECT oxts_dir FROM datasets WHERE tag=?", (r["dataset"],)).fetchone()
    out = {"aligned": False}
    est = rdir / "timed.csv"
    if est.exists():
        t, p = evaluator.load_track_csv(est)
        # 估计轨迹在 LiDAR/IMU 系，用评估得到的 Umeyama 对齐变换转到参考系后再对比
        ev_path = rdir / "evaluation.json"
        if ev_path.exists():
            ate = json.loads(ev_path.read_text()).get("ate", {})
            if ate.get("alignment_R") and ate.get("alignment_t"):
                R = np.array(ate["alignment_R"])
                tr = np.array(ate["alignment_t"])
                p = p @ R.T + tr
                out["aligned"] = True
        out["est"] = [[round(x, 2), round(y, 2)] for x, y in zip(p[:, 0], p[:, 1])]
    ref = DATA / f"kitti/kitti_{r['dataset']}_ref.csv"
    if ref.exists():
        t, p = evaluator.load_track_csv(ref)
        step = max(1, len(p) // 2000)
        out["ref"] = [[round(x, 2), round(y, 2)] for x, y in zip(p[::step, 0], p[::step, 1])]
    return out


@app.get("/api/runs/{run_id}/report.json")
def report_json(run_id: str):
    r = _db_get_run(run_id)
    if not r:
        return JSONResponse({"error": "not found"}, status_code=404)
    r["metrics"] = json.loads(r.pop("metrics") or "{}")
    r["system"] = monitor.snapshot()
    return r


@app.get("/api/runs/{run_id}/report.html")
def report_html(run_id: str):
    r = _db_get_run(run_id)
    if not r:
        return JSONResponse({"error": "not found"}, status_code=404)
    m = json.loads(r.pop("metrics") or "{}")
    ev = m.get("evaluation", {}).get("ate", {})
    tim = m.get("timing", {}) or {}
    rows = "".join(f"<tr><td>{k}</td><td>{v}</td></tr>" for k, v in [
        ("实验 ID", run_id), ("数据集", r["dataset"]), ("状态", r["status"]),
        ("开始时间", time.strftime("%F %T", time.gmtime(r["started"] + 8*3600)) + " (UTC+8)"),
        ("耗时(含容器)", f"{(r['finished'] or 0) - r['started']:.0f}s" if r["finished"] else "-"),
        ("FAST-LIO2 版本", m.get("fastlio", "-")),
        ("算法配置", json.dumps(m.get("config", {}), ensure_ascii=False)),
        ("地图点数", f"{m.get('map_points'):,}" if m.get("map_points") else "-"),
        ("逐帧耗时均值", f"{tim.get('total_ms_mean')} ms" if tim else "-"),
        ("逐帧耗时中位", f"{tim.get('total_ms_median')} ms" if tim else "-"),
        ("单帧点数均值", f"{tim.get('scan_points_mean'):,}" if tim else "-"),
        ("ATE RMSE (GPS/INS 参考)", f"{ev.get('ate_rmse')} m ({ev.get('n_pairs')} 对)" if ev else "未评估"),
        ("参考口径", m.get("evaluation_reference", "-")),
    ])
    html = f"""<html><head><meta charset="utf-8"><title>实验报告 {run_id}</title>
<style>body{{font-family:system-ui;margin:30px;color:#111}}table{{border-collapse:collapse}}
td{{border:1px solid #ccc;padding:6px 14px}}td:first-child{{background:#f5f5f5;font-weight:600}}
h1{{font-size:20px}}</style></head><body>
<h1>三维激光惯性SLAM建图与轨迹评估 — 实验报告</h1><table>{rows}</table>
<p style="color:#888;font-size:12px">ATE 口径：相对 KITTI OXTS GPS/INS 参考轨迹，SE(3) Umeyama 对齐后逐点欧氏误差 RMSE。
闭环检测与位姿图优化为 V2.0 规划功能，本报告不包含。</p></body></html>"""
    return HTMLResponse(html)


@app.get("/api/runs/{run_id}/export/traj.csv")
def export_traj(run_id: str):
    p = RUNS / run_id / "timed.csv"
    if not p.exists():
        p = RUNS / run_id / "traj.csv"
    if not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p, media_type="text/csv", filename=f"{run_id}_traj.csv")


@app.get("/api/runs/{run_id}/export/evaluation.json")
def export_eval(run_id: str):
    p = RUNS / run_id / "evaluation.json"
    if not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p, media_type="application/json", filename=f"{run_id}_evaluation.json")


@app.get("/api/runs/{run_id}/export/map_decimated.pcd")
def export_map_decimated(run_id: str):
    rdir = RUNS / run_id
    src_bin, meta_f, src_pcd = rdir / "map_web.bin", rdir / "map_web.meta.json", rdir / "map.pcd"
    if not (src_pcd.exists() and src_bin.exists() and meta_f.exists()):
        return JSONResponse({"error": "not found"}, status_code=404)
    out = rdir / "map_decimated.pcd"
    if not out.exists():
        meta = json.loads(meta_f.read_text())
        n_tot, n_srv = meta["points_total"], meta["points_served"]
        pts = _read_pcd(src_pcd)
        stride = max(1, n_tot // max(1, n_srv))
        sub = pts[::stride]
        hdr = (f"VERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
               f"WIDTH {len(sub)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(sub)}\nDATA binary\n")
        with open(out, "wb") as f:
            f.write(hdr.encode()); f.write(np.ascontiguousarray(sub).tobytes())
    return FileResponse(out, media_type="application/octet-stream", filename=f"{run_id}_map_decimated.pcd")


@app.get("/api/jobs/{job_id}/log/download")
def download_job_log(job_id: str):
    return export_log(job_id)


@app.get("/api/runs/{run_id}/log/download")
def export_log(run_id: str):
    import re as _re
    r = _db_get_run(run_id)
    p = Path(r["log_path"]) if r and r.get("log_path") else None
    if not p or not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    text = p.read_text(errors="ignore")
    text = _re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", text)   # 剥离 ANSI 终端控制符
    return PlainTextResponse(text, media_type="text/plain",
        headers={"Content-Disposition": f'attachment; filename="{run_id}_log.txt"'})


@app.get("/api/runs/{run_id}/export/map.pcd")
def export_map(run_id: str):
    p = RUNS / run_id / "map.pcd"
    if not p.exists():
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(p, media_type="application/octet-stream", filename=f"{run_id}_map.pcd")


# ---------------- 兼容旧接口（最新成功 run） ----------------

def _latest_done(tag):
    ids = _run_ids_of(tag)
    return _db_get_run(ids[-1]) if ids else None


@app.get("/api/map/{tag}")
def get_map(tag: str):
    r = _latest_done(tag)
    bin_path = RUNS / r["id"] / "map_web.bin" if r else None
    legacy = RESULTS / tag / "map_web.bin"
    if r and bin_path and bin_path.exists():
        meta = json.loads((RUNS / r["id"] / "map_web.meta.json").read_text())
        return Response(content=bin_path.read_bytes(), media_type="application/octet-stream",
                        headers={"X-Meta": json.dumps(meta), "X-Run": r["id"]})
    if legacy.exists():
        meta = json.loads((RESULTS / tag / "map_web.meta.json").read_text())
        return Response(content=legacy.read_bytes(), media_type="application/octet-stream",
                        headers={"X-Meta": json.dumps(meta), "X-Run": "legacy"})
    return JSONResponse({"error": "map not built yet"}, status_code=404)


@app.get("/api/traj/{tag}")
def get_traj(tag: str):
    r = _latest_done(tag)
    p = (RUNS / r["id"] / "timed.csv") if r else None
    if not (p and p.exists()):
        p = DATA / f"kitti/odom_{tag}_traj.csv"
    if not p.exists():
        return JSONResponse({"error": "not built yet"}, status_code=404)
    a = np.genfromtxt(p, delimiter=",", names=True)
    dist = float(np.sum(np.hypot(np.diff(a["x"]), np.hypot(np.diff(a["y"]), np.diff(a["z"])))))
    step = max(1, len(a) // 3000)
    return {"count": int(len(a)), "length_m": round(dist, 1),
            "points": [[round(float(x), 2), round(float(y), 2), round(float(z2), 2)]
                       for x, y, z2 in zip(a["x"][::step], a["y"][::step], a["z"][::step])]}


@app.get("/api/stats/{tag}")
def get_stats(tag: str):
    r = _latest_done(tag)
    out = {"map_points": None}
    if r:
        m = json.loads(r["metrics"] or "{}")
        out.update({"map_points": m.get("map_points"), "run_id": r["id"],
                    "timing": m.get("timing"), "evaluation": m.get("evaluation")})
    return out


# ---------------- 系统监控 / 总览 ----------------

@app.get("/api/monitor")
def api_monitor():
    return monitor.snapshot()


@app.get("/api/overview")
def overview():
    with db() as c:
        n_runs = c.execute("SELECT COUNT(*) c FROM runs").fetchone()["c"]
        n_done = c.execute("SELECT COUNT(*) c FROM runs WHERE status='done'").fetchone()["c"]
        n_ds = c.execute("SELECT COUNT(*) c FROM datasets").fetchone()["c"]
    sysinfo = monitor.snapshot()
    sysinfo["containers_running_count"] = len(sysinfo.pop("containers_running", []))
    return {"datasets": n_ds, "runs_total": n_runs, "runs_done": n_done,
            "fastlio": FASTLIO_COMMIT, "system": sysinfo,
            "modules": {"数据适配": "已实现", "激光惯性里程计(FAST-LIO2)": "已实现",
                        "三维重建": "已实现", "轨迹评估(ATE/RPE)": "已实现",
                        "算法配置": "已实现", "实验管理": "已实现", "报告导出": "已实现(JSON/CSV/HTML)",
                        "闭环检测与位姿图优化": "V2.0 规划中（未实现）"}}


def decimate_to_cache(pcd_path: Path, bin_path: Path, meta_path: Path, max_pts: int = 1_500_000):
    pts = _read_pcd(pcd_path)
    n = len(pts)
    stride = max(1, n // max_pts)
    sub = pts[::stride]
    out = np.empty((len(sub), 4), dtype="<f4")
    out[:, 0], out[:, 1], out[:, 2] = sub["x"], sub["y"], sub["z"]
    out[:, 3] = sub["intensity"] if "intensity" in pts.dtype.names else 0
    out.tofile(bin_path)
    meta_path.write_text(json.dumps({"points_total": int(n), "points_served": int(len(sub)),
                                     "zmin": float(out[:, 2].min()), "zmax": float(out[:, 2].max())}))


def _read_pcd(path: Path):
    with open(path, "rb") as f:
        header = {}
        while True:
            line = f.readline().decode("ascii", errors="ignore").strip()
            if not line:
                continue
            key = line.split()[0]
            if key in ("FIELDS", "SIZE", "TYPE", "COUNT"):
                header[key] = line.split()[1:]
            elif key == "POINTS":
                header["POINTS"] = int(line.split()[1])
            elif key == "DATA":
                break
        dt = np.dtype([(n, t.lower() + s) for n, t, s in
                       zip(header["FIELDS"], header["TYPE"], header["SIZE"])])
        return np.fromfile(f, dtype=dt, count=header["POINTS"])


# SPA 页面路径兜底：/workspace、/eval 等直接访问时返回前端入口（前端 hash 路由接管）
@app.get("/{page_path:path}", include_in_schema=False)
def spa_fallback(page_path: str):
    if page_path.startswith("api/"):
        return JSONResponse({"error": "not found"}, status_code=404)
    return FileResponse(Path(__file__).parent / "static" / "index.html")


app.mount("/", StaticFiles(directory=Path(__file__).parent / "static", html=True), name="static")
