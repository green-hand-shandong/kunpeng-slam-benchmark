"""系统监控（真实数据源：/proc 与 docker CLI）"""
import subprocess
import time
from pathlib import Path


def _cpu_percent(sample_s=0.25):
    def read():
        vals = {}
        for line in Path('/proc/stat').read_text().splitlines():
            if line.startswith('cpu '):
                p = [int(x) for x in line.split()[1:8]]
                vals = {'total': sum(p), 'idle': p[3] + p[4]}
        return vals
    a = read()
    time.sleep(sample_s)
    b = read()
    dt = b['total'] - a['total']
    return round(100 * (1 - (b['idle'] - a['idle']) / dt), 1) if dt > 0 else None


def snapshot():
    meminfo = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        k, v = line.split(':')
        meminfo[k] = int(v.split()[0])  # kB
    total_gb = meminfo['MemTotal'] / 1e6
    avail_gb = meminfo['MemAvailable'] / 1e6
    load1 = float(Path('/proc/loadavg').read_text().split()[0])
    st = Path('/proc/stat').read_text().splitlines()[0].split()
    boot = time.time() - float(Path('/proc/uptime').read_text().split()[0])
    du = subprocess.run(['df', '-B1', '/data0'], capture_output=True, text=True).stdout.splitlines()
    disk_total, disk_used = (int(x) for x in du[1].split()[1:3])
    ncpu = Path('/proc/cpuinfo').read_text().count('processor')
    try:
        boxes = subprocess.run(['docker', 'ps', '--format', '{{.Names}}\t{{.Status}}'],
                               capture_output=True, text=True, timeout=5).stdout.strip().splitlines()
        containers = [b.split('\t') for b in boxes if b]
    except Exception:
        containers = []
    return {'cpu_percent': _cpu_percent(), 'cpu_cores': ncpu,
            'mem_total_gb': round(total_gb, 1), 'mem_used_gb': round(total_gb - avail_gb, 1),
            'load1': load1, 'disk_data0_used_gb': round(disk_used / 1e9, 1),
            'disk_data0_total_gb': round(disk_total / 1e9, 1),
            'boot_time': time.strftime('%Y-%m-%d %H:%M', time.localtime(boot)),
            'containers_running': containers}
