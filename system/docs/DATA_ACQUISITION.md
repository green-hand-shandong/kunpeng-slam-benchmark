# EuRoC / R3LIVE-avia 数据获取指南（2026-11-06）

> 现状：自动脚本化下载的全部路径已穷尽（见下）。剩余路径需要**一次人工操作**（注册账号或浏览器登录网盘），
> 之后数据可长期使用。LIO-SAM 的 sample 已有 watchdog 自动重试，无需人工。

## EuRoC MAV（模态 C：VINS-Fusion 精度评估）

| 路径 | 状态 |
|---|---|
| robotics.ethz.ch 官方直链 | ✗ TCP 不通（路由封锁，DNS 正常 129.132.38.186） |
| ghproxy 系镜像 | ✗ 只代理 GitHub 域，ETH 不适用 |
| GitHub 镜像仓/release | ✗ 无 EuRoC bag（已穷尽搜索） |
| Newer College（替代数据） | ✗ 表单墙 + Google Drive |
| **OpenDataLab（推荐）** | ⚠️ 有 EuRoC 页面（opendatalab.com/OpenDataLab/EuRoC），需免费注册后 CLI 下载 |

**人工操作步骤（5 分钟）**：
1. 浏览器打开 https://opendatalab.com 注册（手机号即可）
2. `pip install openxlab && openxlab login`（输入 AK/SK）
3. `openxlab dataset download --dataset-repo OpenDataLab/EuRoC --target-path /data0/slam-datasets/modality_C/`
4. 下载完告诉我，VINS-Fusion 精度评估管线已就绪

## R3LIVE-avia（模态 D：R3LIVE 精度评估）

| 路径 | 状态 |
|---|---|
| OneDrive 分享 | ✗ API 需认证（已迁移 SharePoint） |
| 百度网盘 | ⚠️ 需登录+客户端（可人工下载后拷入 /data0/slam-datasets/modality_D/） |

**人工操作步骤**：百度网盘链接 https://pan.baidu.com/s/1zmVxkcwOSul8oTBwaHfuFg
（提取码见 r3live_dataset 仓库 README），下载 `hku_park_01.bag`（最小序列）拷入即可。

## LIO-SAM sample（模态 B 补测）

- Watchdog 已在跑（每小时探测 drive.google.com，通了自动下载到 /data0/slam-datasets/lio_sam_samples/）
- 日志：/data0/slam-datasets/lio_sam_watchdog.log
