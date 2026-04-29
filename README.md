# camera03 — Isaac Sim PTZ 虚拟相机推流工程

## 工程概述

本工程基于 **NVIDIA Isaac Sim** 实现了一套完整的 PTZ（Pan-Tilt-Zoom）虚拟安防相机系统：典型链路为 **ptz_launcher.py:8080 → ptz_stream.py:8081 → mediamtx:8554**（见下文架构图）。画面输出形态（如 MJPEG / RTSP / HLS 等）与 **ONVIF** 接入是否同时启用，**以本机 `ptz_config.yaml` 与当前运行进程为准**，部署到新机器时请逐项核对路径与开关。

**第三方平台对接**（服务启停、拉流、PTZ 控制等完整 HTTP/RTSP 说明）见：**[docs/API.md](./docs/API.md)**。

---

## 系统架构

```
┌─────────────────────────────────────────────────────────┐
│  用户浏览器  http://localhost:8080                       │
└──────────────────────┬──────────────────────────────────┘
                       │ HTTP
┌──────────────────────▼──────────────────────────────────┐
│  ptz_launcher.py  :8080  (始终运行，GPU ≈ 0%)           │
│  • 服务 Web UI                                          │
│  • 管理 Isaac Sim 子进程的启动 / 停止                    │
│  • 代理 /control /scene/* /stream.mjpeg 到 :8081        │
└──────────────────────┬──────────────────────────────────┘
                       │ 按需启动（start_new_session）
┌──────────────────────▼──────────────────────────────────┐
│  ptz_stream.py  :8081  (Isaac Sim headless)             │
│  • 加载 scene_4diaolan_ptz.usda 主场景                  │
│  • Replicator RenderProduct 捕获相机画面                 │
│  • MJPEG 推流（浏览器低延迟预览）                        │
│  • RTSP 推流（VLC / NVR 接入）                          │
│  • PTZ 控制 API + 场景控制 API                          │
└────────────┬────────────────────┬───────────────────────┘
             │ stdin pipe         │ RTSP ANNOUNCE
┌────────────▼────────┐  ┌────────▼────────────────────────┐
│  ffmpeg（H.264编码）│  │  mediamtx  :8554  RTSP Server   │
└─────────────────────┘  └─────────────────────────────────┘
```

**关键设计**：`ptz_launcher.py` 用 `start_new_session=True` 为 Isaac Sim 子进程创建独立进程组，停止时通过 `os.killpg(pgid, SIGKILL)` 确保 bash 壳、Python 进程、mediamtx、ffmpeg **全部干净退出**，彻底释放 GPU 资源。

---

## 文件说明

| 文件 | 类型 | 说明 |
|------|------|------|
| `ptz_launcher.py` | Python | 轻量启动器（系统入口，始终运行，无 GPU 占用） |
| `ptz_stream.py` | Python | Isaac Sim 推流主体（按需启动，含 PTZ / 场景控制逻辑） |
| `ptz_config.yaml` | 配置 | 端口、场景路径、分辨率、码率等全局配置 |
| `diaolan_randomizer.py` | Python | 吊篮随机化工具：在多个吊篮候选中选取/切换、与高度与工人等逻辑配合 |
| `ptz_web_control.html` | HTML | Web 控制台页面（由 launcher 提供服务） |
| `mediamtx.yml` | 配置 | MediaMTX RTSP 服务器配置（启用 RTSP:8554 + HLS:8888） |
| `mediamtx` | 二进制 | MediaMTX v1.17.0 可执行文件（.gitignore 忽略） |
| `scene_4diaolan_ptz.usda` | USDA | **当前主场景**：引用子层工地资产并挂载 CameraRig / 灯光等 |
| `setup_camera_v4.py` | Python | 一次性工具：为裸 Camera prim 创建 Pan/Tilt 控制层级 |
| `fix_ptz_orientation.py` | Python | 一次性工具：修复 Y-up 球机模型放入 Z-up 场景时的方向/缩放 |
| `test_onvif.py` | Python | 验证脚本：用 python-onvif-zeep 测试 ONVIF PTZ 控制接口 |
| `PTZ_Camera_Rig.usda` | USDA | PTZ 球机外观参考资产（立柱+云台+镜头+Camera，Z-up，米） |
| `DiaoLan_ChangJing_2026.03.18.usdz` | USDZ | 原始工地场景资产（大文件，.gitignore 忽略） |

---

## USD 场景结构（scene_4diaolan_ptz.usda）

当前 **主场景文件** 为 `scene_4diaolan_ptz.usda`：在文件内通过 `subLayers` 引用子层资产（如打包的工地 USDZ），并在 `World` 下覆盖 `CameraRig/CamTilt/Camera` 与灯光等。

子层资源中通常包含 **多套吊篮实例**（命名上常见为 **DiaoLan01～DiaoLan06** 等，具体以子层 USD 为准）。运行时由 **`diaolan_randomizer.py`** 在候选集合中 **随机选取 / 切换** 当前活跃吊篮，并与 `ptz_config.yaml` 中的相关项（例如 `force_active_diaolan_path`）配合；**请勿在文档中写死未经现场核对的 prim 路径**。

主场景 USDA 内可直接看到的结构示意（以仓库内文件为准）：

```
World（defaultPrim）
├── CameraRig / CamTilt / Camera   # PTZ 渲染相机
├── KeyLight / FillLight / …       # 灯光
└── （几何与多套吊篮主体来自 subLayers 引用的子层资产）
```

> **坐标系**：主场景为 Z-up；子层资产可能自带尺度/轴向约定，以实际 USD 为准。

---

## 快速开始

### 1. 启动轻量服务（始终运行）

```bash
cd /home/uniubi/xuanyuan/camera05/camera03

# 1) 确认 8080 / 8081 / 8554 未被占用（若被占用，请在确认 cwd 与用途后自行处理旧进程）
ss -ltnp | grep -E ':8080|:8081|:8554' || true

# 2) 启动 ONVIF / Web 入口（示例：请按本机 conda / Isaac 环境替换 python 路径）
nohup /path/to/isaac/python3 \
  /home/uniubi/xuanyuan/camera05/camera03/ptz_launcher.py \
  --config /home/uniubi/xuanyuan/camera05/camera03/ptz_config.yaml \
  > /tmp/onvif_debug.log 2>&1 &

# 3) 检查 launcher 是否启动成功
ss -ltnp | grep 8080
ps -ef | grep ptz_launcher.py | grep -v grep
tail -n 50 /tmp/onvif_debug.log
```

打开浏览器访问：**http://localhost:8080/**

### 2. 在 Web UI 中启动 Isaac Sim 推流

点击右上角 **"⏻ 开始推流"** 按钮。

也可以直接用命令启动：

```bash
curl -X POST http://127.0.0.1:8080/start

# 检查 Isaac Sim / 推流侧是否已起来
ss -ltnp | grep -E ':8081|:8554|:8888'
ps -ef | grep -E 'ptz_stream.py|mediamtx' | grep -v grep
tail -n 100 /home/uniubi/xuanyuan/camera05/camera03/isaac_stream.log
```

Isaac Sim 首次冷启动约需 **60~120 秒**，加载完成后 Web UI 自动切换为视频画面。

### 2.1 为 `ws-flv` 启动 HTTPS / WSS 代理

当上层页面是 `https://` 时，浏览器通常不能直接连接 `ws://<host>:8080/ws-flv`。  
可在服务器上额外启动一个 TLS 代理，将 `https://` / `wss://` 转发到本机 `8080`：

```bash
cd /home/uniubi/xuanyuan/camera05/camera03

# 1) 生成自签名证书（仅首次需要；CN/SAN 请替换为 <目标机器 IP>）
mkdir -p /home/uniubi/xuanyuan/camera05/camera03/certs
openssl req -x509 -newkey rsa:2048 -nodes -days 3650 \
  -keyout /home/uniubi/xuanyuan/camera05/camera03/certs/camera03.key \
  -out /home/uniubi/xuanyuan/camera05/camera03/certs/camera03.crt \
  -subj "/CN=<目标机器 IP>" \
  -addext "subjectAltName=IP:<目标机器 IP>"

# 2) 启动 HTTPS / WSS 代理（8443 -> 8080）
nohup /path/to/isaac/python3 \
  /home/uniubi/xuanyuan/camera05/camera03/tls_tcp_proxy.py \
  --listen-host 0.0.0.0 \
  --listen-port 8443 \
  --upstream-host 127.0.0.1 \
  --upstream-port 8080 \
  --cert /home/uniubi/xuanyuan/camera05/camera03/certs/camera03.crt \
  --key /home/uniubi/xuanyuan/camera05/camera03/certs/camera03.key \
  > /tmp/camera03_tls_proxy.log 2>&1 &

# 3) 检查端口与日志
ss -ltnp | grep 8443
tail -n 50 /tmp/camera03_tls_proxy.log
```

启动后可使用（将 `<目标机器 IP>` 换成实际访问 IP，勿写死某一固定环境）：

```text
https://<目标机器 IP>:8443/
wss://<目标机器 IP>:8443/ws-flv
```

说明：
- 自签名证书默认不会被浏览器信任，首次访问需要手动接受证书风险。
- 当前用户无 `sudo`，因此默认使用 `8443`，不是 `443`。

### 3. 停止推流释放 GPU

点击 **"⏹ 停止推流"**，launcher 向整个进程组发送 SIGTERM + SIGKILL，所有资源完全释放。

### 4. 外部客户端接入 RTSP

```bash
# VLC
vlc rtsp://localhost:8554/ptz_cam

# ffplay
ffplay rtsp://localhost:8554/ptz_cam -rtsp_transport tcp
```

---

## Web 控制台功能

### PTZ 相机控制

| 控件 | 功能 | 范围 |
|------|------|------|
| 2D 摇杆 | 同时控制水平角（Pan）和俯仰角（Tilt） | Pan: -170°~+170°，Tilt: -90°~+30° |
| 变焦滑条 | 光学变焦倍数 | 1× ~ 32× |
| 预置位面板 | 调用 / 保存 / 删除 1~5 槽位 | 5 个固定槽位 |
| 键盘 | ← → 调 Pan，↑ ↓ 调 Tilt，+/- 调 Zoom | — |

### 场景控制（吊篮 + 工人）

| 控件 | 功能 | 范围 |
|------|------|------|
| 吊篮高度滑条 | 控制当前活跃吊篮装配体高度（具体 prim 随子层资产而定） | 依场景与配置 |
| 工人数量按钮 | 0：全隐；1：随机显示一人；2：全显 | 0 / 1 / 2 |

工人显示时，其位置**可配置为跟随吊篮**（以运行时代码与场景为准）。

---

## HTTP API 参考

所有接口均在 `http://localhost:8080`（launcher 代理）。

### 启动器管理

| 方法 | 路径 | 说明 |
|------|------|------|
| GET | `/status` | 返回 `isaac_state`（stopped/starting/running/stopping）及 PTZ 状态 |
| POST | `/start` | 启动 Isaac Sim 推流进程 |
| POST | `/stop` | 停止并完全清理 Isaac Sim 进程组 |
| GET | `/log` | 返回 Isaac Sim 启动日志末尾 8 KB |

### PTZ 控制（Isaac Sim 运行时有效）

```http
POST /control
Content-Type: application/json

{"pan": 45.0, "tilt": -30.0, "zoom": 8.0}
```

### 场景控制（Isaac Sim 运行时有效）

```http
# 吊篮高度
POST /scene/gondola
{"y": 1650}

# 工人数量（0/1/2）
POST /scene/workers
{"count": 1}

# 查询场景当前状态
GET /scene/state
```

### 视频流

| 路径 | 说明 |
|------|------|
| `GET /stream.mjpeg` | MJPEG 连续流（浏览器 canvas 低延迟播放，延迟 ~100-300ms） |
| `GET /snapshot.jpg` | 单帧 JPEG 快照 |

### ONVIF 服务（供 VMS / NVR 对接）

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/onvif/device_service` | ONVIF 设备服务（GetCapabilities / GetSystemDateAndTime / GetServices） |
| POST | `/onvif/media_service` | ONVIF 媒体服务（GetProfiles / GetSnapshotUri / GetServiceCapabilities） |
| POST | `/onvif/ptz_service` | ONVIF PTZ 控制服务（AbsoluteMove / RelativeMove / SetPreset / GetPresets / GotoPreset / RemovePreset / GetStatus / GetNodes / GetConfigurations） |
| GET | `/onvif-snap.jpg` | 单帧快照（ONVIF GetSnapshotUri 返回的 URL，代理到 Isaac Sim 内部快照） |
| GET | `/presets` | 返回当前 1~5 预置位槽位 |
| POST | `/presets/{token}` | 保存当前或指定 PTZ 到槽位 |
| POST | `/presets/{token}/goto` | 调用槽位 |
| DELETE | `/presets/{token}` | 删除槽位 |

坐标单位转换（ONVIF 归一化 → 本仓库 PTZ 角度/倍率）：

| ONVIF 参数 | 范围 | PTZ 参数 | 换算 |
|-----------|------|--------------|------|
| pan（x） | \[-1, 1\] | pan_deg | `× 170.0` → \[-170°, 170°\] |
| tilt（y） | \[-1, 1\] | tilt_deg | `-60.0 × norm - 30.0` → \[-90°, 30°\]（与 Java 仿射公式一致） |
| zoom（x） | \[0, 1\] | zoom_x | `1.0 + × 31.0` → \[1×, 32×\] |

预置位默认槽位：`1`=默认位、`2`=水平正视、`3`=左90°、`4`=右90°、`5`=俯视。  
预置位保存后会回写 `ptz_config.yaml` 的 `presets` 段，重启后仍保留。

---

## 配置文件（ptz_config.yaml）

```yaml
launcher_port: 8080          # launcher Web UI 端口
ctrl_port:     8081          # Isaac Sim 内部 HTTP 端口
python_sh: /path/to/python.sh  # Isaac Sim python 解释器路径

scene_path:   ./scene_4diaolan_ptz.usda    # 要加载的 USD 主场景
camera_prim:  /World/CameraRig/CamTilt/Camera  # 推流相机路径

resolution:   [1920, 1080]
fps:          25
bitrate:      "4M"
rtsp_enabled: true           # false 时仅提供 /snapshot.jpg 快照，不启动 ffmpeg/mediamtx
focal_length_1x: 18.14756    # 1× 基准焦距（mm），32× 时自动 ×32
```

---

## 移植到新工程

只需将以下文件复制到目标工程目录：

```
ptz_launcher.py
ptz_stream.py
ptz_config.yaml
diaolan_randomizer.py
ptz_web_control.html
mediamtx.yml
mediamtx（二进制，或设 auto_download: true 自动下载）
scene_4diaolan_ptz.usda（及子层引用的资产路径）
```

然后修改 `ptz_config.yaml` 中：
1. `python_sh` → Isaac Sim 安装路径
2. `scene_path` → 目标机器上 `scene_4diaolan_ptz.usda` 的绝对路径
3. `camera_prim` → 目标场景中的相机 Prim 路径

如果目标场景的相机没有 Pan/Tilt 控制层级，运行一次：
```bash
python3 setup_camera_v4.py --scene ./scene_4diaolan_ptz.usda --cam /World/Camera
```

---

## 开发工具脚本

### setup_camera_v4.py

为场景中已有的裸 Camera prim 添加 PTZ 控制层级（一次性执行）：

```bash
python3 setup_camera_v4.py [--scene ./scene_4diaolan_ptz.usda] [--cam /World/Camera]
```

执行后自动创建 `CameraRig / CamTilt / Camera` 三层结构，原 Camera 被 deactivate。

### fix_ptz_orientation.py

修复将 Y-up / cm 坐标的球机模型（如 PTZ_SecurityDome.usda）放入 Z-up / 米坐标场景时出现的"倒地"和"体积异常"问题：

```bash
python3 fix_ptz_orientation.py [--scene ./YourScene.usd] [--x 5.0] [--y -8.0] [--z 0.0]
```

自动应用 `rotateXYZ=(-90, 0, 0)` 和 `scale=(0.01, 0.01, 0.01)`。

---

## 技术栈

| 组件 | 版本 / 说明 |
|------|------------|
| NVIDIA Isaac Sim | headless 模式，Raytraced Lighting 渲染 |
| USD (pxr) | 场景描述，Z-up / metersPerUnit=1 |
| omni.replicator.core | RenderProduct + RGB Annotator 图像捕获 |
| MediaMTX | v1.17.0，RTSP Server |
| ffmpeg | H.264 编码，yuv420p，零延迟推流 |
| Pillow / OpenCV | MJPEG JPEG 帧编码 |
| Python 3.11 | Isaac Sim 内置 Python 环境 |

---

## 端口占用一览

| 端口 | 进程 | 用途 |
|------|------|------|
| 8080 | ptz_launcher.py | Web UI + API（始终监听） |
| 8081 | ptz_stream.py | Isaac Sim 内部 API（按需） |
| 8554 | mediamtx | RTSP 流（供 VLC/NVR）（按需） |
| 8888 | mediamtx | HLS 流（按需） |
