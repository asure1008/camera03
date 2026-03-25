# Camera03 培训大纲
## 基于 Isaac Sim 的 PTZ 虚拟相机推流系统——设计思路与实现

**适用对象**：熟悉 Python 的研发工程师，了解基本网络协议概念  
**培训目标**：理解系统整体设计思路，能够独立移植到新工程并扩展功能  
**建议时长**：3～4 小时（含动手实验）

---

## 目录

1. [背景与问题定义](#1-背景与问题定义)
2. [需求拆解与约束分析](#2-需求拆解与约束分析)
3. [整体架构设计思路](#3-整体架构设计思路)
4. [模块一：Isaac Sim 图像捕获与帧管线](#4-模块一isaac-sim-图像捕获与帧管线)
5. [模块二：PTZ 控制——USD 场景操控](#5-模块二ptz-控制usd-场景操控)
6. [模块三：视频推流——RTSP + MJPEG 双路输出](#6-模块三视频推流rtsp--mjpeg-双路输出)
7. [模块四：进程管理与 GPU 资源释放](#7-模块四进程管理与-gpu-资源释放)
8. [模块五：ONVIF 协议适配](#8-模块五onvif-协议适配)
9. [配置驱动设计与可移植性](#9-配置驱动设计与可移植性)
10. [关键技术决策回顾](#10-关键技术决策回顾)
11. [动手实验](#11-动手实验)

---

## 1. 背景与问题定义

### 1.1 场景来源

工地安防场景：需要在虚拟施工环境中模拟 PTZ 球机的视角画面，用于：
- 算法训练数据采集（任意视角、任意光照、任意对象状态）
- 安防系统集成测试（替代真实摄像头进行联调）
- 培训演示（场景可重复复现）

### 1.2 核心问题

> **如何将 Isaac Sim 仿真渲染的画面，变成一个"标准摄像头"，让外部系统（VMS、NVR、算法平台）像对待真实摄像头一样接入它？**

拆开来说：
1. 画面怎么"拿出来"？—— 图像捕获
2. 画面怎么"传出去"？—— 视频推流协议
3. 摄像头怎么"转方向"？—— PTZ 控制
4. 外部系统怎么"认出它"？—— 协议适配（ONVIF）
5. GPU 怎么按需使用？—— 进程生命周期管理

---

## 2. 需求拆解与约束分析

### 2.1 功能需求

| 需求 | 说明 |
|------|------|
| F1 视频输出 | 支持 RTSP（供 NVR/VLC 接入）和 MJPEG（浏览器低延迟预览） |
| F2 PTZ 控制 | Pan / Tilt / Zoom 实时调节，有角度范围限制 |
| F3 场景联动 | 吊篮高度、工人可见性可通过 API 动态控制 |
| F4 ONVIF 兼容 | 能被主流 VMS/NVR 自动发现并识别为网络摄像机 |
| F5 Web 控制台 | 浏览器可视化操作界面 |

### 2.2 非功能约束

| 约束 | 影响 |
|------|------|
| **GPU 资源宝贵** | Isaac Sim 必须按需启停，空闲时占用接近 0% |
| **Isaac Sim 启动慢** | 冷启动约 60~120 秒，不能频繁重启 |
| **Isaac Sim 内置 Python** | 需用 `python.sh` 执行，无法直接 `pip install` 任意包 |
| **跨进程通信** | 控制 UI 与仿真进程必须解耦 |

> **讨论**：为什么不把控制 UI 和 Isaac Sim 做在同一个进程里？
> → Isaac Sim 崩溃会把 UI 一起带走；且 Isaac Sim 启动慢，UI 应该随时可用。

---

## 3. 整体架构设计思路

### 3.1 分层设计：控制层与渲染层解耦

```
┌─────────────────────────────────────────────┐
│   控制层  ptz_launcher.py  :8080             │
│   始终运行 · 无 GPU 占用 · 快速响应           │
│   职责：Web UI / ONVIF 模拟 / 进程管理        │
└────────────────────┬────────────────────────┘
                     │ HTTP 代理 / 子进程管理
┌────────────────────▼────────────────────────┐
│   渲染层  ptz_stream.py   :8081              │
│   按需启停 · 占用 GPU · Isaac Sim headless   │
│   职责：场景渲染 / PTZ 控制 / 视频推流        │
└────────────────────────────────────────────┘
```

**设计原则**：控制层对用户"始终在线"；渲染层按需拉起，释放时彻底清理 GPU。

### 3.2 端口分配策略

```
8080  ← 外部唯一入口（用户/ONVIF客户端只需记这一个端口）
8081  ← 内部端口（控制层代理，外部不直接访问）
8554  ← RTSP 标准端口（mediamtx）
8888  ← HLS 端口（mediamtx，备用）
```

**设计原则**：外部依赖最小化，只暴露一个端口。内部端口变化不影响外部调用方。

### 3.3 数据流全景

```
Isaac Sim RenderProduct
        │ RGBA numpy array（每帧）
        ▼
  帧缓冲（_mjpeg / ffmpeg stdin）
        ├── Pillow/OpenCV JPEG 编码 → MJPEG HTTP 流 → 浏览器
        └── ffmpeg H.264 编码 → mediamtx RTSP → VLC / NVR
```

---

## 4. 模块一：Isaac Sim 图像捕获与帧管线

### 4.1 Isaac Sim 的启动顺序约束

**关键约束**：`SimulationApp` 必须在所有 `omni.*` 包导入之前启动。

```python
# ptz_stream.py —— 正确的导入顺序
from isaacsim import SimulationApp          # 第一阶段：启动 Kit 运行时

sim_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", ...})

import omni.replicator.core as rep          # 第二阶段：Kit 就绪后才能导入
import omni.usd
from isaacsim.core.api import World
```

> **常见坑**：在 `SimulationApp()` 之前 `import omni.*` 会导致 Kit 初始化失败。

### 4.2 用 Replicator 捕获相机画面

Isaac Sim 的图像捕获使用 `omni.replicator.core`（Replicator）：

```python
# 创建 RenderProduct：指定相机 prim 和分辨率
rp = rep.create.render_product(camera_prim, (W, H))

# 挂载 RGB Annotator（每帧输出 RGBA numpy 数组）
rgb = rep.AnnotatorRegistry.get_annotator("rgb")
rgb.attach([rp])

# 在仿真主循环中获取帧
sim_app.update()          # 推进一步仿真
data = rgb.get_data()     # 获取 RGBA uint8 array，shape=(H, W, 4)
```

### 4.3 帧率控制：仿真 Hz vs 推流 fps

```python
sim_hz    = 60    # 仿真步进频率
fps       = 25    # 推流帧率
skip_frames = round(sim_hz / fps)   # 每 skip_frames 步推一帧
```

**为什么不把仿真频率直接设为 25Hz？**  
→ 仿真物理精度依赖较高的步进频率；推流帧率是独立需求，两者应解耦。

---

## 5. 模块二：PTZ 控制——USD 场景操控

### 5.1 PTZ 场景层级设计

```
/World/CameraRig          ← Pan 节点（rotateZ）
    └── /CamTilt          ← Tilt 节点（rotateY）
        └── /Camera       ← 渲染相机（focalLength = zoom）
```

**为什么用三层 Xform 而不是直接操作 Camera？**  
→ USD 的 xformOp 是叠加的，三层分离使 Pan/Tilt/Zoom 互不干扰，逻辑清晰。

### 5.2 PTZ 状态写入 USD Stage

```python
def _apply_ptz_state(stage):
    pan_p.GetAttribute("xformOp:rotateZ").Set(float(pan_deg))   # Pan
    tilt_p.GetAttribute("xformOp:rotateY").Set(float(tilt_deg)) # Tilt
    cam_p.GetAttribute("focalLength").Set(FOCAL_LENGTH_1X * zoom)  # Zoom
```

**Zoom 实现原理**：修改相机 `focalLength` 属性模拟光学变焦（焦距 = 基准焦距 × 倍率）。

### 5.3 坐标系适配

| 场景版本 | Up Axis | Pan 轴 | Tilt 轴 |
|---------|---------|--------|--------|
| V4.0（当前） | Z-up | rotateZ | rotateY |
| Y-up 旧场景 | Y-up | rotateY | rotateZ |

代码自动检测场景 `upAxis` 并切换，保证可移植性。

### 5.4 脏标记（Dirty Flag）模式

```python
_ptz_dirty = threading.Event()   # HTTP 收到新控制指令时 set()
# 主循环
if _ptz_dirty.is_set():
    _ptz_dirty.clear()
    _apply_ptz_state(stage)      # 只在有变化时写 USD，避免每帧都写
```

**为什么用 Event 而不是每帧写？**  
→ USD Stage 写操作有开销；用脏标记只在状态变化时才写，减少不必要的 USD I/O。

---

## 6. 模块三：视频推流——RTSP + MJPEG 双路输出

### 6.1 为什么需要两路推流？

| 协议 | 延迟 | 兼容性 | 用途 |
|------|------|-------|------|
| MJPEG（HTTP） | ~100-300ms | 浏览器原生支持 | Web 控制台预览 |
| RTSP（H.264） | ~500ms-1s | NVR/VLC 标准接口 | 外部系统接入 |

### 6.2 RTSP 推流链路

```
Isaac Sim RGBA numpy
    │
    │ stdin pipe（原始 BGR24 数据）
    ▼
ffmpeg（H.264 编码，yuv420p，zero-latency）
    │ RTSP ANNOUNCE
    ▼
mediamtx（RTSP Server :8554）
    │ RTSP DESCRIBE/PLAY
    ▼
VLC / NVR
```

**为什么用 mediamtx 而不是让 ffmpeg 直接做 RTSP 服务器？**  
→ ffmpeg 作为 RTSP Server 并发能力弱；mediamtx 专为多路 RTSP 分发设计，支持多客户端同时拉流。

### 6.3 MJPEG 推流实现

MJPEG 本质是用 HTTP `multipart/x-mixed-replace` 持续发送 JPEG 帧：

```python
# HTTP Response 头
"Content-Type: multipart/x-mixed-replace; boundary=frame"

# 每帧
"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: N\r\n\r\n<jpeg bytes>"
```

**JPEG 编码器优先级**：Pillow → OpenCV → 不可用（日志明确告知）。

### 6.4 ffmpeg 关键参数解析

```bash
ffmpeg -f rawvideo -pixel_format bgr24 -video_size 1920x1080 -framerate 25
       -i pipe:0                          # 从 stdin 读原始帧
       -c:v libx264                       # H.264 编码
       -pix_fmt yuv420p                   # 兼容性最好的色彩格式
       -tune zerolatency                  # 最小化编码延迟
       -b:v 4M                            # 码率
       -f rtsp rtsp://localhost:8554/ptz_cam
```

---

## 7. 模块四：进程管理与 GPU 资源释放

### 7.1 进程组（Process Group）机制

Isaac Sim 启动链：`ptz_launcher.py` → `bash` → `python.sh` → `ptz_stream.py` → `ffmpeg` + `mediamtx`

**问题**：直接 `proc.kill()` 只杀 bash 壳，子进程（ffmpeg/mediamtx）继续占用 GPU。

**解法**：使用 `start_new_session=True` 为整个启动链创建独立进程组，停止时用 `os.killpg()` 一次性杀死整组：

```python
# 启动时
_isaac_proc = subprocess.Popen(
    cmd,
    start_new_session=True,   # 创建新进程组，pgid = 子进程 pid
)

# 停止时
pgid = os.getpgid(_isaac_proc.pid)
os.killpg(pgid, signal.SIGTERM)   # 优雅退出
# 等待 15 秒端口释放
os.killpg(pgid, signal.SIGKILL)   # 强制清理
```

### 7.2 状态机：4 个状态

```
stopped ──[/start]──→ starting ──[HTTP 就绪]──→ running
   ↑                                                │
   └──[进程退出]──── stopping ←──[/stop]────────────┘
```

**为什么需要 `starting` 状态？**  
→ Isaac Sim 启动需要 60~120 秒，这段时间 Web UI 要显示进度，不能返回 `stopped` 也不能说 `running`。

### 7.3 就绪检测

```python
def _is_isaac_http_ready():
    # 轮询 :8081/status，成功则进入 running
    urllib.request.urlopen(f"http://localhost:{ISAAC_PORT}/status", timeout=1)
```

后台线程每秒检测一次，最多等待 300 秒（5 分钟），超时后认为已启动（防止极端场景卡死）。

---

## 8. 模块五：ONVIF 协议适配

### 8.1 什么是 ONVIF？为什么需要它？

ONVIF（Open Network Video Interface Forum）是安防行业标准协议，主流 NVR、VMS 通过 ONVIF 自动发现和控制摄像头。

实现 ONVIF 后，虚拟摄像头在海康 VMS 等系统中**与真实摄像头无区别**。

### 8.2 ONVIF 本质：SOAP over HTTP

```
客户端（VMS）  ──POST /onvif/ptz_service──→  我们的服务器
              Body: XML SOAP 请求
              ←── XML SOAP 响应 ──
```

### 8.3 我们实现了哪些 ONVIF 操作？

| 服务端点 | 支持操作 |
|---------|---------|
| `/onvif/device_service` | GetCapabilities, GetSystemDateAndTime, GetServices |
| `/onvif/media_service` | GetProfiles, GetSnapshotUri, GetStreamUri |
| `/onvif/ptz_service` | AbsoluteMove, RelativeMove, GotoPreset, GetStatus, GetNodes |

### 8.4 坐标归一化转换

ONVIF 使用归一化坐标 `[-1, 1]`，Camera03 使用角度/倍率：

```python
pan_deg  = onvif_pan  * 170.0                       # [-1,1] → [-170°, 170°]
tilt_deg = clamp(onvif_tilt * 90.0, -90.0, 30.0)   # [-1,1] → [-90°,  30°]
zoom_x   = 1.0 + onvif_zoom * 31.0                  # [0,1]  → [1×,   32×]
```

### 8.5 SOAP 版本兼容处理

```python
# 自动检测请求使用的 SOAP 版本并返回对应版本的响应
if "application/soap+xml" in content_type:
    s_ns = "http://www.w3.org/2003/05/soap-envelope"   # SOAP 1.2
else:
    s_ns = "http://schemas.xmlsoap.org/soap/envelope/"  # SOAP 1.1
```

**为什么要兼容两个版本？**  
→ 不同厂商的 VMS 实现不同：中文工具多用 SOAP 1.1，.NET/海外工具多用 SOAP 1.2。

---

## 9. 配置驱动设计与可移植性

### 9.1 配置文件职责

`ptz_config.yaml` 集中管理所有"环境相关"的参数，代码不硬编码路径和端口：

```yaml
python_sh:    /path/to/isaac/python.sh   # Isaac Sim 解释器路径
scene_path:   ./V4.0.usd                 # 场景文件路径
camera_prim:  /World/CameraRig/CamTilt/Camera  # 相机层级路径
resolution:   [1920, 1080]
fps:          25
rtsp_enabled: true                        # 关闭可跳过 ffmpeg/mediamtx
```

### 9.2 移植到新工程的最小步骤

1. 复制 5 个核心文件（launcher、stream、config、html、mediamtx.yml）
2. 修改 `ptz_config.yaml` 中 `python_sh`、`scene_path`、`camera_prim`
3. 如果场景没有 PTZ 层级，运行一次建层工具：
   ```bash
   python3 setup_camera_v4.py --scene ./YourScene.usd --cam /World/Camera
   ```
4. 启动：`python3 ptz_launcher.py`

### 9.3 rtsp_enabled 开关的价值

```yaml
rtsp_enabled: false  # 不启动 ffmpeg 和 mediamtx，仅提供 MJPEG 和快照
```

快速调试场景时关闭 RTSP，节省启动时间和依赖。

---

## 10. 关键技术决策回顾

| 决策 | 选择 | 为什么不用其他方案 |
|------|------|-----------------|
| 进程隔离 | Launcher + Stream 双进程 | 单进程：Isaac Sim 崩溃带走 UI；UI 应始终可用 |
| GPU 释放 | `start_new_session` + `killpg` | 只 kill 主进程：ffmpeg/mediamtx 子进程残留，GPU 无法释放 |
| 图像捕获 | Replicator RenderProduct | Isaac Sim 官方推荐方式，性能最优，支持多路 |
| RTSP Server | mediamtx | ffmpeg 做 RTSP Server 并发差；mediamtx 轻量且支持多客户端 |
| PTZ 实现 | 修改 USD xformOp 属性 | USD Stage 是场景唯一事实来源，直接写属性最简洁 |
| Zoom 实现 | 修改 Camera focalLength | 模拟真实光学变焦，与 Isaac Sim 渲染管线天然一致 |
| ONVIF | 纯 Python SOAP 手写实现 | 依赖 `python-onvif-zeep` 包需要 pip 安装，Isaac Sim 环境受限；手写轻量可控 |

---

## 11. 动手实验

### 实验一：跑通基础链路（30 分钟）

```bash
cd /home/uniubi/xuanyuan/Camera03
python3 ptz_launcher.py
# 浏览器打开 http://localhost:8080
# 点击"开始推流"，等待启动完成
# 用 VLC 打开 rtsp://localhost:8554/ptz_cam 验证画面
```

### 实验二：PTZ 控制 API（20 分钟）

```bash
# 调整 Pan/Tilt/Zoom
curl -X POST http://localhost:8080/control \
  -H "Content-Type: application/json" \
  -d '{"pan": 45.0, "tilt": -30.0, "zoom": 4.0}'

# 查询当前状态
curl http://localhost:8080/status
```

### 实验三：场景联动（20 分钟）

```bash
# 吊篮上升到 1500cm
curl -X POST http://localhost:8080/scene/gondola \
  -H "Content-Type: application/json" \
  -d '{"y": 1500}'

# 显示1个工人
curl -X POST http://localhost:8080/scene/workers \
  -H "Content-Type: application/json" \
  -d '{"count": 1}'

# 查询场景状态
curl http://localhost:8080/scene/state
```

### 实验四：理解进程清理（15 分钟）

```bash
# 启动后查看进程组
ps -eo pid,pgid,cmd | grep -E "ptz|ffmpeg|mediamtx"

# 停止推流
curl -X POST http://localhost:8080/stop

# 再次查看，确认所有相关进程已退出
ps -eo pid,pgid,cmd | grep -E "ptz|ffmpeg|mediamtx"
```

### 实验五：移植练习（60 分钟，高阶）

将系统移植到另一个 USD 场景：
1. 准备一个有 Camera 的 USD 场景
2. 运行 `setup_camera_v4.py` 为场景添加 PTZ 控制层级
3. 修改 `ptz_config.yaml` 的 `scene_path` 和 `camera_prim`
4. 启动验证

---

## 附录：常见问题

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| 点击"开始推流"后长时间 `starting` | Isaac Sim 冷启动慢 | 正常现象，等待 60~120 秒；查看 `/log` 接口确认进度 |
| VLC 无法拉流 | mediamtx 未启动或 ffmpeg 崩溃 | 检查 `isaac_stream.log`；确认 `rtsp_enabled: true` |
| 停止后 GPU 内存未释放 | 进程组未完全清理 | 查看 `ps -ef | grep python`；手动 `kill -9 <pgid>` |
| ONVIF 设备被 VMS 识别但无画面 | GetStreamUri 返回地址不可达 | 检查 mediamtx 端口 8554 是否防火墙放行 |
| PTZ 控制无效 | camera_prim 路径错误 | 用 Isaac Sim Stage 面板确认 prim 路径，更新配置文件 |

---

*文档版本：2026-03-24 | Camera03 v4.0*
