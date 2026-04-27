# PTZ 仿真 ONVIF 服务 — API 文档

## 1. 服务角色

| 进程 | 端口 | 说明 |
|------|------|------|
| **ptz_launcher.py** | 8080 | 始终运行；内置 ONVIF SOAP 服务器 + Web UI；按需启停 Isaac Sim |
| **ptz_stream.py** | 8081 | 按需启动；Isaac Sim 仿真进程；提供内部 HTTP 控制 API |

外部客户端（包括 `onvif_client.py`）只需连接 **8080**，无需感知 8081。

---

## 2. 快速启动

```bash
# 启动 Launcher（始终运行，GPU 占用≈0%）
python3 ptz_launcher.py --config ./ptz_config.yaml

# 通过 Web UI 或 API 启动 Isaac Sim
curl -X POST http://localhost:8080/start

# 停止 Isaac Sim
curl -X POST http://localhost:8080/stop
```

---

## 3. ONVIF 服务接口

### 3.1 服务端点

| 端点 | 方法 | 说明 |
|------|------|------|
| `/onvif/device_service` | POST (SOAP) | ONVIF 设备服务 |
| `/onvif/media_service`  | POST (SOAP) | ONVIF 媒体服务 |
| `/onvif/ptz_service`    | POST (SOAP) | ONVIF PTZ 服务 |
| `/onvif-snap.jpg`       | GET         | 单帧 JPEG 快照（代理 Isaac Sim 内部快照） |

### 3.2 Device Service 支持的动作

| SOAPAction / Body 根元素 | 说明 |
|--------------------------|------|
| `GetSystemDateAndTime` | 返回当前 UTC 时间 |
| `GetCapabilities` | 返回 Media / PTZ 服务地址 |
| `GetServices` | 返回所有服务描述 |
| `GetDeviceInformation` | 返回设备型号（Isaac-Sim / PTZ-Camera-V4） |

### 3.3 Media Service 支持的动作

| SOAPAction / Body 根元素 | 说明 |
|--------------------------|------|
| `GetProfiles` | 返回 Profile_1（含 VideoEncoder + PTZConfiguration） |
| `GetSnapshotUri` | 返回快照地址 `http://<host>:8080/onvif-snap.jpg` |
| `GetServiceCapabilities` | 返回 SnapshotUri=true |

### 3.4 PTZ Service 支持的动作

| SOAPAction / Body 根元素 | 说明 |
|--------------------------|------|
| `AbsoluteMove` | 绝对位置控制（见坐标换算） |
| `RelativeMove` | 相对位移控制 |
| `ContinuousMove` | 连续运动（单步近似） |
| `Stop` | 停止运动（无操作，直接返回） |
| `SetPreset` | 保存当前 PTZ 到预置位（token 1~5） |
| `GotoPreset` | 跳转到预置位（token 1~5） |
| `GetPresets` | 返回当前已保存的预置位 |
| `RemovePreset` | 删除预置位 |
| `GetStatus` | 返回当前 Pan/Tilt/Zoom（ONVIF 坐标系） |
| `GetNodes` | 返回 PTZ 节点能力描述 |
| `GetConfigurations` | 返回 PTZ 配置信息 |

### 3.5 坐标换算

ONVIF 使用归一化坐标，Isaac Sim 使用物理单位（度 / 倍率）。

| 维度 | ONVIF 范围 | Isaac 范围 | 换算公式 |
|------|-----------|-----------|---------|
| Pan（水平）  | [-1, 1]  | [-170°, 170°] | `pan_deg = onvif_pan × 170.0` |
| Tilt（俯仰） | [-1, 1]  | [-90°, +30°]  | `tilt_deg = clamp(-60.0 × onvif_tilt - 30.0, -90, 30)` |
| Zoom（变焦） | [0, 1]   | [1×, 32×]     | `zoom_x = 1.0 + onvif_zoom × 31.0` |

反向换算（GetStatus 输出）：

```
onvif_pan  = pan_deg  / 170.0
onvif_tilt = -(2.0 * (tilt_deg - (-90.0)) / (30.0 - (-90.0)) - 1.0)
onvif_zoom = (zoom_x - 1.0) / 31.0
```

### 3.6 预置位

| Token | 名称 | Pan | Tilt | Zoom |
|-------|------|-----|------|------|
| `1` | 默认位   | -56°  | 9°   | 1× |
| `2` | 水平正视 | 0°    |   0° | 1× |
| `3` | 左90°    | -90°  | -45° | 1× |
| `4` | 右90°    | +90°  | -45° | 1× |
| `5` | 俯视     | 0°    |  30° | 1× |

预置位持久化在 `ptz_config.yaml` 的 `presets` 段；Web 保存与 ONVIF `SetPreset` 使用同一份数据。

### 3.7 ONVIF 客户端连接示例

```python
# 使用 onvif-zeep 客户端
from onvif import ONVIFCamera

cam = ONVIFCamera('localhost', 8080, '', '',
                  wsdl_dir='/path/to/python-onvif-zeep/wsdl/')

# 获取设备信息
device = cam.create_devicemgmt_service()
info = device.GetDeviceInformation()

# PTZ 绝对控制
ptz = cam.create_ptz_service()
ptz.AbsoluteMove({
    'ProfileToken': 'Profile_1',
    'Position': {
        'PanTilt': {'x': 0.5, 'y': -0.5},   # Pan 85°, Tilt -45°
        'Zoom':    {'x': 0.1},                # Zoom 4.1×
    }
})

# 获取快照
import urllib.request
urllib.request.urlretrieve('http://localhost:8080/onvif-snap.jpg', 'snap.jpg')
```

---

## 4. Web UI

| 地址 | 说明 |
|------|------|
| `http://localhost:8080/` | Web 控制面板 |

Web UI 功能：
- 启动 / 停止 Isaac Sim
- 实时查看快照（每 500ms 自动刷新 `/onvif-snap.jpg`）
- Pan / Tilt / Zoom 拖拽控制
- 吊篮高度 + 工人数量控制
- ONVIF 接入信息展示

---

## 5. 启动器管理 API

| 路径 | 方法 | 说明 |
|------|------|------|
| `/status` | GET | 返回 Isaac Sim 状态 + PTZ 当前值 |
| `/start`  | POST | 启动 Isaac Sim |
| `/stop`   | POST | 停止 Isaac Sim |
| `/log`    | GET | 返回 Isaac Sim 启动日志（末尾 8KB） |

`/status` 响应示例：

```json
{
  "isaac_state": "running",
  "isaac_port": 8081,
  "uptime_s": 120,
  "ptz": { "pan": -56.0, "tilt": 9.0, "zoom": 1.0 }
}
```

---

## 6. 场景控制 API（透明代理至 Isaac Sim）

| 路径 | 方法 | Body | 说明 |
|------|------|------|------|
| `/control` | POST | `{"pan":0,"tilt":-45,"zoom":1}` | 直接设置 PTZ（度 / 倍率） |
| `/scene/gondola` | POST | `{"y": 1650}` | 设置吊篮高度（0~3300 cm） |
| `/scene/workers` | POST | `{"count": 2}` | 设置工人数量（0 / 1 / 2） |

---

## 7. 配置文件（ptz_config.yaml）

| 字段 | 默认值 | 说明 |
|------|--------|------|
| `launcher_port` | 8080 | Launcher 监听端口 |
| `ctrl_port` | 8081 | Isaac Sim 内部 HTTP 端口 |
| `python_sh` | — | Isaac Sim Python 可执行文件路径 |
| `scene_path` | `./V4.0.usd` | USD 场景文件路径 |
| `camera_prim` | `/World/CameraRig/CamTilt/Camera` | 相机 Prim 路径 |
| `resolution` | `[1920, 1080]` | 渲染分辨率 |
| `fps` | 25 | 渲染帧率 |
| `focal_length_1x` | 18.14756 | 1× 变焦基准焦距（mm） |

---

## 8. 集成流程

1. 启动 Launcher：`python3 ptz_launcher.py`
2. 调用 `POST /start` 启动 Isaac Sim（首次约需 60~120 秒加载场景）
3. 轮询 `GET /status` 直到 `isaac_state == "running"`
4. ONVIF 客户端连接 `http://<host>:8080/onvif/device_service`，正常调用 ONVIF API

---

## 9. 变更说明

**v4.0（当前版本）**：
- 接入方式由 RTSP + MJPEG 改为标准 **ONVIF**（Profile S）
- 新增内置 ONVIF SOAP 服务器（无需外部 onvif 库）
- 已移除：RTSP 推流（ffmpeg + MediaMTX）、MJPEG 流、HLS
- 配置文件重命名：`ptz_rtsp_config.yaml` → `ptz_config.yaml`
- 流脚本重命名：`ptz_rtsp_stream.py` → `ptz_stream.py`
