# DEPLOY.md

## 1. 部署目标

本目录（camera03）部署完成后，典型进程链为：

- **ptz_launcher.py** 监听 **:8080**（Web / ONVIF 入口与进程管理）
- **ptz_stream.py** 监听 **:8081**（Isaac Sim 侧 HTTP，由 launcher 按需拉起）
- **mediamtx** 提供 **:8554**（RTSP 等，与推流链路配合）

即：**8080（launcher）→ 8081（stream）→ 8554（mediamtx）**。

## 2. 前置条件

- **Isaac Sim** 已在目标机器安装，且可用的 headless/脚本启动方式明确。
- 可用的 **conda** 或 **Python** 环境，且与 Isaac 官方要求版本一致。
- **GPU 驱动**可用（`nvidia-smi` 正常），显存满足场景与分辨率需求。
- **mediamtx** 可执行文件与 **mediamtx.yml** 已就位（路径与权限正确）。
- 端口 **8080、8081、8554**（及文档中涉及的 **8888** 等）在目标环境未被误占用，或已在防火墙/安全组中放行。

## 3. 关键文件

| 文件 | 说明 |
|------|------|
| `ptz_launcher.py` | 常驻 launcher，:8080 |
| `ptz_stream.py` | Isaac 推流与控制逻辑，:8081 |
| `ptz_config.yaml` | 全局配置（端口、场景路径、Python 路径等） |
| `scene_4diaolan_ptz.usda` | 当前主场景 |
| `diaolan_randomizer.py` | 吊篮随机化相关逻辑 |
| `mediamtx.yml` | RTSP 服务配置 |
| `ptz_web_control.html` | Web 控制台页面 |

## 4. `ptz_config.yaml` 需要按机器修改的字段

以下仅说明含义，**部署时请在目标机器上自行编辑**该 YAML（勿将敏感路径提交到错误环境）：

- **python_sh**：目标机器上用于启动 Isaac / `ptz_stream.py` 的 Python 或 `python.sh` 绝对路径。
- **scene_path**：目标机器上 **scene_4diaolan_ptz.usda** 的绝对路径（与子层资产相对关系一致）。
- **onvif_rtsp_host**：对外声明或拉流使用的本机 IP，例如部署机为 `192.168.41.178` 时填该地址；其他机器则填对应网段 IP。
- **rtsp_enabled**（若配置中存在）：控制是否参与 RTSP / mediamtx / ffmpeg 等相关链路；其它布尔或端口类字段**以目标机实际 `ptz_config.yaml` 为准**，勿照搬示例仓库路径。

## 5. 启动命令

工作目录应为本仓库根目录：

```bash
cd /home/uniubi/xuanyuan/camera05/camera03
nohup python3 ptz_launcher.py > ptz_launcher.log 2>&1 &
```

说明：请将 `python3` 替换为与本机 Isaac 环境一致的解释器；**本文档仅给出命令示例，部署时由运维在目标机自行执行**，不在文档生成环节代为执行 `nohup` 或启停进程。

## 6. 健康检查

```bash
ss -ltnp | grep -E ':8080|:8081|:8554'
curl -sS http://127.0.0.1:8080/status
curl -sS http://127.0.0.1:8081/status
```

第二、三条为 **GET**，用于确认 launcher 与 stream 侧 HTTP 是否响应（stream 仅在 Isaac 进程已拉起后有效）。

## 7. 常见问题

- **8080 已被占用**：检查是否已有其他目录下的 launcher 在监听；结合 `pwd` 与进程命令行确认 cwd，**不要盲目结束进程**，确认后再处理。
- **8081 未就绪**：Isaac 冷启动与场景加载较慢，查看 `isaac_stream.log` 或 launcher 日志中的报错与堆栈。
- **GPU 初始化超时**：检查 `nvidia-smi`、Isaac 环境变量、显存是否被其他任务占满。
- **scene_path 错误**：核对 `scene_4diaolan_ptz.usda` 在目标机上的绝对路径及子层引用文件是否一并拷贝。
- **RTSP 不出流**：检查 mediamtx 是否在运行、8554 是否监听，以及 ffmpeg / mediamtx 相关日志中的推流与鉴权错误。
