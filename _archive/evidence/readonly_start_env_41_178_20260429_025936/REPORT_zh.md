# camera03 @ 192.168.41.178 只读排查报告

**采集时间戳目录**：`readonly_start_env_41_178_20260429_025936`  
**采集端**：192.168.46.144（本机写入证据）  
**目标端**：192.168.41.178（经 ssh 只读命令）

---

## 1. 本轮是否修改文件

**否。** 仅在 46.144 上新建证据目录并追加采集文件；未对 41.178 或本仓库业务文件做任何修改。

---

## 2. kill / stop / start / nohup / POST 等约束执行情况

| 约束项 | 本轮是否执行 |
|--------|----------------|
| kill / pkill / killall / xkill | **否** |
| stop / start / restart / nohup | **否**（`ps` 中可见历史 `nohup` 为机上既有进程，非本轮启动） |
| POST / PUT / PATCH / DELETE | **否**（`curl` 仅 **GET**） |
| rsync / scp / cp / mv / rm / mkdir（目标机） | **否**（本机为落证据目录曾 `mkdir -p`，未改目标机） |
| conda / pip / apt 安装 | **否** |
| 改代码 / 改配置 | **否** |

---

## 3. 8080 / 8081 / 8554 当前状态（41.178）

- **8080**：`LISTEN`，进程为 `python3 ptz_launcher.py --config ./ptz_config.yaml`（launcher 正常在跑）。
- **8081**：`ss -ltnp | grep` **无**匹配 → **未监听**（与 Isaac 子进程未起来一致）。
- **8554**：同上 **无**匹配 → **未监听**（RTSP 未起；与 Isaac 未运行一致）。

---

## 4. `/api/health`、`/status`、`/api/scene/random-config`（GET）

- **`GET /api/health`**：返回 JSON，`ok: true`；`isaac_state` 相关为 **stopped**，8081 未就绪（`skipped_port_free` 等字段与端口空闲一致）。
- **`GET /status`**：`isaac_state: stopped`；`stream_config.scene_path` 已为  
  `/home/uniubi/xuanyuan/camera03/scene_4diaolan_ptz.usda`（与已知修改一致）。
- **`GET /api/scene/random-config`**：`{"ok": false, "error": "Isaac Sim 未就绪"}`（因 8081 无控制面）。

---

## 5. `/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3` 是否存在

**不存在。**  
`ls` 报错 `No such file or directory`；`/home/uniubi/miniconda3/envs/` 下 **仅有空目录结构、无任何已创建 env 子目录**（`find` 未找到任何 `envs/.../python3`）。

---

## 6. 41.178 当前 conda 环境列表

`conda env list` 显示 **仅有**：

- `base` → `/home/uniubi/miniconda3`

**不存在**名为 `env_isaaclab` 的环境。

---

## 7. 当前可用的 python3 路径（证据内）

- `which python3` → **`/usr/bin/python3`**，`Python 3.12.3`
- `/home/uniubi/miniconda3/bin/python3` → **`Python 3.13.12`**（base 解释器存在）

---

## 8. `env_isaaclab` 路径来源（文件与行号）

在目录 `/home/uniubi/xuanyuan/camera03` 内检索（见 `D_grep_camera03.txt`），**直接决定 launcher 所用解释器** 的为：

| 文件 | 行号 | 内容要点 |
|------|------|----------|
| **`ptz_config.yaml`** | **27** | `python_sh: /home/uniubi/miniconda3/envs/env_isaaclab/bin/python3` |

其它脚本/备份中为同类硬编码或默认路径引用（如 `run_*`、`start_safe.sh`、`generate_scene_usd.py` 等），**与本次 `/start` 失败同一配置键同源**。

`ptz_launcher.py` 顶部（约 69–71 行）从 YAML 读取：

- `PYTHON_SH = cfg.get("python_sh", "/home/uniubi/projects/issac/.isaac_sim_unzip/python.sh")`

因 YAML 已提供 `python_sh`，故 **不会**走代码里的 `issac` 解压默认路径。

---

## 9. `ptz_launcher.py` 的 `/start` 如何构造子进程命令

路由 `POST /start` 调用 `start_isaac()`（见 `ptz_launcher_index.txt` 约 3214、3231 行附近）。在需 **spawn 新子进程** 时，命令组成为（证据 `ptz_launcher_start_isaac_popen.txt`）：

```text
cmd = [
    PYTHON_SH,                    # 即 cfg["python_sh"]，当前为 conda env_isaaclab 的 python3 绝对路径
    "-u",
    STREAM_SCRIPT,                # .../camera03/ptz_stream.py
    "--config", config_abs,       # ptz_config.yaml 绝对路径
    "--ctrl-port", str(ISAAC_PORT),
]
# 可选：环境变量 PTZ_SCENE_OVERRIDE / PTZ_CAMERA_PRIM_OVERRIDE
# 若从磁盘读到 scene_path，再追加：["--scene", sc["scene_path"]]

subprocess.Popen(cmd, stdout/stderr→isaac_stream.log, cwd=script_dir,
                 start_new_session=True, env=_env_for_ptz_stream_subprocess())
```

`_env_for_ptz_stream_subprocess()` 会用 **`os.path.abspath(PYTHON_SH)`** 推导相邻 `lib` 并拼到 `LD_LIBRARY_PATH`。  
当 `PYTHON_SH` 指向 **不存在的文件** 时，`Popen` 在创建子进程阶段即抛出 **`FileNotFoundError`**，与日志一致。

---

## 10. 41.178 上 Isaac / Kit / `python.sh` 可疑可用路径（未启动）

`find` 摘要（`F_isaac_kit_paths.txt` + 补充 `G_fallback_paths.txt`）：

- **存在且可执行**：`/home/uniubi/xuanyuan/python.sh`、`/home/uniubi/xuanyuan/kit/python.sh`；另有 `/home/uniubi/siwei/isaac-sim/` 下 `python.sh`、`kit/kit` 等。
- **代码默认回退路径不存在**：`/home/uniubi/projects/issac/.isaac_sim_unzip/python.sh` → **No such file**（与 workspace 规则中「issac 在 ~/projects/issac」不一致，**178 上该路径未部署**）。

---

## 11. 日志中最近一次 `/start` 失败栈（摘录）

`ptz_launcher.log` 中（`E_ptz_launcher_log.txt`）：

- 行 348：`========== 本次 Isaac 子进程将加载（ptz_stream 读盘）==========`
- 随后 **Traceback** → `subprocess` → **`FileNotFoundError: ... '/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3'`**

与「配置把 `python_sh` 指到不存在的 conda env」**完全吻合**。客户端 `POST /start` 若遇 worker 线程异常，可能出现 **Empty reply** 类现象（本轮未复现 POST）。

---

## 12. 下一轮最小修复建议（本轮不执行）

1. **若优先改配置（最小动配置）**  
   - 修改 **`ptz_config.yaml` 中的配置键 `python_sh`**，指向 **真实存在** 且能加载 Isaac/omni 栈的解释器或包装脚本，例如（需下一轮在机上验证后再定稿）：  
     - 若采用本机已解压 Isaac：`/home/uniubi/xuanyuan/python.sh` 或 `/home/uniubi/xuanyuan/kit/python.sh`（与 `ptz_stream.py` 文档中 kit 用法一致）；或  
     - 若采用 siwei 安装树：`/home/uniubi/siwei/isaac-sim/python.sh`。  
   - **注意**：仅用系统 `/usr/bin/python3` 通常 **无法** 直接运行 `ptz_stream.py`（缺 `omni` 等），除非另有完整运行时注入。

2. **若坚持 conda 方案**  
   - 在 41.178 上 **创建/恢复** 名为 `env_isaaclab` 的 conda 环境，使  
     `/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3` **存在**，并与 Isaac 版本匹配；**或** 将 `python_sh` 改为实际 env 名称对应路径。

3. **若考虑代码层默认路径**  
   - `ptz_launcher.py` 中 `cfg.get("python_sh", ...)` 的默认值当前为 `.../projects/issac/...`；178 上该文件 **不存在**，故仅改代码默认值 **不能**解决当前机已用 YAML 覆盖的问题；**根本仍是 `python_sh` 与机况一致**。

4. **`projects/issac` 部署缺口**  
   - 若规范要求使用 `~/projects/issac`，需在 178 上 **部署该目录** 或改配置指向实际 `python.sh`，与本轮证据一致。

---

**证据文件清单**：`A_ports_health.txt`、`B_conda_env_paths.txt`、`C_system_python.txt`、`D_grep_camera03.txt`、`E_ptz_launcher_log.txt`、`F_isaac_kit_paths.txt`、`G_fallback_paths.txt`、`ptz_launcher_head.txt`、`ptz_launcher_index.txt`、`ptz_launcher_start_isaac_popen.txt`、本 `REPORT_zh.md`。
