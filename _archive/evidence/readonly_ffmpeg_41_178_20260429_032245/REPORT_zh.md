# 只读排查报告：41.178 `ptz_stream` 启动找不到 `ffmpeg`

## 1. 证据目录完整路径

`/home/uniubi/xuanyuan/camera05/camera03/_archive/evidence/readonly_ffmpeg_41_178_20260429_032245`

## 2. 本轮是否修改文件

- **远程 192.168.41.178**：未修改任何业务文件、配置或代码（仅执行只读命令）。
- **本机 camera05**：仅新建上述证据目录及其中采集输出文件与本报告；未改动仓库内业务源码（本任务要求的证据落盘）。

## 3. 是否执行 stop / start / restart / kill / nohup

**否。** 未执行。

## 4. 是否发送 POST / PUT / PATCH / DELETE

**否。** 仅对 `127.0.0.1:8080` 使用 **GET**（见 `08_curl_get.txt`）。

## 5. 是否触发 randomize / PTZ / snapshot / RTSP

**否。** 未调用相关接口或推流操作；仅只读 GET 状态类 URL。

---

## 6. 41.178 上 `ffmpeg` 是否存在

**存在，但不在系统默认 PATH 的常见路径下。**

- 以下路径 **不存在** `ffmpeg`：`/usr/bin/ffmpeg`、`/usr/local/bin/ffmpeg`、`/bin/ffmpeg`、`/snap/bin/ffmpeg`、`/home/uniubi/anaconda3/bin/ffmpeg`、`/home/uniubi/xuanyuan/ffmpeg`、`/home/uniubi/xuanyuan/camera03/ffmpeg`（见 `02_common_paths_ffmpeg.txt`）。
- **`/home/uniubi/miniconda3/bin/ffmpeg` 存在**（`-rwxrwxr-x`，见 `02_common_paths_ffmpeg.txt`）。

## 7. `ffmpeg` 完整路径（若存在）

**`/home/uniubi/miniconda3/bin/ffmpeg`**

## 8. 8080 launcher 进程 `PATH` 是否包含 `ffmpeg` 所在目录

**否。**

- 监听 **8080** 的进程：`python3`，**PID=560720**，`cmdline`：`python3 ptz_launcher.py --config ./ptz_config.yaml`（见 `03_launcher_proc_8080.txt`）。
- `/proc/560720/environ` 中 **PATH** 为：  
  `PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin`  
  **不包含** `/home/uniubi/miniconda3/bin`（见 `03_launcher_proc_8080.txt`、`03b_proc560720_environ_selected.txt`）。
- 同机 **`python3 -c`** 默认环境：`shutil.which("ffmpeg")` 为 **`None`**，`PATH` 与上一致（见 `09_python3_readonly_which.txt`）。

## 9. `ptz_stream.py` 调用裸 `ffmpeg` 还是绝对路径

**裸命令名 `"ffmpeg"`**（依赖 `PATH` 解析）。

- `_get_ffmpeg_version()`：`subprocess.check_output(["ffmpeg", "-version"], ...)`（见 `11_ptz_stream_ffmpeg_snippet.txt`）。
- `_build_nvenc_cmd` / `_build_x264_cmd`：命令列表首元素为 **`"ffmpeg"`**（同上文件）。

## 10. `python.sh` 是否影响 `PATH`

**未在该脚本中把 Miniconda 的 `bin` 加入 `PATH`。**

- 文件存在：`/home/uniubi/xuanyuan/python.sh`，`-rwxr-xr-x`（见 `06_python_sh.txt`）。
- 内容摘要：`export CARB_APP_PATH` / `ISAAC_PATH` / `EXP_PATH`，`source ${SCRIPT_DIR}/setup_python_env.sh`，选用 **`${SCRIPT_DIR}/kit/python/bin/python3`**；若存在 **`CONDA_PREFIX`** 仅打印警告，**无**将 `miniconda3/bin` 写入 `PATH` 的逻辑（见 `06_python_sh.txt`）。

## 11. 8080 / 8081 / 8554 当前状态（只读 `ss` / `ps`）

- **8080**：监听，`python3` pid=560720（`ptz_launcher`）（见 `07_ss_ps.txt`）。
- **8081**：**未**在 `ss -lntp | grep` 结果中出现（与 health 中 Isaac 未运行一致）（见 `07_ss_ps.txt`、`08_curl_get.txt`）。
- **8554**：监听，`mediamtx` pid=564456（见 `07_ss_ps.txt`）。

## 12. GET 结果摘要（`08_curl_get.txt`）

| URL | 摘要 |
|-----|------|
| `/api/health` | `ok: true`；**Isaac 未运行**（`isaac_process_running: false`，`isaac_state: stopped` 等） |
| `/status` | `isaac_state: stopped`；`config_path` 指向 `/home/uniubi/xuanyuan/camera03/ptz_config.yaml` |
| `/api/scene/random-config` | `ok: false`，**`error: Isaac Sim 未就绪`** |

## 13. `ptz_config.yaml` 中与 ffmpeg / 启动相关（`04_ptz_config_grep.txt`）

- **无** `ffmpeg` 关键字行。
- **`python_sh: /home/uniubi/xuanyuan/python.sh`**（Isaac 子进程用，不解决系统 `PATH` 中裸 `ffmpeg`）。
- 含 **`mediamtx`**、`**rtsp**` 等常规模块（路径 `./mediamtx`，`rtsp_enabled: true` 等）。

## 14. `ptz_launcher.py` 与子进程环境（`12`、`13` 证据）

- Isaac 子进程使用 **`env=_env_for_ptz_stream_subprocess()`**（见 `12_ptz_launcher_env_subprocess_grep.txt`）。
- `_env_for_ptz_stream_subprocess()`：**`env = os.environ.copy()`**，主要处理 IsaacLab 相关键与 **`LD_LIBRARY_PATH`**，**未**向 `PATH` 注入 `miniconda3/bin`（见 `13_ptz_launcher_env_func_snippet.txt`）。

---

## 15. 根因判断（单选对应）

- **A**：系统标准前缀（`/usr/bin` 等）下 **未** 安装 `ffmpeg` —— **成立**（与 `02` 一致）。
- **B**：**`ffmpeg` 已在 Miniconda 中存在，但 launcher / Isaac 子进程继承的 `PATH` 无法解析裸 `ffmpeg`** —— **主因（与现象一致）**。
- **C**：代码硬编码路径错误 —— **不成立**；代码为裸 **`"ffmpeg"`**，问题在解析路径而非写死错误路径。
- **D**：其它 —— 不作为主因；8554 mediamtx 当前在跑，与「找不到 ffmpeg 可执行文件」无直接矛盾。

**结论代号：以 B 为主（A 为并列事实：系统路径无 ffmpeg，唯 conda 有）。**

---

## 16. 下一轮最小修复建议（本轮不执行）

1. **若接受使用现有 Miniconda 内二进制**：在 **启动 `ptz_launcher` 的 shell** 或 **`_env_for_ptz_stream_subprocess()`** 中为子进程 **`PATH` 前置** `/home/uniubi/miniconda3/bin`（或配置项指定 **`ffmpeg` 绝对路径** 并让 `_build_*_cmd` / `_get_ffmpeg_version` 使用）。
2. **若希望与 conda 解耦**：由运维 **`apt` 安装 `ffmpeg`** 到 `/usr/bin`（满足当前 launcher `PATH`），或提供项目内固定二进制并在配置/代码中使用绝对路径。
3. **8554 残留 mediamtx**：当前 **8554 有 mediamtx 监听**；若曾遇端口/发布者冲突，下一轮可单独规划 **停启 mediamtx** 策略；**本轮按要求未处理**。

---

## 17. 证据文件索引

| 文件 | 内容 |
|------|------|
| `01_shell_ffmpeg_resolve.txt` | 登录 shell：`command -v` / `which` / `type` ffmpeg |
| `02_common_paths_ffmpeg.txt` | 常见路径 `ls` 探测 |
| `03_launcher_proc_8080.txt` | 8080 PID、`cwd`、`cmdline`、`PATH` |
| `03b_proc560720_environ_selected.txt` | 选中 environ 键（无 `LD_LIBRARY_PATH`/`CONDA_PREFIX`/`VIRTUAL_ENV`） |
| `04_ptz_config_grep.txt` | `ptz_config.yaml` grep |
| `05_code_grep_stream_launcher.txt` | `ptz_stream.py` / `ptz_launcher.py` grep 行号 |
| `06_python_sh.txt` | `python.sh` 权限与 head/tail |
| `07_ss_ps.txt` | `ss`、`ps` 摘要 |
| `08_curl_get.txt` | GET health/status/random-config |
| `09_python3_readonly_which.txt` | `python3 -c`：`sys.executable`、`shutil.which`、`PATH` |
| `10_git_readonly.txt` | 远程 `camera03` 目录 `git status/diff --stat`（只读） |
| `11_ptz_stream_ffmpeg_snippet.txt` | `_get_ffmpeg_version` / `_build_*` 片段 |
| `12_ptz_launcher_env_subprocess_grep.txt` | `_env_for_ptz_stream_subprocess` 引用处 |
| `13_ptz_launcher_env_func_snippet.txt` | `_env_for_ptz_stream_subprocess` 函数体片段 |

---

*报告生成时间以证据目录名中时间戳为准：`20260429_032245`（本机采集时刻）。*
