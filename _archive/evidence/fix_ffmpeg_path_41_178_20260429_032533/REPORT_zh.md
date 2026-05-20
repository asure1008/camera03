# 41.178 ffmpeg PATH 最小修复 — 执行报告

## 1. 证据目录完整路径

`/home/uniubi/xuanyuan/camera05/camera03/_archive/evidence/fix_ffmpeg_path_41_178_20260429_032533`

## 2. 本轮是否修改文件

是。已修改目标机 **192.168.41.178** 上 `/home/uniubi/xuanyuan/camera03/ptz_launcher.py`（磁盘内容）。本机 camera05 仓库内其它路径未改，仅证据目录保留备份与 diff。

## 3. 修改了哪些文件

- `ptz_launcher.py`（仅上述远程路径）

## 4. 每个文件具体改了什么

在 `_env_for_ptz_stream_subprocess()` 末尾、`return env` 之前增加：

- 常量：`ffmpeg_dir = "/home/uniubi/miniconda3/bin"`
- 将 `ffmpeg_dir` **前置**拼接到 `env["PATH"]`，保留原有 `PATH` 片段（若原 `PATH` 为空则仅为 `ffmpeg_dir`）

未改动 scene_path、随机化、Web、RTSP 参数、mediamtx 配置等其它逻辑。

## 5. 备份文件路径

- `ptz_launcher.py.before`（修复前从 41.178 拉取的完整备份）
- `ptz_launcher.py.patched`（与当前远程磁盘文件一致的可审计副本）
- `ptz_launcher.diff`（`diff -u` 相对 `.before`）
- `phase1_readonly.txt`、`health_pre_start.json`、`post_start_response.json`、`poll_final.txt`、`poll_90s.txt`（POST /start 之后 90 秒内每 5 秒一轮的 ss/health/status 记录）、`log_ffmpeg_errors_tail.txt`、`path_parent_shell_hint.txt`

## 6. 是否执行 install / download / compile

否。未执行 `apt install` / `pip install` / `conda install` / 下载 / 编译。

## 7. 是否执行 stop / start / restart / kill / nohup

- 未对系统或进程执行 `kill` / `pkill` / `killall`。
- 未手动 `nohup` 重启 launcher、未 `systemctl` 类重启。
- 未对 launcher 进程做「停止再拉起」的 shell 级操作。
- 仅通过 HTTP 调用 launcher 的 **`POST /start` 一次**（见下条；属受控 API，非本机对 launcher 的二进制重启）。

## 8. 是否发送 `POST /start`，次数

是，**1 次**：`curl -sS -X POST http://127.0.0.1:8080/start`。  
按约束未发送第二次（首次返回失败后已停止）。

## 9. 是否发送 `POST randomize`

否。未调用 `/api/scene/randomize` 或相关 randomize 接口。

## 10. 是否触发 PTZ / preset / snapshot / RTSP

否。未做 PTZ、preset、snapshot、RTSP 画面验证。

## 11. `ffmpeg -version` 是否成功

是。在 41.178 上执行 `/home/uniubi/miniconda3/bin/ffmpeg -version` 成功（见 `phase1_readonly.txt`）。

## 12. PATH 修复前后对比

| 阶段 | 说明 |
|------|------|
| **修复前（子进程实际继承）** | launcher 子进程环境由 `_env_for_ptz_stream_subprocess()` 基于 `os.environ.copy()` 构造；未改 PATH 时与父进程一致，**不含** `/home/uniubi/miniconda3/bin`。登录 shell 的 `PATH` 示例见 `path_parent_shell_hint.txt`（无 miniconda）。 |
| **修复后（磁盘代码）** | 在上述函数内显式设置 `env["PATH"] = ffmpeg_dir + os.pathsep + 原PATH`，使 `ffmpeg` 裸调用可被解析。 |

**重要说明**：当前 **8080 上的 `python3 ptz_launcher.py` 进程**（例如 pid=560720）在启动时已加载旧版函数对象；**仅替换磁盘 `.py` 不会自动让长驻进程重新加载该函数**。因此单次 `POST /start` 仍可能走内存中的旧 `_env_for_ptz_stream_subprocess()`，与本次观测一致。

## 13. `py_compile` 是否通过

是。远程执行：`python3 -m py_compile ptz_launcher.py` → `PY_COMPILE_OK`。

## 14. 8080 / 8081 / 8554 最终状态（验证尾声）

- **8080**：监听（launcher）。
- **8081**：未监听（Isaac/ptz_stream 未保持运行）。
- **8554**：监听（已有 mediamtx；未做清理；未报告端口冲突阻断 launcher）。

## 15. health / status 最终摘要

- **health**：`launcher` 正常；`isaac_state` 为 **stopped**；8081 未监听、Isaac 相关 ready 均为 false。
- **status**：`isaac_state: stopped`，`isaac_port_listening: false`，与上一致。

## 16. 是否启动成功

**否**（相对任务成功条件）。  
单次 `POST /start` 后子进程仍报 **`FileNotFoundError: ... 'ffmpeg'`**（见 `post_start_response.json` 内 `isaac_log_tail` 与 `log_ffmpeg_errors_tail.txt`）。  
磁盘上的 PATH 修复已生效于**新启动的 launcher 进程**加载代码之后；当前长驻 launcher 需重新加载进程方可使该次修复参与子进程 `env`。

## 17. 若失败，下一步最小建议

1. **在运维策略允许的前提下，重启 8080 上的 `ptz_launcher.py` 进程**（使解释器重新从磁盘加载 `ptz_launcher.py`，从而使用新的 `_env_for_ptz_stream_subprocess()`）。重启方式取决于现有部署（例如交接窗口内的服务脚本 / systemd），**本回合未使用 kill**。
2. 重启 launcher 后，**最多再发 1 次** `POST /start`（仍遵守「不 randomize、不做 RTSP/PTZ 验证」等约束即可）复查 8081 与日志中是否仍出现 `'ffmpeg'` 找不到。
3. 若重启 launcher 后仍失败，再单独排查 `ptz_stream.py` 内是否还有不经过该 `env` 的 `ffmpeg` 调用路径（本次按约束未改 `ptz_stream.py`）。
