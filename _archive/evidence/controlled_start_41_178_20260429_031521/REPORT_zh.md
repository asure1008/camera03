# 41.178 单次受控 POST /start 启动 Isaac 验证 — 中文报告

## 1. 本轮是否修改文件

**否。** 未修改任何业务文件、配置、代码或日志（仅在本机证据目录内写入采集文件与本文档）。

## 2. 是否发送 POST

- **POST /start：是，且仅 1 次**（`curl -X POST http://127.0.0.1:8080/start`，`--max-time 180`）。
- **POST randomize 等：否。** 未调用 `/api/scene/randomize`、`/api/scene/randomize/last` 或其它 POST/PUT/PATCH/DELETE。

## 3. stop / restart / kill / nohup

- **stop：否**
- **restart：否**
- **kill / pkill / killall / xkill：否**
- **nohup：否**
- **start：是**，仅通过 launcher 的 **POST /start** 触发 Isaac 子进程拉起逻辑。

## 4. POST /start 返回结果

- **HTTP 状态码：200**（见 `G_post_start.txt` 中 `CURL_HTTP_CODE:200`）。
- **curl 退出码：0**（`CURL_EXIT_CODE=0`）。
- **耗时：** 约 `CURL_TIME_TOTAL:9.45` 秒；`ELAPSED_SEC=10`。
- **响应体摘要：** JSON 中 `ok: false`，`error` 为「Isaac 子进程在就绪等待期退出 exit_code=0」，`action_taken: spawn_ready_timeout`，`ready_wait_s` 约 9.39s。`isaac_log_tail` 指向 **`FileNotFoundError: ... 'ffmpeg'`**（`ptz_stream.py` 中 `_start_ffmpeg` / `_launch` 调用 `subprocess.Popen` 时找不到可执行文件 `ffmpeg`）。
- **说明：** `G_post_start.txt` 首行出现 `bash: -c: option requires an argument`，为远端 shell 包装时的无害噪声；紧随其后的 JSON 为有效 POST 响应。

## 5. 8080 / 8081 / 8554 启动前后状态

| 阶段 | 8080 | 8081 | 8554 |
|------|------|------|------|
| 启动前（`B_before_ports_processes.txt`） | 监听（`python3` launcher） | 未在 `ss` 中出现 | 未在 `ss` 中出现 |
| 启动后终态（`H_after_ports_processes.txt`） | 仍监听（launcher） | 仍未监听 | **监听**（`mediamtx`，`camera03/mediamtx.yml`，子进程由本次启动流程拉起后残留） |

**8081：** 终态仍未监听；Isaac 控制面未就绪。

## 6. `/api/health` 结果摘要（`I_get_health.json`）

- Launcher 层：`ok: true`，8080 正常。
- Isaac：`isaac_process_running: false`，`isaac_port_listening: false`，`isaac_state: "stopped"`，`ctrl_probe` 中 `health_err: "skipped_port_free"`（8081 未起）。

## 7. `/status` 结果摘要（`J_get_status.json`）

- `isaac_state: "stopped"`，`isaac_port_listening: false`，`uptime_s: 0`。
- `config_path` 与 `stream_config.scene_path` 等仍为 `camera03` 下磁盘配置，无本轮写盘变更。

## 8. `/api/scene/random-config` 是否可用（`K_get_random_config.json`）

**不可用（业务语义未就绪）：** `{"ok": false, "error": "Isaac Sim 未就绪"}`。

## 9. 失败判定与日志依据（未自动修复）

- **失败点：** Isaac 子进程在就绪等待期内退出；根本错误为 **`ffmpeg` 不在 PATH / 不可执行**（`isaac_stream.log` 尾部与 POST 响应内嵌 `isaac_log_tail` 一致）。
- **附带现象：** `scan_diaolan_prims` 报 `/World not found`（可能为阶段加载时序或场景问题，但直接导致退出的是 **`FileNotFoundError: 'ffmpeg'`**）。
- **8554：** `mediamtx` 仍在监听，与「流栈启动过但主进程随后崩溃」一致；**未**自动清理进程或改配置。

## 10. 若视为「成功」时的下一步（本轮实际为失败，此处仅作预案）

本轮**不成功**，故**不应**进入「Web + snapshot/RTSP 只读验证」。待 **`ffmpeg` 可用**（或等价满足 `ptz_stream` 启动 RTSP 栈的前置条件）且 launcher 策略允许再次 **POST /start** 后，再安排该轮验证；**不得**自动 randomize。

---

**证据文件清单：** `A_context.txt` … `M_gpu_after.txt`，`G_post_start.txt`，`P_poll_180s.txt`（180s 轮询日志），`REPORT_zh.md`。
