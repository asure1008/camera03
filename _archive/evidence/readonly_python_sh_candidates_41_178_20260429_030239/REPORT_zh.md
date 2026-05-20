# 只读定版排查报告：192.168.41.178 `python_sh` 候选

## 本轮是否修改文件

**否。** 未在 192.168.41.178 上修改任何业务文件、配置、代码或场景；证据仅写入本机目录  
`/home/uniubi/xuanyuan/camera05/camera03/_archive/evidence/readonly_python_sh_candidates_41_178_20260429_030239/`。

## 是否 kill / stop / start / nohup / POST

| 项 | 是否执行 |
|----|----------|
| kill | 否 |
| stop | 否 |
| start | 否 |
| nohup | 否 |
| POST（及 PUT/PATCH/DELETE） | 否 |

## 8080 / 8081 / 8554 状态（`ss -lntp` 摘要）

| 端口 | 状态 |
|------|------|
| **8080** | 监听 `0.0.0.0:8080`，进程 `python3 ptz_launcher.py --config ./ptz_config.yaml`（pid 554536） |
| **8081** | 本轮输出中**无** LISTEN 行 |
| **8554** | 本轮输出中**无** LISTEN 行 |

详见 `B_ports_processes.txt`。

## 当前 `python_sh` 指向什么、为何失败

- **当前值**（`ptz_config.yaml`）：`/home/uniubi/miniconda3/envs/env_isaaclab/bin/python3`
- **存在性**：**不存在**（`stat` 报 No such file or directory）
- **可执行性**：不适用（路径无效）
- **与运行态关系**：`/api/health` 显示 `isaac_process_running: false`、`isaac_port_listening: false`、`isaac_state: stopped`；与解释器路径无效、无法成功拉起 `ptz_stream` 一致。该路径为 **miniconda 环境 python3**，不符合「Isaac/Kit 的 `python.sh`」入口要求。

详见 `C_current_config.txt`、`F_get_status.txt`。

## 候选 `python.sh` 汇总表

| 路径 | 存在 | 可执行 | realpath | 备注 |
|------|------|--------|----------|------|
| `/home/uniubi/xuanyuan/python.sh` | 是 | 是 | 同左 | NVIDIA Isaac 风格顶层入口；`ISAAC_PATH`/`CARB_APP_PATH` 指向 `SCRIPT_DIR`（xuanyuan 根下 kit） |
| `/home/uniubi/xuanyuan/kit/python.sh` | 是 | 是 | 同左 | Kit 侧入口；`CARB_APP_PATH=${MY_DIR}`（kit 目录） |
| `/home/uniubi/siwei/isaac-sim/python.sh` | 是 | 是 | 同左 | 独立 siwei 安装，与他人环境风险见下 |
| `/home/uniubi/projects/issac/.isaac_sim_unzip/python.sh` | **否** | N/A | — | 与 launcher 代码默认路径一致，但**本机该文件不存在** |
| `find /home/uniubi -maxdepth 4 -name python.sh` 额外命中 | `/home/uniubi/siwei/isaac-sim/kit/python.sh` | 未在本轮逐文件展开 | 同 siwei 树 | 与 siwei 根 `python.sh` 同类 |

完整 `stat`/`head`/`grep` 见 `D_python_sh_candidates.txt`。

## 推荐优先候选（不直接改配置）

1. **首选**：`/home/uniubi/xuanyuan/python.sh`  
   - 位于 **`/home/uniubi/xuanyuan`**，存在且可执行；脚本内容设置 `ISAAC_PATH`、`CARB_APP_PATH=$SCRIPT_DIR/kit`、`EXP_PATH`、`setup_python_env.sh` 及 `kit/python/bin/python3`，符合 **本机 Kit/Isaac Sim 安装** 的标准启动方式。  
   - 符合「优先 xuanyuan 路径、不优先 siwei」的约束。

2. **次选**：`/home/uniubi/xuanyuan/kit/python.sh`  
   - 同属 xuanyuan，为 **kit 目录内**入口；若业务约定始终从 kit 子目录启动，可选用；一般顶层 `python.sh` 更常见。

3. **不推荐作默认**（除非确认 xuanyuan 两套均不可用）：`/home/uniubi/siwei/isaac-sim/python.sh`  
   - 虽存在且为完整 Isaac 脚本，但属 **siwei** 路径，可能涉及他人/共享环境；本轮未验证与 `camera03` 场景版本是否一致。

**明确不推荐**：`/usr/bin/python3`、miniconda **base** 的 `python3` 作为 Isaac 启动解释器（当前配置误用 conda env 的 `python3` 且路径已不存在）。

## 若仍无法确认时的下一轮最小验证

- 在**仍不启动 Isaac 业务**的前提下：对选定候选仅做 `bash -n /path/python.sh` 或 `test -x` + 确认 `kit/python/bin/python3` 与 `setup_python_env.sh` 存在（只读 `ls`/`stat`）。  
- 若允许**受控启动**（超出本轮范围）：备份配置后只改 `python_sh` 一行，再观察 `isaac_stream.log` 与 `/api/health`。

## 下一轮最小修复范围说明

下一轮最小修复应：**备份 `ptz_config.yaml` 后，仅修改其中 `python_sh` 一行**，且**前提**为候选 `/home/uniubi/xuanyuan/python.sh`（或经确认的等价 Kit 入口）已通过存在性、可执行性及与场景/版本匹配的检查。

---

*证据文件：`A_context.txt` … `F_get_status.txt`、本 `REPORT_zh.md`。*
