# 只读排查报告：HDRI `sky_sun.hdr` 与 Isaac 启动失败（关联 192.168.41.178）

## 合规声明（本轮）

| 项 | 是否 |
|----|------|
| 修改业务代码 / 配置 / YAML / HTML / USD / HDR / 模型 | **否** |
| stop / start / kill / nohup | **否** |
| install / download / compile / rsync / scp / git | **否** |
| POST / PUT / PATCH / DELETE | **否** |
| randomize / PTZ / preset / snapshot / RTSP 抓流 / HDRI 写入 / renderer 操作 | **否** |

证据目录（完整路径）：

`/home/uniubi/xuanyuan/camera05/camera03/_archive/evidence/readonly_hdri_missing_41_178_20260429_033817/`

---

## 1. 采集环境与 41.178 的关系（重要）

本轮 **shell 证据均在当前 Cursor 工作区主机采集**，`hostname` 为 `uniubi`，`hostname -I` 含 **192.168.46.144**，**不包含 192.168.41.178**。

在仅允许 `ls/find/grep/sed/.../curl GET`、禁止 SSH 的前提下，**无法对 41.178 磁盘做同等 `find/stat`**。对 `http://192.168.41.178:8080/8081/health` 的 `curl GET` 在本环境失败（见证据 `09_curl_41_178.txt`，疑似代理指向 `127.0.0.1:17890`）。

因此：**关于 41.178 上「文件是否真实缺失」的结论为推断**；**关于「代码为何报 `HDRI file not found`」的结论为仓库只读确定**。

用户优先工作目录 `/home/uniubi/xuanyuan/camera03` **在本机不存在**；实际项目为 **`/home/uniubi/xuanyuan/camera05/camera03`**（与历史证据中 41.178 曾出现的 `/home/uniubi/xuanyuan/camera03/...` 路径层级 **不一致**，见下）。

---

## 2. `FileNotFoundError: HDRI file not found: …` 的确定技术根因（代码层）

`ptz_stream.py` 中 `_apply_hdri_texture` 在应用贴图前执行：

- `normalized_path = _normalize_hdri_path(hdri_path)`，其中 `_normalize_hdri_path` 使用 `os.path.abspath(_resolve_path(...))`；
- `_resolve_path` 对非绝对路径拼 **`script_dir`（`ptz_stream.py` 所在目录）**；
- 若 `os.path.isfile(normalized_path)` 为假，则抛出 **`FileNotFoundError(f"HDRI file not found: {normalized_path}")`**。

因此 **直接触发条件**：解析后的 **`normalized_path` 在运行时文件系统上不是常规文件**（未部署、路径错、`changjing` 目录漏拷、盘符/前缀与 yaml 不一致等）。

若你看到的异常字面量仅为 `sky_sun.hdr`，请注意源码抛错使用的是 **`normalized_path` 整串**；若现场日志被截断或经上层包装，可能只露出 basename。以仓库为准应核对 **完整路径**。

---

## 3. 在本采集主机上：`sky_sun.hdr` 是否存在

**存在。**

完整路径（`find /home/uniubi/xuanyuan -name 'sky_sun.hdr'` 命中主副本）：

`/home/uniubi/xuanyuan/camera05/camera03/changjing/sky_sun.hdr`

`stat` 显示为约 **97 466 049** 字节的常规文件（证据 `01_find_sky_sun_and_hdr.txt`）。

---

## 4. 若 41.178 报错：是否像「迁移漏文件」

在 **配置仍指向 `hdri_whitelist` 中 `.../camera05/camera03/changjing/sky_sun.hdr`** 的前提下，若 41.178 实际部署在 **`/home/uniubi/xuanyuan/camera03`**（历史证据 `G_post_start.txt` 曾出现 `.../camera03/isaac_stream.log`），则可能出现两类「像迁移问题」的情况：

1. **`changjing/` 或大体积 `.hdr` 未同步** → 目标绝对路径上 `os.path.isfile` 失败 → 与代码逻辑一致。  
2. **仅同步了代码与 yaml，但 yaml 内绝对路径仍指向 `camera05/camera03`，而机上无该前缀** → 同样 `isfile` 失败。

二者均属 **部署/迁移与资源路径不一致**；需在 **41.178** 上对 `ptz_config.yaml` 中每条 `hdri_whitelist` 路径执行 `stat`/`test -f` 验证（本轮未在 41.178 执行）。

---

## 5. `ptz_config.yaml` 中 HDRI 相关摘要

- **`hdri_whitelist`**：四条 **绝对路径**，均指向  
  `/home/uniubi/xuanyuan/camera05/camera03/changjing/*.hdr`（含 `sky_sun.hdr`）。  
- **`environment_mode`**：`hdri`（默认走 HDRI，不用 Dynamic Sky）。  
- **`random_config.random_hdri`**：`true`（启动随机化流程会尝试从 **磁盘存在的候选** 中选一张应用；若 **四条都不存在** 会先表现为 `RuntimeError: no valid HDRI candidates...`，与单文件 `FileNotFoundError` 场景略有差别，但若某路径仍被单独传入 `_apply_hdri_texture` 仍可能直接 FileNotFoundError）。

详见证据 `02_ptz_config_hdri_grep_and_head.txt`。

---

## 6. 代码中 HDRI 路径解析依据（摘要）

- 内置默认槽位 A1 相对名为 `changjing/sky_sun.hdr`，经 `_normalize_hdri_path` → **`{脚本目录}/changjing/sky_sun.hdr`**。  
- 当前 yaml **用绝对路径覆盖** 白名单，故 **以 yaml 绝对路径为准**（再 `abspath`）。  
- `exists` 标志与随机候选均依赖 **`os.path.isfile`**。

证据：`03_code_hdri_path_resolution.txt`。

---

## 7. 「8081 未启动是否仍指向 HDRI 缺失」

- **在本采集主机（46.144）上**：`ss` 显示 **8081 已在监听**，且 `ptz_stream` 进程在跑，**与「HDRI 缺失导致未监听」不一致**。  
- **在 41.178 上**：若 Isaac/子进程在应用 HDRI 前即因该异常退出，可能出现 **就绪等待超时 / 控制面未起来**，表现为 8081 无监听或 `/start` 失败；**需在目标机结合日志与端口再判**，本轮无 41.178 直接端口证据。

---

## 8. 下一步最小修复建议（仅建议，本轮未执行修复）

**A.** 若源机（或本仓库）存在完整 `changjing`（至少含 `sky_sun.hdr` 及白名单其余三张），在 **41.178** 上使 **`ptz_config.yaml` 中 `hdri_whitelist` 每条路径经 `stat` 均存在**；路径前缀与真实部署根（`camera05/camera03` vs `camera03`）必须一致。

**B.** 若暂时不想同步大 HDR：在 **评估可接受性后**（非本轮操作）可考虑将运行模式改为 **不依赖缺失 HDRI**（例如 `environment_mode: dynamic_sky` 并保证 preset USD 存在，或关闭 `random_hdri`/收窄候选）；**任何配置修改均超出本轮只读范围**，此处仅为方向性最小化描述。

**C.** 在 **41.178** 上未确认文件与路径一致前，避免反复 **`/start`** 造成无意义的就绪等待与日志噪声。

---

## 9. 证据文件列表

| 文件 | 内容概要 |
|------|------------|
| `00_scope_and_host.txt` | 主机名、IP、端口、进程、工作目录说明 |
| `01_find_sky_sun_and_hdr.txt` | `sky_sun.hdr` 与 `changjing` 目录 |
| `02_ptz_config_hdri_grep_and_head.txt` | yaml HDRI 摘要 |
| `03_code_hdri_path_resolution.txt` | 路径解析与抛错条件 |
| `04_isaac_stream_log_hdri.txt` | 本机 isaac_stream.log 与 HDRI 相关结论 |
| `05_archive_41_178_crossref.txt` | 历史 41.178 证据路径交叉引用 |
| `06_wide_grep_note.txt` | 超宽 grep 策略说明 |
| `09_curl_41_178.txt` | 对 41.178 health 的 curl 结果 |

---

*报告生成时间：与证据目录时间戳一致（20260429_033817 批次）。*
