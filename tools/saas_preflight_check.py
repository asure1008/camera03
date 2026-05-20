#!/usr/bin/env python3
"""
SaaS 检测前置验证工具

在把正确答案发给哨兵基座之前，验证 camera03 当前场景能否生成可发送的标准答案。
覆盖 rule_id: 1、2、3、4、5、10、12

三层校验：
  1. 服务可达性  — GET /status, /api/scene/randomize/last, /onvif-snap.jpg
  2. 标准答案完整性 — hazard 三态、必填字段、evidence 关键字段
  3. 快照可用性  — JPEG 合法性、大小、耗时

输出: 哨兵基座 pushDetectionDecision arguments 格式的 payload（可落盘 / --send 发送）

退出码:
  0 = 全部 PASS（含 warning）
  1 = 至少一个 rule BLOCK
  2 = 网络/JSON 解析错误 或 SENTINEL_API_KEY 缺失
"""
from __future__ import annotations

import argparse
import datetime
import json
import os
import sys
import time
import urllib.error
import urllib.request
import uuid
from typing import Optional

# ── 有效 rule_id 集合 ──────────────────────────────────────────────────────
VALID_RULE_IDS: list[int] = [1, 2, 3, 4, 5, 10, 12]

# 每个 rule 在 evidence 中必须存在的关键字段（缺失时 warning: evidence_incomplete）
RULE_EVIDENCE_KEYS: dict[int, list[str]] = {
    1:  ["slots"],
    2:  ["worker_count"],
    3:  ["worker_count"],
    4:  ["worker_count", "gondola_world_height"],
    5:  ["safety_rope_count", "workers_count"],
    10: ["overexposed_ratio"],
    12: ["per_steel_rope_or_fallback"],
}

RUNTIME_LOG_MAX_BYTES = 8 * 1024   # 8KB
EVIDENCE_MAX_BYTES    = 4 * 1024   # 4KB

DEFAULT_CAMERA03   = "http://127.0.0.1:8080"
DEFAULT_SENTINEL   = "http://192.168.6.30:8079"
SENTINEL_PATH      = "/api/v2/mcp/bridge/call/tool"
HTTP_TIMEOUT       = 30   # 普通 JSON 接口
SNAP_TIMEOUT       = 30   # 快照接口
RANDOMIZE_TIMEOUT  = 180  # POST randomize 等待场景渲染


# ── HTTP 工具 ──────────────────────────────────────────────────────────────

def _http_get_json(base: str, path: str, timeout: int = HTTP_TIMEOUT) -> dict:
    url = base.rstrip("/") + path
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def _http_post_json(base: str, path: str, body: dict,
                    timeout: int = HTTP_TIMEOUT,
                    extra_headers: Optional[dict] = None) -> tuple[dict, int]:
    url = base.rstrip("/") + path
    data = json.dumps(body, ensure_ascii=False).encode("utf-8")
    headers = {"Content-Type": "application/json"}
    if extra_headers:
        headers.update(extra_headers)
    req = urllib.request.Request(url, data=data, method="POST", headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8")), resp.status
    except urllib.error.HTTPError as e:
        body_raw = e.read().decode("utf-8", errors="replace")
        try:
            return json.loads(body_raw), e.code
        except Exception:
            return {"error": body_raw[:500]}, e.code


def _http_get_bytes(base: str, path: str, timeout: int = SNAP_TIMEOUT) -> tuple[bytes, dict]:
    """返回 (响应体字节, headers 字典)。"""
    url = base.rstrip("/") + path
    req = urllib.request.Request(url, method="GET")
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        data = resp.read()
        headers = dict(resp.headers)
    return data, headers


# ── 校验工具 ───────────────────────────────────────────────────────────────

def _check_evidence_keys(rule_id: int, evidence: dict) -> list[str]:
    """返回缺失的 evidence 关键字段名列表。"""
    required = RULE_EVIDENCE_KEYS.get(rule_id, [])
    missing = []
    for k in required:
        if k not in evidence:
            missing.append(k)
    return missing


def _truncate_runtime_log(log_obj: dict) -> dict:
    """evidence 超 4KB 时截断，整体超 8KB 时裁剪 evidence。"""
    ev = log_obj.get("evidence")
    if isinstance(ev, dict):
        ev_str = json.dumps(ev, ensure_ascii=False)
        if len(ev_str.encode("utf-8")) > EVIDENCE_MAX_BYTES:
            log_obj = dict(log_obj)
            log_obj["evidence"] = {"_truncated": True, "_note": "evidence too large"}
            log_obj["evidence_truncated"] = True
    total = json.dumps(log_obj, ensure_ascii=False).encode("utf-8")
    if len(total) > RUNTIME_LOG_MAX_BYTES:
        log_obj = dict(log_obj)
        log_obj["evidence"] = {"_truncated": True, "_note": "runtimeLog size exceeded 8KB"}
        log_obj["evidence_truncated"] = True
    return log_obj


def _fmt_ms(seconds: float) -> str:
    return f"{int(seconds * 1000)}ms"


def _dot_line(label: str, value: str, ok: bool = True, width: int = 42) -> str:
    dots = "." * max(2, width - len(label))
    mark = "✓" if ok else "✗"
    return f"      {label} {dots} {value} {mark}"


# ── 第一层：服务可达性 ─────────────────────────────────────────────────────

def check_reachability(base: str, verbose: bool = False) -> tuple[bool, dict]:
    """
    返回 (ok, info)
    info 含: isaac_state, last_ok, snap_ok, snap_size_kb, snap_ms, errors
    """
    info: dict = {"errors": []}
    ok_all = True

    # 1a. GET /status
    t0 = time.monotonic()
    try:
        status = _http_get_json(base, "/status")
        isaac_state = status.get("isaac_state", "unknown")
    except Exception as e:
        info["errors"].append(f"/status 请求失败: {e}")
        info["isaac_state"] = "unreachable"
        return False, info
    info["isaac_state"] = isaac_state
    info["status_ms"] = _fmt_ms(time.monotonic() - t0)

    if isaac_state != "running":
        info["errors"].append(
            f"isaac_state={isaac_state!r}，需要 running。"
            f"启动命令: python3 ptz_launcher.py --config ./ptz_config.yaml"
        )
        return False, info

    # 1b. GET /api/scene/randomize/last
    t0 = time.monotonic()
    try:
        last_resp = _http_get_json(base, "/api/scene/randomize/last")
        info["last_ms"] = _fmt_ms(time.monotonic() - t0)
        info["last_ok"] = True
        if verbose:
            info["last_raw_preview"] = json.dumps(last_resp, ensure_ascii=False)[:4096]
    except Exception as e:
        info["errors"].append(f"/api/scene/randomize/last 请求失败: {e}")
        info["last_ok"] = False
        ok_all = False

    # 1c. GET /onvif-snap.jpg（仅探活，不做 JPEG 校验，留给第三层）
    t0 = time.monotonic()
    try:
        snap_bytes, snap_hdrs = _http_get_bytes(base, "/onvif-snap.jpg")
        snap_ms = time.monotonic() - t0
        info["snap_ok"] = True
        info["snap_size_kb"] = round(len(snap_bytes) / 1024, 1)
        info["snap_ms"] = _fmt_ms(snap_ms)
        info["snap_content_type"] = snap_hdrs.get("Content-Type", "")
        info["_snap_bytes"] = snap_bytes   # 传给第三层复用
    except Exception as e:
        info["errors"].append(f"/onvif-snap.jpg 请求失败: {e}")
        info["snap_ok"] = False
        info["_snap_bytes"] = b""

    return ok_all and info.get("last_ok", False), info


# ── 第二层：标准答案完整性 ─────────────────────────────────────────────────

def check_standard_answer(last_resp: dict, target_rule_id: Optional[int] = None
                          ) -> tuple[str, list[str], dict]:
    """
    返回 (status, warnings, answer_fields)
    status: "PASS" | "BLOCK"
    warnings: 非空时为警告描述列表
    answer_fields: 从 last_resp 提取的顶层标准答案字段
    """
    warnings: list[str] = []

    hazard    = last_resp.get("hazard")
    rule_id   = last_resp.get("rule_id")
    hazard_id = last_resp.get("hazard_id")
    htype     = last_resp.get("hazard_type")
    hname     = last_resp.get("hazard_name")
    rfactor   = last_resp.get("random_factor")
    evidence  = last_resp.get("evidence") or {}
    std_ans   = last_resp.get("standard_answer") or {}
    event_id  = last_resp.get("event_id")

    fields = {
        "hazard":        hazard,
        "rule_id":       rule_id,
        "hazard_id":     hazard_id,
        "hazard_type":   htype,
        "hazard_name":   hname,
        "random_factor": rfactor,
        "evidence":      evidence,
        "standard_answer": std_ans,
        "event_id":      event_id,
    }

    # hazard=null → BLOCK
    if hazard is None:
        return "BLOCK", ["hazard=null，当前场景无法判断隐患，建议重新随机"], fields

    # hazard=false → PASS（无隐患对照答案）
    if hazard is False:
        return "PASS", [], fields

    # hazard=true 专项检查
    if hazard is True:
        try:
            rid_int = int(rule_id) if rule_id is not None else None
        except (TypeError, ValueError):
            rid_int = None

        if rid_int is None:
            return "BLOCK", ["hazard=true 但 rule_id 为空，标准答案不可发送"], fields

        if rid_int not in VALID_RULE_IDS:
            warnings.append(f"unknown_rule: rule_id={rid_int} 不在有效列表 {VALID_RULE_IDS}")

        # target_rule_id 与实际不一致（--rule-id 触发随机后验证）
        if target_rule_id is not None and rid_int != target_rule_id:
            warnings.append(
                f"rule_id 不一致：请求 {target_rule_id}，实际得到 {rid_int}，可能是随机未生效"
            )

        for field_name in ("hazard_type", "hazard_name", "random_factor"):
            if not last_resp.get(field_name):
                warnings.append(f"缺少必填字段: {field_name}")

        # standard_answer 一致性
        sa_rule_id = std_ans.get("rule_id")
        if sa_rule_id is not None and sa_rule_id != rid_int:
            warnings.append(
                f"standard_answer.rule_id={sa_rule_id} 与顶层 rule_id={rid_int} 不一致"
            )

        # evidence 关键字段
        missing_ev = _check_evidence_keys(rid_int, evidence)
        if missing_ev:
            warnings.append(f"evidence_incomplete: 缺少字段 {missing_ev}")

        # 若有 BLOCK 级 warning（当前：无），返回 BLOCK
        # 现阶段所有警告仅降级为 warning，不 BLOCK
        return "PASS", warnings, fields

    return "BLOCK", [f"hazard 取值异常: {hazard!r}"], fields


# ── 第三层：快照可用性 ─────────────────────────────────────────────────────

def check_snapshot(snap_bytes: bytes, snap_info: dict) -> tuple[bool, list[str]]:
    """
    返回 (jpeg_ok, warnings)
    快照不可用仅 warning，不阻断。
    """
    warnings: list[str] = []

    if not snap_bytes:
        warnings.append("快照字节为空，imageUrl 将标记为 unavailable")
        return False, warnings

    if not (snap_bytes[:2] == b"\xff\xd8"):
        warnings.append(
            f"响应不是合法 JPEG（magic bytes: {snap_bytes[:4].hex()}），imageUrl 将标记为 unavailable"
        )
        return False, warnings

    size_kb = snap_info.get("snap_size_kb", 0)
    ct = snap_info.get("snap_content_type", "")
    if "image" not in ct.lower():
        warnings.append(f"Content-Type={ct!r}，不是 image/*")

    return True, warnings


# ── Payload 生成 ───────────────────────────────────────────────────────────

def build_payload(
    run_id: str,
    answer_fields: dict,
    preflight_status: str,
    all_warnings: list[str],
    snap_ok: bool,
    base: str,
) -> dict:
    """构建哨兵基座 pushDetectionDecision 外层结构。"""
    rule_id   = answer_fields.get("rule_id")
    htype     = answer_fields.get("hazard_type") or ""
    hname     = answer_fields.get("hazard_name") or ""
    rfactor   = answer_fields.get("random_factor") or ""
    hazard    = answer_fields.get("hazard")
    evidence  = answer_fields.get("evidence") or {}
    std_ans   = answer_fields.get("standard_answer") or {}
    event_id  = answer_fields.get("event_id")

    image_url = (
        f"{base.rstrip('/')}/onvif-snap.jpg" if snap_ok else "unavailable"
    )

    event_id_out = str(event_id) if event_id else str(uuid.uuid4())

    title = f"前置验证：rule_id={rule_id} {htype}" if rule_id else "前置验证：无隐患场景"

    if preflight_status == "PASS":
        ev_summary = ", ".join(
            f"{k}={v}" for k, v in list(evidence.items())[:6]
        ) if evidence else "—"
        reason = (
            f"preflight_status=PASS；hazard={hazard}；"
            f"hazard_name={hname}；evidence: {ev_summary}"
        )
    else:
        reason = f"preflight_status=BLOCK；" + "；".join(all_warnings[:3])

    runtime_log_obj = {
        "preflight_status": preflight_status,
        "rule_id":          rule_id,
        "hazard":           hazard,
        "hazard_type":      htype or None,
        "hazard_name":      hname or None,
        "random_factor":    rfactor or None,
        "evidence":         evidence,
        "standard_answer":  std_ans,
        "preflight_warnings": all_warnings,
        "camera03_fetched_at": datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
    }
    runtime_log_obj = _truncate_runtime_log(runtime_log_obj)
    runtime_log_str = json.dumps(runtime_log_obj, ensure_ascii=False)

    return {
        "label": "safety_mcp_server",
        "name":  "pushDetectionDecision",
        "arguments": {
            "runId":      run_id,
            "eventId":    event_id_out,
            "timestamp":  datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "title":      title,
            "reason":     reason,
            "imageUrl":   image_url,
            "regionNo":   "",
            "gridNo":     "",
            "runtimeLog": runtime_log_str,
        },
    }


# ── 单次 rule 验证流程 ─────────────────────────────────────────────────────

def run_one_rule(
    base: str,
    rule_id: Optional[int],
    wait_s: int,
    verbose: bool,
) -> tuple[str, list[str], dict, dict]:
    """
    执行随机（若 rule_id 非 None）→ 读 last → 校验。
    返回 (preflight_status, all_warnings, answer_fields, reachability_info)
    preflight_status: "PASS" | "BLOCK" | "ERROR"
    """
    # 触发随机
    # rule_id 是 camera03 的顶层元字段（_RANDOMIZE_REQ_META_KEYS），不能放在 random_config 内
    if rule_id is not None:
        rand_body: dict = {"rule_id": rule_id}
        try:
            resp, code = _http_post_json(base, "/api/scene/randomize", rand_body,
                                          timeout=RANDOMIZE_TIMEOUT)
            if code not in (200, 201) or not resp.get("ok"):
                return "ERROR", [
                    f"POST /api/scene/randomize 失败 (HTTP {code}): {resp.get('error','')}"
                ], {}, {}
        except Exception as e:
            return "ERROR", [f"POST /api/scene/randomize 异常: {e}"], {}, {}
        if wait_s > 0:
            time.sleep(wait_s)

    # 读 last
    try:
        last_resp = _http_get_json(base, "/api/scene/randomize/last")
    except Exception as e:
        return "ERROR", [f"GET /api/scene/randomize/last 异常: {e}"], {}, {}

    if not last_resp.get("has_last_result"):
        return "BLOCK", ["has_last_result=false，尚无随机结果"], {}, {}

    status, warnings, answer_fields = check_standard_answer(last_resp, rule_id)
    return status, warnings, answer_fields, last_resp


# ── 打印工具 ───────────────────────────────────────────────────────────────

def print_layer_header(n: int, total: int, title: str) -> None:
    print(f"\n[{n}/{total}] {title}")


def print_reach_result(info: dict) -> None:
    isaac_ok = info.get("isaac_state") == "running"
    print(_dot_line(
        "isaac_state", info.get("isaac_state", "?") + f" ({info.get('status_ms','')})",
        ok=isaac_ok
    ))
    last_ok = info.get("last_ok", False)
    print(_dot_line(
        "/api/scene/randomize/last",
        f"{'OK' if last_ok else 'FAIL'} ({info.get('last_ms','')})",
        ok=last_ok
    ))
    snap_ok_reach = info.get("snap_ok", False)
    snap_desc = (
        f"{info.get('snap_size_kb',0)}KB ({info.get('snap_ms','')})"
        if snap_ok_reach else "FAIL"
    )
    print(_dot_line("/onvif-snap.jpg", snap_desc, ok=snap_ok_reach))
    for err in info.get("errors", []):
        print(f"      [BLOCK] {err}")


def print_answer_result(status: str, warnings: list[str], fields: dict) -> None:
    hazard = fields.get("hazard")
    print(f"      hazard 三态: {hazard}")
    if fields.get("rule_id") is not None:
        print(f"      rule_id={fields['rule_id']} .... {fields.get('hazard_type','?')}")
    if fields.get("hazard_name"):
        print(f"      hazard_name .... {fields['hazard_name']}")
    if fields.get("random_factor"):
        print(f"      random_factor .. {fields['random_factor']}")
    ev = fields.get("evidence") or {}
    if ev:
        ev_str = ", ".join(f"{k}={v}" for k, v in list(ev.items())[:6])
        print(f"      evidence ....... {ev_str}")
    if status == "BLOCK":
        for w in warnings:
            print(f"      [BLOCK] {w}")
    elif warnings:
        for w in warnings:
            print(f"      [WARN]  {w}")
    else:
        print("      标准答案一致性 ✓")


def print_payload_result(payload: dict, output_file: Optional[str], snap_ok: bool) -> None:
    args = payload.get("arguments", {})
    print(f"      runId   .... {args.get('runId','')}")
    print(f"      eventId .... {args.get('eventId','')}")
    runtime_log = args.get("runtimeLog", "{}")
    try:
        rl = json.loads(runtime_log)
        pf_status = rl.get("preflight_status", "?")
    except Exception:
        pf_status = "?"
    print(f"      preflight_status .. {pf_status}")
    if not snap_ok:
        print("      [WARN]  imageUrl=unavailable（快照不可用）")
    if output_file:
        try:
            with open(output_file, "w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
            size_kb = round(os.path.getsize(output_file) / 1024, 1)
            print(f"\n[OUTPUT] → {output_file} ({size_kb}KB)")
        except Exception as e:
            print(f"[ERROR] 落盘失败: {e}", file=sys.stderr)


# ── 单条 rule 完整运行（含打印） ───────────────────────────────────────────

def run_and_print_rule(
    base: str,
    run_id: str,
    rule_id: Optional[int],
    wait_s: int,
    verbose: bool,
    output: Optional[str],
    reach_info: dict,
) -> tuple[str, dict]:
    """
    在可达性已确认的前提下，完成随机 → 校验 → payload 三层并打印。
    返回 (preflight_status, payload)
    preflight_status: "PASS" | "BLOCK" | "ERROR"
    """
    rule_label = f"rule_id={rule_id}" if rule_id else "last-only"
    print(f"\n[PREFLIGHT] camera03: {base}  {rule_label}")

    # 层 1（打印，已在外部确认）
    print_layer_header(1, 3, "服务可达性")
    print_reach_result(reach_info)

    # 层 2（随机 + 校验）
    print_layer_header(2, 3, f"标准答案校验 ({rule_label})")
    status, warnings, answer_fields, _last = run_one_rule(
        base, rule_id, wait_s, verbose
    )
    if status == "ERROR":
        for w in warnings:
            print(f"      [ERROR] {w}")
        return "ERROR", {}

    print_answer_result(status, warnings, answer_fields)

    # 层 3（快照）
    print_layer_header(3, 3, "快照可用性")
    snap_bytes = reach_info.get("_snap_bytes", b"")
    snap_ok, snap_warns = check_snapshot(snap_bytes, reach_info)
    if snap_ok:
        print(_dot_line(
            "/onvif-snap.jpg",
            f"合法 JPEG {reach_info.get('snap_size_kb',0)}KB ({reach_info.get('snap_ms','')})",
            ok=True
        ))
    else:
        for w in snap_warns:
            print(f"      [WARN]  {w}")
    all_warnings = warnings + snap_warns

    # 如果 status=BLOCK，快照不影响 BLOCK 判断
    payload = build_payload(run_id, answer_fields, status, all_warnings, snap_ok, base)

    print_layer_header(3, 3, "Payload 生成") if False else None  # 已在第 3 层里
    print(f"\n[3/3] Payload 生成")
    print_payload_result(payload, output if rule_id is not None else output, snap_ok)

    return status, payload


# ── --all-rules 汇总 ───────────────────────────────────────────────────────

def run_all_rules(
    base: str,
    wait_s: int,
    verbose: bool,
    output: Optional[str],
    reach_info: dict,
) -> tuple[int, list[dict]]:
    """
    依次对 VALID_RULE_IDS 各随机一次并校验。
    返回 (max_exit_code, payloads_list)
    """
    run_id = "preflight_" + str(uuid.uuid4())
    rows: list[tuple] = []
    payloads: list[dict] = []
    max_code = 0

    for rid in VALID_RULE_IDS:
        try:
            status, warnings, answer_fields, _ = run_one_rule(
                base, rid, wait_s, verbose
            )
        except Exception as e:
            status, warnings, answer_fields = "ERROR", [str(e)], {}

        snap_bytes = reach_info.get("_snap_bytes", b"")
        snap_ok, snap_warns = check_snapshot(snap_bytes, reach_info)
        all_warnings = warnings + snap_warns

        payload = build_payload(run_id, answer_fields, status, all_warnings, snap_ok, base)
        payloads.append(payload)

        htype = answer_fields.get("hazard_type") or "-"
        warn_str = "; ".join(all_warnings) if all_warnings else "—"
        rows.append((rid, htype, status, warn_str))

        if status == "ERROR":
            max_code = max(max_code, 2)
        elif status == "BLOCK":
            max_code = max(max_code, 1)

    # 打印汇总表
    print("\n")
    print(f"{'rule':<6}  {'hazard_type':<48}  {'status':<8}  {'warnings'}")
    print(f"{'----':<6}  {'-------------------------------------------':<48}  {'-------':<8}  {'--------'}")
    for rid, htype, status, warn_str in rows:
        st_str = status
        print(f"{rid:<6}  {htype:<48}  {st_str:<8}  {warn_str[:80]}")

    if output:
        all_payload_path = output
        try:
            with open(all_payload_path, "w", encoding="utf-8") as f:
                json.dump(payloads, f, ensure_ascii=False, indent=2)
            size_kb = round(os.path.getsize(all_payload_path) / 1024, 1)
            print(f"\n[OUTPUT] → {all_payload_path} ({size_kb}KB, {len(payloads)} rules)")
        except Exception as e:
            print(f"[ERROR] 落盘失败: {e}", file=sys.stderr)

    return max_code, payloads


# ── 哨兵基座发送 ───────────────────────────────────────────────────────────

def send_to_sentinel(sentinel_url: str, api_key: str, payload: dict) -> int:
    """
    发送 payload 到哨兵基座。
    返回 HTTP 状态码（失败时返回 0）。
    """
    try:
        _, code = _http_post_json(
            sentinel_url, SENTINEL_PATH, payload,
            timeout=HTTP_TIMEOUT,
            extra_headers={"apiKey": api_key},
        )
        return code
    except Exception as e:
        print(f"[ERROR] 发送哨兵基座异常: {e}", file=sys.stderr)
        return 0


# ── CLI ────────────────────────────────────────────────────────────────────

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="SaaS 检测前置验证工具 —— 覆盖 rule_id 1/2/3/4/5/10/12",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    p.add_argument(
        "--camera03",
        default=DEFAULT_CAMERA03,
        metavar="URL",
        help=f"camera03 launcher 地址（默认 {DEFAULT_CAMERA03}）",
    )

    # 场景控制互斥组（三选一）
    scene_grp = p.add_mutually_exclusive_group()
    scene_grp.add_argument(
        "--last-only",
        action="store_true",
        help="只读最近结果，不触发随机",
    )
    scene_grp.add_argument(
        "--rule-id",
        type=int,
        choices=VALID_RULE_IDS,
        metavar="N",
        help=f"触发指定 rule_id 随机并校验，有效值: {VALID_RULE_IDS}",
    )
    scene_grp.add_argument(
        "--all-rules",
        action="store_true",
        help=f"依次随机并校验全部有效 rule_id {VALID_RULE_IDS}，输出汇总",
    )

    p.add_argument(
        "--wait-s",
        type=int,
        default=5,
        metavar="N",
        help="随机后等待 N 秒再读 last（默认 5）",
    )
    p.add_argument(
        "--output",
        metavar="FILE",
        help="payload 落盘到 FILE.json，默认只打印",
    )
    p.add_argument(
        "--verbose",
        action="store_true",
        help="打印完整 raw 响应（限 4KB，超出截断）",
    )

    # 哨兵基座探测互斥组
    sent_grp = p.add_mutually_exclusive_group()
    sent_grp.add_argument(
        "--dry-run",
        action="store_true",
        default=False,
        help="校验 payload 但不发送（默认模式）",
    )
    sent_grp.add_argument(
        "--send",
        action="store_true",
        default=False,
        help=(
            "发送到哨兵基座 POST /api/v2/mcp/bridge/call/tool；"
            "apiKey 从环境变量 SENTINEL_API_KEY 读取"
        ),
    )
    p.add_argument(
        "--sentinel-url",
        default=DEFAULT_SENTINEL,
        metavar="URL",
        help=f"哨兵基座地址（默认 {DEFAULT_SENTINEL}）",
    )

    return p


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    base    = args.camera03
    wait_s  = args.wait_s
    verbose = args.verbose
    output  = args.output

    # --send 时校验 apiKey
    api_key = ""
    if args.send:
        api_key = os.environ.get("SENTINEL_API_KEY", "")
        if not api_key:
            print("[ERROR] SENTINEL_API_KEY 未配置，--send 需要先 export SENTINEL_API_KEY=<你的apiKey>",
                  file=sys.stderr)
            sys.exit(2)

    # 默认行为：last-only
    if not args.rule_id and not args.all_rules:
        args.last_only = True

    # ── 第一层（所有模式公用） ────────────────────────────────
    try:
        reach_ok, reach_info = check_reachability(base, verbose)
    except Exception as e:
        print(f"[ERROR] 可达性检查异常: {e}", file=sys.stderr)
        sys.exit(2)

    if not reach_ok:
        print(f"\n[PREFLIGHT] camera03: {base}")
        print_layer_header(1, 3, "服务可达性")
        print_reach_result(reach_info)
        sys.exit(2)

    # ── --all-rules ───────────────────────────────────────────
    if args.all_rules:
        print(f"\n[PREFLIGHT] camera03: {base}  --all-rules {VALID_RULE_IDS}")
        print_layer_header(1, 3, "服务可达性")
        print_reach_result(reach_info)

        exit_code, payloads = run_all_rules(base, wait_s, verbose, output, reach_info)

        if args.send and payloads:
            print("\n[SEND] 逐条发送到哨兵基座...")
            any_fail = False
            for pl in payloads:
                code = send_to_sentinel(args.sentinel_url, api_key, pl)
                rid = json.loads(pl["arguments"]["runtimeLog"]).get("rule_id", "?")
                ok = 200 <= code < 300
                print(f"      rule_id={rid} → HTTP {code} {'✓' if ok else '✗'}")
                if not ok:
                    any_fail = True
            if any_fail:
                exit_code = max(exit_code, 1)
        elif args.send:
            print("[WARN] 无可发送的 payload")

        sys.exit(exit_code)

    # ── --last-only 或 --rule-id ──────────────────────────────
    run_id    = "preflight_" + str(uuid.uuid4())
    rule_id   = args.rule_id     # None 时为 last-only

    status, payload = run_and_print_rule(
        base, run_id, rule_id, wait_s, verbose, output, reach_info
    )

    if args.send and payload:
        print("\n[SEND] 发送到哨兵基座...")
        code = send_to_sentinel(args.sentinel_url, api_key, payload)
        ok = 200 <= code < 300
        print(f"      HTTP {code} {'✓' if ok else '✗'}")
        if not ok:
            status = "BLOCK"

    if status == "PASS":
        sys.exit(0)
    elif status == "BLOCK":
        sys.exit(1)
    else:
        sys.exit(2)


if __name__ == "__main__":
    main()
