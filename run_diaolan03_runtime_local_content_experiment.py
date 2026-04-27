#!/usr/bin/env python3
import datetime as dt
import json
import subprocess
import time
import urllib.request
from pathlib import Path

import yaml

import run_constrained_random_acceptance as base


OUT_DIR = Path("/home/uniubi/xuanyuan/camera05/camera03")
CTRL_PORT = 18081
CTRL_BASE = f"http://127.0.0.1:{CTRL_PORT}"
RTSP_PORT = 18554

ACTIVE_PATH = "/World/Diaolan_Ver1_0_2026_07"
FORCE_WORKERS_COUNT = 1
FORCE_GONDOLA_HEIGHT = 24.98
INITIAL_PAN = 10.0
INITIAL_TILT = 0.0
BASELINE_XYZ = [88.0, -55.0, 33.0]

Z_DELTAS = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
Y_DELTAS = [0.0, -1.5, -1.0, -0.5, 0.5, 1.0, 1.5]

RUN_TAG = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
REPORT_PATH = OUT_DIR / f"_rtsp_diaolan03_runtime_report_{RUN_TAG}.json"
LOG_PATH = OUT_DIR / f"_rtsp_diaolan03_runtime_log_{RUN_TAG}.jsonl"
STREAM_LOG_PATH = OUT_DIR / f"_rtsp_diaolan03_runtime_stream_{RUN_TAG}.log"
TMP_CFG_PATH = OUT_DIR / f"_ptz_config_diaolan03_runtime_{RUN_TAG}.yaml"


def log_event(event: str, **payload) -> None:
    row = {
        "ts": dt.datetime.now().isoformat(timespec="milliseconds"),
        "event": event,
        **payload,
    }
    text = json.dumps(row, ensure_ascii=False)
    print(text, flush=True)
    with LOG_PATH.open("a", encoding="utf-8") as fp:
        fp.write(text + "\n")


def http_json(url: str, method: str = "GET", payload: dict | None = None, timeout: int = 30) -> dict:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as resp:
        return json.loads(resp.read().decode("utf-8"))


def status() -> dict:
    return http_json(f"{CTRL_BASE}/status", timeout=10)


def projection_metrics() -> dict:
    return ((status().get("stream") or {}).get("target_projection_metrics") or {})


def ctrl_post(path: str, payload: dict) -> dict:
    return http_json(f"{CTRL_BASE}{path}", method="POST", payload=payload, timeout=15)


def wait_stream_ready(timeout_s: int = 360) -> dict:
    deadline = time.time() + timeout_s
    last = {}
    while time.time() < deadline:
        try:
            last = status()
        except Exception as exc:
            last = {"error": repr(exc)}
            time.sleep(1.0)
            continue
        stream = last.get("stream") or {}
        if stream.get("camera_bound_ok") is True and stream.get("ffmpeg_alive") is True:
            time.sleep(2.0)
            return last
        time.sleep(1.0)
    raise RuntimeError(f"stream not ready: {json.dumps(last, ensure_ascii=False)}")


def make_temp_cfg(original_cfg: dict) -> dict:
    cfg = json.loads(json.dumps(original_cfg, ensure_ascii=False))
    cfg["ctrl_port"] = CTRL_PORT
    cfg["rtsp_url"] = f"rtsp://localhost:{RTSP_PORT}/ptz_cam"
    mediamtx = dict(cfg.get("mediamtx") or {})
    mediamtx["port"] = RTSP_PORT
    cfg["mediamtx"] = mediamtx
    cfg["diaolan_camera_sampling_enabled"] = False
    cfg["force_active_diaolan_path"] = ACTIVE_PATH
    cfg["camera_rig_translate_xyz"] = [float(v) for v in BASELINE_XYZ]
    cfg["force_gondola_height"] = FORCE_GONDOLA_HEIGHT
    cfg["force_workers_count"] = FORCE_WORKERS_COUNT
    cfg["initial_pan"] = INITIAL_PAN
    cfg["initial_tilt"] = INITIAL_TILT
    return cfg


def stream_tail() -> str:
    if not STREAM_LOG_PATH.exists():
        return ""
    return "\n".join(STREAM_LOG_PATH.read_text(encoding="utf-8", errors="ignore").splitlines()[-120:])


def start_stream(temp_cfg: dict) -> subprocess.Popen:
    log_fp = STREAM_LOG_PATH.open("w", encoding="utf-8")
    try:
        proc = subprocess.Popen(
            [
                str(temp_cfg["python_sh"]),
                "-u",
                str(OUT_DIR / "ptz_stream.py"),
                "--config",
                str(TMP_CFG_PATH),
                "--ctrl-port",
                str(CTRL_PORT),
                "--scene",
                str(temp_cfg["scene_path"]),
            ],
            cwd=str(OUT_DIR),
            stdout=log_fp,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        log_fp.close()
    log_event("stream_started", pid=proc.pid, stream_log_path=str(STREAM_LOG_PATH), temp_config_path=str(TMP_CFG_PATH))
    return proc


def stop_stream(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=10)
    log_event("stream_stopped", returncode=proc.returncode, stream_tail=stream_tail())


def set_camera_rig(xyz: list[float]) -> dict:
    resp = ctrl_post("/scene/camera_rig", {"xyz": [float(v) for v in xyz]})
    if not resp.get("ok"):
        raise RuntimeError(f"set_camera_rig failed: {resp}")
    time.sleep(0.8)
    return resp


def set_workers(count: int) -> None:
    ctrl_post("/scene/workers", {"count": int(count)})
    time.sleep(0.8)


def scene_post(action: str, payload: dict | None = None) -> dict:
    req = {"action": action}
    if payload:
        req.update(payload)
    return ctrl_post("/scene/experiment", req)


def scene_describe() -> dict:
    return http_json(f"{CTRL_BASE}/scene/describe", timeout=15)


def apply_fixed_ptz(initial_zoom: float) -> None:
    ctrl_post("/control", {"pan": INITIAL_PAN, "tilt": INITIAL_TILT, "zoom": float(initial_zoom)})
    time.sleep(0.8)


def capture_best_rtsp_frame_set(case_tag: str, rtsp_url: str, frame_count: int = base.RTSP_MULTI_FRAME_COUNT) -> dict:
    time.sleep(base.RTSP_STABILIZE_WAIT_S)
    frames = []
    for idx in range(frame_count):
        pm = projection_metrics()
        snap_path, img = base.grab_rtsp_snapshot(f"{case_tag}_f{idx + 1:02d}", rtsp_url)
        bbox_px = (pm or {}).get("屏幕包围框px", [0.0, 0.0, 1.0, 1.0])
        iq = base.image_quality_metrics(img, bbox_px)
        geom = base.geometry_rules(pm) if pm else {"投影指标缺失": False}
        rules = base.full_acceptance_rules(pm, iq) if pm else {"投影指标缺失": False}
        image_rules = base.image_rule_bools(iq)
        frame = {
            "帧序号": idx + 1,
            "RTSP抓帧路径": str(snap_path),
            "distance_to_camera": (pm or {}).get("distance_to_camera"),
            "2D占屏": {
                "屏幕包围框px": (pm or {}).get("屏幕包围框px"),
                "中心点px": (pm or {}).get("中心点px"),
                "宽度占比%": (pm or {}).get("宽度占比%"),
                "高度占比%": (pm or {}).get("高度占比%"),
                "越界面积占比": (pm or {}).get("越界面积占比"),
                "目标像素占比": (pm or {}).get("目标像素占比"),
            },
            "几何规则": geom,
            "几何通过": all(geom.values()),
            "图像验收": iq,
            "图像规则": image_rules,
            "图像通过": all(image_rules.values()),
            "验收规则": rules,
        }
        frame["是否通过验收"] = all(rules.values())
        frame["评分"] = base.frame_acceptance_score(pm, iq, rules)
        frames.append(frame)
        log_event("frame_scored", case_tag=case_tag, frame=frame)
        if idx + 1 < frame_count:
            time.sleep(base.RTSP_FRAME_INTERVAL_S)
    best = max(frames, key=base.frame_sort_key)
    return {
        "抓帧数量": frame_count,
        "逐帧结果": frames,
        "最佳帧": best,
        "最佳帧路径": best["RTSP抓帧路径"],
        "最佳帧评分": best["评分"],
        "最佳帧通过验收": best["是否通过验收"],
        "最佳帧图像通过": best["图像通过"],
    }


def failed_rules(best_frame: dict) -> list[str]:
    return [k for k, ok in best_frame["验收规则"].items() if not ok]


def evaluate_pose(case_tag: str, xyz: list[float], rtsp_url: str) -> dict:
    set_camera_rig(xyz)
    capture = capture_best_rtsp_frame_set(case_tag, rtsp_url)
    best = capture["最佳帧"]
    row = {
        "分组标签": case_tag,
        "camera_rig_translate_xyz": [round(float(v), 2) for v in xyz],
        "抓帧数量": capture["抓帧数量"],
        "逐帧结果": capture["逐帧结果"],
        "最佳帧路径": capture["最佳帧路径"],
        "最佳帧评分": capture["最佳帧评分"],
        "最佳帧通过验收": capture["最佳帧通过验收"],
        "最佳帧图像通过": capture["最佳帧图像通过"],
        "最佳帧结果": best,
        "最佳帧失败规则": failed_rules(best),
    }
    log_event("pose_case_done", case_tag=case_tag, xyz=row["camera_rig_translate_xyz"], best_score=row["最佳帧评分"], best_path=row["最佳帧路径"], best_pass=row["最佳帧通过验收"], failed_rules=row["最佳帧失败规则"], best_metrics=row["最佳帧结果"]["图像验收"])
    return row


def content_case_specs(desc: dict) -> list[dict]:
    model_path = desc.get("model_path") or ""
    target_branch_path = desc.get("target_branch_path") or ""
    worker_paths = list(desc.get("worker_paths") or [])
    auxiliary_model_children = list(desc.get("auxiliary_model_children") or [])
    model_children = list(desc.get("model_children") or [])
    suspicious_named_paths = list(desc.get("suspicious_named_paths") or [])
    flat_candidate_paths = [item["path"] for item in (desc.get("flat_candidate_paths") or []) if isinstance(item, dict) and item.get("path")]
    suspicious_paths = []
    for path in suspicious_named_paths + flat_candidate_paths:
        if path not in suspicious_paths and path not in worker_paths:
            suspicious_paths.append(path)
    keep_root = target_branch_path
    if model_path and target_branch_path.startswith(model_path + "/"):
        keep_root = f"{model_path}/{target_branch_path[len(model_path) + 1:].split('/')[0]}"
    keep_only_hide = [path for path in model_children if path != keep_root]
    return [
        {"case_id": "content_baseline", "说明": "最佳机位基线", "workers": 1, "hide_paths": []},
        {"case_id": "workers_0", "说明": "workers=0", "workers": 0, "hide_paths": []},
        {"case_id": "hide_auxiliary_model_children", "说明": "隐藏吊篮附属对象", "workers": 1, "hide_paths": auxiliary_model_children},
        {"case_id": "hide_suspicious_card_like", "说明": "隐藏疑似 card/plane/quad/billboard/decal/screen 类", "workers": 1, "hide_paths": suspicious_paths},
        {"case_id": "hide_entire_gondola", "说明": "隐藏整个吊篮仅保留场景", "workers": 1, "hide_paths": [model_path] if model_path else []},
        {"case_id": "keep_only_target_branch", "说明": "仅保留吊篮主分支", "workers": 1, "hide_paths": keep_only_hide},
    ]


def evaluate_content_case(case: dict, rtsp_url: str) -> dict:
    scene_post("clear")
    set_workers(case["workers"])
    apply_resp = {"ok": True}
    if case["hide_paths"]:
        apply_resp = scene_post("apply_visibility", {"hide_paths": case["hide_paths"]})
    time.sleep(1.0)
    capture = capture_best_rtsp_frame_set(case["case_id"], rtsp_url)
    best = capture["最佳帧"]
    row = {
        "case_id": case["case_id"],
        "说明": case["说明"],
        "workers": case["workers"],
        "hide_paths": case["hide_paths"],
        "apply_response": apply_resp,
        "抓帧数量": capture["抓帧数量"],
        "逐帧结果": capture["逐帧结果"],
        "最佳帧路径": capture["最佳帧路径"],
        "最佳帧评分": capture["最佳帧评分"],
        "最佳帧通过验收": capture["最佳帧通过验收"],
        "最佳帧图像通过": capture["最佳帧图像通过"],
        "最佳帧结果": best,
        "最佳帧失败规则": failed_rules(best),
    }
    log_event("content_case_done", case_id=case["case_id"], best_score=row["最佳帧评分"], best_path=row["最佳帧路径"], best_pass=row["最佳帧通过验收"], failed_rules=row["最佳帧失败规则"], best_metrics=row["最佳帧结果"]["图像验收"])
    return row


def judge_binding(content_rows: list[dict]) -> tuple[str, dict | None]:
    rows = {row["case_id"]: row for row in content_rows}
    baseline = rows.get("content_baseline")
    if not baseline:
        return "无法判断", None
    baseline_score = float(baseline["最佳帧评分"])
    deltas = []
    for row in content_rows:
        if row["case_id"] == "content_baseline":
            continue
        deltas.append((round(float(row["最佳帧评分"]) - baseline_score, 3), row))
    if not deltas:
        return "无法判断", None
    best_delta, best_row = max(deltas, key=lambda item: item[0])
    if best_delta < 6.0:
        scene_only = rows.get("hide_entire_gondola")
        if scene_only and float(scene_only["最佳帧评分"]) <= baseline_score + 2.0:
            return "场景前景对象", best_row
        return "无法判断", best_row
    if best_row["case_id"] in ("hide_auxiliary_model_children", "hide_suspicious_card_like", "keep_only_target_branch"):
        return "吊篮附属对象", best_row
    if best_row["case_id"] == "hide_entire_gondola":
        keep_only = rows.get("keep_only_target_branch")
        if keep_only and float(keep_only["最佳帧评分"]) <= baseline_score + 3.0:
            return "吊篮主体", best_row
        return "吊篮附属对象", best_row
    return "无法判断", best_row


def summarize_row(row: dict | None) -> dict | None:
    if row is None:
        return None
    return {
        "分组标签": row.get("分组标签") or row.get("case_id"),
        "camera_rig_translate_xyz": row.get("camera_rig_translate_xyz"),
        "最佳帧路径": row["最佳帧路径"],
        "最佳帧评分": row["最佳帧评分"],
        "最佳帧失败规则": row["最佳帧失败规则"],
        "最佳帧图像验收": row["最佳帧结果"]["图像验收"],
    }


def main() -> None:
    original_cfg = yaml.safe_load((OUT_DIR / "ptz_config.yaml").read_text(encoding="utf-8")) or {}
    temp_cfg = make_temp_cfg(original_cfg)
    TMP_CFG_PATH.write_text(yaml.safe_dump(temp_cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
    rtsp_url = str(temp_cfg["rtsp_url"])
    initial_zoom = float(temp_cfg.get("initial_zoom", 1.5))

    stream_proc = None
    z_rows = []
    y_rows = []
    content_rows = []
    try:
        log_event("experiment_start", report_path=str(REPORT_PATH), log_path=str(LOG_PATH), temp_config_path=str(TMP_CFG_PATH))
        stream_proc = start_stream(temp_cfg)
        wait_stream_ready()
        apply_fixed_ptz(initial_zoom)
        log_event("stream_ready", status=status())

        for delta in Z_DELTAS:
            xyz = [BASELINE_XYZ[0], BASELINE_XYZ[1], round(BASELINE_XYZ[2] + delta, 2)]
            z_rows.append(evaluate_pose(f"z_{delta:+.1f}", xyz, rtsp_url))
        z_best = max(z_rows, key=lambda row: base.frame_sort_key(row["最佳帧结果"]))

        z_best_xyz = list(z_best["camera_rig_translate_xyz"])
        for delta in Y_DELTAS:
            xyz = [z_best_xyz[0], round(BASELINE_XYZ[1] + delta, 2), z_best_xyz[2]]
            y_rows.append(evaluate_pose(f"y_{delta:+.1f}", xyz, rtsp_url))
        y_best = max(y_rows, key=lambda row: base.frame_sort_key(row["最佳帧结果"]))

        best_pose_xyz = list(y_best["camera_rig_translate_xyz"])
        set_camera_rig(best_pose_xyz)
        apply_fixed_ptz(initial_zoom)
        desc = scene_describe()
        log_event("scene_describe_best_pose", describe=desc)

        for case in content_case_specs(desc):
            content_rows.append(evaluate_content_case(case, rtsp_url))
            scene_post("clear")
            set_workers(1)

        best_pass_candidates = [row for row in (z_rows + y_rows + content_rows) if row["最佳帧通过验收"]]
        best_pass = max(best_pass_candidates, key=lambda row: base.frame_sort_key(row["最佳帧结果"])) if best_pass_candidates else None
        closest_fail = max([row for row in (z_rows + y_rows) if not row["最佳帧通过验收"]], key=lambda row: base.frame_sort_key(row["最佳帧结果"]), default=None)

        content_baseline = next((row for row in content_rows if row["case_id"] == "content_baseline"), None)
        best_content_improvement = None
        if content_baseline is not None:
            base_score = float(content_baseline["最佳帧评分"])
            candidates = []
            for row in content_rows:
                if row["case_id"] == "content_baseline":
                    continue
                candidates.append(
                    {
                        "case_id": row["case_id"],
                        "说明": row["说明"],
                        "评分提升": round(float(row["最佳帧评分"]) - base_score, 3),
                        "中心区黑像素占比改善": round(float(content_baseline["最佳帧结果"]["图像验收"]["中心区黑像素占比"]) - float(row["最佳帧结果"]["图像验收"]["中心区黑像素占比"]), 4),
                        "row": row,
                    }
                )
            if candidates:
                best_content_improvement = max(candidates, key=lambda item: (item["评分提升"], item["中心区黑像素占比改善"], item["row"]["最佳帧评分"]))

        binding_judgement, binding_support = judge_binding(content_rows)

        summary = {
            "z微扫最优点": summarize_row(z_best),
            "y微扫最优点": summarize_row(y_best),
            "是否拿到1张通过验收RTSP稳定帧": best_pass is not None,
            "通过验收最佳结果": summarize_row(best_pass),
            "最接近通过的一组参数": summarize_row(closest_fail),
            "内容隔离改善最大": {
                "case_id": best_content_improvement["case_id"],
                "说明": best_content_improvement["说明"],
                "评分提升": best_content_improvement["评分提升"],
                "中心区黑像素占比改善": best_content_improvement["中心区黑像素占比改善"],
                "最佳帧路径": best_content_improvement["row"]["最佳帧路径"],
                "最佳帧评分": best_content_improvement["row"]["最佳帧评分"],
                "最佳帧失败规则": best_content_improvement["row"]["最佳帧失败规则"],
            } if best_content_improvement else None,
            "黑块更像绑定在": binding_judgement,
            "绑定判断依据最佳组": {
                "case_id": binding_support["case_id"],
                "说明": binding_support["说明"],
                "最佳帧路径": binding_support["最佳帧路径"],
                "最佳帧评分": binding_support["最佳帧评分"],
            } if binding_support else None,
            "报告路径": str(REPORT_PATH),
            "实验日志路径": str(LOG_PATH),
            "流日志路径": str(STREAM_LOG_PATH),
            "临时配置路径": str(TMP_CFG_PATH),
            "全部新增RTSP抓帧路径": [frame["RTSP抓帧路径"] for row in (z_rows + y_rows + content_rows) for frame in row["逐帧结果"]],
        }

        report = {
            "summary": summary,
            "fixed_baseline": {
                "active_path": ACTIVE_PATH,
                "force_workers_count": FORCE_WORKERS_COUNT,
                "force_gondola_height": FORCE_GONDOLA_HEIGHT,
                "initial_pan": INITIAL_PAN,
                "initial_tilt": INITIAL_TILT,
                "baseline_camera_rig_translate_xyz": BASELINE_XYZ,
            },
            "scene_describe_best_pose": desc,
            "z_scan_results": z_rows,
            "y_scan_results": y_rows,
            "content_isolation_results": content_rows,
        }
        REPORT_PATH.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        log_event("experiment_report_written", report_path=str(REPORT_PATH))
        print(json.dumps(report, ensure_ascii=False), flush=True)
    except Exception as exc:
        log_event("experiment_failed", error=repr(exc), stream_tail=stream_tail())
        raise
    finally:
        stop_stream(stream_proc)


if __name__ == "__main__":
    main()
