#!/usr/bin/env python3
import datetime as dt
import json
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import yaml

import run_constrained_random_acceptance as base


OUT_DIR = Path("/home/uniubi/xuanyuan/camera05/camera03")
LAUNCHER_PORT = 18080
CTRL_PORT = 18081
RTSP_PORT = 18554
LAUNCHER_BASE = f"http://127.0.0.1:{LAUNCHER_PORT}"
CTRL_BASE = f"http://127.0.0.1:{CTRL_PORT}"

ACTIVE_PATH = "/World/Diaolan_Ver1_0_2026_07"
FORCE_WORKERS_COUNT = 1
FORCE_GONDOLA_HEIGHT = 24.98
INITIAL_PAN = 10.0
INITIAL_TILT = 0.0
BASELINE_XYZ = [88.0, -55.0, 33.0]

Z_DELTAS = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
Y_DELTAS = [0.0, -1.5, -1.0, -0.5, 0.5, 1.0, 1.5]


def now_tag() -> str:
    return dt.datetime.now().strftime("%Y%m%d_%H%M%S")


RUN_TAG = now_tag()
REPORT_PATH = OUT_DIR / f"_rtsp_diaolan03_local_content_report_{RUN_TAG}.json"
LOG_PATH = OUT_DIR / f"_rtsp_diaolan03_local_content_log_{RUN_TAG}.jsonl"
LAUNCHER_LOG_PATH = OUT_DIR / f"_rtsp_diaolan03_local_content_launcher_{RUN_TAG}.log"
TMP_CFG_PATH = OUT_DIR / f"_ptz_config_diaolan03_local_content_{RUN_TAG}.yaml"


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


def http_json(url: str, method: str = "GET", payload: dict | None = None) -> dict:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=30) as resp:
        return json.loads(resp.read().decode("utf-8"))


def make_temp_cfg(original_cfg: dict) -> dict:
    cfg = json.loads(json.dumps(original_cfg, ensure_ascii=False))
    cfg["launcher_port"] = LAUNCHER_PORT
    cfg["ctrl_port"] = CTRL_PORT
    cfg["rtsp_url"] = f"rtsp://localhost:{RTSP_PORT}/ptz_cam"
    mediamtx = dict(cfg.get("mediamtx") or {})
    mediamtx["port"] = RTSP_PORT
    cfg["mediamtx"] = mediamtx
    return cfg


def patch_base_transport() -> None:
    def patched_http_json(url: str, method: str = "GET", payload: dict | None = None) -> dict:
        url = url.replace("http://127.0.0.1:8080", LAUNCHER_BASE).replace("http://localhost:8080", LAUNCHER_BASE)
        return http_json(url, method=method, payload=payload)

    base.http_json = patched_http_json
    base.CFG_PATH = TMP_CFG_PATH


def launcher_tail() -> str:
    if not LAUNCHER_LOG_PATH.exists():
        return ""
    text = LAUNCHER_LOG_PATH.read_text(encoding="utf-8", errors="ignore")
    return "\n".join(text.splitlines()[-80:])


def start_local_launcher() -> subprocess.Popen:
    log_fp = LAUNCHER_LOG_PATH.open("w", encoding="utf-8")
    try:
        proc = subprocess.Popen(
            [
                sys.executable,
                "-u",
                str(OUT_DIR / "ptz_launcher.py"),
                "--config",
                str(TMP_CFG_PATH),
            ],
            cwd=str(OUT_DIR),
            stdout=log_fp,
            stderr=subprocess.STDOUT,
            text=True,
        )
    finally:
        log_fp.close()
    log_event("launcher_started", pid=proc.pid, launcher_log_path=str(LAUNCHER_LOG_PATH), temp_config_path=str(TMP_CFG_PATH))
    return proc


def wait_launcher_ready(proc: subprocess.Popen, timeout_s: int = 30) -> None:
    deadline = time.time() + timeout_s
    last_error = None
    while time.time() < deadline:
        if proc.poll() is not None:
            raise RuntimeError(f"临时 launcher 提前退出，tail=\n{launcher_tail()}")
        try:
            status = http_json(f"{LAUNCHER_BASE}/status")
            log_event("launcher_ready", status=status)
            return
        except Exception as exc:
            last_error = exc
            time.sleep(1.0)
    raise RuntimeError(f"临时 launcher 未就绪: {last_error!r}\nlauncher_tail=\n{launcher_tail()}")


def stop_local_launcher(proc: subprocess.Popen | None) -> None:
    if proc is None:
        return
    try:
        http_json(f"{LAUNCHER_BASE}/stop", method="POST", payload={})
    except Exception:
        pass
    if proc.poll() is None:
        proc.terminate()
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=10)
    log_event("launcher_stopped", returncode=proc.returncode, launcher_tail=launcher_tail())


def scene_post(action: str, payload: dict | None = None) -> dict:
    body = {"action": action}
    if payload:
        body.update(payload)
    return http_json(f"{CTRL_BASE}/scene/experiment", method="POST", payload=body)


def scene_get_describe() -> dict:
    return http_json(f"{CTRL_BASE}/scene/describe")


def set_workers(count: int) -> dict:
    return http_json(f"{CTRL_BASE}/scene/workers", method="POST", payload={"count": int(count)})


def candidate_case(case_tag: str, xyz: list[float]) -> dict:
    return {
        "实验类型": "本轮局部实验",
        "候选名": "DiaoLan_03_本轮最佳邻域",
        "分组标签": case_tag,
        "active_path": ACTIVE_PATH,
        "camera_rig_translate_xyz": [round(float(v), 2) for v in xyz],
        "force_gondola_height": FORCE_GONDOLA_HEIGHT,
        "force_workers_count": FORCE_WORKERS_COUNT,
        "initial_pan": INITIAL_PAN,
        "initial_tilt": INITIAL_TILT,
        "扰动维度": case_tag,
        "扰动量": 0.0,
        "case_tag": case_tag,
    }


def sort_key(row: dict):
    return base.frame_sort_key(row["最佳帧结果"])


def failed_rules(best_frame: dict) -> list[str]:
    return [k for k, ok in best_frame["验收规则"].items() if not ok]


def evaluate_pose(original_cfg: dict, rtsp_url: str, initial_zoom: float, case_tag: str, xyz: list[float]) -> dict:
    case = candidate_case(case_tag, xyz)
    row = base.evaluate_case(original_cfg, rtsp_url, initial_zoom, case)
    row["最佳帧失败规则"] = failed_rules(row["最佳帧结果"])
    log_event(
        "pose_case_done",
        case_tag=case_tag,
        xyz=row["camera_rig_translate_xyz"],
        best_score=row["最佳帧评分"],
        best_path=row["最佳帧路径"],
        best_pass=row["最佳帧通过验收"],
        failed_rules=row["最佳帧失败规则"],
        best_metrics=row["最佳帧结果"]["图像验收"],
    )
    return row


def prepare_best_pose(original_cfg: dict, initial_zoom: float, xyz: list[float]) -> None:
    candidate = {
        "active_path": ACTIVE_PATH,
        "camera_rig_translate_xyz": [float(v) for v in xyz],
        "force_gondola_height": FORCE_GONDOLA_HEIGHT,
        "force_workers_count": FORCE_WORKERS_COUNT,
        "initial_pan": INITIAL_PAN,
        "initial_tilt": INITIAL_TILT,
    }
    base.write_cfg(base.build_candidate_cfg(original_cfg, candidate, initial_zoom))
    base.restart_stream()
    base.wait_stream_ready()


def reset_runtime_state(workers: int = 1) -> dict:
    scene_post("clear")
    set_workers(workers)
    time.sleep(1.0)
    desc = scene_get_describe()
    log_event("runtime_reset", workers=workers, scene=desc)
    return desc


def root_child_for_target(model_path: str, target_branch_path: str) -> str:
    if not model_path or not target_branch_path or not target_branch_path.startswith(model_path + "/"):
        return target_branch_path
    suffix = target_branch_path[len(model_path) + 1 :]
    return f"{model_path}/{suffix.split('/')[0]}"


def content_hide_groups(desc: dict) -> list[dict]:
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
    keep_root = root_child_for_target(model_path, target_branch_path)
    keep_only_hide = [path for path in model_children if path != keep_root]
    return [
        {
            "case_id": "content_baseline",
            "说明": "最佳机位基线",
            "workers": 1,
            "hide_paths": [],
        },
        {
            "case_id": "workers_0",
            "说明": "workers=0",
            "workers": 0,
            "hide_paths": [],
        },
        {
            "case_id": "hide_auxiliary_model_children",
            "说明": "隐藏吊篮附属对象",
            "workers": 1,
            "hide_paths": auxiliary_model_children,
        },
        {
            "case_id": "hide_suspicious_card_like",
            "说明": "隐藏疑似 card/plane/quad/billboard/decal/screen 类",
            "workers": 1,
            "hide_paths": suspicious_paths,
        },
        {
            "case_id": "hide_entire_gondola",
            "说明": "隐藏整个吊篮仅保留场景",
            "workers": 1,
            "hide_paths": [model_path] if model_path else [],
        },
        {
            "case_id": "keep_only_target_branch",
            "说明": "仅保留吊篮主分支",
            "workers": 1,
            "hide_paths": keep_only_hide,
        },
    ]


def apply_runtime_visibility(case_id: str, workers: int, hide_paths: list[str]) -> dict:
    scene_post("clear")
    set_workers(workers)
    time.sleep(1.0)
    apply_resp = {"ok": True, "data": {}}
    if hide_paths:
        apply_resp = scene_post("apply_visibility", {"hide_paths": hide_paths})
    time.sleep(1.0)
    log_event(
        "runtime_visibility_applied",
        case_id=case_id,
        workers=workers,
        hide_paths=hide_paths,
        response=apply_resp,
    )
    return apply_resp


def evaluate_content_case(rtsp_url: str, case: dict) -> dict:
    apply_resp = apply_runtime_visibility(case["case_id"], case["workers"], case["hide_paths"])
    capture = base.capture_best_rtsp_frame_set(case["case_id"], rtsp_url)
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
    log_event(
        "content_case_done",
        case_id=case["case_id"],
        desc=case["说明"],
        best_score=row["最佳帧评分"],
        best_path=row["最佳帧路径"],
        best_pass=row["最佳帧通过验收"],
        failed_rules=row["最佳帧失败规则"],
        best_metrics=best["图像验收"],
    )
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
        delta = round(float(row["最佳帧评分"]) - baseline_score, 3)
        deltas.append((delta, row))
    if not deltas:
        return "无法判断", None
    best_delta, best_row = max(deltas, key=lambda item: item[0])
    if best_delta < 6.0:
        if rows.get("hide_entire_gondola") and float(rows["hide_entire_gondola"]["最佳帧评分"]) <= baseline_score + 2.0:
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


def summarize_best_fail(row: dict | None) -> dict | None:
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
    original_cfg_text = base.CFG_PATH.read_text(encoding="utf-8")
    original_cfg = yaml.safe_load(original_cfg_text) or {}
    temp_cfg = make_temp_cfg(original_cfg)
    TMP_CFG_PATH.write_text(yaml.safe_dump(temp_cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
    patch_base_transport()
    rtsp_url = str(temp_cfg["rtsp_url"])
    initial_zoom = float(original_cfg.get("initial_zoom", 1.5))
    launcher_proc: subprocess.Popen | None = None

    z_rows = []
    y_rows = []
    content_rows = []

    try:
        log_event("experiment_start", report_path=str(REPORT_PATH), log_path=str(LOG_PATH))
        launcher_proc = start_local_launcher()
        wait_launcher_ready(launcher_proc)

        for delta in Z_DELTAS:
            xyz = [BASELINE_XYZ[0], BASELINE_XYZ[1], round(BASELINE_XYZ[2] + delta, 2)]
            z_rows.append(evaluate_pose(original_cfg, rtsp_url, initial_zoom, f"z_{delta:+.1f}", xyz))
        z_best = max(z_rows, key=sort_key)

        z_best_xyz = list(z_best["camera_rig_translate_xyz"])
        for delta in Y_DELTAS:
            xyz = [z_best_xyz[0], round(BASELINE_XYZ[1] + delta, 2), z_best_xyz[2]]
            y_rows.append(evaluate_pose(original_cfg, rtsp_url, initial_zoom, f"y_{delta:+.1f}", xyz))
        y_best = max(y_rows, key=sort_key)

        best_pose_xyz = list(y_best["camera_rig_translate_xyz"])
        prepare_best_pose(original_cfg, initial_zoom, best_pose_xyz)
        desc = reset_runtime_state(workers=1)
        log_event("scene_describe_after_best_pose", describe=desc)

        for case in content_hide_groups(desc):
            content_rows.append(evaluate_content_case(rtsp_url, case))
            scene_post("clear")
            set_workers(1)
            time.sleep(1.0)

        best_pass_candidates = [row for row in (z_rows + y_rows + content_rows) if row.get("最佳帧通过验收")]
        best_pass = max(best_pass_candidates, key=sort_key) if best_pass_candidates else None
        pose_fail_rows = z_rows + y_rows
        closest_fail = max([row for row in pose_fail_rows if not row["最佳帧通过验收"]], key=sort_key, default=None)

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
                        "中心区黑像素占比改善": round(
                            float(content_baseline["最佳帧结果"]["图像验收"]["中心区黑像素占比"])
                            - float(row["最佳帧结果"]["图像验收"]["中心区黑像素占比"]),
                            4,
                        ),
                        "row": row,
                    }
                )
            if candidates:
                best_content_improvement = max(
                    candidates,
                    key=lambda item: (
                        item["评分提升"],
                        item["中心区黑像素占比改善"],
                        item["row"]["最佳帧评分"],
                    ),
                )

        binding_judgement, binding_support = judge_binding(content_rows)

        summary = {
            "z微扫最优点": {
                "分组标签": z_best["分组标签"],
                "camera_rig_translate_xyz": z_best["camera_rig_translate_xyz"],
                "最佳帧路径": z_best["最佳帧路径"],
                "最佳帧评分": z_best["最佳帧评分"],
                "最佳帧失败规则": z_best["最佳帧失败规则"],
                "最佳帧图像验收": z_best["最佳帧结果"]["图像验收"],
            },
            "y微扫最优点": {
                "分组标签": y_best["分组标签"],
                "camera_rig_translate_xyz": y_best["camera_rig_translate_xyz"],
                "最佳帧路径": y_best["最佳帧路径"],
                "最佳帧评分": y_best["最佳帧评分"],
                "最佳帧失败规则": y_best["最佳帧失败规则"],
                "最佳帧图像验收": y_best["最佳帧结果"]["图像验收"],
            },
            "是否拿到1张通过验收RTSP稳定帧": best_pass is not None,
            "通过验收最佳结果": summarize_best_fail(best_pass) if best_pass else None,
            "最接近通过的一组参数": summarize_best_fail(closest_fail),
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
            "临时launcher日志路径": str(LAUNCHER_LOG_PATH),
            "临时配置路径": str(TMP_CFG_PATH),
            "全部新增RTSP抓帧路径": [
                frame["RTSP抓帧路径"]
                for row in (z_rows + y_rows + content_rows)
                for frame in row["逐帧结果"]
            ],
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

        prepare_best_pose(original_cfg, initial_zoom, best_pose_xyz)
        reset_runtime_state(workers=1)
        print(json.dumps(report, ensure_ascii=False), flush=True)
    except Exception as exc:
        try:
            scene_post("clear")
        except Exception:
            pass
        log_event("experiment_failed", error=repr(exc))
        stop_local_launcher(launcher_proc)
        raise
    else:
        stop_local_launcher(launcher_proc)


if __name__ == "__main__":
    main()
