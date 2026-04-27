#!/usr/bin/env python3
import datetime as dt
import json
import math
import random
import subprocess
import time
import urllib.request
from pathlib import Path

import numpy as np
import yaml
from PIL import Image


CFG_PATH = Path("/home/uniubi/xuanyuan/camera05/camera03/ptz_config.yaml")
OUT_DIR = Path("/home/uniubi/xuanyuan/camera05/camera03")

MAX_ATTEMPTS = 8
DEFAULT_FORCE_WORKERS = 1
RTSP_STABILIZE_WAIT_S = 2.5
RTSP_MULTI_FRAME_COUNT = 7
RTSP_FRAME_INTERVAL_S = 0.35
LOCAL_SCAN_PERTURBATIONS = [
    ("y", -2.0),
    ("y", 2.0),
    ("z", -2.0),
    ("z", 2.0),
]
SEED_CANDIDATES = [
    {
        "候选名": "Diaolan_07_近通过",
        "active_path": "/World/Diaolan_Ver1_0_2026_07",
        "camera_rig_translate_xyz": [88.0, -55.0, 33.0],
        "force_gondola_height": 24.98,
        "force_workers_count": 1,
        "initial_pan": 10.0,
        "initial_tilt": 0.0,
    },
    {
        "候选名": "Diaolan_05_近通过",
        "active_path": "/World/Diaolan_Ver1_0_2026_05",
        "camera_rig_translate_xyz": [78.0, -45.0, 30.0],
        "force_gondola_height": 24.98,
        "force_workers_count": 1,
        "initial_pan": 20.0,
        "initial_tilt": 0.0,
    },
    {
        "候选名": "Diaolan_01_近通过",
        "active_path": "/World/Diaolan_Ver1_0_2026_01",
        "camera_rig_translate_xyz": [65.0, -35.0, 30.0],
        "force_gondola_height": 24.98,
        "force_workers_count": 1,
        "initial_pan": 10.0,
        "initial_tilt": 0.0,
    },
]
COARSE_POSES = [
    (0.0, -15.0),
    (0.0, -6.0),
    (0.0, 10.0),
    (0.0, 20.0),
    (0.0, 30.0),
    (-150.0, -6.0),
    (-120.0, -6.0),
    (-90.0, -6.0),
    (-60.0, -6.0),
    (-30.0, -6.0),
    (30.0, -6.0),
    (60.0, -6.0),
    (90.0, -6.0),
    (120.0, -6.0),
    (150.0, -6.0),
    (-120.0, 10.0),
    (-60.0, 10.0),
    (60.0, 10.0),
    (120.0, 10.0),
]


def http_json(url: str, method: str = "GET", payload: dict | None = None) -> dict:
    data = None
    headers = {}
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=20) as resp:
        return json.loads(resp.read().decode("utf-8"))


def restart_stream() -> None:
    try:
        http_json("http://127.0.0.1:8080/stop", method="POST", payload={})
    except Exception:
        pass
    for _ in range(45):
        try:
            st = http_json("http://127.0.0.1:8080/status")
            if st.get("isaac_state") == "stopped":
                break
        except Exception:
            pass
        time.sleep(1)
    http_json("http://127.0.0.1:8080/start", method="POST", payload={})
    for _ in range(150):
        st = http_json("http://127.0.0.1:8080/status")
        if st.get("isaac_state") == "running":
            time.sleep(2)
            return
        time.sleep(1)
    raise RuntimeError("Isaac stream did not enter running state")


def write_cfg(cfg: dict) -> None:
    CFG_PATH.write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")


def build_candidate_cfg(base_cfg: dict, candidate: dict, initial_zoom: float) -> dict:
    cfg = dict(base_cfg)
    cfg["diaolan_camera_sampling_enabled"] = False
    cfg["force_active_diaolan_path"] = candidate["active_path"]
    cfg["camera_rig_translate_xyz"] = [float(v) for v in candidate["camera_rig_translate_xyz"]]
    cfg["initial_pan"] = float(candidate["initial_pan"])
    cfg["initial_tilt"] = float(candidate["initial_tilt"])
    cfg["initial_zoom"] = float(initial_zoom)
    cfg["force_gondola_height"] = float(candidate["force_gondola_height"])
    cfg["force_workers_count"] = int(candidate["force_workers_count"])
    return cfg


def rtsp_local_url(rtsp_url: str) -> str:
    return rtsp_url.replace("localhost", "127.0.0.1")


def grab_rtsp_snapshot(tag: str, rtsp_url: str) -> tuple[Path, np.ndarray]:
    ts = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out_path = OUT_DIR / f"_rtsp_constrained_{tag}_{ts}.jpg"
    cmd = [
        "ffmpeg",
        "-y",
        "-rtsp_transport",
        "tcp",
        "-i",
        rtsp_local_url(rtsp_url),
        "-frames:v",
        "1",
        "-q:v",
        "2",
        str(out_path),
    ]
    last_error = None
    for _ in range(5):
        try:
            subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=30)
            return out_path, np.array(Image.open(out_path).convert("RGB"))
        except Exception as exc:
            last_error = exc
            time.sleep(1.5)
    raise RuntimeError(f"RTSP单帧抓取失败: {last_error!r}")


def geometry_rules(pm: dict) -> dict:
    return {
        "frustum内可见": pm.get("frustum内可见") is True,
        "中心在画面内": pm.get("中心在画面内") is True,
        "宽度占比阈值": 5.0 <= float(pm.get("宽度占比%", -1.0)) <= 25.0,
        "高度占比阈值": 8.0 <= float(pm.get("高度占比%", -1.0)) <= 35.0,
        "越界面积阈值": float(pm.get("越界面积占比", 1.0)) < 0.15,
        "距离阈值": 50.0 <= float(pm.get("distance_to_camera", -1.0)) <= 90.0,
    }


def candidate_score(pm: dict) -> float:
    cx, cy = pm.get("中心点px", [9999.0, 9999.0])
    return (
        abs(float(pm.get("宽度占比%", 999.0)) - 15.0)
        + abs(float(pm.get("高度占比%", 999.0)) - 20.0)
        + float(pm.get("越界面积占比", 1.0)) * 100.0
        + abs(float(cx) - 480.0) / 30.0
        + abs(float(cy) - 270.0) / 30.0
        + abs(float(pm.get("distance_to_camera", 999.0)) - 70.0) / 5.0
    )


def get_status() -> dict:
    return http_json("http://127.0.0.1:8080/status")


def wait_stream_ready(timeout_s: int = 60) -> dict:
    deadline = time.time() + timeout_s
    last = {}
    while time.time() < deadline:
        last = get_status()
        ptz = last.get("ptz") or {}
        stream = ptz.get("stream") or {}
        if (
            last.get("isaac_state") == "running"
            and stream.get("camera_bound_ok") is True
            and stream.get("ffmpeg_alive") is True
        ):
            time.sleep(2)
            return last
        time.sleep(1)
    raise RuntimeError(f"推流未就绪: {json.dumps(last, ensure_ascii=False)}")


def get_projection_metrics() -> dict:
    status = get_status()
    return ((status.get("ptz") or {}).get("stream") or {}).get("target_projection_metrics") or {}


def apply_ptz(pan: float, tilt: float, zoom: float) -> dict:
    http_json(
        "http://127.0.0.1:8080/control",
        method="POST",
        payload={"pan": float(pan), "tilt": float(tilt), "zoom": float(zoom)},
    )
    time.sleep(0.5)
    return get_projection_metrics()


def build_refine_poses(best_pan: float, best_tilt: float) -> list[tuple[float, float]]:
    out = []
    seen = set()
    for pan_off in (-30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0):
        for tilt_off in (-15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0):
            pan = round(max(-170.0, min(170.0, best_pan + pan_off)), 2)
            tilt = round(max(-90.0, min(30.0, best_tilt + tilt_off)), 2)
            key = (pan, tilt)
            if key not in seen:
                seen.add(key)
                out.append(key)
    return out


def search_live_geometry(initial_zoom: float) -> dict:
    best = None

    def consider(pan: float, tilt: float) -> None:
        nonlocal best
        pm = apply_ptz(pan, tilt, initial_zoom)
        if not pm or pm.get("error"):
            return
        row = {
            "initial_pan": round(pan, 2),
            "initial_tilt": round(tilt, 2),
            "projection_metrics": pm,
            "geometry_rules": geometry_rules(pm),
        }
        row["geometry_pass"] = all(row["geometry_rules"].values())
        row["score"] = round(candidate_score(pm), 3)
        if best is None or row["geometry_pass"] and not best["geometry_pass"] or row["score"] < best["score"]:
            best = row

    for pan, tilt in COARSE_POSES:
        consider(pan, tilt)
        if best and best["geometry_pass"]:
            break

    if best is not None:
        for pan, tilt in build_refine_poses(best["initial_pan"], best["initial_tilt"]):
            consider(pan, tilt)

    return best or {
        "initial_pan": 0.0,
        "initial_tilt": -15.0,
        "projection_metrics": {},
        "geometry_rules": {"投影指标缺失": False},
        "geometry_pass": False,
        "score": 9999.0,
    }


def largest_component_ratio(mask: np.ndarray) -> float:
    h, w = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    best = 0
    for y in range(h):
        for x in range(w):
            if not mask[y, x] or visited[y, x]:
                continue
            stack = [(y, x)]
            visited[y, x] = True
            size = 0
            while stack:
                cy, cx = stack.pop()
                size += 1
                for ny, nx in ((cy - 1, cx), (cy + 1, cx), (cy, cx - 1), (cy, cx + 1)):
                    if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True
                        stack.append((ny, nx))
            best = max(best, size)
    return float(best) / float(h * w)


def image_quality_metrics(img: np.ndarray, bbox_px: list[float]) -> dict:
    gray = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1] + 0.114 * img[:, :, 2]).astype(np.float32)
    mean_luma = float(np.mean(gray))
    black_mask = gray < 20.0
    black_ratio = float(np.mean(black_mask))
    h, w = gray.shape
    c0x, c1x = int(w * 0.2), int(w * 0.8)
    c0y, c1y = int(h * 0.2), int(h * 0.8)
    center = gray[c0y:c1y, c0x:c1x]
    center_black_ratio = float(np.mean(center < 20.0))

    small_mask = np.array(Image.fromarray((black_mask.astype(np.uint8) * 255)).resize((48, 27))).astype(bool)
    largest_black_component = largest_component_ratio(small_mask)

    x0 = max(0, min(w - 1, int(math.floor(bbox_px[0]))))
    y0 = max(0, min(h - 1, int(math.floor(bbox_px[1]))))
    x1 = max(x0 + 1, min(w, int(math.ceil(bbox_px[2]))))
    y1 = max(y0 + 1, min(h, int(math.ceil(bbox_px[3]))))
    crop = gray[y0:y1, x0:x1]
    if crop.size == 0:
        crop_std = 0.0
        crop_grad = 0.0
    else:
        crop_std = float(np.std(crop))
        gx = np.abs(np.diff(crop, axis=1)).mean() if crop.shape[1] > 1 else 0.0
        gy = np.abs(np.diff(crop, axis=0)).mean() if crop.shape[0] > 1 else 0.0
        crop_grad = float((gx + gy) * 0.5)

    return {
        "平均亮度": round(mean_luma, 2),
        "全图黑像素占比": round(black_ratio, 4),
        "中心区黑像素占比": round(center_black_ratio, 4),
        "最大黑块连通占比": round(largest_black_component, 4),
        "目标区域亮度标准差": round(crop_std, 2),
        "目标区域梯度均值": round(crop_grad, 2),
        "非整帧黑": mean_luma > 18.0 and black_ratio < 0.90,
        "非主黑块吞屏": center_black_ratio < 0.65 and black_ratio < 0.72 and largest_black_component < 0.40,
        "目标基本轮廓可见": crop_std > 14.0 and crop_grad > 5.5,
    }


def full_acceptance_rules(pm: dict, iq: dict) -> dict:
    rules = geometry_rules(pm)
    rules.update(
        {
            "非整帧黑": iq["非整帧黑"] is True,
            "非主黑块吞屏": iq["非主黑块吞屏"] is True,
            "目标基本轮廓可见": iq["目标基本轮廓可见"] is True,
        }
    )
    return rules


def target_paths() -> list[str]:
    return [
        "/World/Diaolan_Ver1_0_2026_01",
        "/World/Diaolan_Ver1_0_2026_05",
        "/World/Diaolan_Ver1_0_2026_06",
        "/World/Diaolan_Ver1_0_2026_07",
    ]


def image_rule_bools(iq: dict) -> dict:
    return {
        "非整帧黑": iq["非整帧黑"] is True,
        "非主黑块吞屏": iq["非主黑块吞屏"] is True,
        "目标基本轮廓可见": iq["目标基本轮廓可见"] is True,
    }


def frame_acceptance_score(pm: dict, iq: dict, rules: dict) -> float:
    geom = geometry_rules(pm) if pm else {"投影指标缺失": False}
    image_rules = image_rule_bools(iq)
    image_pass_count = sum(1 for ok in image_rules.values() if ok)
    score = 0.0
    score += 120.0 if all(rules.values()) else 0.0
    score += 20.0 if all(geom.values()) else 0.0
    score += image_pass_count * 18.0
    score += max(0.0, 30.0 - float(iq["中心区黑像素占比"]) * 40.0)
    score += max(0.0, 25.0 - float(iq["全图黑像素占比"]) * 25.0)
    score += max(0.0, 20.0 - float(iq["最大黑块连通占比"]) * 30.0)
    score += min(float(iq["目标区域亮度标准差"]), 40.0) * 0.5
    score += min(float(iq["目标区域梯度均值"]), 12.0) * 2.0
    score += min(float(iq["平均亮度"]), 60.0) * 0.2
    return round(score, 3)


def frame_sort_key(frame: dict) -> tuple:
    iq = frame["图像验收"]
    image_pass_count = sum(1 for ok in frame["图像规则"].values() if ok)
    return (
        1 if frame["是否通过验收"] else 0,
        1 if frame["几何通过"] else 0,
        image_pass_count,
        float(frame["评分"]),
        -float(iq["中心区黑像素占比"]),
        -float(iq["全图黑像素占比"]),
        -float(iq["最大黑块连通占比"]),
        float(iq["目标区域梯度均值"]),
        float(iq["目标区域亮度标准差"]),
        float(iq["平均亮度"]),
    )


def capture_best_rtsp_frame_set(case_tag: str, rtsp_url: str, frame_count: int = RTSP_MULTI_FRAME_COUNT) -> dict:
    time.sleep(RTSP_STABILIZE_WAIT_S)
    frames: list[dict] = []
    for idx in range(frame_count):
        pm = get_projection_metrics()
        snap_path, img = grab_rtsp_snapshot(f"{case_tag}_f{idx + 1:02d}", rtsp_url)
        bbox_px = (pm or {}).get("屏幕包围框px", [0.0, 0.0, 1.0, 1.0])
        iq = image_quality_metrics(img, bbox_px)
        geom = geometry_rules(pm) if pm else {"投影指标缺失": False}
        rules = full_acceptance_rules(pm, iq) if pm else {"投影指标缺失": False}
        image_rules = image_rule_bools(iq)
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
        frame["评分"] = frame_acceptance_score(pm, iq, rules)
        frames.append(frame)
        print(json.dumps({"逐帧打分": frame}, ensure_ascii=False))
        if idx + 1 < frame_count:
            time.sleep(RTSP_FRAME_INTERVAL_S)
    best = max(frames, key=frame_sort_key)
    return {
        "抓帧数量": frame_count,
        "逐帧结果": frames,
        "最佳帧": best,
        "最佳帧路径": best["RTSP抓帧路径"],
        "最佳帧评分": best["评分"],
        "最佳帧通过验收": best["是否通过验收"],
        "最佳帧图像通过": best["图像通过"],
    }


def apply_xyz_perturbation(base_xyz: list[float], axis: str | None, delta: float) -> list[float]:
    xyz = [float(v) for v in base_xyz]
    if axis is None:
        return [round(v, 2) for v in xyz]
    idx = {"x": 0, "y": 1, "z": 2}[axis]
    xyz[idx] = round(xyz[idx] + float(delta), 2)
    return [round(v, 2) for v in xyz]


def evaluate_case(base_cfg: dict, rtsp_url: str, initial_zoom: float, case: dict) -> dict:
    candidate = {
        "active_path": case["active_path"],
        "camera_rig_translate_xyz": [float(v) for v in case["camera_rig_translate_xyz"]],
        "force_gondola_height": float(case["force_gondola_height"]),
        "force_workers_count": int(case["force_workers_count"]),
        "initial_pan": float(case["initial_pan"]),
        "initial_tilt": float(case["initial_tilt"]),
    }
    write_cfg(build_candidate_cfg(base_cfg, candidate, initial_zoom))
    restart_stream()
    wait_stream_ready()
    capture = capture_best_rtsp_frame_set(case["case_tag"], rtsp_url)
    best = capture["最佳帧"]
    row = {
        "实验类型": case["实验类型"],
        "候选名": case["候选名"],
        "分组标签": case["分组标签"],
        "active_path": case["active_path"],
        "chosen_target_prim": http_json("http://127.0.0.1:8080/scene/describe")["data"]["target_branch_path"],
        "camera_rig_translate_xyz": candidate["camera_rig_translate_xyz"],
        "force_gondola_height": candidate["force_gondola_height"],
        "force_workers_count": candidate["force_workers_count"],
        "initial_pan": candidate["initial_pan"],
        "initial_tilt": candidate["initial_tilt"],
        "扰动维度": case["扰动维度"],
        "扰动量": case["扰动量"],
        "抓帧数量": capture["抓帧数量"],
        "逐帧结果": capture["逐帧结果"],
        "最佳帧路径": capture["最佳帧路径"],
        "最佳帧评分": capture["最佳帧评分"],
        "最佳帧通过验收": capture["最佳帧通过验收"],
        "最佳帧图像通过": capture["最佳帧图像通过"],
        "最佳帧结果": best,
        "是否通过验收": capture["最佳帧通过验收"],
    }
    print(json.dumps({"候选结果": row}, ensure_ascii=False))
    return row


def worker_group_specs(seed: dict) -> list[dict]:
    original_workers = int(seed["force_workers_count"])
    return [
        {"分组标签": "force_workers_count=0", "分组代号": "w0", "force_workers_count": 0},
        {"分组标签": "force_workers_count=1", "分组代号": "w1", "force_workers_count": 1},
        {
            "分组标签": f"force_workers_count=原值({original_workers})",
            "分组代号": f"worig{original_workers}",
            "force_workers_count": original_workers,
        },
    ]


def summarize_worker_effect(rows: list[dict], candidate_name: str) -> dict:
    target_rows = [row for row in rows if row["候选名"] == candidate_name and row["实验类型"] == "人数剥离基线"]
    by_group = {}
    for row in target_rows:
        best = row["最佳帧结果"]
        by_group[row["分组标签"]] = {
            "最佳帧路径": row["最佳帧路径"],
            "最佳帧评分": row["最佳帧评分"],
            "最佳帧通过验收": row["最佳帧通过验收"],
            "中心区黑像素占比": best["图像验收"]["中心区黑像素占比"],
            "全图黑像素占比": best["图像验收"]["全图黑像素占比"],
            "最大黑块连通占比": best["图像验收"]["最大黑块连通占比"],
            "目标区域梯度均值": best["图像验收"]["目标区域梯度均值"],
        }
    numeric_groups = [v for v in by_group.values()]
    if len(numeric_groups) >= 2:
        score_span = round(max(v["最佳帧评分"] for v in numeric_groups) - min(v["最佳帧评分"] for v in numeric_groups), 3)
        center_black_span = round(max(v["中心区黑像素占比"] for v in numeric_groups) - min(v["中心区黑像素占比"] for v in numeric_groups), 4)
    else:
        score_span = 0.0
        center_black_span = 0.0
    significant = score_span >= 18.0 or center_black_span >= 0.12
    return {
        "候选名": candidate_name,
        "分组结果": by_group,
        "评分跨度": score_span,
        "中心黑占比跨度": center_black_span,
        "人数影响显著": significant,
    }


def judge_black_block_cause(rows: list[dict], worker_summaries: list[dict]) -> dict:
    timing_hits = 0
    content_hits = 0
    timing_cases = []
    content_cases = []
    for row in rows:
        frames = row["逐帧结果"]
        if not frames:
            continue
        scores = [float(frame["评分"]) for frame in frames]
        center_black = [float(frame["图像验收"]["中心区黑像素占比"]) for frame in frames]
        if max(scores) - min(scores) >= 20.0 or max(center_black) - min(center_black) >= 0.18:
            timing_hits += 1
            timing_cases.append(row["候选名"] + "/" + row["分组标签"])
        best = row["最佳帧结果"]["图像验收"]
        if best["中心区黑像素占比"] >= 0.75 and best["最大黑块连通占比"] >= 0.60:
            content_hits += 1
            content_cases.append(row["候选名"] + "/" + row["分组标签"])
    worker_significant = sum(1 for item in worker_summaries if item["人数影响显著"])
    content_hits += worker_significant
    if timing_hits >= max(2, content_hits):
        conclusion = "更像时序问题"
    elif content_hits >= max(2, timing_hits + 1):
        conclusion = "更像内容级问题"
    else:
        conclusion = "时序与内容级混合，但暂不能单边定性"
    return {
        "结论": conclusion,
        "时序迹象命中数": timing_hits,
        "内容级迹象命中数": content_hits,
        "时序迹象样例": timing_cases,
        "内容级迹象样例": content_cases,
    }


def best_row(rows: list[dict], passed: bool | None = None) -> dict | None:
    filtered = rows
    if passed is not None:
        filtered = [row for row in rows if row["是否通过验收"] is passed]
    if not filtered:
        return None
    return max(filtered, key=lambda row: frame_sort_key(row["最佳帧结果"]))


def main() -> None:
    original_cfg_text = CFG_PATH.read_text(encoding="utf-8")
    original_cfg = yaml.safe_load(original_cfg_text) or {}
    rtsp_url = str(original_cfg["rtsp_url"])
    initial_zoom = float(original_cfg.get("initial_zoom", 1.5))
    report_path = OUT_DIR / f"_rtsp_local_scan_report_{dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.json"

    baseline_rows: list[dict] = []
    local_scan_rows: list[dict] = []
    best_overall: dict | None = None

    try:
        for seed in SEED_CANDIDATES:
            for group in worker_group_specs(seed):
                case = {
                    **seed,
                    **group,
                    "实验类型": "人数剥离基线",
                    "扰动维度": "baseline",
                    "扰动量": 0.0,
                    "camera_rig_translate_xyz": [float(v) for v in seed["camera_rig_translate_xyz"]],
                    "case_tag": f"{seed['active_path'].split('/')[-1]}_{group['分组代号']}_baseline",
                }
                row = evaluate_case(original_cfg, rtsp_url, initial_zoom, case)
                baseline_rows.append(row)
                if best_overall is None or frame_sort_key(row["最佳帧结果"]) > frame_sort_key(best_overall["最佳帧结果"]):
                    best_overall = row

        for seed in SEED_CANDIDATES:
            candidate_rows = [row for row in baseline_rows if row["候选名"] == seed["候选名"]]
            seed_best = max(candidate_rows, key=lambda row: frame_sort_key(row["最佳帧结果"]))
            for axis, delta in LOCAL_SCAN_PERTURBATIONS:
                case = {
                    **seed,
                    "实验类型": "局部扫描",
                    "分组标签": seed_best["分组标签"],
                    "force_workers_count": int(seed_best["force_workers_count"]),
                    "camera_rig_translate_xyz": apply_xyz_perturbation(seed_best["camera_rig_translate_xyz"], axis, delta),
                    "扰动维度": axis,
                    "扰动量": delta,
                    "case_tag": f"{seed['active_path'].split('/')[-1]}_{axis}_{delta:+.1f}_{int(seed_best['force_workers_count'])}",
                }
                row = evaluate_case(original_cfg, rtsp_url, initial_zoom, case)
                local_scan_rows.append(row)
                if best_overall is None or frame_sort_key(row["最佳帧结果"]) > frame_sort_key(best_overall["最佳帧结果"]):
                    best_overall = row

        all_rows = baseline_rows + local_scan_rows
        worker_summaries = [summarize_worker_effect(baseline_rows, seed["候选名"]) for seed in SEED_CANDIDATES]
        cause = judge_black_block_cause(all_rows, worker_summaries)
        best_pass = best_row(all_rows, passed=True)
        closest_fail = best_row(all_rows, passed=False)

        report = {
            "报告路径": str(report_path),
            "黑块判断": cause,
            "人物数量影响分析": worker_summaries,
            "是否拿到ODM可用RTSP稳定帧": best_pass is not None,
            "第一张通过验收的稳定帧": {
                "候选名": best_pass["候选名"],
                "active_path": best_pass["active_path"],
                "分组标签": best_pass["分组标签"],
                "camera_rig_translate_xyz": best_pass["camera_rig_translate_xyz"],
                "force_gondola_height": best_pass["force_gondola_height"],
                "force_workers_count": best_pass["force_workers_count"],
                "initial_pan": best_pass["initial_pan"],
                "initial_tilt": best_pass["initial_tilt"],
                "最佳帧路径": best_pass["最佳帧路径"],
                "最佳帧评分": best_pass["最佳帧评分"],
                "最佳帧结果": best_pass["最佳帧结果"],
            } if best_pass else None,
            "最接近通过但未通过": {
                "候选名": closest_fail["候选名"],
                "active_path": closest_fail["active_path"],
                "分组标签": closest_fail["分组标签"],
                "camera_rig_translate_xyz": closest_fail["camera_rig_translate_xyz"],
                "force_gondola_height": closest_fail["force_gondola_height"],
                "force_workers_count": closest_fail["force_workers_count"],
                "initial_pan": closest_fail["initial_pan"],
                "initial_tilt": closest_fail["initial_tilt"],
                "最佳帧路径": closest_fail["最佳帧路径"],
                "最佳帧评分": closest_fail["最佳帧评分"],
                "仍未满足的规则": [k for k, v in closest_fail["最佳帧结果"]["验收规则"].items() if not v],
                "最佳帧结果": closest_fail["最佳帧结果"],
            } if closest_fail else None,
            "人数剥离基线结果": baseline_rows,
            "局部扫描结果": local_scan_rows,
            "全部新增RTSP抓帧路径": [frame["RTSP抓帧路径"] for row in all_rows for frame in row["逐帧结果"]],
        }
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False))

        if best_overall is not None:
            write_cfg(build_candidate_cfg(original_cfg, best_overall, initial_zoom))
            restart_stream()
    except Exception:
        write_cfg(yaml.safe_load(original_cfg_text) or {})
        try:
            restart_stream()
        except Exception:
            pass
        raise


if __name__ == "__main__":
    main()
