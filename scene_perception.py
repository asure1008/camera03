# -*- coding: utf-8 -*-
"""
随机事件隐患判定：**以场景模型 / USD 运行时状态为准**（作业人员启用数量、吊篮世界高度等）。

- **legacy_rule_result / hazard_eval**：与 `_safe_build_hazard_eval` 同源语义的对照字段，**不是**最终 hazard 来源。
- **最终 hazard**（`final_hazard_result` / `random_event_hazard`）：默认仅来自 `evaluate_random_event_hazard_from_scene_state`（场景态）；
  **rule_id=11** 过曝为相机 snapshot 统计链路，`final_hazard_result.source` 为 `camera_overexposure`。
  相机投影、`target_projection_metrics`、`camera_view_perception` 等**不参与**场景态 1/2/3/4/5/12 判定，仅作调试信息写入 `scene_observation`。
"""

from __future__ import annotations

from copy import deepcopy
from typing import Any

import numpy as np


def _safe_plain_dict(raw: Any) -> dict[str, Any]:
    """
    将 hazard_eval['evidence'] 等字段规范为浅层原生 dict。

    Kit / carb 的 `carb.dictionary._dictionary.Item` 等对象常无 `.keys()`，直接 `dict(x)` 会抛
    AttributeError；此处优先用「可迭代键 + __getitem__」方式拷贝，失败则降级为 {}。
    """
    if raw is None:
        return {}
    if isinstance(raw, dict):
        return dict(raw)
    try:
        return {str(k): raw[k] for k in raw}
    except Exception:
        pass
    try:
        return dict(raw)
    except Exception:
        pass
    return {}

# Rule 11：snapshot JPEG 全图亮度（与 Isaac 解耦，供单测与 hazard_eval 复用）
_RULE11_BRIGHT_THRESH = 240.0 / 255.0
_RULE11_SAT_THRESH = 250.0 / 255.0
_RULE11_SAT_RATIO_HAZARD = 0.08
_RULE11_BRIGHT_RATIO_HAZARD = 0.35
_RULE11_BRIGHT_WITH_MEAN_MEAN = 0.82
_RULE11_MEAN_ALONE_HAZARD = 0.90


def analyze_jpeg_overexposure_metrics(jpeg_bytes: bytes | None) -> dict[str, Any]:
    """
    归一化 mean/bright/sat 占比与 capture_effective；仅对过亮/饱和判负。
    """
    out: dict[str, Any] = {
        "mean_brightness": None,
        "bright_pixel_ratio": None,
        "saturated_pixel_ratio": None,
        "capture_effective": None,
        "decode_error": None,
    }
    if not jpeg_bytes:
        out["decode_error"] = "no_jpeg"
        return out
    arr = None
    try:
        from io import BytesIO

        from PIL import Image as _PILImage  # noqa

        im = _PILImage.open(BytesIO(jpeg_bytes)).convert("L")
        arr = np.asarray(im, dtype=np.float32) / 255.0
    except Exception as pil_exc:
        try:
            import cv2 as _cv2  # noqa

            nparr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            bgr = _cv2.imdecode(nparr, _cv2.IMREAD_GRAYSCALE)
            if bgr is None:
                raise RuntimeError("cv2 imdecode failed") from pil_exc
            arr = bgr.astype(np.float32) / 255.0
        except Exception as cv_exc:
            out["decode_error"] = f"pil:{pil_exc};cv:{cv_exc}"
            return out
    if arr is None or arr.size == 0:
        out["decode_error"] = "empty_array"
        return out
    mean_b = float(np.mean(arr))
    bright = float(np.mean(arr >= _RULE11_BRIGHT_THRESH))
    sat = float(np.mean(arr >= _RULE11_SAT_THRESH))
    over_stress = bool(
        sat >= _RULE11_SAT_RATIO_HAZARD
        or (bright >= _RULE11_BRIGHT_RATIO_HAZARD and mean_b >= _RULE11_BRIGHT_WITH_MEAN_MEAN)
        or mean_b >= _RULE11_MEAN_ALONE_HAZARD
    )
    out["mean_brightness"] = round(mean_b, 4)
    out["bright_pixel_ratio"] = round(bright, 4)
    out["saturated_pixel_ratio"] = round(sat, 4)
    out["capture_effective"] = not over_stress
    return out


def rule11_overexposure_thresholds_meta() -> dict[str, Any]:
    return {
        "bright_norm": round(_RULE11_BRIGHT_THRESH, 4),
        "sat_norm": round(_RULE11_SAT_THRESH, 4),
        "sat_ratio_hazard": _RULE11_SAT_RATIO_HAZARD,
        "bright_ratio_hazard": _RULE11_BRIGHT_RATIO_HAZARD,
        "bright_with_mean_mean": _RULE11_BRIGHT_WITH_MEAN_MEAN,
        "mean_alone_hazard": _RULE11_MEAN_ALONE_HAZARD,
    }

# ---------------------------------------------------------------------------
# 事件注册表：归类与目标组织（非真值）
# ---------------------------------------------------------------------------
EVENT_REGISTRY: dict[int, dict[str, Any]] = {
    1: {
        "event_id": 1,
        "event_type": "防护栏/挡脚板缺失或不可用",
        "hazard_category": "设备状态与防护",
        "target_objects": ["active_gondola", "fanghulangan_prims"],
        "required_signals": ["usd_prim_visibility_guardrails"],
        "description": "基于活动吊篮载荷下 Fanghulangan/Front_01/02 代表性 Mesh 的存在性与可见性（场景状态）。",
    },
    2: {
        "event_id": 2,
        "event_type": "吊篮内作业人员超过2人",
        "hazard_category": "人员数量",
        "target_objects": ["active_gondola", "visible_workers"],
        "required_signals": [
            "active_gondola_scene_worker_count",
            "workers_scalar_or_per_diaolan_map",
        ],
        "description": "超员：当前活动吊篮上已启用作业人员数量（场景状态）>2。",
    },
    3: {
        "event_id": 3,
        "event_type": "吊篮单人作业",
        "hazard_category": "人员数量",
        "target_objects": ["active_gondola", "visible_workers"],
        "required_signals": [
            "active_gondola_scene_worker_count",
        ],
        "description": "单人作业：当前活动吊篮上已启用作业人员数量（场景状态）==1。",
    },
    4: {
        "event_id": 4,
        "event_type": "作业结束后吊篮未降至地面",
        "hazard_category": "设备状态与高度",
        "target_objects": ["active_gondola"],
        "required_signals": [
            "active_gondola_scene_worker_count",
            "gondola_world_height_axis",
        ],
        "description": "未落地：作业人员数为 0 时，用活动吊篮篮体世界高度轴与地面对照（场景状态）。",
    },
    5: {
        "event_id": 5,
        "event_type": "吊篮作业人员安全绳未单人使用（数量口径）",
        "hazard_category": "设备状态与防护",
        "target_objects": ["active_gondola", "safety_rope_anchors"],
        "required_signals": [
            "active_gondola_scene_worker_count",
            "safety_rope_anchor_effective_count",
        ],
        "description": "单人单绳（场景态数量）：有效安全绳实例数 < 作业人数则隐患为真；不做人物-绳索绑定跟踪。",
    },
    12: {
        "event_id": 12,
        "event_type": "限位装置不符",
        "hazard_category": "设备状态与防护",
        "target_objects": ["active_gondola", "limitstop_prims"],
        "required_signals": ["usd_prim_visibility_limitstops_per_steel_rope"],
        "description": "各钢丝绳分支下 Limitstop 几何启用且可见（场景状态）。",
    },
    13: {
        "event_id": 13,
        "event_type": "防坠安全锁不符",
        "hazard_category": "设备状态与防护",
        "target_objects": ["active_gondola", "fallarrestor_prims"],
        "required_signals": ["usd_prim_visibility_fallarrestors_per_steel_rope"],
        "description": "各钢丝绳分支下 FallArrestor 几何启用且可见（场景状态）。",
    },
    11: {
        "event_id": 11,
        "event_type": "光照过大导致模拟相机过曝，无法有效捕捉目标画面",
        "hazard_category": "环境与成像",
        "target_objects": ["camera_view", "environment_lighting"],
        "required_signals": ["snapshot_jpeg_luma_histogram"],
        "description": "基于 snapshot 缓存 JPEG 的全图亮度/饱和占比；与 USD 场景人数等无耦合。",
    },
}

_DEFAULT_GROUND_Z_BASELINE = 0.12
_DEFAULT_GROUND_EPS = 0.5

# 已由场景状态评估实现的事件（其余 rule_id → unsupported_by_scene_state_evaluation_yet）
SCENE_STATE_SUPPORTED_RULE_IDS: frozenset[int] = frozenset({1, 2, 3, 4, 5, 12, 13})
# 相机几何/工人投影链路仅覆盖 2/3/4；1/5/12/13 为纯场景态 USD 判定
CAMERA_GEOMETRY_PERCEPTION_RULE_IDS: frozenset[int] = frozenset({2, 3, 4})
# 兼容旧名（ptz_stream 等仍从本模块导入该名）
CAMERA_PERCEPTION_SUPPORTED_RULE_IDS: frozenset[int] = CAMERA_GEOMETRY_PERCEPTION_RULE_IDS


def _default_perception_cfg(raw: Any) -> dict[str, Any]:
    base = {
        "include_projection_in_observation": True,
        "occlusion": {"enabled": False},
        "min_target_area_ratio": 0.001,
        "min_worker_area_ratio": 0.00012,
    }
    if not isinstance(raw, dict):
        return base
    out = deepcopy(base)
    out["include_projection_in_observation"] = bool(
        raw.get("include_projection_in_observation", base["include_projection_in_observation"])
    )
    for key in ("min_target_area_ratio", "min_worker_area_ratio"):
        try:
            v = float(raw.get(key, base[key]))
            if v > 0:
                out[key] = v
        except (TypeError, ValueError):
            pass
    occ = raw.get("occlusion")
    if isinstance(occ, dict):
        out["occlusion"] = {"enabled": bool(occ.get("enabled", False))}
    return out


def _resolve_rule_id(
    hazard_eval: dict,
    request_meta: dict | None,
    *,
    randomize_event_meta: dict[str, Any] | None = None,
) -> int | None:
    """
    解析本次要走的规则 id：优先 hazard_eval（已与 randomize 侧写入的 rule 对齐），
    其次 HTTP 请求元字段，最后仅消费 `result.randomize_event_meta`（由事件生成侧写入，非几何推断）。
    """
    rid = hazard_eval.get("rule_id")
    if rid is not None:
        try:
            return int(rid)
        except (TypeError, ValueError):
            pass
    if isinstance(request_meta, dict):
        for key in ("rule_id", "event_id"):
            if key in request_meta:
                try:
                    return int(request_meta[key])
                except (TypeError, ValueError):
                    return None
    if isinstance(randomize_event_meta, dict) and randomize_event_meta.get("rule_id") is not None:
        try:
            return int(randomize_event_meta["rule_id"])
        except (TypeError, ValueError):
            return None
    return None


def _registry_entry(rule_id: int | None) -> dict[str, Any] | None:
    if rule_id is None:
        return None
    e = EVENT_REGISTRY.get(int(rule_id))
    return deepcopy(e) if e else None


def _build_legacy_rule_result(hazard_eval: dict) -> dict[str, Any]:
    return {
        "hazard": hazard_eval.get("has_hazard"),
        "reason": str(hazard_eval.get("reason") or ""),
        "supported": hazard_eval.get("supported"),
        "rule_id": hazard_eval.get("rule_id"),
        "rule_name": hazard_eval.get("rule_name"),
    }


def _ground_params_from_hazard_evidence(hazard_eval: dict) -> tuple[float, float]:
    ev = hazard_eval.get("evidence")
    if not isinstance(ev, dict):
        return _DEFAULT_GROUND_Z_BASELINE, _DEFAULT_GROUND_EPS
    gz = ev.get("ground_z_baseline")
    ge = ev.get("ground_eps")
    try:
        z0 = float(gz) if gz is not None else _DEFAULT_GROUND_Z_BASELINE
    except (TypeError, ValueError):
        z0 = _DEFAULT_GROUND_Z_BASELINE
    try:
        e0 = float(ge) if ge is not None else _DEFAULT_GROUND_EPS
    except (TypeError, ValueError):
        e0 = _DEFAULT_GROUND_EPS
    return z0, e0


def _rule4_ground_hazard(gondola_world_z: float, worker_count: int, z_base: float, z_eps: float) -> bool:
    is_on_ground = gondola_world_z < z_base + z_eps
    if not is_on_ground and int(worker_count) == 0:
        return True
    return False


def _extract_scene_worker_count(result: dict[str, Any]) -> tuple[int | None, str | None]:
    """
    活动吊篮上参与当前场景逻辑的作业人员数量（与 randomize 写入的 `workers` 一致）。
    来源为 USD/场景随机状态，**不是**相机视锥内可见性。
    """
    w = result.get("workers")
    if w is not None:
        try:
            return int(w), "workers"
        except (TypeError, ValueError):
            pass
    ap = str(result.get("active_diaolan_path") or "").strip()
    wb = result.get("workers_visible_count_by_diaolan_path")
    if isinstance(wb, dict) and ap:
        raw = wb.get(ap)
        if raw is not None:
            try:
                return int(raw), "workers_visible_count_by_diaolan_path"
            except (TypeError, ValueError):
                pass
    return None, None


def _extract_scene_gondola_world_height(result: dict[str, Any]) -> tuple[float | None, str | None]:
    """活动吊篮篮体控制 prim 的世界高度轴读数（场景状态）。"""
    ap = str(result.get("active_diaolan_path") or "").strip()
    gh = result.get("gondola_heights")
    if isinstance(gh, dict) and ap and ap in gh:
        try:
            return float(gh[ap]), "gondola_heights[active]"
        except (TypeError, ValueError):
            pass
    hd = result.get("height_debug")
    if isinstance(hd, dict) and hd.get("final_world_z") is not None:
        try:
            return float(hd["final_world_z"]), "height_debug.final_world_z"
        except (TypeError, ValueError):
            pass
    gy = result.get("gondola_y")
    if gy is not None:
        try:
            return float(gy), "gondola_y"
        except (TypeError, ValueError):
            pass
    return None, None


def evaluate_random_event_hazard_from_scene_state(
    result: dict[str, Any],
    hazard_eval: dict[str, Any],
    request_meta: dict[str, Any] | None,
) -> dict[str, Any]:
    """
    基于 randomize 结果中的场景模型字段计算隐患，**不**使用相机投影、覆盖或 camera_view_perception。
    返回结构与旧版「评估子集」兼容：supported、hazard、reason、evidence。
    """
    gen_meta = result.get("randomize_event_meta") if isinstance(result.get("randomize_event_meta"), dict) else None
    rule_id = _resolve_rule_id(hazard_eval, request_meta, randomize_event_meta=gen_meta)
    common_base: dict[str, Any] = {
        "resolved_rule_id": rule_id,
        "evaluation": "scene_state",
    }
    if rule_id is None:
        return {
            **common_base,
            "supported": False,
            "observable": None,
            "coverage_sufficient": None,
            "hazard": None,
            "reason": "unsupported_by_scene_state_evaluation_yet",
            "evidence": {"note": "缺少 rule_id/event_id"},
        }
    rid = int(rule_id)
    if rid == 11:
        he = hazard_eval if isinstance(hazard_eval, dict) else {}
        supported = bool(he.get("supported"))
        haz = he.get("hazard")
        if haz is None:
            haz = he.get("has_hazard")
        ev11 = _safe_plain_dict(he.get("evidence"))
        reason11 = str(he.get("reason") or "")
        return {
            "resolved_rule_id": rid,
            "evaluation": "camera_overexposure_snapshot",
            "supported": supported,
            "observable": None,
            "coverage_sufficient": None,
            "hazard": haz,
            "reason": reason11,
            "evidence": ev11,
        }
    if rid not in SCENE_STATE_SUPPORTED_RULE_IDS:
        return {
            **common_base,
            "supported": False,
            "observable": None,
            "coverage_sufficient": None,
            "hazard": None,
            "reason": "unsupported_by_scene_state_evaluation_yet",
            "evidence": {"rule_id": rid},
        }

    wc, wc_src = _extract_scene_worker_count(result)
    gh, gh_src = _extract_scene_gondola_world_height(result)
    z0, zeps = _ground_params_from_hazard_evidence(hazard_eval)
    th = z0 + zeps

    if rid == 2:
        ev: dict[str, Any] = {
            "worker_count": wc,
            "worker_count_source": wc_src,
            "active_diaolan_path": str(result.get("active_diaolan_path") or ""),
        }
        if wc is None:
            return {
                **common_base,
                "supported": True,
                "observable": None,
                "coverage_sufficient": None,
                "hazard": None,
                "reason": "当前缺少活动吊篮作业人员数量（场景状态），无法判断是否超过 2 人",
                "evidence": ev,
            }
        hz = int(wc) > 2
        reason = (
            f"场景状态：当前活动吊篮作业人员数={wc}，超过 2 人"
            if hz
            else f"场景状态：当前活动吊篮作业人员数={wc}，未超过 2 人"
        )
        return {
            **common_base,
            "supported": True,
            "observable": None,
            "coverage_sufficient": None,
            "hazard": hz,
            "reason": reason,
            "evidence": ev,
        }

    if rid == 3:
        ev = {
            "worker_count": wc,
            "worker_count_source": wc_src,
            "active_diaolan_path": str(result.get("active_diaolan_path") or ""),
        }
        if wc is None:
            return {
                **common_base,
                "supported": True,
                "observable": None,
                "coverage_sufficient": None,
                "hazard": None,
                "reason": "当前缺少活动吊篮作业人员数量（场景状态），无法判断是否为单人作业",
                "evidence": ev,
            }
        hz = int(wc) == 1
        reason = (
            f"场景状态：当前活动吊篮作业人员数={wc}，构成单人作业"
            if hz
            else f"场景状态：当前活动吊篮作业人员数={wc}，非单人作业"
        )
        return {
            **common_base,
            "supported": True,
            "observable": None,
            "coverage_sufficient": None,
            "hazard": hz,
            "reason": reason,
            "evidence": ev,
        }

    if rid == 1:
        try:
            import omni.usd

            st = omni.usd.get_context().get_stage()
        except Exception:
            st = None
        ap = str(result.get("active_diaolan_path") or "")
        if st is None:
            return {
                **common_base,
                "supported": False,
                "observable": None,
                "coverage_sufficient": None,
                "hazard": None,
                "reason": "usd_stage_unavailable",
                "evidence": {"active_diaolan_path": ap, "rule_id": rid},
            }
        from diaolan_randomizer import evaluate_scene_rule_guardrails

        r = evaluate_scene_rule_guardrails(st, ap)
        ev1 = _safe_plain_dict(r.get("evidence"))
        ev1["active_diaolan_path"] = ap
        return {
            **common_base,
            "supported": bool(r.get("supported")),
            "observable": None,
            "coverage_sufficient": None,
            "hazard": r.get("has_hazard"),
            "reason": f"场景状态（USD，非相机）：{r.get('reason')}",
            "evidence": ev1,
        }

    if rid == 5:
        try:
            import omni.usd

            st = omni.usd.get_context().get_stage()
        except Exception:
            st = None
        ap = str(result.get("active_diaolan_path") or "")
        if st is None:
            return {
                **common_base,
                "supported": False,
                "observable": None,
                "coverage_sufficient": None,
                "hazard": None,
                "reason": "usd_stage_unavailable",
                "evidence": {"active_diaolan_path": ap, "rule_id": rid},
            }
        from diaolan_randomizer import evaluate_scene_rule_safety_ropes

        r = evaluate_scene_rule_safety_ropes(st, ap, wc)
        ev5 = _safe_plain_dict(r.get("evidence"))
        ev5["active_diaolan_path"] = ap
        ev5["worker_count_source"] = wc_src
        return {
            **common_base,
            "supported": bool(r.get("supported")),
            "observable": None,
            "coverage_sufficient": None,
            "hazard": r.get("has_hazard"),
            "reason": f"场景状态（USD，非相机）：{r.get('reason')}",
            "evidence": ev5,
        }

    if rid == 12:
        try:
            import omni.usd

            st = omni.usd.get_context().get_stage()
        except Exception:
            st = None
        ap = str(result.get("active_diaolan_path") or "")
        if st is None:
            return {
                **common_base,
                "supported": False,
                "observable": None,
                "coverage_sufficient": None,
                "hazard": None,
                "reason": "usd_stage_unavailable",
                "evidence": {"active_diaolan_path": ap, "rule_id": rid},
            }
        from diaolan_randomizer import evaluate_scene_rule_limitstops

        r = evaluate_scene_rule_limitstops(st, ap)
        ev12 = _safe_plain_dict(r.get("evidence"))
        ev12["active_diaolan_path"] = ap
        return {
            **common_base,
            "supported": bool(r.get("supported")),
            "observable": None,
            "coverage_sufficient": None,
            "hazard": r.get("has_hazard"),
            "reason": f"场景状态（USD，非相机）：{r.get('reason')}",
            "evidence": ev12,
        }

    if rid == 4:
        ev = {
            "worker_count": wc,
            "worker_count_source": wc_src,
            "gondola_world_height": gh,
            "gondola_world_height_source": gh_src,
            "ground_z_baseline": z0,
            "ground_eps": zeps,
            "ground_threshold": th,
            "active_diaolan_path": str(result.get("active_diaolan_path") or ""),
        }
        if wc is None or gh is None:
            return {
                **common_base,
                "supported": True,
                "observable": None,
                "coverage_sufficient": None,
                "hazard": None,
                "reason": "当前缺少活动吊篮作业人员数量或篮体世界高度（场景状态），无法判断是否已降至地面",
                "evidence": ev,
            }
        if int(wc) != 0:
            return {
                **common_base,
                "supported": True,
                "observable": None,
                "coverage_sufficient": None,
                "hazard": False,
                "reason": f"场景状态：当前吊篮作业人员数={wc}，作业未结束，本规则不构成「未降至地面」隐患",
                "evidence": ev,
            }
        hz = _rule4_ground_hazard(float(gh), 0, z0, zeps)
        reason = (
            f"场景状态：人数=0，篮体世界高度轴={float(gh):.4f}，地面阈值={z0:.4f}+eps{zeps:.4f}={th:.4f}，未落地隐患={hz}"
        )
        return {
            **common_base,
            "supported": True,
            "observable": None,
            "coverage_sufficient": None,
            "hazard": bool(hz),
            "reason": reason,
            "evidence": ev,
        }

    return {
        **common_base,
        "supported": False,
        "observable": None,
        "coverage_sufficient": None,
        "hazard": None,
        "reason": "unsupported_by_scene_state_evaluation_yet",
        "evidence": {"rule_id": rid, "note": "unexpected_rule_branch"},
    }


def _finalize_hazard_from_scene_evaluation(ev: dict[str, Any]) -> dict[str, Any]:
    """整理为对外 `final_hazard_result`；真值来源为场景状态，source 恒为 scene_state。"""
    if not ev.get("supported"):
        return {
            "source": "scene_state",
            "supported": False,
            "hazard": None,
            "conclusive": False,
            "reason": str(ev.get("reason") or "unsupported_by_scene_state_evaluation_yet"),
        }
    haz = ev.get("hazard")
    if isinstance(haz, bool):
        return {
            "source": "scene_state",
            "supported": True,
            "hazard": haz,
            "conclusive": True,
            "reason": str(ev.get("reason") or ""),
        }
    return {
        "source": "scene_state",
        "supported": True,
        "hazard": None,
        "conclusive": False,
        "reason": str(ev.get("reason") or "insufficient_scene_state_evidence"),
    }


def _gondola_observable_from_projection(
    projection_metrics: dict | None,
    min_area_ratio: float,
) -> tuple[bool | None, str]:
    """篮体是否在成像平面内且包围盒占比足够（不含深度真遮挡，仅几何裁剪）。"""
    if not isinstance(projection_metrics, dict):
        return None, "缺少篮体 target_projection_metrics"
    if "error" in projection_metrics:
        return None, f"篮体 USD 投影失败：{projection_metrics.get('error')}"
    if not projection_metrics.get("frustum内可见"):
        return False, "篮体世界包围盒与当前相机视锥不相交"
    if not projection_metrics.get("中心在画面内"):
        return False, "篮体投影中心不在成像矩形内"
    try:
        ar = float(projection_metrics.get("目标像素占比") or 0.0)
    except (TypeError, ValueError):
        ar = 0.0
    try:
        onr = float(projection_metrics.get("画面内交集像素占比") or 0.0)
    except (TypeError, ValueError):
        onr = 0.0
    eff = max(ar, onr)
    if eff < float(min_area_ratio):
        return False, f"篮体在画面内有效占比 {eff:.6f} < 阈值 {min_area_ratio}，覆盖不足"
    return True, "篮体视锥命中且画面占比满足阈值"


def _build_detection_placeholder() -> dict[str, Any]:
    return {
        "ran": False,
        "objects": [],
        "confidence": None,
        "supports_event_hazard": None,
        "note": "reserved_for_future_detector",
    }


def evaluate_random_event_from_camera_view(
    *,
    rule_id: int | None,
    hazard_eval: dict,
    projection_metrics: dict | None,
    camera_view: dict | None,
    perception_cfg: dict[str, Any],
) -> dict[str, Any]:
    """
    统一入口：仅使用相机位姿/视锥/投影与 `camera_view_perception`（作业人员逐 prim 投影）。
    不使用 workers_visible_count_by_diaolan_path / rule 真值包装 hazard。
    """
    min_g = float(perception_cfg.get("min_target_area_ratio") or 0.001)
    occlusion_note = (
        "几何遮挡分析已打开但尚未接入射线/深度"
        if bool((perception_cfg.get("occlusion") or {}).get("enabled"))
        else "未启用深度遮挡；结论仅基于视锥与成像矩形裁剪"
    )

    if rule_id is None or int(rule_id) not in CAMERA_GEOMETRY_PERCEPTION_RULE_IDS:
        return {
            "supported": False,
            "observable": None,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": "unsupported_by_camera_perception_yet",
            "evidence": {"rule_id": rule_id, "occlusion_note": occlusion_note},
        }

    if not hazard_eval.get("supported"):
        return {
            "supported": False,
            "observable": None,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": "unsupported_by_camera_perception_yet",
            "evidence": {"rule_id": rule_id, "hazard_eval_supported": False},
        }

    if not isinstance(camera_view, dict) or camera_view.get("error"):
        err = camera_view.get("error") if isinstance(camera_view, dict) else "camera_view_perception_missing"
        return {
            "supported": True,
            "observable": None,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": f"camera_view_perception 不可用：{err}",
            "evidence": {"camera_view": camera_view, "occlusion_note": occlusion_note},
        }

    g_obs, g_note = _gondola_observable_from_projection(projection_metrics, min_g)
    n_cam = int(camera_view.get("camera_view_worker_count") or 0)
    n_ren = int(camera_view.get("rendered_worker_paths_under_active_count") or 0)
    all_seen = bool(camera_view.get("workers_all_projected_in_view"))

    ev_common: dict[str, Any] = {
        "gondola_observable": g_obs,
        "gondola_observable_reason": g_note,
        "camera_view_worker_count": n_cam,
        "rendered_worker_paths_under_active_count": n_ren,
        "workers_all_projected_in_view": all_seen,
        "camera_view_perception": {
            "image_wh": camera_view.get("image_wh"),
            "camera_prim_path": camera_view.get("camera_prim_path"),
            "workers": camera_view.get("workers"),
            "gondola_world_height_axis": camera_view.get("gondola_world_height_axis"),
        },
        "target_projection_metrics": projection_metrics if isinstance(projection_metrics, dict) else {},
        "occlusion_note": occlusion_note,
    }

    rid = int(rule_id)

    if rid == 2:
        if g_obs is not True:
            return {
                "supported": True,
                "observable": g_obs,
                "coverage_sufficient": False,
                "hazard": None,
                "reason": f"超员判定需先确认篮体在相机画面内：{g_note}",
                "evidence": ev_common,
            }
        if n_cam >= 3:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": True,
                "hazard": True,
                "reason": f"相机几何可见作业人员投影数={n_cam}（≥3），判定超员隐患",
                "evidence": ev_common,
            }
        if n_ren == 0:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": True,
                "hazard": False,
                "reason": "当前无已渲染作业人员（USD 可见集合为空），相机画面不构成超员",
                "evidence": ev_common,
            }
        if all_seen and n_cam <= 2:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": True,
                "hazard": False,
                "reason": f"全部已渲染作业人员均在画面几何可见，计数={n_cam}，未构成超员",
                "evidence": ev_common,
            }
        return {
            "supported": True,
            "observable": True,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": "部分已渲染作业人员未在画面内或占比过低，无法确认是否仍有未入画人员，不足以判定超员",
            "evidence": ev_common,
        }

    if rid == 3:
        if g_obs is not True:
            return {
                "supported": True,
                "observable": g_obs,
                "coverage_sufficient": False,
                "hazard": None,
                "reason": f"单人作业判定需篮体在相机画面内：{g_note}",
                "evidence": ev_common,
            }
        if n_ren == 1 and all_seen and n_cam == 1:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": True,
                "hazard": True,
                "reason": "仅一名已渲染作业人员且几何投影完整入画，判定单人作业隐患",
                "evidence": ev_common,
            }
        if n_ren >= 2 and all_seen:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": True,
                "hazard": False,
                "reason": f"已渲染作业人员≥2 且均在画面可见（{n_ren}），不构成单人作业隐患语义",
                "evidence": ev_common,
            }
        if n_ren == 0:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": True,
                "hazard": False,
                "reason": "无已渲染作业人员，不构成「单人作业」隐患",
                "evidence": ev_common,
            }
        if n_ren == 1 and not all_seen:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": False,
                "hazard": None,
                "reason": "仅渲染一人但其投影未充分入画，无法确认单人作业",
                "evidence": ev_common,
            }
        return {
            "supported": True,
            "observable": True,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": "无法从当前画面唯一确定单人/多人状态（作业人员投影不完整）",
            "evidence": ev_common,
        }

    # rid == 4
    if n_ren > 0:
        if all_seen:
            return {
                "supported": True,
                "observable": True,
                "coverage_sufficient": True,
                "hazard": False,
                "reason": f"仍可见已渲染作业人员（{n_ren} 人），作业未结束，本规则不构成「未降至地面」隐患",
                "evidence": ev_common,
            }
        return {
            "supported": True,
            "observable": None,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": "存在已渲染作业人员但未能全部在画面几何确认，无法判定作业是否已结束",
            "evidence": ev_common,
        }

    if g_obs is not True:
        return {
            "supported": True,
            "observable": g_obs,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": f"作业结束场景需观察篮体高度，但篮体未满足相机几何可观测：{g_note}",
            "evidence": ev_common,
        }

    gh = camera_view.get("gondola_world_height_axis")
    try:
        gh_f = float(gh) if gh is not None else None
    except (TypeError, ValueError):
        gh_f = None
    if gh_f is None:
        return {
            "supported": True,
            "observable": True,
            "coverage_sufficient": False,
            "hazard": None,
            "reason": "篮体已入画但缺少可读取的世界高度轴读数",
            "evidence": ev_common,
        }

    z0, zeps = _ground_params_from_hazard_evidence(hazard_eval)
    hz = _rule4_ground_hazard(float(gh_f), 0, z0, zeps)
    th = z0 + zeps
    return {
        "supported": True,
        "observable": True,
        "coverage_sufficient": True,
        "hazard": hz,
        "reason": (
            f"零作业人员入画且篮体可观测：世界高度轴={float(gh_f):.4f}，"
            f"地面阈值={z0:.4f}+eps{zeps:.4f}={th:.4f}，未落地隐患={hz}"
        ),
        "evidence": {**ev_common, "ground_z_baseline": z0, "ground_eps": zeps},
    }


def _finalize_hazard_from_camera_evaluation(ev: dict[str, Any]) -> dict[str, Any]:
    """最终对外块：永不回退 legacy_rule.hazard。"""
    if not ev.get("supported"):
        return {
            "source": "unsupported",
            "hazard": None,
            "conclusive": False,
            "reason": str(ev.get("reason") or "unsupported_by_camera_perception_yet"),
        }
    obs = ev.get("observable")
    cov = ev.get("coverage_sufficient")
    haz = ev.get("hazard")
    if obs is True and cov is True and isinstance(haz, bool):
        return {
            "source": "camera_perception",
            "hazard": haz,
            "conclusive": True,
            "reason": str(ev.get("reason") or ""),
        }
    return {
        "source": "camera_perception_inconclusive",
        "hazard": None,
        "conclusive": False,
        "reason": str(ev.get("reason") or "insufficient_camera_evidence"),
    }


def _canonical_random_event_hazard(final_h: dict[str, Any]) -> dict[str, Any]:
    h = final_h.get("hazard")
    return {
        "has_hazard": h,
        "source": final_h.get("source"),
        "conclusive": bool(final_h.get("conclusive", isinstance(h, bool))),
        "reason": final_h.get("reason"),
    }


def _build_scene_observation(
    result: dict,
    *,
    event_entry: dict[str, Any] | None,
    rule_id: int | None,
    resolution_wh: tuple[int, int],
    projection_metrics: dict | None,
    camera_view: dict | None,
    camera_prim_path: str,
    focal_length_ref: float,
    include_projection: bool,
) -> dict[str, Any]:
    from diaolan_randomizer import count_logical_workers_from_paths as _count_logical_workers_from_paths

    w, h = int(resolution_wh[0]), int(resolution_wh[1])
    cam_pose = result.get("camera_pose") if isinstance(result.get("camera_pose"), dict) else {}
    targets: list[dict[str, Any]] = [
        {
            "role": "active_gondola",
            "diaolan_root_path": result.get("active_diaolan_path"),
            "gondola_group1_prim_path": result.get("target_prim_path")
            or result.get("selected_target_path"),
            "state": {
                "gondola_y": result.get("gondola_y"),
                "gondola_height_cm": result.get("gondola_height_cm"),
                "height_debug": result.get("height_debug"),
            },
        },
        {
            "role": "visible_workers",
            "worker_prim_paths": list(result.get("visible_worker_paths") or []),
            "count": _count_logical_workers_from_paths(result.get("visible_worker_paths") or []),
        },
    ]
    obs: dict[str, Any] = {
        "event_id": rule_id,
        "event_type": (event_entry or {}).get("event_type"),
        "targets": targets,
        "debug_projection_fields_not_authoritative": True,
        "debug_projection_note": "target_projection_metrics / camera_view_perception 仅作几何调试，不参与 final_hazard 判定",
        "camera_params": {
            "camera_pose": cam_pose,
            "camera_xyz": result.get("camera_xyz"),
            "orientation": result.get("orientation"),
            "camera_meta": result.get("camera_meta"),
            "resolution_wh": [w, h],
            "camera_prim_path": camera_prim_path,
            "focal_length_1x_config": float(focal_length_ref),
        },
        "visibility_check": result.get("visibility_check"),
        "workers_visible_count_by_diaolan_path": result.get(
            "workers_visible_count_by_diaolan_path"
        ),
        "camera_view_perception": camera_view,
    }
    if include_projection and isinstance(projection_metrics, dict):
        obs["projection_metrics_snapshot"] = projection_metrics
        obs["target_projection_metrics"] = projection_metrics
    al = result.get("perception_camera_alignment")
    if isinstance(al, dict):
        obs["perception_camera_alignment"] = al
    return obs


def _build_perception_inputs(stream_diag: dict) -> dict[str, Any]:
    return {
        "image_inputs": {
            "rtsp_url": stream_diag.get("rtsp_url"),
            "snapshot_http": stream_diag.get("snapshot_url"),
            "mjpeg_http": stream_diag.get("mjpeg_url"),
            "resolution_wh": stream_diag.get("resolution_wh"),
            "frame_pipeline_note": stream_diag.get("frame_pipeline"),
        },
        "protocol_note": "帧级检测可并行；最终 hazard 以场景模型状态为准，影像/投影为辅助",
    }


def _best_ptz_from_alignment_and_result(
    alignment_diag: dict | None,
    result: dict[str, Any],
) -> dict[str, Any] | None:
    """从随机后 PTZ 搜索尝试与当前 orientation 拼出 best_ptz / best_camera_pose。"""
    ori = result.get("orientation") if isinstance(result.get("orientation"), dict) else {}
    pan = ori.get("applied_pan")
    tilt = ori.get("applied_tilt")
    zoom = None
    attempts: list = []
    if isinstance(alignment_diag, dict) and not alignment_diag.get("skipped"):
        attempts = list(alignment_diag.get("attempts") or [])
        idx = int(alignment_diag.get("stopped_after_attempt") or 0)
        if attempts and 1 <= idx <= len(attempts):
            last = attempts[idx - 1]
            if isinstance(last, dict) and last.get("zoom") is not None:
                zoom = last.get("zoom")
    out: dict[str, Any] = {
        "pan_deg": pan,
        "tilt_deg": tilt,
        "zoom": zoom,
        "camera_xyz": result.get("camera_xyz"),
    }
    if all(x is None for x in (pan, tilt, zoom)) and result.get("camera_xyz") is None:
        return None
    return out


def _visible_targets_summary(
    projection_metrics: dict | None,
    camera_view: dict | None,
    *,
    gondola_observable: bool | None,
) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = [
        {
            "role": "active_gondola",
            "gondola_geometry_observable": gondola_observable,
        }
    ]
    if isinstance(camera_view, dict) and not camera_view.get("error"):
        rows.append(
            {
                "role": "workers_under_active_diaolan",
                "camera_view_worker_count": camera_view.get("camera_view_worker_count"),
                "rendered_worker_paths_under_active_count": camera_view.get(
                    "rendered_worker_paths_under_active_count"
                ),
                "workers_all_projected_in_view": camera_view.get("workers_all_projected_in_view"),
            }
        )
    if isinstance(projection_metrics, dict) and "error" not in projection_metrics:
        rows[0]["target_projection_metrics_present"] = True
    return rows


def evaluate_random_event_camera_observability(
    *,
    scene_ev: dict[str, Any],
    rule_id: int | None,
    hazard_eval: dict[str, Any],
    projection_metrics: dict | None,
    camera_view: dict | None,
    perception_cfg: dict[str, Any],
    alignment_diag: dict | None,
    result: dict[str, Any],
) -> dict[str, Any]:
    """
    在「不改变场景态隐患真值」前提下，判断经允许的 PTZ / look-at 补救后，
    当前相机几何与投影是否足以**拍到**本次隐患证据（与 hazard 真值解耦）。
    """
    min_g = float(perception_cfg.get("min_target_area_ratio") or 0.001)
    g_obs, g_note = _gondola_observable_from_projection(projection_metrics, min_g)
    n_cam = int((camera_view or {}).get("camera_view_worker_count") or 0) if isinstance(camera_view, dict) else 0
    n_ren = int((camera_view or {}).get("rendered_worker_paths_under_active_count") or 0) if isinstance(
        camera_view, dict
    ) else 0
    all_seen = bool((camera_view or {}).get("workers_all_projected_in_view")) if isinstance(camera_view, dict) else False

    coverage_goal_reached = bool(
        isinstance(alignment_diag, dict)
        and (not alignment_diag.get("skipped"))
        and alignment_diag.get("coverage_goal_reached")
    )
    best_view_found = coverage_goal_reached
    best_ptz = _best_ptz_from_alignment_and_result(alignment_diag, result)

    projection_evidence = projection_metrics if isinstance(projection_metrics, dict) else {}
    camera_obs_snap: dict[str, Any] = {}
    if isinstance(camera_view, dict):
        camera_obs_snap = {
            "image_wh": camera_view.get("image_wh"),
            "camera_prim_path": camera_view.get("camera_prim_path"),
            "gondola_world_height_axis": camera_view.get("gondola_world_height_axis"),
            "camera_view_worker_count": camera_view.get("camera_view_worker_count"),
            "rendered_worker_paths_under_active_count": camera_view.get(
                "rendered_worker_paths_under_active_count"
            ),
            "workers_all_projected_in_view": camera_view.get("workers_all_projected_in_view"),
            "error": camera_view.get("error"),
        }

    visible_targets = _visible_targets_summary(
        projection_metrics, camera_view, gondola_observable=g_obs
    )

    base_out: dict[str, Any] = {
        "source": "camera_ptz_observation",
        "best_view_found": best_view_found,
        "best_ptz": best_ptz,
        "best_camera_pose": {"camera_pose": result.get("camera_pose"), "camera_xyz": result.get("camera_xyz")},
        "coverage_goal_reached": coverage_goal_reached,
        "target_projection_metrics": projection_evidence,
        "projection_evidence": projection_evidence,
        "camera_observation": camera_obs_snap,
        "visible_targets": visible_targets,
        "perception_camera_alignment": alignment_diag if isinstance(alignment_diag, dict) else None,
    }

    if rule_id is not None and int(rule_id) == 11:
        return {
            **base_out,
            "supported": True,
            "observability_applicable": False,
            "can_observe_hazard": None,
            "reason": "规则 11 为全图曝光统计，不使用篮体/工人几何投影链路作可观测性结论",
        }

    if not scene_ev.get("supported"):
        return {
            **base_out,
            "supported": False,
            "can_observe_hazard": None,
            "reason": "场景态规则不支持评估，相机可观测性同步标记为不支持",
        }

    sh = scene_ev.get("hazard")
    if sh is False:
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "observability_applicable": False,
            "reason": "场景态判定无隐患，不作「镜头已拍到隐患」断言（can_observe_hazard=false）",
        }

    if sh is None:
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": None,
            "observability_applicable": False,
            "reason": "场景态证据不足，无法与「是否拍到隐患」对齐，相机层不断言可观测隐患",
        }

    # sh is True：仅在几何上判断是否足以在画面中体现本次隐患
    if rule_id is None or int(rule_id) not in SCENE_STATE_SUPPORTED_RULE_IDS:
        return {
            **base_out,
            "supported": False,
            "can_observe_hazard": None,
            "reason": "unsupported_by_scene_state_evaluation_yet",
        }

    if not hazard_eval.get("supported"):
        return {
            **base_out,
            "supported": False,
            "can_observe_hazard": None,
            "reason": "hazard_eval 不支持，相机可观测性未定义",
        }

    if not isinstance(camera_view, dict) or camera_view.get("error"):
        err = camera_view.get("error") if isinstance(camera_view, dict) else "camera_view_perception_missing"
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "reason": f"camera_view_perception 不可用，无法在 PTZ 后确认隐患画面：{err}",
        }

    rid = int(rule_id)
    if rid in (1, 5, 12):
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": None,
            "observability_applicable": False,
            "reason": (
                "规则 1/5/12 为场景态 USD 判定（护栏、安全绳、限位），"
                "不使用相机工人投影链路断言是否入画"
            ),
        }

    z0, zeps = _ground_params_from_hazard_evidence(hazard_eval)

    if rid == 2:
        if g_obs is not True:
            return {
                **base_out,
                "supported": True,
                "can_observe_hazard": False,
                "reason": f"超员隐患在场景态为真，但篮体几何可观测性不足：{g_note}",
            }
        if n_cam >= 3:
            return {
                **base_out,
                "supported": True,
                "can_observe_hazard": True,
                "reason": f"PTZ 后篮体可观测且几何投影作业人员数≥3（n={n_cam}），可拍到超员隐患证据",
            }
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "reason": f"场景态超员为真，但当前画面几何可见作业人员投影数={n_cam}<3，不足以在镜头中确认超员",
        }

    if rid == 3:
        if g_obs is not True:
            return {
                **base_out,
                "supported": True,
                "can_observe_hazard": False,
                "reason": f"单人作业隐患在场景态为真，但篮体几何可观测性不足：{g_note}",
            }
        if n_ren == 1 and all_seen and n_cam == 1:
            return {
                **base_out,
                "supported": True,
                "can_observe_hazard": True,
                "reason": "PTZ 后单名已渲染作业人员且投影完整入画，可拍到单人作业隐患证据",
            }
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "reason": f"场景态单人作业为真，但镜头几何无法唯一确认单人入画（n_ren={n_ren}, all_seen={all_seen}, n_cam={n_cam}）",
        }

    # rid == 4
    if n_ren > 0:
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "reason": f"场景态「未落地」为真，但画面中仍可见已渲染作业人员（{n_ren}），镜头语义与「作业结束」场景不一致，不足以作为可复现场景证据",
        }
    if g_obs is not True:
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "reason": f"未落地隐患在场景态为真，但篮体几何可观测性不足：{g_note}",
        }
    gh = camera_view.get("gondola_world_height_axis")
    try:
        gh_f = float(gh) if gh is not None else None
    except (TypeError, ValueError):
        gh_f = None
    if gh_f is None:
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "reason": "篮体已几何入画但缺少世界高度轴读数，无法在画面中确认未落地",
        }
    off_ground = _rule4_ground_hazard(float(gh_f), 0, z0, zeps)
    if not off_ground:
        return {
            **base_out,
            "supported": True,
            "can_observe_hazard": False,
            "reason": f"场景态未落地为真，但相机可读高度轴={float(gh_f):.4f} 相对地面阈值未呈现「未落地」几何证据",
        }
    return {
        **base_out,
        "supported": True,
        "can_observe_hazard": True,
        "reason": f"PTZ 后零作业人员入画且篮体高度轴={float(gh_f):.4f} 相对地面阈值呈现未落地几何证据",
    }


def attach_perception_to_randomize_result(
    result: dict[str, Any],
    request_meta: dict | None,
    *,
    perception_cfg_raw: Any,
    stream_diag: dict[str, Any],
    resolution_wh: tuple[int, int],
    camera_prim_path: str,
    focal_length_ref: float,
) -> None:
    """
    在已有 hazard_eval 与 camera_pose 等字段写入 result 之后调用。
    仅追加字段；**最终 hazard 仅来自场景模型状态**（`evaluate_random_event_hazard_from_scene_state`）。
    """
    if not isinstance(result, dict):
        return
    hazard_eval = result.get("hazard_eval")
    if not isinstance(hazard_eval, dict):
        return

    pcfg = _default_perception_cfg(perception_cfg_raw)
    gen_meta = result.get("randomize_event_meta") if isinstance(result.get("randomize_event_meta"), dict) else None
    rule_id = _resolve_rule_id(hazard_eval, request_meta, randomize_event_meta=gen_meta)
    entry = _registry_entry(rule_id)

    projection_metrics = stream_diag.get("target_projection_metrics")
    if not isinstance(projection_metrics, dict):
        projection_metrics = None

    camera_view = stream_diag.get("camera_view_perception")
    if not isinstance(camera_view, dict):
        camera_view = None

    legacy_rr = _build_legacy_rule_result(hazard_eval)

    ev = evaluate_random_event_hazard_from_scene_state(result, hazard_eval, request_meta)

    perception_pr: dict[str, Any] = {
        "supported": ev["supported"],
        "observable": ev.get("observable"),
        "coverage_sufficient": ev.get("coverage_sufficient"),
        "hazard": ev.get("hazard"),
        "reason": ev.get("reason"),
        "evidence": _safe_plain_dict(ev.get("evidence")),
        "method": "scene_state",
        "detection": _build_detection_placeholder(),
    }
    if rule_id is not None and int(rule_id) == 11:
        perception_pr["method"] = "camera_overexposure_snapshot"

    if rule_id is not None and int(rule_id) == 11:
        haz11 = ev.get("hazard")
        final_h = {
            "source": "camera_overexposure",
            "supported": bool(ev.get("supported")),
            "hazard": haz11,
            "conclusive": bool(ev.get("supported")) and isinstance(haz11, bool),
            "reason": str(ev.get("reason") or ""),
        }
    else:
        final_h = _finalize_hazard_from_scene_evaluation(ev)
    canonical = _canonical_random_event_hazard(final_h)

    _ev_coerced = _safe_plain_dict(ev.get("evidence"))
    scene_obs_for_eval: dict[str, Any] = {
        "evaluation": "scene_state",
        "worker_count": _ev_coerced.get("worker_count"),
        "worker_count_source": _ev_coerced.get("worker_count_source"),
        "gondola_world_height": _ev_coerced.get("gondola_world_height"),
        "gondola_world_height_source": _ev_coerced.get("gondola_world_height_source"),
        "active_diaolan_path": str(result.get("active_diaolan_path") or ""),
        "workers_scalar": result.get("workers"),
    }

    alignment_diag = result.get("perception_camera_alignment")
    cam_obs = evaluate_random_event_camera_observability(
        scene_ev=ev,
        rule_id=rule_id,
        hazard_eval=hazard_eval,
        projection_metrics=projection_metrics,
        camera_view=camera_view,
        perception_cfg=pcfg,
        alignment_diag=alignment_diag if isinstance(alignment_diag, dict) else None,
        result=result,
    )

    result["legacy_rule_result"] = legacy_rr
    result["perception_result"] = perception_pr
    result["scene_state_evaluation"] = {
        "rule_id": rule_id,
        "rule_name": (entry or {}).get("event_type") if entry else None,
        "supported": ev.get("supported"),
        "hazard": ev.get("hazard"),
        "reason": ev.get("reason"),
        "evidence": _safe_plain_dict(ev.get("evidence")),
        "method": "scene_state",
        "scene_observation": scene_obs_for_eval,
    }
    result["camera_observability"] = cam_obs
    result["final_hazard_result"] = final_h
    result["random_event_hazard"] = canonical
    current_event = None
    if isinstance(gen_meta, dict):
        current_event = {
            "rule_id": gen_meta.get("rule_id"),
            "rule_name": gen_meta.get("rule_name"),
            "event_id": gen_meta.get("event_id"),
            "source": gen_meta.get("source"),
        }
    result["event_registry"] = {
        "resolved_rule_id": rule_id,
        "current_event": current_event,
        "registry_entry": entry,
        "note": "隐患真值见 final_hazard_result / scene_state_evaluation（scene_state）；能否入画见 camera_observability（camera_ptz_observation）。current_event 为本次 randomize 事件生成侧写入的元数据。",
    }
    result["scene_observation"] = _build_scene_observation(
        result,
        event_entry=entry,
        rule_id=rule_id,
        resolution_wh=resolution_wh,
        projection_metrics=projection_metrics,
        camera_view=camera_view,
        camera_prim_path=camera_prim_path,
        focal_length_ref=focal_length_ref,
        include_projection=bool(pcfg.get("include_projection_in_observation")),
    )
    result["perception_inputs"] = _build_perception_inputs(stream_diag)
    result["hazard_evaluation"] = {
        "legacy_eval": hazard_eval,
        "random_event_hazard": canonical,
        "final_source": final_h.get("source"),
        "note": "final_hazard_result / scene_state_evaluation 仅 scene_state；camera_observability 单独回答「PTZ 后能否拍到隐患」，二者不得混用。",
    }

    result["event_id"] = rule_id
    result["event_type"] = (entry or {}).get("event_type")
    result["hazard_category"] = (entry or {}).get("hazard_category")


def export_full_event_registry() -> dict[int, dict[str, Any]]:
    return deepcopy(EVENT_REGISTRY)
