from __future__ import annotations

import argparse
import os
import sys
from typing import Any

from .common import (
    load_yaml_config,
    normalize_renderer_mode,
    resolve_repo_path,
    sanitize_windows_path_env,
    simulation_renderer_name,
)


def setup_isaac_env(script_dir: str) -> None:
    sanitize_windows_path_env()

    cache_root = os.path.join(script_dir, ".runtime_cache")
    warp_cache_path = os.path.join(cache_root, "warp")
    ov_cache_path = os.path.join(cache_root, "ov_cache")
    kit_cache_path = os.path.join(cache_root, "kit_cache")
    optix_cache_path = os.path.join(cache_root, "optix_cache")
    shader_cache_path = os.path.join(cache_root, "shader_cache")
    os.makedirs(warp_cache_path, exist_ok=True)
    os.makedirs(ov_cache_path, exist_ok=True)
    os.makedirs(kit_cache_path, exist_ok=True)
    os.makedirs(optix_cache_path, exist_ok=True)
    os.makedirs(shader_cache_path, exist_ok=True)
    os.environ.setdefault("WARP_CACHE_PATH", warp_cache_path)
    # Hint Kit/RTX subsystems to prefer local project cache paths on Windows.
    os.environ.setdefault("OV_CACHE_ROOT", ov_cache_path.replace("\\", "/"))
    os.environ.setdefault("OMNI_GLOBAL_CACHE_DIR", ov_cache_path.replace("\\", "/"))
    os.environ.setdefault("OMNI_KIT_CACHE_DIR", kit_cache_path.replace("\\", "/"))
    # Make token-backed cache resolution use local writable folders from process start.
    os.environ.setdefault("cache", kit_cache_path.replace("\\", "/"))
    os.environ.setdefault("omni_global_cache", ov_cache_path.replace("\\", "/"))
    # RTX / OptiX and shader cache redirection to avoid AppData permission failures.
    os.environ.setdefault("OPTIX_CACHE_PATH", optix_cache_path.replace("\\", "/"))
    os.environ.setdefault("CUDA_CACHE_PATH", shader_cache_path.replace("\\", "/"))
    os.environ.setdefault("DXVK_STATE_CACHE_PATH", shader_cache_path.replace("\\", "/"))

    # Try to bind token values as early as possible, before SimulationApp startup.
    try:
        import carb.tokens

        tokens = carb.tokens.get_tokens_interface()
        tokens.set_value("cache", kit_cache_path.replace("\\", "/"))
        tokens.set_value("omni_global_cache", ov_cache_path.replace("\\", "/"))
    except Exception:
        pass

    carb_app_path = os.environ.get("CARB_APP_PATH", "")
    if not carb_app_path:
        return

    isaac_root = os.path.dirname(carb_app_path)
    os.environ.setdefault("ISAAC_PATH", isaac_root)

    exp_path = os.path.join(isaac_root, "apps")
    if os.path.isdir(exp_path):
        os.environ.setdefault("EXP_PATH", exp_path)


def build_arg_parser(script_dir: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PTZ Camera RTSP Streamer for Isaac Sim")
    parser.add_argument("--config", default=os.path.join(script_dir, "ptz_rtsp_config.yaml"))
    parser.add_argument("--scene", default=None)
    parser.add_argument("--camera", default=None)
    parser.add_argument("--rtsp", default=None)
    parser.add_argument("--ffmpeg", default=None)
    parser.add_argument("--fps", type=int, default=None)
    parser.add_argument("--renderer-mode", default=None, dest="renderer_mode")
    parser.add_argument("--ctrl-port", type=int, default=None, dest="ctrl_port")
    parser.add_argument("--path-tracing-spp", type=int, default=None, dest="path_tracing_spp")
    return parser


def build_stream_settings(args: argparse.Namespace, script_dir: str) -> dict[str, Any]:
    cfg = load_yaml_config(args.config)
    ffmpeg_cfg = cfg.get("ffmpeg", {}) if isinstance(cfg.get("ffmpeg", {}), dict) else {}
    ffmpeg_path_raw = args.ffmpeg or ffmpeg_cfg.get("path") or cfg.get("ffmpeg_path")
    renderer_mode_raw = args.renderer_mode or cfg.get("renderer_mode", "PathTracing")
    try:
        renderer_mode = normalize_renderer_mode(renderer_mode_raw)
    except ValueError:
        renderer_mode = "PathTracing"
        print(f"[PTZ-RTSP] invalid renderer_mode={renderer_mode_raw!r}, fallback to PathTracing")
    try:
        path_tracing_spp = int(args.path_tracing_spp if args.path_tracing_spp is not None else cfg.get("path_tracing_spp", 1))
    except Exception:
        path_tracing_spp = 1
    path_tracing_spp = max(1, min(512, path_tracing_spp))

    scene_path = resolve_repo_path(script_dir, args.scene or cfg["scene_path"])
    settings: dict[str, Any] = {
        "script_dir": script_dir,
        "config_path": os.path.abspath(args.config),
        "cfg": cfg,
        "scene_path": scene_path,
        "camera_prim": args.camera or cfg["camera_prim"],
        "rtsp_url": args.rtsp or cfg["rtsp_url"],
        "fps": args.fps or int(cfg.get("fps", 25)),
        "resolution": tuple(cfg.get("resolution", [1920, 1080])),
        "bitrate": cfg.get("bitrate", "4M"),
        "sim_hz": int(cfg.get("sim_hz", 60)),
        "mediamtx_cfg": cfg.get("mediamtx", {}),
        "ffmpeg_path": resolve_repo_path(script_dir, ffmpeg_path_raw) if ffmpeg_path_raw else None,
        "rtsp_enabled": bool(cfg.get("rtsp_enabled", True)),
        "mjpeg_quality": int(cfg.get("mjpeg_quality", 80)),
        "focal_length_1x": float(cfg.get("focal_length_1x", 18.14756)),
        "ctrl_port": args.ctrl_port or int(cfg.get("ctrl_port", 8081)),
        "renderer_mode": renderer_mode,
        "path_tracing_spp": path_tracing_spp,
        "rt_subframes": int(cfg.get("rt_subframes", 2)),
    }
    return settings


def main(argv: list[str] | None = None) -> None:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    script_dir = os.path.dirname(script_dir)

    parser = build_arg_parser(script_dir)
    args, _ = parser.parse_known_args(argv)

    setup_isaac_env(script_dir)
    settings = build_stream_settings(args, script_dir)

    # Apply Kit startup overrides before SimulationApp reads process argv.
    cache_token = os.environ.get("cache", "").replace("\\", "/")
    global_cache_token = os.environ.get("omni_global_cache", "").replace("\\", "/")
    startup_overrides = [
        "--/exts/omni.kit.material.library/enabled=false",
    ]
    if cache_token:
        startup_overrides.append(f"--/app/tokens/cache={cache_token}")
    if global_cache_token:
        startup_overrides.append(f"--/app/tokens/omni_global_cache={global_cache_token}")
    existing = set(sys.argv)
    for arg in startup_overrides:
        if arg not in existing:
            sys.argv.append(arg)

    renderer_mode = str(settings.get("renderer_mode", "PathTracing")).strip() or "PathTracing"
    sim_renderer = simulation_renderer_name(renderer_mode)
    print(f"[PTZ-RTSP] renderer mode: {renderer_mode} (backend={sim_renderer})")

    from isaacsim import SimulationApp

    width, height = settings["resolution"]
    sim_app_settings: dict[str, Any] = {
        "headless": True,
        "renderer": sim_renderer,
        "anti_aliasing": 0,
        "width": width,
        "height": height,
    }
    if renderer_mode == "PathTracing":
        sim_app_settings["samples_per_pixel_per_frame"] = int(settings.get("path_tracing_spp", 1))
        sim_app_settings["denoiser"] = False
        print(f"[PTZ-RTSP] path tracing spp/frame: {sim_app_settings['samples_per_pixel_per_frame']}")
    sim_app = SimulationApp(sim_app_settings)

    from .stream_runtime import run_stream_runtime

    run_stream_runtime(sim_app, settings)
