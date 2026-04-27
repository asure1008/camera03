#!/usr/bin/env python3
from dataclasses import dataclass
import sys
from pxr import Gf, Usd, UsdGeom
from diaolan_randomizer import scan_diaolan_prims

SCENE_PATH = "/home/uniubi/xuanyuan/camera05/camera03/scene_4diaolan_ptz.usda"
CAMERA_PATH = "/World/CameraRig/CamTilt/Camera"
WIDTH = 960
HEIGHT = 540

@dataclass
class ProjectionResult:
    target_path: str
    bbox_min: tuple[float, float, float]
    bbox_max: tuple[float, float, float]
    bbox_mid: tuple[float, float, float]
    distance_to_camera: float
    points_in_front: int
    points_total: int
    screen_bbox_px: tuple[float, float, float, float] | None
    screen_fill_w_pct: float
    screen_fill_h_pct: float
    crosses_boundary: bool
    very_close_face: bool

def compute_projection(stage: Usd.Stage, target_path: str) -> ProjectionResult | None:
    cam_prim = stage.GetPrimAtPath(CAMERA_PATH)
    tgt_prim = stage.GetPrimAtPath(target_path)
    if not cam_prim.IsValid() or not tgt_prim.IsValid():
        return None

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "proxy", "render"])
    target_bound = bbox_cache.ComputeWorldBound(tgt_prim)
    target_range = target_bound.ComputeAlignedRange()
    if target_range.IsEmpty():
        return None

    target_min = target_range.GetMin()
    target_max = target_range.GetMax()
    target_mid = target_range.GetMidpoint()

    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    cam_world = xcache.GetLocalToWorldTransform(cam_prim)
    cam_pos = cam_world.ExtractTranslation()
    dist = float((target_mid - cam_pos).GetLength())

    gf_cam = UsdGeom.Camera(cam_prim).GetCamera(Usd.TimeCode.Default())
    frustum = gf_cam.frustum
    frustum.Transform(cam_world)

    vm = frustum.ComputeViewMatrix()
    pm = frustum.ComputeProjectionMatrix()

    points_to_test = [
        Gf.Vec3d(target_min[0], target_min[1], target_min[2]),
        Gf.Vec3d(target_min[0], target_min[1], target_max[2]),
        Gf.Vec3d(target_min[0], target_max[1], target_min[2]),
        Gf.Vec3d(target_min[0], target_max[1], target_max[2]),
        Gf.Vec3d(target_max[0], target_min[1], target_min[2]),
        Gf.Vec3d(target_max[0], target_min[1], target_max[2]),
        Gf.Vec3d(target_max[0], target_max[1], target_min[2]),
        Gf.Vec3d(target_max[0], target_max[1], target_max[2]),
        Gf.Vec3d(target_mid[0], target_mid[1], target_mid[2]),
    ]

    pixels = []
    points_in_front = 0
    very_close_face = False

    for pt in points_to_test:
        clip = Gf.Vec4d(pt[0], pt[1], pt[2], 1.0) * vm * pm
        w = float(clip[3])
        if w > 0:
            points_in_front += 1
            if w < 1.0:
                very_close_face = True
            ndc_x = float(clip[0]) / w
            ndc_y = float(clip[1]) / w
            sx = (ndc_x * 0.5 + 0.5) * WIDTH
            sy = (1.0 - (ndc_y * 0.5 + 0.5)) * HEIGHT
            pixels.append((sx, sy))

    if not pixels:
        return ProjectionResult(
            target_path=target_path,
            bbox_min=(target_min[0], target_min[1], target_min[2]),
            bbox_max=(target_max[0], target_max[1], target_max[2]),
            bbox_mid=(target_mid[0], target_mid[1], target_mid[2]),
            distance_to_camera=dist,
            points_in_front=points_in_front,
            points_total=len(points_to_test),
            screen_bbox_px=None,
            screen_fill_w_pct=0.0,
            screen_fill_h_pct=0.0,
            crosses_boundary=False,
            very_close_face=very_close_face,
        )

    min_x = min(v[0] for v in pixels)
    min_y = min(v[1] for v in pixels)
    max_x = max(v[0] for v in pixels)
    max_y = max(v[1] for v in pixels)

    crosses_boundary = min_x < 0 or max_x > WIDTH or min_y < 0 or max_y > HEIGHT

    bbox_w = max_x - min_x
    bbox_h = max_y - min_y

    return ProjectionResult(
        target_path=target_path,
        bbox_min=(target_min[0], target_min[1], target_min[2]),
        bbox_max=(target_max[0], target_max[1], target_max[2]),
        bbox_mid=(target_mid[0], target_mid[1], target_mid[2]),
        distance_to_camera=dist,
        points_in_front=points_in_front,
        points_total=len(points_to_test),
        screen_bbox_px=(min_x, min_y, max_x, max_y),
        screen_fill_w_pct=(bbox_w / WIDTH) * 100.0,
        screen_fill_h_pct=(bbox_h / HEIGHT) * 100.0,
        crosses_boundary=crosses_boundary,
        very_close_face=very_close_face,
    )

def main():
    scene_path = sys.argv[1] if len(sys.argv) > 1 else SCENE_PATH
    print(f"Loading stage: {scene_path}...")
    stage = Usd.Stage.Open(scene_path)
    if not stage:
        print("Failed to open stage.")
        return

    diaolans = scan_diaolan_prims(stage)
    if not diaolans:
        print("No diaolans found.")
        return

    targets = [d["group1"] for d in diaolans]
    
    print("\n==================================================")
    print("Camera Projection Analysis")
    print(f"Camera: {CAMERA_PATH}")
    print(f"Resolution: {WIDTH}x{HEIGHT}")
    print("==================================================\n")

    for target in targets:
        print(f"Target: {target}")
        res = compute_projection(stage, target)
        if not res:
            print("  -> ERROR: Could not compute projection (prim missing or empty bbox).")
            print()
            continue

        print(f"  Distance to Camera: {res.distance_to_camera:.2f} m")
        print(f"  World BBox Mid:     ({res.bbox_mid[0]:.2f}, {res.bbox_mid[1]:.2f}, {res.bbox_mid[2]:.2f})")
        print(f"  Points in front:    {res.points_in_front} / {res.points_total}")

        if res.screen_bbox_px:
            x0, y0, x1, y1 = res.screen_bbox_px
            print(f"  Screen BBox (px):   ({x0:.1f}, {y0:.1f}) -> ({x1:.1f}, {y1:.1f})")
            print(f"  Screen Fill (W/H):  {res.screen_fill_w_pct:.1f}% / {res.screen_fill_h_pct:.1f}%")
            if res.crosses_boundary:
                print("  -> WARNING: BBox crosses screen boundary.")
            if res.very_close_face:
                print("  -> WARNING: Target is very close to near clip plane (w < 1.0).")
        else:
            print("  -> Target is completely behind the camera.")
        print()

if __name__ == "__main__":
    main()
