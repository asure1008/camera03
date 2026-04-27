from pxr import Usd, UsdGeom

def main():
    stage = Usd.Stage.Open("scene_6diaolan_ptz.usda")
    if not stage:
        return
        
    # We need to compute world bounds of the camera and the target
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'proxy', 'render'])
    
    cam_prim = stage.GetPrimAtPath("/World/CameraRig/CamTilt/Camera")
    target_prim = stage.GetPrimAtPath("/World/DiaoLan_01/Model/Group8761")
    
    cam_bbox = bbox_cache.ComputeWorldBound(cam_prim)
    cam_range = cam_bbox.ComputeAlignedRange()
    print("Camera World Bounds:")
    print("  Min:", cam_range.GetMin())
    print("  Max:", cam_range.GetMax())
    print("  Mid:", cam_range.GetMidpoint())
    
    target_bbox = bbox_cache.ComputeWorldBound(target_prim)
    target_range = target_bbox.ComputeAlignedRange()
    print("\nTarget World Bounds:")
    print("  Min:", target_range.GetMin())
    print("  Max:", target_range.GetMax())
    print("  Mid:", target_range.GetMidpoint())

if __name__ == "__main__":
    main()
