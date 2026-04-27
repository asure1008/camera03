import sys
from pxr import Usd, UsdGeom, UsdShade
from diaolan_randomizer import scan_diaolan_prims

def inspect_prim(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"Prim {path} not found.")
        return
        
    print(f"Inspecting {path}:")
    
    # Check visibility
    imageable = UsdGeom.Imageable(prim)
    if imageable:
        vis = imageable.GetVisibilityAttr().Get()
        purpose = imageable.GetPurposeAttr().Get()
        print(f"  Visibility: {vis}")
        print(f"  Purpose: {purpose}")
        
    # Check material binding
    material_binding = UsdShade.MaterialBindingAPI(prim)
    direct_binding = material_binding.GetDirectBinding()
    if direct_binding.GetMaterial():
        print(f"  Material Binding: {direct_binding.GetMaterial().GetPath()}")
    else:
        print(f"  Material Binding: None")
        
    # Check display color
    if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Gprim):
        gprim = UsdGeom.Gprim(prim)
        color = gprim.GetDisplayColorAttr().Get()
        opacity = gprim.GetDisplayOpacityAttr().Get()
        print(f"  DisplayColor: {color}")
        print(f"  DisplayOpacity: {opacity}")

def main():
    scene_path = sys.argv[1] if len(sys.argv) > 1 else "scene_4diaolan_ptz.usda"
    stage = Usd.Stage.Open(scene_path)
    if not stage:
        print("Failed to open stage.")
        return
        
    diaolans = scan_diaolan_prims(stage)
    if not diaolans:
        print("No diaolans found.")
        return
        
    for d in diaolans:
        root_path = d["group1"]
        print(f"\n======================================")
        print(f"Target Prim: {root_path}")
        print(f"======================================")
        inspect_prim(stage, root_path)
        
        # Inspect all meshes under it
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim.IsValid():
            continue
            
        mesh_count = 0
        for desc in Usd.PrimRange(root_prim):
            if desc.IsA(UsdGeom.Mesh):
                mesh_count += 1
                if mesh_count <= 5: # Just print first 5
                    print("\n---")
                    inspect_prim(stage, desc.GetPath().pathString)
                    
        print(f"\nTotal meshes found under {root_path}: {mesh_count}")

if __name__ == "__main__":
    main()
