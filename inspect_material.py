from pxr import Usd, UsdGeom, UsdShade

def inspect_material(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"Material {path} not found.")
        return
        
    print(f"Inspecting Material {path}:")
    mat = UsdShade.Material(prim)
    
    # Check all outputs
    for out in mat.GetOutputs():
        print(f"  Output: {out.GetFullName()}")
        connected_sources, _ = out.GetConnectedSources()
        for source in connected_sources:
            print(f"    Connected Source: {source.source.GetPrim().GetPath()}")
            shader_prim = source.source.GetPrim()
            shader = UsdShade.Shader(shader_prim)
            print(f"      Shader ID: {shader.GetIdAttr().Get()}")
            
            # Print inputs
            for inp in shader.GetInputs():
                print(f"      Input: {inp.GetFullName()} = {inp.Get()}")

def main():
    stage = Usd.Stage.Open("scene_6diaolan_ptz.usda")
    if not stage:
        print("Failed to open stage.")
        return
        
    inspect_material(stage, "/World/DiaoLan_01/Looks/material___3")
    inspect_material(stage, "/World/DiaoLan_01/Looks/material_____5")

if __name__ == "__main__":
    main()
