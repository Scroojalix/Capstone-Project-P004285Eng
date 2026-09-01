from pxr import UsdGeom, Gf
import omni.usd

# Get the current USD stage
stage = omni.usd.get_context().get_stage()
mesh_prim = stage.GetPrimAtPath("/World/Warehouse/Floor/FloorPlane")

# Access the UsdGeom.Mesh schema
mesh = UsdGeom.Mesh(mesh_prim)

# Get current points
points_attr = mesh.GetPointsAttr()
points = points_attr.Get()

half_width = 30
half_height = 10

# Modify a specific point (e.g., move the first vertex up on Z-axis)
if points:
    points[0] = Gf.Vec3f(-half_width, -half_height, 0)
    points[1] = Gf.Vec3f(half_width, -half_height, 0)
    points[2] = Gf.Vec3f(-half_width, half_height, 0)
    points[3] = Gf.Vec3f(half_width, half_height, 0)
    
    points_attr.Set(points)




