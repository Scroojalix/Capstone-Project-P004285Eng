from pxr import Usd, UsdPhysics

stage = omni.usd.get_context().get_stage()

# Loop through all prims on the stage
for prim in stage.Traverse():
  # Filter by name or path pattern matching your similar objects
  if "SM_SafetyRailing" in prim.GetName():
    # Ensure it has a collision API applied, or update its approximation
    UsdPhysics.CollisionAPI.Apply(prim)
    
    mesh_collision_api = UsdPhysics.MeshCollisionAPI.Apply(prim)
    
    # Set collision approximation (e.g., ConvexHull, Mesh, BoundingBox)
    # Reference: https://docs.omniverse.nvidia.com/isaacsim/latest/features/physics/ext_omni_isaac_physics_utils.html
    mesh_collision_api.CreateApproximationAttr("boundingCube")

