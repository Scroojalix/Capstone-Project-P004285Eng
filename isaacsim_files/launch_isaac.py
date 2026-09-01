from isaacsim import SimulationApp
kit = SimulationApp({"headless": False})

import sys
import omni
from pxr import Sdf
from isaacsim.storage.native import is_file
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleXFormPrim
import omni.graph.core as og

# Enable the ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

keys = og.Controller.Keys

# Path to the USD files
WORLD_USD = "IsaacWarehouseSmall.usd"
ROBOT_USD = "DingoRobot.usd"

STRIP_HEAVY_MESH = True
DISABLE_SENSORS = True
HEAVY_MESHES = ("mesh_1",)
SENSOR_FRAMES = ("velodyne_frame", "realsense_frame")

# Open the world USD file
if is_file(WORLD_USD):
    omni.usd.get_context().open_stage(WORLD_USD)
else:
    print(f"Error: World USD file not found at {WORLD_USD}")
    kit.close()
    sys.exit(1)
stage = omni.usd.get_context().get_stage()

# Robot start positions: 3 corner blocks of 9 + a middle-left column of 3 (30 total),
# placed on open floor from the occupancy map.
START_POSITIONS = [
  [ -35.175,  -25.225, 0.0],   # robot0  cell (1, 5)
    [ -33.175,  -25.225, 0.0],   # robot1  cell (3, 5)
    [ -31.175,  -25.225, 0.0],   # robot2  cell (5, 5)
    [ -29.175,  -25.225, 0.0],   # robot3  cell (7, 5)
    [ -26.175,  -25.225, 0.0],   # robot4  cell (10, 5)
    [ -24.175,  -25.225, 0.0],   # robot5  cell (12, 5)
    [ -21.175,  -25.225, 0.0],   # robot6  cell (15, 5)
    [ -19.175,  -25.225, 0.0],   # robot7  cell (17, 5)
    [ -17.175,  -25.225, 0.0],   # robot8  cell (19, 5)
    [ -34.175,  -24.225, 0.0],   # robot9  cell (2, 6)
    [ -32.175,  -26.225, 0.0],   # robot10 cell (4, 4)
    [ -30.175,  -26.225, 0.0],   # robot11 cell (6, 4)
    [ -25.175,  -26.225, 0.0],   # robot12 cell (11, 4)
    [ -20.175,  -26.225, 0.0],   # robot13 cell (16, 4)
    [ -16.175,  -26.225, 0.0],   # robot14 cell (20, 4)
    [ -32.175,   26.775, 0.0],   # robot15 cell (4, 57)
    [ -30.175,   26.775, 0.0],   # robot16 cell (6, 57)
    [ -28.175,   26.775, 0.0],   # robot17 cell (8, 57)
    [ -26.175,   26.775, 0.0],   # robot18 cell (10, 57)
    [ -24.175,   26.775, 0.0],   # robot19 cell (12, 57)
    [ -22.175,   26.775, 0.0],   # robot20 cell (14, 57)
    [ -20.175,   26.775, 0.0],   # robot21 cell (16, 57)
    [ -18.175,   26.775, 0.0],   # robot22 cell (18, 57)
    [ -16.175,   26.775, 0.0],   # robot23 cell (20, 57)
    [ -31.175,   27.775, 0.0],   # robot24 cell (5, 58)
    [ -27.175,   27.775, 0.0],   # robot25 cell (9, 58)
    [ -23.175,   27.775, 0.0],   # robot26 cell (13, 58)
    [ -19.175,   27.775, 0.0],   # robot27 cell (17, 58)
    [ -17.175,   27.775, 0.0],   # robot28 cell (19, 58)
    [ -29.175,   27.775, 0.0],   # robot29 cell (7, 58)
]


FACE_NORTH = (0.70710678, 0.0, 0.0,  0.70710678)
FACE_SOUTH = (0.70710678, 0.0, 0.0, -0.70710678)

stripped_meshes = 0
disabled_frames = 0

# Spawn the robot models at the specified positions
for i, pos in enumerate(START_POSITIONS):
    # Add the robot USD reference to the stage
    add_reference_to_stage(ROBOT_USD, f"/World/robot{i}")

    # Set the robot's position and heading in the world
    robot_xform = SingleXFormPrim(f"/World/robot{i}")
    robot_xform.set_world_pose(
        position=pos,
        orientation=FACE_NORTH if i < 15 else FACE_SOUTH,
    )

    if STRIP_HEAVY_MESH:
        for name in HEAVY_MESHES:
            prim = stage.GetPrimAtPath(
                f"/World/robot{i}/dingo/base_link/visuals/{name}")
            if prim.IsValid():
                prim.SetActive(False)
                stripped_meshes += 1
            else:
                print(f"WARNING: robot{i} has no visuals/{name}")

    if DISABLE_SENSORS:
        for frame in SENSOR_FRAMES:
            prim = stage.GetPrimAtPath(
                f"/World/robot{i}/dingo/base_link/{frame}")
            if prim.IsValid():
                prim.SetActive(False)
                disabled_frames += 1
            else:
                print(f"WARNING: robot{i} has no base_link/{frame}")

    kit.update()
    
    # Set the robot's namespace attribute
    # FIXME: namespace is set correctly, but the ROS topics are not prefixed with the namespace.
    # I think it may be 
    # May submit a bug report to NVIDIA if this is not expected behavior.
    # Current workaround is to manually set the topic names in the graph after spawning the robot.
    robot_prim = stage.GetPrimAtPath(f"/World/robot{i}")
    namespace_attr = robot_prim.CreateAttribute("isaac:namespace", Sdf.ValueTypeNames.Token)
    namespace_attr.SetCustom(False)
    namespace_attr.Set(f"robot{i}")
    
    # Tick sim once to ensure the robot's graph is created before we try to edit it
    kit.update()
    
    # Edit the robot's graph to set the ROS topic names with the correct namespace
    robot_graph = og.get_graph_by_path(f"/World/robot{i}/dingo/RobotController")   
    if robot_graph is not None:    
        edit_nodes_config = {
            keys.SET_VALUES: [
                (f"/World/robot{i}/dingo/RobotController/ros2_subscribe_twist.inputs:topicName", f"robot{i}/cmd_vel"),
                (f"/World/robot{i}/dingo/RobotController/ros2_publish_transform_tree.inputs:topicName", f"robot{i}/tf"),
            ]   
        }
        og.Controller.edit(robot_graph, edit_nodes_config)
    else:
        print(f"Error: Robot graph not found for robot{i}")

n = len(START_POSITIONS)
print(f"Stripped {stripped_meshes}/{n * len(HEAVY_MESHES)} heavy meshes, "
      f"deactivated {disabled_frames}/{n * len(SENSOR_FRAMES)} sensor frames.")

# Play Simulation
omni.timeline.get_timeline_interface().play()

while kit.is_running():
    # Run in realtime mode, we don't specify a timestep, so it will run as fast as possible
    kit.update()
    
omni.timeline.get_timeline_interface().stop()
kit.close()