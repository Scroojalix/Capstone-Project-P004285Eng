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
# TODO: allow selecting between small and large warehouse
WORLD_USD = "SmallWarehouse.usd"
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

# 3 corner blocks of 9 + middle-left column of 3. 
# START_POS_LRG_WAREHOUSE = [
#     [-33.0, 26.0, 0],
#     [-33.0, 25.0, 0],
#     [-33.0, 24.0, 0],
#     [-34.0, 26.0, 0],
#     [-34.0, 25.0, 0],
#     [-34.0, 24.0, 0],
#     [-35.0, 26.0, 0],
#     [-35.0, 25.0, 0],
#     [-35.0, 24.0, 0],
#     [36.0, 26.0, 0],
#     [36.0, 25.0, 0],
#     [36.0, 24.0, 0],
#     [35.0, 27.0, 0],
#     [35.0, 24.0, 0],
#     [34.0, 24.0, 0],
#     [37.0, 26.0, 0],
#     [33.0, 24.0, 0],
#     [34.0, 23.0, 0],
#     [33.0, -26.0, 0],
#     [34.0, -28.0, 0],
#     [34.0, -27.0, 0],
#     [32.0, -27.0, 0],
#     [33.0, -28.0, 0],
#     [33.0, -27.0, 0],
#     [32.0, -26.0, 0],
#     [32.0, -28.0, 0],
#     [34.0, -26.0, 0],
#     [-34.0, -2.0, 0],
#     [-34.0, -3.0, 0],
#     [-34.0, -4.0, 0],
# ]

START_POS = []

for x in range(6):
    for y in range(5):
        X = -28 + x
        Y = -9 + y
        START_POS.append([X, Y, 0])


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