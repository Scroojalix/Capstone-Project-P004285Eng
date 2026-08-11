from isaacsim import SimulationApp
kit = SimulationApp({"headless": False})

import sys
import omni
from pxr import Sdf
from isaacsim.storage.native import is_file
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.storage.native import get_assets_root_path
import omni.graph.core as og

# Enable the ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

assets_root_path = get_assets_root_path()
keys = og.Controller.Keys

# Path to the USD files
WORLD_USD = "Warehouse.usd"
ROBOT_USD = "DingoRobot.usd"

# Open the world USD file
if is_file(WORLD_USD):
    omni.usd.get_context().open_stage(WORLD_USD)
else:
    print(f"Error: World USD file not found at {WORLD_USD}")
    kit.close()
    sys.exit(1)
stage = omni.usd.get_context().get_stage()

# 3 corner blocks of 9 + middle-left column of 3. 
START_POSITIONS = [
    [-33.0, 26.0, 0],
    [-33.0, 25.0, 0],
    [-33.0, 24.0, 0],
    [-34.0, 26.0, 0],
    [-34.0, 25.0, 0],
    [-34.0, 24.0, 0],
    [-35.0, 26.0, 0],
    [-35.0, 25.0, 0],
    [-35.0, 24.0, 0],
    [36.0, 26.0, 0],
    [36.0, 25.0, 0],
    [36.0, 24.0, 0],
    [35.0, 27.0, 0],
    [35.0, 24.0, 0],
    [34.0, 24.0, 0],
    [37.0, 26.0, 0],
    [33.0, 24.0, 0],
    [34.0, 23.0, 0],
    [33.0, -26.0, 0],
    [34.0, -28.0, 0],
    [34.0, -27.0, 0],
    [32.0, -27.0, 0],
    [33.0, -28.0, 0],
    [33.0, -27.0, 0],
    [32.0, -26.0, 0],
    [32.0, -28.0, 0],
    [34.0, -26.0, 0],
    [-34.0, -2.0, 0],
    [-34.0, -3.0, 0],
    [-34.0, -4.0, 0],
]

# Spawn the robot models at the specified positions
for i, pos in enumerate(START_POSITIONS):    
    # Add the robot USD reference to the stage
    add_reference_to_stage(ROBOT_USD, f"/World/robot{i}")
    
    # Set the robot's position in the world
    robot_xform = SingleXFormPrim(f"/World/robot{i}")
    robot_xform.set_world_pose(position=pos)
    
    kit.update()
    
    # Set the robot's namespace attribute
    # FIXME: namespace is set correctly, but the ROS topics are not prefixed with the namespace.
    # I think it may be to do with the simulation cache needing a refresh
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

# Play Simulation
# omni.timeline.get_timeline_interface().play()

while kit.is_running():
    # Run in realtime mode, we don't specify a timestep, so it will run as fast as possible
    kit.update()
    
omni.timeline.get_timeline_interface().stop()
kit.close()