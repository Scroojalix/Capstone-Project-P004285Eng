from isaacsim import SimulationApp
kit = SimulationApp({"headless": False})

import os
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

# Publish one shared simulation clock for the external fleet controller.
# Wiring follows NVIDIA's ROS 2 Clock tutorial for the bridge used by this project:
# https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_clock.html
# Do not run a second /clock publisher alongside this graph.
CLOCK_GRAPH = "/World/WHCAClock"
if not stage.GetPrimAtPath(CLOCK_GRAPH).IsValid():
    og.Controller.edit(
        {"graph_path": CLOCK_GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("SimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "/clock"),
                ("SimTime.inputs:resetOnStop", False),
            ],
        },
    )
print("Publishing simulation time on /clock for WHCA execution.")

# Spawns come from scenarios.py so they always match the controller's goals.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from scenarios import START_POSITIONS, SCENARIO
print(f"Scenario: {SCENARIO} ({len(START_POSITIONS)} robots)")


FACE_NORTH = (0.70710678, 0.0, 0.0,  0.70710678)
FACE_SOUTH = (0.70710678, 0.0, 0.0, -0.70710678)
SHARE_ROS_CONTEXT = True
SPAWN_BATCH = 10           # robots per batch (0 disables staggering)
SPAWN_SETTLE_TICKS = 60    # app updates to idle between batches (~1 s)

stripped_meshes = 0
disabled_frames = 0
shared_contexts = 0

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

        if SHARE_ROS_CONTEXT:
            ctx = f"/World/robot{i}/dingo/RobotController/ros2_context.outputs:context"
            try:
                og.Controller.edit(robot_graph, {
                    keys.DISCONNECT: [
                        (ctx, f"/World/robot{i}/dingo/RobotController"
                              f"/ros2_subscribe_twist.inputs:context"),
                        (ctx, f"/World/robot{i}/dingo/RobotController"
                              f"/ros2_publish_transform_tree.inputs:context"),
                    ],
                })
                shared_contexts += 1
            except Exception as exc:
                print(f"WARNING: robot{i} context disconnect failed: {exc}")
    else:
        print(f"Error: Robot graph not found for robot{i}")

    # Let DDS discovery catch up before spawning the next batch.
    if SPAWN_BATCH and (i + 1) % SPAWN_BATCH == 0 and (i + 1) < len(START_POSITIONS):
        print(f"  spawned {i + 1}/{len(START_POSITIONS)}, settling...")
        for _ in range(SPAWN_SETTLE_TICKS):
            kit.update()

n = len(START_POSITIONS)
if SHARE_ROS_CONTEXT:
    print(f"Shared ROS context on {shared_contexts}/{n} robots "
          f"({n - shared_contexts} still on their own DDS participant).")
print(f"Stripped {stripped_meshes}/{n * len(HEAVY_MESHES)} heavy meshes, "
      f"deactivated {disabled_frames}/{n * len(SENSOR_FRAMES)} sensor frames.")

# Give every robot's ROS graph time to register before the controller connects.
print("Settling before play...")
for _ in range(SPAWN_SETTLE_TICKS * 3):
    kit.update()

# Play Simulation
omni.timeline.get_timeline_interface().play()

while kit.is_running():
    # Run in realtime mode, we don't specify a timestep, so it will run as fast as possible
    kit.update()
    
omni.timeline.get_timeline_interface().stop()
kit.close()