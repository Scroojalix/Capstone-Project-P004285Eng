import argparse
import random

# Add argument parser to allow spawning a custom number of robots
parser = argparse.ArgumentParser(description="Launch Isaac Sim with a warehouse world and multiple Dingo robots.")
parser.add_argument("--num_robots", type=int, default=20, help="Number of Dingo robots to spawn in the warehouse.")
args = parser.parse_args()

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

SCENARIO = "Narrow"

# Path to the USD files
WORLD_USD = ""
ROBOT_USD = "DingoRobot.usd"
match SCENARIO:
    case "Small":
        WORLD_USD = "SmallWarehouse.usd"
    case "Narrow":
        WORLD_USD = "NarrowCorridor.usd"

# Open the world USD file
if is_file(WORLD_USD):
    omni.usd.get_context().open_stage(WORLD_USD)
else:
    print(f"Error: World USD file not found at {WORLD_USD}")
    kit.close()
    sys.exit(1)
stage = omni.usd.get_context().get_stage()

START_POS = []

NUM_ROBOTS = max(0, min(args.num_robots, 100))

match SCENARIO:
    case "Small":
        for x in range(20):
            for y in range(5):
                X = -28.5 + 3 * x
                Y = -8.5 + 4 * y
                START_POS.append([X, Y, 0])
    case "Narrow":
        groups = [(-13.5, -9), (-13.5, 5),(9.5, -9), (9.5, 5)]
        for x in range(5):
            for y in range(5):
                for group in groups:
                    X = group[0] + x
                    Y = group[1] + y
                    START_POS.append([X, Y, 0])
        

# Randomise order of START_POS to avoid robots spawning in a grid pattern
random.shuffle(START_POS)

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

FACE_NORTH = (0.70710678, 0.0, 0.0,  0.70710678)
FACE_SOUTH = (0.70710678, 0.0, 0.0, -0.70710678)
SHARE_ROS_CONTEXT = True
SPAWN_BATCH = 10           # robots per batch (0 disables staggering)
SPAWN_SETTLE_TICKS = 60    # app updates to idle between batches (~1 s)

disabled_frames = 0
shared_contexts = 0

# Spawn the robot models at the specified positions
for i, pos in enumerate(START_POS[:NUM_ROBOTS]):
    # Add the robot USD reference to the stage
    add_reference_to_stage(ROBOT_USD, f"/World/robot{i}")

    # Set the robot's position and heading in the world
    robot_xform = SingleXFormPrim(f"/World/robot{i}/base_link")
    robot_xform.set_world_pose(
        position=pos,
        orientation=FACE_NORTH if i < 15 else FACE_SOUTH,
    )

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
    robot_graph = og.get_graph_by_path(f"/World/robot{i}/RobotController")   
    if robot_graph is not None:    
        edit_nodes_config = {
            keys.SET_VALUES: [
                (f"/World/robot{i}/RobotController/ros2_subscribe_twist.inputs:topicName", f"robot{i}/cmd_vel"),
                (f"/World/robot{i}/RobotController/ros2_publish_transform_tree.inputs:topicName", f"robot{i}/tf"),
            ]   
        }
        og.Controller.edit(robot_graph, edit_nodes_config)

        if SHARE_ROS_CONTEXT:
            ctx = f"/World/robot{i}/RobotController/ros2_context.outputs:context"
            try:
                og.Controller.edit(robot_graph, {
                    keys.DISCONNECT: [
                        (ctx, f"/World/robot{i}/RobotController"
                              f"/ros2_subscribe_twist.inputs:context"),
                        (ctx, f"/World/robot{i}/RobotController"
                              f"/ros2_publish_transform_tree.inputs:context"),
                    ],
                })
                shared_contexts += 1
            except Exception as exc:
                print(f"WARNING: robot{i} context disconnect failed: {exc}")
    else:
        print(f"Error: Robot graph not found for robot{i}")

    # Let DDS discovery catch up before spawning the next batch.
    if SPAWN_BATCH and (i + 1) % SPAWN_BATCH == 0 and (i + 1) < NUM_ROBOTS:
        print(f"  spawned {i + 1}/{NUM_ROBOTS}, settling...")
        for _ in range(SPAWN_SETTLE_TICKS):
            kit.update()

if SHARE_ROS_CONTEXT:
    print(f"Shared ROS context on {shared_contexts}/{NUM_ROBOTS} robots "
          f"({NUM_ROBOTS - shared_contexts} still on their own DDS participant).")

# Give every robot's ROS graph time to register before the controller connects.
print("Settling before play...")
for _ in range(SPAWN_SETTLE_TICKS * 3):
    kit.update()

omni.timeline.get_timeline_interface().play()

while kit.is_running():
    # Run in realtime mode, we don't specify a timestep, so it will run as fast as possible
    kit.update()
    
omni.timeline.get_timeline_interface().stop()
kit.close()