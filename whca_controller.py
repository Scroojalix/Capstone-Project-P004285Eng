
"""
WHCA fleet controller — drives namespaced Isaac Sim Dingo robots along WHCA* plans.

Pipeline:
    /robotN/tf (pose)  ->  snap to grid cell  ->  run_whca(...)
    ->  world waypoints  ->  synchronous go-to-waypoint controller  ->  /robotN/cmd_vel

This is the ROS-side "fleet controller". Isaac Sim (on Windows) provides the robot
bodies; this node (in WSL, ROS 2 Jazzy) provides the brain. It reuses Owen's planner
in WHCABaseline/whca_functions.py unchanged.

HOW TO RUN (ROS 2 Jazzy sourced, Isaac playing with robots spawned):
    python3 whca_controller.py

"""

import math
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

# --- planner (WHCABaseline/whca_functions.py) -------------------
HERE = os.path.dirname(os.path.abspath(__file__))
for cand in (os.path.join(HERE, "WHCABaseline"),          # file at repo root
             os.path.join(HERE, "..", "WHCABaseline")):    # file in gazebo_test/
    if os.path.isdir(cand):
        sys.path.insert(0, cand)
from whca_functions import run_whca  # noqa: E402

# ============================== CONFIG =======================================
# --- World <-> grid mapping (metres, warehouse frame) ---
# Cell (cx, cy) centre sits at world (ORIGIN_X + cx*CELL, ORIGIN_Y + cy*CELL).
# Owen spawns robots at x = -33.5 - j, y = 26.5 - i for j,i in 0..2, so with the
# origin below, robots 0..8 land on integer cells near the (2, *) column.
#
# 
ORIGIN_X = -35.5
ORIGIN_Y = 24.5
CELL = 1.0            # metres per cell (Dingo is ~0.5 m, so 1 m gives clearance)

GRID_DIMX = 8         # cells along world +x
GRID_DIMY = 6         # cells along world +y
OBSTACLES = []        # list of (cx, cy) blocked cells; empty = open floor for now

# three robots with crossing goals. Exercises WHCA coordination
# (they must route around each other). Robots 0,1,2 start ~cells (2,2),(2,1),(2,0).
ROBOT_GOALS = {
    0: (6, 0),
    1: (6, 1),
    2: (6, 2),
}

WINDOW_SIZE = 8              # WHCA window W
BASE_FRAME_HINT = "base_link"   # pose = frame_id=world -> child_frame_id=base_link

# --- Controller gains ---
CONTROL_HZ = 20.0
ARRIVE_TOL = 0.25           # m: within this of the current step cell -> arrived
ALIGN_TOL = 0.30            # rad: turn in place until heading error below this
K_LIN, K_ANG = 1.2, 2.0
MAX_LIN, MAX_ANG = 0.6, 1.5
REVERSE_FORWARD = False     # set True if a robot drives AWAY from its target
# =============================================================================


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class WHCAController(Node):
    def __init__(self):
        super().__init__("whca_fleet_controller")
        self.robot_ids = list(ROBOT_GOALS.keys())

        self.grid = np.zeros((GRID_DIMX, GRID_DIMY), dtype=int)
        for (cx, cy) in OBSTACLES:
            self.grid[cx, cy] = 1

        self.pose = {}            # rid -> (x, y, yaw) in world
        self.frames_logged = set()
        self.pubs = {}
        for rid in self.robot_ids:
            self.pubs[rid] = self.create_publisher(Twist, f"/robot{rid}/cmd_vel", 10)
            self.create_subscription(
                TFMessage, f"/robot{rid}/tf",
                lambda m, r=rid: self.tf_cb(m, r), 10)

        self.traj = None          # rid -> list of (wx, wy), one per timestep
        self.T = 0                # number of timesteps
        self.step = 0             # current synchronous timestep
        self.done = False
        self.create_timer(1.0 / CONTROL_HZ, self.loop)
        self.get_logger().info(
            f"WHCA controller up. Robots {self.robot_ids}. Waiting for /robotN/tf...")

    # --- pose intake ---------------------------------------------------------
    def tf_cb(self, msg, rid):
        if rid not in self.frames_logged:
            self.frames_logged.add(rid)
            self.get_logger().info(
                f"robot{rid} /tf frames: {[t.child_frame_id for t in msg.transforms]}")
        base = None
        for tr in msg.transforms:
            if BASE_FRAME_HINT in tr.child_frame_id:
                base = tr
                break
        if base is None and msg.transforms:
            base = msg.transforms[0]
        if base is None:
            return
        t = base.transform.translation
        q = base.transform.rotation
        self.pose[rid] = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    # --- grid <-> world ------------------------------------------------------
    def world_to_cell(self, wx, wy):
        cx = int(round((wx - ORIGIN_X) / CELL))
        cy = int(round((wy - ORIGIN_Y) / CELL))
        return (min(max(cx, 0), GRID_DIMX - 1), min(max(cy, 0), GRID_DIMY - 1))

    def cell_to_world(self, cx, cy):
        return (ORIGIN_X + cx * CELL, ORIGIN_Y + cy * CELL)

    # --- planning (runs once, when every robot's pose is known) ---------------
    def plan(self):
        if any(rid not in self.pose for rid in self.robot_ids):
            return False
        starts, goals = [], []
        for rid in self.robot_ids:
            wx, wy, _ = self.pose[rid]
            starts.append(self.world_to_cell(wx, wy))
            goals.append(ROBOT_GOALS[rid])
        self.get_logger().info(f"Planning WHCA  starts={starts}  goals={goals}")

        arrival, trajectories, t0, _ = run_whca(starts, goals, self.grid, WINDOW_SIZE)

        self.T = max(len(tr) for tr in trajectories)
        self.traj = {}
        for k, rid in enumerate(self.robot_ids):
            cells = list(trajectories[k])
            cells += [cells[-1]] * (self.T - len(cells))          # pad with final cell
            self.traj[rid] = [self.cell_to_world(cx, cy) for (cx, cy) in cells]
            self.get_logger().info(
                f"robot{rid}: {len(trajectories[k])} steps, arrival_t={arrival[k]}")
        self.get_logger().info(f"Plan ready ({t0*1000:.1f} ms). Executing {self.T} steps.")
        return True

    # --- control loop --------------------------------------------------------
    def loop(self):
        if self.done:
            return
        if self.traj is None:
            if not self.plan():
                return
        if self.step >= self.T:
            self.finish()
            return

        everyone_arrived = True
        for rid in self.robot_ids:
            if rid not in self.pose:
                everyone_arrived = False
                continue
            wx, wy, yaw = self.pose[rid]
            tx, ty = self.traj[rid][self.step]
            dist = math.hypot(tx - wx, ty - wy)

            cmd = Twist()
            if dist < ARRIVE_TOL:
                self.pubs[rid].publish(cmd)                      # hold position
                continue
            everyone_arrived = False
            hd_err = wrap(math.atan2(ty - wy, tx - wx) - yaw)
            if REVERSE_FORWARD:
                hd_err = wrap(hd_err + math.pi)
            if abs(hd_err) > ALIGN_TOL:                          # turn in place first
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd_err))
            else:                                                # aligned -> drive
                fwd = max(0.0, min(MAX_LIN, K_LIN * dist))
                cmd.linear.x = -fwd if REVERSE_FORWARD else fwd
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd_err))
            self.pubs[rid].publish(cmd)

        if everyone_arrived:
            self.step += 1

    def finish(self):
        self.done = True
        for rid in self.robot_ids:
            self.pubs[rid].publish(Twist())
        self.get_logger().info("All robots reached their goals. Done.")


def main():
    rclpy.init()
    node = WHCAController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for rid in node.robot_ids:                               # stop everything
            node.pubs[rid].publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
