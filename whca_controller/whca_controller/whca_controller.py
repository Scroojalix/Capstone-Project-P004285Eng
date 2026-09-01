#!/usr/bin/env python3
"""
WHCA* fleet controller for Isaac Sim.

Receding-horizon execution of turn-aware WHCA* (Silver 2005) on a fleet of
namespaced differential-drive robots:

    every cycle:  read live poses from /robotN/tf
               -> plan one window with plan_window() (turn-aware, W//2 commit)
               -> each robot tracks its committed waypoints on a shared clock
               -> re-plan from real poses

Safety stack:
    1. Planning   — turn-aware WHCA*: plans are collision-free in space-time,
                    including rotation steps (WHCABaseline/whca_functions.py).
    2. Execution  — sequential waypoint tracking: robots visit exactly the
                    planned cells in order; the clock is a ceiling, never a
                    reason to skip cells. Sustained lag triggers an early re-plan.
    3. Safeguards — vacancy gate + headway control (toggle: SAFEGUARDS).
                    vacancy gate (don't enter a cell until physically clear)
                    and headway control (taper speed behind a slow robot).

Run (Windows, ROS-sourced pixi shell, Isaac playing with robots spawned):
    call C:\\pixi_ws\\ros2-windows\\local_setup.bat
    set ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    python whca_controller.py
"""
import math
import random
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

from whca_controller.whca_functions import plan_window, RRAstar
from whca_controller.helpers import *

# TODO: add argument to change between small and large warehouse
yaml_name = 'SmallWarehouseOccMap.yaml'

PLANNING_CELL = 1.0        # m per planning cell; must exceed the robot footprint

ROBOTS = list(range(20))
GOALS_WORLD = {            # robot id -> goal 
    0: (-30.0, -28.0),
    1: (-31.0, -16.0),
    2: (-31.0, -3.0),
    3: (-31.0, 9.0),
    4: (-30.0, 22.0),
    5: (-17.0, -28.0),
    6: (-14.0, -16.0),
    7: (-18.0, -3.0),
    8: (-18.0, 9.0),
    9: (-17.0, 22.0),
    10: (-4.0, -28.0),
    11: (-1.0, -16.0),
    12: (-4.0, -3.0),
    13: (-4.0, 9.0),
    14: (-4.0, 22.0),
    15: (8.0, -28.0),
    16: (8.0, -16.0),
    17: (8.0, -4.0),
    18: (8.0, 9.0),
    19: (8.0, 22.0),
    20: (21.0, -28.0),
    21: (21.0, -16.0),
    22: (21.0, -4.0),
    23: (21.0, 9.0),
    24: (20.0, -14.0),
    25: (33.0, -29.0),
    26: (34.0, -16.0),
    27: (34.0, -3.0),
    28: (34.0, 9.0),
    29: (34.0, 22.0),
}

WINDOW_SIZE = 32            # WHCA window W; commit/re-plan every W//2 steps
STEP_SECONDS = 1.9         # wall-clock length of one plan timestep (one cell
                           # traverse OR one 90-degree rotation)
LAG_REPLAN = 1.5           # re-plan early if any robot falls this many steps behind
DEADLOCK_CYCLES = 12       # stop if no robot has moved for this many windows

BASE_FRAME = "base_link"   # /tf: frame_id "world" -> this child frame = robot pose

# Drive controller
CONTROL_HZ = 20.0
ARRIVE_TOL = 0.18          # m: waypoint reached
ALIGN_TOL = 0.30           # rad: rotate in place until heading error below this
K_LIN, K_ANG = 1.2, 2.0
MAX_LIN, MAX_ANG = 0.6, 1.5

# Execution safeguards
SAFEGUARDS = True           # master switch for the execution-layer safeguards below.
                            # True  = vacancy gate + headway control (our method).
                            # False = pure WHCA* execution, no safeguards (baseline for
                            #         comparison vs k-robust / ADG). Collisions may occur
                            #         when off 
CLEAR_RADIUS = 0.635        # cell counts occupied while any robot centre is within this
                           #  CELL of its centre. 0.635; headway covers the final approach.
HEADWAY = 0.8             # m: taper speed to zero behind a robot ahead
COLLIDE_DIST = 0.68        # m: contact event -> forensic log
# =============================================================================

class WHCAController(Node):
    def __init__(self):
        super().__init__("whca_fleet_controller")
        self.robot_ids = list(ROBOTS)
        self.step_size = max(1, WINDOW_SIZE // 2)

        self.pose = {}                       # rid -> (x, y, yaw), live from /tf
        self.pubs = {}
        self._seen_frames = set()
        for rid in self.robot_ids:
            self.pubs[rid] = self.create_publisher(Twist, f"/robot{rid}/cmd_vel", 10)
            self.create_subscription(TFMessage, f"/robot{rid}/tf",
                                     lambda m, r=rid: self._tf_cb(m, r), 10)

        self.goals = None                    # rid -> goal cell (set once)
        self.rra = None                      # rid -> persistent RRAstar
        self.planning = True                 # True: plan next tick; False: executing
        self.waypoints = {}                  # rid -> committed world waypoints [0..W//2]
        self.progress = {}                   # rid -> last waypoint index reached
        self.sched_cells = {}                # rid -> committed cells (forensics)
        self.window_t0 = 0.0
        self.replans = 0
        self.stuck_windows = 0
        self.done = False
        self._contacts_logged = set()
        self._gate_log_t = 0.0
        
        self.map: Map = load_map(yaml_name, PLANNING_CELL)

        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self.get_logger().info(
            f"Map {yaml_name}: {self.map.dimx}x{self.map.dimy} @ {self.map.cell_size:.2f} m, "
            f"{int(self.map.grid.sum())} blocked. Robots {self.robot_ids}, "
            f"W={WINDOW_SIZE} (commit {self.step_size}).")

    # ---------------- pose intake ----------------
    def _tf_cb(self, msg, rid):
        if rid not in self._seen_frames:
            self._seen_frames.add(rid)
            self.get_logger().info(
                f"robot{rid} /tf frames: {[t.child_frame_id for t in msg.transforms]}")
        for tr in msg.transforms:
            if tr.child_frame_id == BASE_FRAME and tr.header.frame_id == "world":
                t, q = tr.transform.translation, tr.transform.rotation
                self.pose[rid] = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))
                return

    def _cell(self, rid):
        wx, wy, _ = self.pose[rid]
        return self.map.nearest_free(*self.map.world_to_cell(wx, wy))

    # ---------------- planning ----------------
    def _setup_goals(self):
        """One-time: fix each robot's goal cell (deduplicated) and its RRA*."""
        goals, taken = {}, set()
        for rid in self.robot_ids:
            g = self.map.nearest_free(*self.map.world_to_cell(*GOALS_WORLD[rid]), taken=taken)
            goals[rid] = g
            taken.add(g)
        self.goals = goals
        self.rra = {rid: RRAstar(goals[rid][0], goals[rid][1], self.map.grid)
                    for rid in self.robot_ids}
        self.get_logger().info(f"Goals: {sorted(goals.items())}")

    def _distinct_starts(self):
        """Current cells, made unique so the planner never gets two agents in one cell."""
        starts, taken = [], set()
        for rid in self.robot_ids:
            c = self._cell(rid)
            if c in taken:
                c = self.map.nearest_free(*c, taken=taken)
            starts.append(c)
            taken.add(c)
        return starts

    def _plan(self):
        """Plan one window and commit the first W//2 steps as timed waypoints."""
        if any(rid not in self.pose for rid in self.robot_ids):
            return False                       # still waiting for /tf
        if self.goals is None:
            self._setup_goals()

        starts = self._distinct_starts()
        at_goal = [s == self.goals[rid] for s, rid in zip(starts, self.robot_ids)]
        if all(at_goal):
            self._finish()
            return True

        # Silver-faithful planning:
        #  - EVERY agent plans every window, including agents at their goals
        #    (they plan zero-cost waits, but will step aside if a higher-priority
        #    agent needs their cell — no permanent parking).
        #  
        n = len(self.robot_ids)
        # priorities are assigned randomly

        order = random.sample(self.robot_ids, n)
        idx = {rid: self.robot_ids.index(rid) for rid in order}
        o_starts = [starts[idx[r]] for r in order]
        o_goals = [self.goals[r] for r in order]
        o_rra = [self.rra[r] for r in order]
        o_head = [yaw_to_heading(self.pose[r][2]) for r in order]

        t0 = time.perf_counter()
        o_paths = plan_window(o_starts, o_goals, self.map.grid, WINDOW_SIZE,
                              [False] * n, o_rra, start_headings=o_head)
        dt_ms = (time.perf_counter() - t0) * 1000
        self.replans += 1
        paths = {rid: p for rid, p in zip(order, o_paths)}

        moved = 0
        for rid in self.robot_ids:
            path = paths[rid]
            by_t = {st.t: (st.x, st.y) for st in path}
            cells = [by_t.get(0, starts[self.robot_ids.index(rid)])]
            for t in range(1, self.step_size + 1):
                cells.append(by_t.get(t, cells[-1]))
                moved += cells[t] != cells[t - 1]
            self.sched_cells[rid] = cells
            self.waypoints[rid] = [self.map.cell_to_world(*c) for c in cells]
        self.progress = {rid: 0 for rid in self.robot_ids}
        self.window_t0 = time.monotonic()
        self.planning = False

        self.stuck_windows = 0 if moved else self.stuck_windows + 1
        if self.stuck_windows >= DEADLOCK_CYCLES:
            self.get_logger().error("No robot has moved for several windows — "
                                    "likely deadlock. Stopping.")
            self._finish()
        self.get_logger().info(
            f"[replan {self.replans}] {dt_ms:.1f} ms | commit {self.step_size} steps"
            f" | at goal {sum(at_goal)}/{len(at_goal)} | prio {order}")
        return True

    # ---------------- execution safeguards ----------------
    def _cell_occupant(self, rid, tx, ty):
        """Another robot still physically inside the cell centred (tx, ty), or None."""
        for other in self.robot_ids:
            if other != rid and other in self.pose:
                ox, oy, _ = self.pose[other]
                if math.hypot(ox - tx, oy - ty) < CLEAR_RADIUS * self.map.cell_size:
                    return other
        return None

    def _gap_ahead(self, rid, wx, wy, tx, ty):
        """Distance to the nearest robot in a ~60-degree cone toward my target."""
        vx, vy = tx - wx, ty - wy
        vn = math.hypot(vx, vy)
        if vn < 1e-6:
            return None
        best = None
        for other in self.robot_ids:
            if other != rid and other in self.pose:
                dx, dy = self.pose[other][0] - wx, self.pose[other][1] - wy
                d = math.hypot(dx, dy)
                if 1e-6 < d <= HEADWAY and (dx * vx + dy * vy) / (d * vn) > 0.5:
                    best = d if best is None else min(best, d)
        return best

    def _check_contacts(self, plan_now):
        """Forensics: log full context on the first contact between any pair."""
        ids = [r for r in self.robot_ids if r in self.pose]
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = ids[i], ids[j]
                ax, ay, _ = self.pose[a]
                bx, by, _ = self.pose[b]
                d = math.hypot(ax - bx, ay - by)
                if d < COLLIDE_DIST and (a, b) not in self._contacts_logged:
                    self._contacts_logged.add((a, b))
                    self.get_logger().error(
                        f"CONTACT robots {a}&{b} d={d:.2f} m at plan_t={plan_now:.2f} "
                        f"(replan #{self.replans})\n"
                        f"  r{a}: at {self.map.world_to_cell(ax, ay)}, "
                        f"sched={fmt_sched(self.sched_cells[a])}\n"
                        f"  r{b}: at {self.map.world_to_cell(bx, by)}, "
                        f"sched={fmt_sched(self.sched_cells[b])}")

    # ---------------- control loop ----------------
    def _tick(self):
        if self.done:
            return
        if self.planning:
            self._plan()
            return

        plan_now = (time.monotonic() - self.window_t0) / STEP_SECONDS
        self._check_contacts(plan_now)
        due = min(int(plan_now) + 1, self.step_size)   # furthest step the clock allows
        max_lag = 0

        for rid in self.robot_ids:
            if rid not in self.pose:
                continue
            wx, wy, yaw = self.pose[rid]
            prog = self.progress[rid]
            max_lag = max(max_lag, due - prog - 1)
            target = min(prog + 1, due)                # next waypoint only, never skip
            tx, ty = self.waypoints[rid][target]
            dist = math.hypot(tx - wx, ty - wy)
            cmd = Twist()

            if dist < ARRIVE_TOL:                      # at waypoint
                if target > prog:
                    self.progress[rid] = target
                self._pre_rotate(rid, wx, wy, yaw, cmd)   # planned rotation step
                self.pubs[rid].publish(cmd)
                continue

            occ = self._cell_occupant(rid, tx, ty) if SAFEGUARDS else None
            if occ is not None:
                # STRICT vacancy gate: never enter a cell while any robot is
                # physically inside it, regardless of whether it plans to leave.
                now = time.monotonic()
                if now - self._gate_log_t > 2.0:
                    self._gate_log_t = now
                    self.get_logger().warn(
                        f"vacancy gate: r{rid} holding for r{occ} in "
                        f"{self.map.world_to_cell(tx, ty)}")
                self.pubs[rid].publish(cmd)
                continue

            hd = wrap(math.atan2(ty - wy, tx - wx) - yaw)
            if abs(hd) > ALIGN_TOL:                    # face the cell first
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd))
            else:                                      # drive, with headway taper
                fwd = max(0.0, min(MAX_LIN, K_LIN * dist))
                if SAFEGUARDS:
                    gap = self._gap_ahead(rid, wx, wy, tx, ty)
                    if gap is not None:
                        fwd = min(fwd, MAX_LIN * max(0.0, (gap - 0.6) / (HEADWAY - 0.6)))
                cmd.linear.x = fwd
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd))
            self.pubs[rid].publish(cmd)

        window_done = all(self.progress[r] >= self.step_size
                          for r in self.robot_ids if r in self.pose)
        if (plan_now >= self.step_size and window_done) or max_lag > LAG_REPLAN:
            self.planning = True                       # re-plan from real poses

    def _pre_rotate(self, rid, wx, wy, yaw, cmd):
        """During a planned rotation/wait step, pre-align toward the next new cell."""
        for wp in self.waypoints[rid][self.progress[rid] + 1:]:
            if math.hypot(wp[0] - wx, wp[1] - wy) > ARRIVE_TOL:
                hd = wrap(math.atan2(wp[1] - wy, wp[0] - wx) - yaw)
                if abs(hd) > 0.08:
                    cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd))
                return

    def _finish(self):
        self.done = True
        for rid in self.robot_ids:
            self.pubs[rid].publish(Twist())
        self.get_logger().info(f"Done. Total re-plans: {self.replans}.")


def main():
    rclpy.init()
    node = WHCAController()
    node.get_logger().info('Starting WHCA Controller. Awaiting /tf frames...')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            for rid in node.robot_ids:
                node.pubs[rid].publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
