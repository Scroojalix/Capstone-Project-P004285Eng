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
import time
import random

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

from whca_controller.whca_functions import *
from whca_controller.helpers import *

# TODO: add argument to change between small and large warehouse
yaml_name = 'SmallWarehouseOccMap.yaml'

PLANNING_CELL = 1.0        # m per planning cell; must exceed the robot footprint

GOALS = []
for x in range(8):
    for y in range(5):
        X = -28.5 + 8 * x
        Y = -8.5 + 4 * y
        GOALS.append([X, Y])

WINDOW_SIZE = 32            # WHCA window W; commit/re-plan every W//2 steps
# FIXME: should not be assigning a fix time to a time step. It should be allowed to vary.
STEP_SECONDS = 1.9         # wall-clock length of one plan timestep (one cell
                           # traverse OR one 90-degree rotation)
LAG_REPLAN = 10           # re-plan early if any robot falls this many steps behind
DEADLOCK_CYCLES = 12       # stop if no robot has moved for this many windows
STALL_TIMEOUT = 300.0      # s: hard cap on a run with no execution progress at all
MAX_REPLANS = 200         # stop if this many re-plans have been attempted

# Drive controller
CONTROL_HZ = 20.0
ARRIVE_TOL = 0.18          # m: waypoint reached
ALIGN_TOL = 0.30           # rad: rotate in place until heading error below this
K_LIN, K_ANG = 1.2, 2.0
MAX_LIN, MAX_ANG = 0.6, 1.5

# Execution safeguards
# SAFEGUARDS = False           # master switch for the execution-layer safeguards below.
                            # True  = vacancy gate + headway control (our method).
                            # False = pure WHCA* execution, no safeguards (baseline for
                            #         comparison vs k-robust / ADG). Collisions may occur
                            #         when off -- that is the purpose of the baseline.
CLEAR_RADIUS = 0.635        # cell counts occupied while any robot centre is within this
                           #  CELL of its centre. 0.635; headway covers the final approach.
HEADWAY = 0.8             # m: taper speed to zero behind a robot ahead
COLLIDE_DIST = 0.62        # m: contact event -> forensic log
INFLATE_M = 0.30           # m: obstacle inflation. Dingo radius is 0.389 m
# K_ROBUST = 0                # 0 = standard WHCA* (Silver 2005). >=1 = k-robust WHCA* (Atzmon et al. 2018)
# =============================================================================

class Robot():
    def __init__(self, id: int, map: Map, node: Node):
        self.id: int = id
        self.map: Map = map

        # Pose of robot (x, y, yaw) live from /tf, or None if not yet received
        self.pose: tuple[float, float, float] | None = None
        
        self.goal: tuple[int, int] | None = None
        self.rra: RRAstar | None = None
        
        self.frst_arrv_t = None
        
        self.waypoints = []
        self.progress = 0
        self.sched_cells = []
        self.history = []
        self.starved = 0
        
        # Publisher and subscriber for each robot
        self.pub = node.create_publisher(Twist, f"/robot{id}/cmd_vel", 10)
        node.create_subscription(TFMessage, f"/robot{id}/tf",
            lambda msg: self.update_pose(msg), 10)

    def update_pose(self, msg: TFMessage):
        for tf in msg.transforms:
            if tf.child_frame_id == "base_link" and tf.header.frame_id == "world":
                t, q = tf.transform.translation, tf.transform.rotation
                self.pose = (t.x, t.y, yaw_from_quat(q.x, q.y, q.z, q.w))
                return
            
    def cell(self):
        """Current cell of robot."""
        wx, wy, _ = self.pose
        cx, cy = self.map.world_to_cell(wx, wy)
        cell =  self.map.nearest_free(cx, cy)
        return cell
    
    def set_goal(self, goal: tuple[int, int]):
        self.goal = goal
        self.rra = RRAstar(goal, self.map.grid)
    
    def at_goal(self):
        """Returns true if this robot is at its goal"""
        if self.pose is None:
            return False
        else:
            return self.cell() == self.goal

class WHCAController(Node):
    def __init__(self):
        super().__init__("whca_fleet_controller")
        
        self.map: Map = load_map(yaml_name, PLANNING_CELL)
  
        # Get parameters from launch file
        self.declare_parameter('num_robots', 20)
        self.declare_parameter('safeguards', False)
        self.declare_parameter('k_robust', 0)
        self.declare_parameter('debug', False)
        self.num_robots = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.safeguards = self.get_parameter('safeguards').get_parameter_value().bool_value
        self.k = max(0, self.get_parameter('k_robust').get_parameter_value().integer_value)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        # TODO: determine number of robots from number of /robotN/tf topics
        self.num_robots = max(1, self.num_robots)        
        self.robots: list[Robot] = [Robot(rid, self.map, self) for rid in range(self.num_robots)]
        
        # True: plan next tick; False: executing
        self.planning = False
        
        self.commit_size = max(1, WINDOW_SIZE // 2)

        self.t = 0

        self.total_advances = 0
        self._advances_at_last_plan = 0
        self.plan_stats = {}                 # planner diagnostics
        self.window_t0 = 0.0
        self.replans = 0
        self.stuck_windows = 0
        self._contacts_logged = set()
        self.lag_samples = []                # max_lag per tick, for the run summary
        
        # Timing
        self.t_start = None                  # wall clock at first committed window
        self.last_exec_t = None
        self._gate_log_t = 0.0
        
        # Metrics
        self.planning_times = []
        self.hundred_cycle_success = None

        # Log startup information
        self.get_logger().info(f"Map {yaml_name}: {self.map.dimx}x{self.map.dimy} @ {self.map.cell_size:.2f} m."
            f"{int(self.map.grid.sum())} blocked")
        self.get_logger().info(f"Num Robots: {len(self.robots)}")
        self.get_logger().info(f"W={WINDOW_SIZE} (commit {self.commit_size}).")
        self.get_logger().info(f"Safeguards={self.safeguards}, k_robust={self.k}")
        
        # Start control loop
        self.get_logger().info('Starting WHCA Controller. Awaiting /tf frames...\n')
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.tick)
        
    def setup_goals(self):
        """Set robot goal positions"""
        self.get_logger().info("Robot goal positions:")
        taken = set()
        for r in self.robots:
            # Select distinct random goal, ensuring not already taken
            found_goal = False
            while not found_goal:
                goal = random.choice(GOALS)
                goal_cell = self.map.world_to_cell(goal[0], goal[1])
                
                # Ensure goal is defined, not taken, and not current robot start pos
                if goal_cell is not None and goal_cell not in taken and goal_cell != r.cell():
                    found_goal = True
                    r.set_goal(goal_cell)
                    taken.add(goal_cell)
            
            # Log result
            self.get_logger().info(f"r{r.id}: {r.cell()} -> {r.goal}")
    
    def tick(self):
        """Main control loop of controller node"""   
        # Check if all robots at goal. If so, finish node.
        if self.num_at_goal() == self.num_robots:
            self.finish()
            return
        
        if self.planning:
            # This tick will be used to replan window and commit waypoints to robots
            self.plan_and_commit_window()
            return
        
        now = time.time()
        for r in self.robots:
            if r.pose is None:
                # Robot pose undefined. Still awaiting /tf
                return
            if r.goal is None:
                # If any robot has undefined goal, setup all goals
                self.setup_goals()
                return
            if not r.waypoints:
                # Enable planning, only once all starting poses and goals are determined
                # FIXME: if a robot is at its goal, it will have no waypoints, triggering a replan.
                self.planning = True
                return
            if r.at_goal() and r.frst_arrv_t is None:
                # Print first arrival time of robot
                r.frst_arrv_t = now - self.t_start
                self.get_logger().info(
                    f"robot{r.id} at goal {r.goal} ({r.frst_arrv_t:.1f} s) [{self.num_at_goal()}/{self.num_robots}]")
        
        plan_now = (time.monotonic() - self.window_t0) / STEP_SECONDS
        self._check_contacts(plan_now)
        due = min(int(plan_now) + 1, self.commit_size)   # furthest step the clock allows
        max_lag = 0

        # FIXME: loops through robots twice. This should be refactored.
        for robot in self.robots:
            if robot.pose is None:
                continue
            wx, wy, yaw = robot.pose
            prog = robot.progress
            max_lag = max(max_lag, due - prog - 1)
            target = min(prog + 1, due)                # next waypoint only, never skip
            tx, ty = robot.waypoints[target]
            dist = math.hypot(tx - wx, ty - wy)
            cmd = Twist()

            if dist < ARRIVE_TOL:                      # at waypoint
                if target > prog:
                    robot.progress = target
                    self.total_advances += 1
                    self.last_exec_t = time.monotonic()
                self._pre_rotate(robot, wx, wy, yaw, cmd)   # planned rotation step
                robot.pub.publish(cmd)
                continue

            occ = self._cell_occupant(robot.id, tx, ty) if self.safeguards else None
            if occ is not None:
                # STRICT vacancy gate: never enter a cell while any robot is
                # physically inside it, regardless of whether it plans to leave.
                now = time.monotonic()
                if now - self._gate_log_t > 2.0:
                    self._gate_log_t = now
                    self.get_logger().warn(
                        f"vacancy gate: r{robot.id} holding for r{occ} in "
                        f"{self.map.world_to_cell(tx, ty)}")
                robot.pub.publish(cmd)
                continue

            hd = wrap(math.atan2(ty - wy, tx - wx) - yaw)
            if abs(hd) > ALIGN_TOL:                    # face the cell first
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd))
            else:                                      # drive, with headway taper
                fwd = max(0.0, min(MAX_LIN, K_LIN * dist))
                if self.safeguards:
                    gap = self._gap_ahead(robot, wx, wy, tx, ty)
                    if gap is not None:
                        fwd = min(fwd, MAX_LIN * max(0.0, (gap - 0.6) / (HEADWAY - 0.6)))
                cmd.linear.x = fwd
                cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd))
            robot.pub.publish(cmd)

        self.lag_samples.append(max_lag)

        
        now = time.monotonic()
        if self.last_exec_t is not None and now - self.last_exec_t > STALL_TIMEOUT:
            self.get_logger().error(
                f"No robot has reached a waypoint in {STALL_TIMEOUT:.0f} s - stopping.")
            self.finish(reason="stalled")
            return

        window_done = all(r.progress >= self.commit_size for r in self.robots if r.pose is not None)
        if (plan_now >= self.commit_size and window_done) or max_lag > LAG_REPLAN:
            self.planning = True                       # re-plan from real poses  
        
    def num_at_goal(self):
        """Return number of robots at their goals."""
        return sum(1 for robot in self.robots if robot.at_goal())

    def plan_and_commit_window(self):
        """
        Plan one window and commit the first W//2 steps as timed waypoints.

        Planning is faithful to original David Silver (2005) paper:
         - EVERY agent plans every window, including agents at their goals
           (they plan zero-cost waits, but will step aside if a higher-priority
           agent needs their cell — no permanent parking).
         - Priority is randomised each window (Silver 2005) so no fixed right-of-way
           pattern can repeat forever; this breaks symmetric deadlocks.
        
        """
        
        # Snapshot each robot's last K_ROBUST executed cells before the schedule
        # FIXME: what is the purpose of this?
        for robot in self.robots:
            if robot.sched_cells:
                advanced = robot.progress > 0
                robot.starved = 0 if advanced else robot.starved + 1
                
                if self.k > 0:
                    cells = robot.sched_cells
                    p = robot.progress
                    if cells:
                        robot.history = [cells[p - j] for j in range(1, self.k + 1) if p - j >= 0]
                                         
        
        # FIXME: what if robots are in same cell.
        # Should be impossible if safeguards are on, but can happen if off.
                
        # Determine which robots are at their goals
        at_goal = [robot.cell() == robot.goal for robot in self.robots]
        
        # Program exits when all robots are at their goals, or if max replans reached
        if all(at_goal):
            self.finish()
            return True
        if self.replans >= MAX_REPLANS:
            self.get_logger().error(
                f"Reached {self.replans} replans without all robots at goal. "
                f"Stopping to avoid infinite loop.")
            self.finish(reason="max replans reached")
            return True
        
        self.replans += 1

        # Randomise the order of robots for this planning window
        idx = list(range(len(self.robots)))
        random.shuffle(idx)
        
        # FIXME: what is this? Is it necessary?
        # idx.sort(key=lambda r: -self.starved.get(r, 0))
                
        # FIXME: what if robots are in same cell. Start cell will not be distinct.
        
        # Order robots list in planning priority order
        o_robots = [self.robots[v] for k, v in enumerate(idx)]
        
        o_curr = [r.cell() for r in o_robots]
        o_goals = [r.goal for r in o_robots]
        o_rra = [r.rra for r in o_robots]
        o_head = [yaw_to_heading(r.pose[2]) for r in o_robots]
        o_hist = [r.history for r in o_robots] if self.k > 0 else None

        # TODO: refactor plan_window to take Robot class and order
        # to avoid this mess of parallel lists. Then we can remove the idx mapping and the o_* lists.
        t0 = time.perf_counter() # Planning time t0
        o_paths = plan_window(o_curr, o_goals, self.map.grid, WINDOW_SIZE,
                              [False] * len(self.robots), o_rra, start_headings=o_head,
                              commit_horizon=self.commit_size,
                              k=self.k, history=o_hist)
        
        if self.debug:
            print_window(o_paths)
        
        # Add plan time to metrics
        plan_time = (time.perf_counter() - t0) * 1000
        self.planning_times.append(plan_time)
                
        # Record success rate at 100 replans (metric used by David Silver in his WHCA* paper)
        # FIXME: program should terminate at 100 replans
        if self.replans == 100:
            self.hundred_cycle_success = self.num_at_goal() / len(self.robots) * 100

        
        # Push the next W//2 steps into the schedule, filling in with last known cell if no plan
        # TODO: refactor this
        for rid, path in zip(idx, o_paths):
            robot = self.robots[rid]
            
            by_t = {state.t: (state.x, state.y) for state in path}
            
            cells = [by_t.get(0, robot.cell())]
            
            for t in range(1, self.commit_size + 1):
                cells.append(by_t.get(t, cells[-1]))
                
            robot.sched_cells = cells
            robot.waypoints = [self.map.cell_to_world(*c) for c in cells]
            robot.progress = 0
        
        # Updating timing info
        self.window_t0 = time.monotonic()
        if self.last_exec_t is None:
            self.last_exec_t = self.window_t0
        if self.t_start is None:
            self.t_start = self.window_t0
        self.planning = False
        
        # Did any robot execute at least one step since the last plan?
        # If not, increment the stuck counter.
        # If robot stuck for too long, log warning.
        executed = self.total_advances > self._advances_at_last_plan
        self._advances_at_last_plan = self.total_advances
        self.stuck_windows = 0 if executed else self.stuck_windows + 1
        if self.stuck_windows >= DEADLOCK_CYCLES:
            self.get_logger().warn(
                f"No waypoint reached by any robot for {self.stuck_windows} windows. "
                f"Still running - the run ends on its own at goal, or after "
                f"{STALL_TIMEOUT:.0f} s without progress.")

        # Get list of robots that have been starved for >=3 windows (i.e. no execution)
        blocked = [r for r in self.robots if r.starved >= 3 and r.cell() != r.goal]
        if blocked:
            self.get_logger().warn(
                f"starved (no execution for >=3 windows): "
                + ", ".join(f"r{r.id}x{r.starved}@{r.cell()}" for r in blocked))
        self.get_logger().info(
            f"[replan {self.replans}] {plan_time:.1f} ms | commit {self.commit_size} steps"
            f" | k={self.k}"
            f" | at goal {sum(at_goal)}/{len(at_goal)} | prio {idx}")
        return True

    # ---------------- execution safeguards ----------------
    def _cell_occupant(self, rid, tx, ty):
        """Another robot still physically inside the cell centred (tx, ty), or None."""
        for other in self.robots:
            if other != rid and other.pose:
                ox, oy, _ = other.pose
                if math.hypot(ox - tx, oy - ty) < CLEAR_RADIUS * self.map.cell_size:
                    return other
        return None

    def _gap_ahead(self, robot: Robot, wx, wy, tx, ty):
        """Distance to the nearest robot in a ~60-degree cone toward my target."""
        vx, vy = tx - wx, ty - wy
        vn = math.hypot(vx, vy)
        if vn < 1e-6:
            return None
        best = None
        for other in self.robots:
            if other.id != robot.id and other.pose is not None:
                dx, dy = other.pose[0] - wx, other.pose[1] - wy
                d = math.hypot(dx, dy)
                if 1e-6 < d <= HEADWAY and (dx * vx + dy * vy) / (d * vn) > 0.5:
                    best = d if best is None else min(best, d)
        return best

    def _check_contacts(self, plan_now):
        """Forensics: log full context on the first contact between any pair."""
        
        # Loop through all pairs of robots
        for a in self.robots:
            if a.pose is None:
                continue
            for b in self.robots:
                if b.pose is None or a.id == b.id:
                    continue
                
                # Compute distance between the two robots
                ax, ay, _ = a.pose
                bx, by, _ = b.pose
                d = math.hypot(ax - bx, ay - by)
                
                # Log contact event
                # FIXME: this only logs the first robots come into contact.
                # If they separate and collide again, it won't log.
                if d < COLLIDE_DIST and (a.id, b.id) not in self._contacts_logged:
                    self._contacts_logged.add((a.id, b.id))
                    self.get_logger().error(
                        f"CONTACT robots {a.id}&{b.id} d={d:.2f} m at plan_t={plan_now:.2f} "
                        f"(replan #{self.replans})\n"
                        f"  r{a.id}: at {self.map.world_to_cell(ax, ay)}, "
                        f"sched={fmt_sched(a.sched_cells)}\n"
                        f"  r{b.id}: at {self.map.world_to_cell(bx, by)}, "
                        f"sched={fmt_sched(b.sched_cells)}")

    def _pre_rotate(self, robot: Robot, wx: float, wy: float, yaw: float, cmd: Twist):
        """During a planned rotation/wait step, pre-align toward the next new cell."""
        for wp in robot.waypoints[robot.progress + 1:]:
            if math.hypot(wp[0] - wx, wp[1] - wy) > ARRIVE_TOL:
                hd = wrap(math.atan2(wp[1] - wy, wp[0] - wx) - yaw)
                if abs(hd) > 0.08:
                    cmd.angular.z = max(-MAX_ANG, min(MAX_ANG, K_ANG * hd))
                return
            
    def arrival_times(self):
        """Retrieve all robot arrival times (of those robots that reached goal)"""
        arv_times = []
        for r in self.robots:
            if r.frst_arrv_t is not None:
                arv_times.append(r.frst_arrv_t)
        return arv_times
        

    def finish(self, reason="all robots at goal"):
        # Prevent tick() function from being called
        self.timer.cancel()
        
        metrics = {}
        arrived: list[tuple[int, tuple[int, int]]] = []
        stragglers: list[tuple[int, tuple[int, int]]] = []
        for robot in self.robots:
            # Publish empty Twist to stop all robots
            robot.pub.publish(Twist())
        
            if robot.pose and robot.goal:
                cell = robot.cell()
                (arrived if cell == robot.goal else stragglers).append((robot.id, cell))
            else:
                stragglers.append((robot.id, None))
        
        # Compute completion time metrics
        elapsed = (time.monotonic() - self.t_start) if self.t_start else 0.0
        mm, ss = divmod(elapsed, 60)
        mean_lag = (sum(self.lag_samples) / len(self.lag_samples)) if self.lag_samples else 0.0
        peak_lag = max(self.lag_samples) if self.lag_samples else 0
        
        metrics["K Robust Constant"] = self.k
        metrics["Safeguards Enabled"] = self.safeguards
        metrics["Outcome"] = reason
        metrics["# Robots at Goal"] = f"{len(arrived)}/{len(self.robots)}"
        metrics["Success Rate (%)"] = len(arrived) / len(self.robots) * 100.0
        metrics["Completion Time"] = f"{int(mm)}m {ss:04.1f}s  ({elapsed:.1f} s)"
        metrics["First Arrival Time (s)"] = min(self.arrival_times())
        metrics["Last Arrival Time (s)"] = max(self.arrival_times())
        metrics["Num Replans"] = self.replans
        metrics["Num Contacts"] = len(self._contacts_logged)
        metrics["Mean Tracking Lag (steps)"] = mean_lag
        metrics["Peak Tracking Lag (steps)"] = peak_lag
        metrics["Average Planning Time"] = np.mean(self.planning_times)
        
        if self.hundred_cycle_success:
            metrics["Success (%) at 100 Cycles"] = self.hundred_cycle_success
        
        if stragglers:
            stragglers_txt = []
            for r, c in stragglers:
                rob = self.robots[r]
                stragglers_txt.append(f"r{r} at {c} (goal {rob.goal}, starved {rob.starved}w))")
                                
            metrics["Stragglers"] = ", ".join(stragglers_txt)

        self.get_logger().info("==================== RUN COMPLETE ====================")
        for metric, value in metrics.items():
            trailing_spaces = (30 - len(metric)) * " "
            self.get_logger().info(f"  {metric}{trailing_spaces}: {value}")
        self.get_logger().info("======================================================")
        

def main():
    rclpy.init()
    node = WHCAController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node.finish(reason="interrupted by user")
        except Exception:
            pass
    finally:
        try:
            for robot in node.robots:
                robot.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
