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
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "WHCABaseline"))
from whca_functions import plan_window, RRAstar  # noqa: E402

# ================================ CONFIG =====================================
# Map lives in the repo (isaacsim_files/), resolved relative to this file so it
# works on any machine. Override with the WHCA_MAP env var if yours is elsewhere.
MAP_YAML = os.environ.get("WHCA_MAP",
    os.path.join(HERE, "isaacsim_files", "IssacWarehouseSmallOccupancyMap.yaml"))
PLANNING_CELL = 1.0        # m per planning cell; must exceed the robot footprint

ROBOTS = list(range(30))
GOALS_WORLD = {            # robot id -> goal 
     0: ( -32.175,   28.775),   # cell (4, 59)
     1: ( -31.175,   28.775),   # cell (5, 59)
     2: ( -30.175,   28.775),   # cell (6, 59)
     3: ( -29.175,   28.775),   # cell (7, 59)
     4: ( -28.175,   28.775),   # cell (8, 59)
     5: ( -27.175,   28.775),   # cell (9, 59)
     6: ( -26.175,   28.775),   # cell (10, 59)
     7: ( -25.175,   28.775),   # cell (11, 59)
     8: ( -24.175,   28.775),   # cell (12, 59)
     9: ( -23.175,   28.775),   # cell (13, 59)
    10: ( -22.175,   28.775),   # cell (14, 59)
    11: ( -21.175,   28.775),   # cell (15, 59)
    12: ( -20.175,   28.775),   # cell (16, 59)
    13: ( -19.175,   28.775),   # cell (17, 59)
    14: ( -18.175,   28.775),   # cell (18, 59)
    15: ( -29.175,  -27.225),   # cell (7, 3)
    16: ( -28.175,  -27.225),   # cell (8, 3)
    17: ( -27.175,  -27.225),   # cell (9, 3)
    18: ( -26.175,  -27.225),   # cell (10, 3)
    19: ( -25.175,  -27.225),   # cell (11, 3)
    20: ( -24.175,  -27.225),   # cell (12, 3)
    21: ( -23.175,  -27.225),   # cell (13, 3)
    22: ( -22.175,  -27.225),   # cell (14, 3)
    23: ( -21.175,  -27.225),   # cell (15, 3)
    24: ( -20.175,  -27.225),   # cell (16, 3)
    25: ( -19.175,  -27.225),   # cell (17, 3)
    26: ( -18.175,  -27.225),   # cell (18, 3)
    27: ( -17.175,  -27.225),   # cell (19, 3)
    28: ( -16.175,  -27.225),   # cell (20, 3)
    29: ( -29.175,  -28.225),   # cell (7, 2)
}

WINDOW_SIZE = 32            # WHCA window W; commit/re-plan every W//2 steps
STEP_SECONDS = 1.9         # wall-clock length of one plan timestep (one cell
                           # traverse OR one 90-degree rotation)
LAG_REPLAN = 1.5           # re-plan early if any robot falls this many steps behind
DEADLOCK_CYCLES = 12       # stop if no robot has moved for this many windows
STALL_TIMEOUT = 300.0      # s: hard cap on a run with no execution progress at all

BASE_FRAME = "base_link"   # /tf: frame_id "world" -> this child frame = robot pose

# Drive controller
CONTROL_HZ = 20.0
ARRIVE_TOL = 0.18          # m: waypoint reached
ALIGN_TOL = 0.30           # rad: rotate in place until heading error below this
K_LIN, K_ANG = 1.2, 2.0
MAX_LIN, MAX_ANG = 0.6, 1.5

# Execution safeguards
SAFEGUARDS = False           # master switch for the execution-layer safeguards below.
                            # True  = vacancy gate + headway control (our method).
                            # False = pure WHCA* execution, no safeguards (baseline for
                            #         comparison vs k-robust / ADG). Collisions may occur
                            #         when off -- that is the purpose of the baseline.
CLEAR_RADIUS = 0.635        # cell counts occupied while any robot centre is within this
                           #  CELL of its centre. 0.635; headway covers the final approach.
HEADWAY = 0.8             # m: taper speed to zero behind a robot ahead
COLLIDE_DIST = 0.62        # m: contact event -> forensic log
INFLATE_M = 0.30           # m: obstacle inflation. Dingo radius is 0.389 m
K_ROBUST = 0                # 0 = standard WHCA* (Silver 2005). >=1 = k-robust WHCA* (Atzmon et al. 2018)
# =============================================================================


# ------------------------------ map loading ----------------------------------
def load_map(yaml_path, planning_cell):
    """Load a ROS map (.yaml + image) -> (grid[x, y] 1=blocked, origin, cell)."""
    cfg = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.split("#")[0].strip()
            if ":" in line:
                k, v = (p.strip() for p in line.split(":", 1))
                cfg[k] = ([float(x) for x in v.strip("[]").split(",")]
                          if v.startswith("[") else v)
    from PIL import Image, ImageFilter
    img_path = os.path.join(os.path.dirname(yaml_path), cfg["image"])
    arr = np.array(Image.open(img_path).convert("L"), dtype=np.float32)

    occ = arr / 255.0 if int(float(cfg.get("negate", 0))) else (255.0 - arr) / 255.0
    occupied = occ > float(cfg.get("occupied_thresh", 0.65))
    res = float(cfg["resolution"])

    # Inflate obstacles by the robot's radius before downsampling,

    kern = int(round(INFLATE_M / res)) * 2 + 1
    if kern > 1:
        occupied = np.array(
            Image.fromarray((occupied * 255).astype(np.uint8))
                 .filter(ImageFilter.MaxFilter(kern))
        ) > 127

    grid = np.flipud(occupied).T                      # image (row 0 = top) -> [x, y]
    f = max(1, int(round(planning_cell / res)))
    if f > 1:                                         # downsample: blocked if ANY sub-cell is
        w, h = (grid.shape[0] // f) * f, (grid.shape[1] // f) * f
        grid = grid[:w, :h].reshape(w // f, f, h // f, f).any(axis=(1, 3))
    return grid.astype(int), float(cfg["origin"][0]), float(cfg["origin"][1]), f * res


GRID, ORIGIN_X, ORIGIN_Y, CELL = load_map(MAP_YAML, PLANNING_CELL)
DIMX, DIMY = GRID.shape


# ------------------------------ small helpers ---------------------------------
def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_heading(yaw):
    """Snap yaw (rad, 0 = +x) to grid heading 0=E, 1=N, 2=W, 3=S."""
    return int(round(yaw / (math.pi / 2))) % 4


def world_to_cell(wx, wy):
    cx = int((wx - ORIGIN_X) // CELL)
    cy = int((wy - ORIGIN_Y) // CELL)
    return (min(max(cx, 0), DIMX - 1), min(max(cy, 0), DIMY - 1))

def cell_to_world(cx, cy):
    return (ORIGIN_X + (cx + 0.5) * CELL, ORIGIN_Y + (cy + 0.5) * CELL)


def nearest_free(cx, cy, taken=frozenset()):
    """BFS to the closest free cell not in `taken` (returns input if none found)."""
    q, seen = deque([(cx, cy)]), {(cx, cy)}
    while q:
        x, y = q.popleft()
        if GRID[x, y] == 0 and (x, y) not in taken:
            return (x, y)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            n = (x + dx, y + dy)
            if 0 <= n[0] < DIMX and 0 <= n[1] < DIMY and n not in seen:
                seen.add(n)
                q.append(n)
    return (cx, cy)


def fmt_sched(cells):
    """Compact schedule string: only the timesteps where the cell changes."""
    out, last = [], None
    for t, c in enumerate(cells):
        if c != last:
            out.append(f"t{t}:{c}")
            last = c
    return " ".join(out)


# ------------------------------ controller ------------------------------------
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
        self.total_advances = 0
        self._advances_at_last_plan = 0
        self.sched_cells = {}                # rid -> committed cells (forensics)
        self.history = {}                    # rid -> last K_ROBUST occupied cells
        self.plan_stats = {}                 # planner diagnostics
        self.window_t0 = 0.0
        self.replans = 0
        self.stuck_windows = 0
        self.done = False
        self._contacts_logged = set()
        self.last_exec_t = None
        self.arrived_at = {}                 # rid -> seconds from start when it first
                                             # reached its goal cell
        self.t_start = None                  # wall clock at first committed window
        self.starved = {}                    # rid -> consecutive windows with no progress
        self.lag_samples = []                # max_lag per tick, for the run summary
        self._gate_log_t = 0.0

        self.create_timer(1.0 / CONTROL_HZ, self._tick)
        self.get_logger().info(
            f"Map {os.path.basename(MAP_YAML)}: {DIMX}x{DIMY} @ {CELL:.2f} m, "
            f"{int(GRID.sum())} blocked. Robots {self.robot_ids}, "
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
        return nearest_free(*world_to_cell(wx, wy))

    # ---------------- planning ----------------
    def _setup_goals(self):
        """One-time: fix each robot's goal cell (deduplicated) and its RRA*."""
        goals, taken = {}, set()
        for rid in self.robot_ids:
            g = nearest_free(*world_to_cell(*GOALS_WORLD[rid]), taken=taken)
            goals[rid] = g
            taken.add(g)
        self.goals = goals
        self.rra = {rid: RRAstar(goals[rid][0], goals[rid][1], GRID)
                    for rid in self.robot_ids}
        self.get_logger().info(f"Goals: {sorted(goals.items())}")

    def _distinct_starts(self):
        """Current cells, made unique so the planner never gets two agents in one cell."""
        starts, taken = [], set()
        for rid in self.robot_ids:
            c = self._cell(rid)
            if c in taken:
                c = nearest_free(*c, taken=taken)
            starts.append(c)
            taken.add(c)
        return starts

    def _plan(self):
        """Plan one window and commit the first W//2 steps as timed waypoints."""
        if any(rid not in self.pose for rid in self.robot_ids):
            return False                       # still waiting for /tf
        if self.goals is None:
            self._setup_goals()
        
         # Snapshot each robot's last K_ROBUST executed cells before the schedule
        if self.sched_cells:
            for rid in self.robot_ids:
                advanced = self.progress.get(rid, 0) > 0
                self.starved[rid] = 0 if advanced else self.starved.get(rid, 0) + 1
        if K_ROBUST and self.sched_cells:
            for rid in self.robot_ids:
                cells = self.sched_cells.get(rid)
                p = self.progress.get(rid, 0)
                if cells:
                    self.history[rid] = [cells[p - j]
                                         for j in range(1, K_ROBUST + 1) if p - j >= 0]
        starts = self._distinct_starts()
        at_goal = [s == self.goals[rid] for s, rid in zip(starts, self.robot_ids)]
        if all(at_goal):
            self._finish()
            return True

        # Silver-faithful planning:
        #  - EVERY agent plans every window, including agents at their goals
        #    (they plan zero-cost waits, but will step aside if a higher-priority
        #    agent needs their cell — no permanent parking).
        #  - Priority ROTATES each window (Silver 2005) so no fixed right-of-way
        #    pattern can repeat forever; this breaks symmetric deadlocks.
        n = len(self.robot_ids)
        # Silver (2005): priorities are assigned randomly, re-drawn each planning
        # window, so no fixed right-of-way pattern can persist.
        order = random.sample(self.robot_ids, n)
        order.sort(key=lambda r: -self.starved.get(r, 0))
        idx = {rid: self.robot_ids.index(rid) for rid in order}
        o_starts = [starts[idx[r]] for r in order]
        o_goals = [self.goals[r] for r in order]
        o_rra = [self.rra[r] for r in order]
        o_head = [yaw_to_heading(self.pose[r][2]) for r in order]
        o_hist = [self.history.get(r, []) for r in order] if K_ROBUST else None

        t0 = time.perf_counter()
        o_paths = plan_window(o_starts, o_goals, GRID, WINDOW_SIZE,
                              [False] * n, o_rra, start_headings=o_head,
                              commit_horizon=self.step_size,
                              k=K_ROBUST, history=o_hist)
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
            self.waypoints[rid] = [cell_to_world(*c) for c in cells]
        self.progress = {rid: 0 for rid in self.robot_ids}
        self.window_t0 = time.monotonic()
        if self.last_exec_t is None:
            self.last_exec_t = self.window_t0
        if self.t_start is None:
            self.t_start = self.window_t0
        self.planning = False

        executed = self.total_advances > self._advances_at_last_plan
        self._advances_at_last_plan = self.total_advances
        self.stuck_windows = 0 if executed else self.stuck_windows + 1
        if self.stuck_windows >= DEADLOCK_CYCLES:
            self.get_logger().warn(
                f"No waypoint reached by any robot for {self.stuck_windows} windows. "
                f"Still running - the run ends on its own at goal, or after "
                f"{STALL_TIMEOUT:.0f} s without progress.")

        blocked = [r for r in self.robot_ids if self.starved.get(r, 0) >= 3
                   and starts[self.robot_ids.index(r)] != self.goals[r]]
        if blocked:
            self.get_logger().warn(
                f"starved (no execution for >=3 windows): "
                + ", ".join(f"r{r}x{self.starved[r]}@{starts[self.robot_ids.index(r)]}"
                            for r in blocked))
        self.get_logger().info(
            f"[replan {self.replans}] {dt_ms:.1f} ms | commit {self.step_size} steps"
            f" | k={K_ROBUST}"
            f" | at goal {sum(at_goal)}/{len(at_goal)} | prio {order}")
        return True

    # ---------------- execution safeguards ----------------
    def _cell_occupant(self, rid, tx, ty):
        """Another robot still physically inside the cell centred (tx, ty), or None."""
        for other in self.robot_ids:
            if other != rid and other in self.pose:
                ox, oy, _ = self.pose[other]
                if math.hypot(ox - tx, oy - ty) < CLEAR_RADIUS * CELL:
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
                        f"  r{a}: at {world_to_cell(ax, ay)}, "
                        f"sched={fmt_sched(self.sched_cells[a])}\n"
                        f"  r{b}: at {world_to_cell(bx, by)}, "
                        f"sched={fmt_sched(self.sched_cells[b])}")

    # ---------------- control loop ----------------
    def _tick(self):
        if self.done:
            return
        if self.planning:
            self._plan()
            return

         # Completion is checked here, every tick.
        if self.goals and self.replans > 0:
            now = time.monotonic()
            for rid in self.robot_ids:
                if rid not in self.pose:
                    continue
                at = world_to_cell(self.pose[rid][0], self.pose[rid][1]) == self.goals[rid]
                if rid in self.arrived_at:
                    if not at:
                        del self.arrived_at[rid]      # stepped aside; count it again
                    continue
                if at:
                    self.arrived_at[rid] = now - (self.t_start or now)
                    self.get_logger().info(
                        f"robot{rid} at goal {self.goals[rid]} "
                        f"({self.arrived_at[rid]:.1f} s) "
                        f"[{len(self.arrived_at)}/{len(self.robot_ids)}]")
            if len(self.arrived_at) == len(self.robot_ids):
                self._finish()
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
                    self.total_advances += 1
                    self.last_exec_t = time.monotonic()
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
                        f"{world_to_cell(tx, ty)}")
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

        self.lag_samples.append(max_lag)

        
        now = time.monotonic()
        if self.last_exec_t is not None and now - self.last_exec_t > STALL_TIMEOUT:
            self.get_logger().error(
                f"No robot has reached a waypoint in {STALL_TIMEOUT:.0f} s - stopping.")
            self._finish(reason="stalled")
            return

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

    def _finish(self, reason="all robots at goal"):
        if self.done:
            return
        self.done = True
        for rid in self.robot_ids:
            self.pubs[rid].publish(Twist())

        arrived, stragglers = [], []
        for rid in self.robot_ids:
            if rid in self.pose and self.goals:
                cell = world_to_cell(self.pose[rid][0], self.pose[rid][1])
                (arrived if cell == self.goals[rid] else stragglers).append((rid, cell))
            else:
                stragglers.append((rid, None))

        elapsed = (time.monotonic() - self.t_start) if self.t_start else 0.0
        mm, ss = divmod(elapsed, 60)
        mean_lag = (sum(self.lag_samples) / len(self.lag_samples)) if self.lag_samples else 0.0
        peak_lag = max(self.lag_samples) if self.lag_samples else 0

        self.get_logger().info(
            "\n"
            "==================== RUN COMPLETE ====================\n"
            f"  K_ROBUST        : {K_ROBUST}\n"
            f"  SAFEGUARDS      : {SAFEGUARDS}\n"
            f"  outcome         : {reason}\n"
            f"  robots at goal  : {len(arrived)}/{len(self.robot_ids)}\n"
            f"  completion time : {int(mm)}m {ss:04.1f}s  ({elapsed:.1f} s)\n"
            f"  re-plans        : {self.replans}\n"
            f"  contacts        : {len(self._contacts_logged)}\n"
            f"  tracking lag    : mean {mean_lag:.2f} steps, peak {peak_lag} steps\n"
            + (f"  first / last    : {min(self.arrived_at.values()):.1f} s / "
               f"{max(self.arrived_at.values()):.1f} s\n" if self.arrived_at else "")
            + ("" if not stragglers else
               "  NOT AT GOAL     : " + ", ".join(
                   f"r{r} at {c} (goal {self.goals[r] if self.goals else '?'}"
                   f", starved {self.starved.get(r, 0)}w)" for r, c in stragglers) + "\n")
            + "======================================================")

def main():
    rclpy.init()
    node = WHCAController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        try:
            node._finish(reason="interrupted by user")
        except Exception:
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
