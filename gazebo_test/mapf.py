"""MAPF / WHCA* controller node — online windowed loop.

True WHCA* execution: instead of precomputing the whole trajectory, the
controller runs the windowed loop in real time. Every cycle it:

  1. reads each robot's CURRENT cell from ground-truth pose,
  2. reads the CURRENT grid  (<-- newly detected obstacles enter here),
  3. plans one window of depth W with plan_window() (reservation table rebuilt
     fresh each window, exactly as Silver's WHCA* prescribes),
  4. commits and executes only the first W/2 steps, tick-synced so the
     space-time deconfliction holds on the wall clock,
  5. replans from wherever the robots actually ended up.


Grid convention matches whca_functions exactly: numpy array indexed grid[x, y],
value 1 = obstacle, positions are integer (x, y) tuples.
"""

import math
from time import perf_counter

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray

from gazebo_test.whca_functions import plan_window, RRAstar


def yaw_from_quat(q):
    """Yaw (rad) from a geometry_msgs Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MAPFNode(Node):
    def __init__(self):
        super().__init__('mapf_whca_node')

        # ---- Parameters -------------------------------------------------
        self.declare_parameter('num_robots', 30)
        self.declare_parameter('grid_res', 3.0)        # metres per cell
        self.declare_parameter('origin_x', -30.0)      # world x of grid cell 0
        self.declare_parameter('origin_y', -30.0)
        self.declare_parameter('grid_w', 20)           # cells in x
        self.declare_parameter('grid_h', 20)           # cells in y
        self.declare_parameter('window_size', 24)      # W (window depth)
        self.declare_parameter('max_turns', 200)       # safety cap on total steps
        self.declare_parameter('goal_dx_cells', 14)    # goal = start +N cells in x
        self.declare_parameter('priority', 'closest')  # 'closest' = nearest-goal
                                                       # first (avoids target
                                                       # conflicts); 'static' =
                                                       # agent index order
        self.declare_parameter('control_rate', 20.0)   # Hz
        self.declare_parameter('pos_tol', 0.6)         # m to count a waypoint hit
        self.declare_parameter('max_lin', 0.8)         # m/s
        self.declare_parameter('max_ang', 1.2)         # rad/s
        self.declare_parameter('k_lin', 0.6)
        self.declare_parameter('k_ang', 2.0)
        self.declare_parameter('align_thresh', 0.6)    # rad; turn-in-place gate
        self.declare_parameter('tick_timeout', 15.0)   # s; force-advance fallback

        g = self.get_parameter
        self.n = g('num_robots').value
        self.res = g('grid_res').value
        self.ox = g('origin_x').value
        self.oy = g('origin_y').value
        self.gw = g('grid_w').value
        self.gh = g('grid_h').value
        self.window_size = g('window_size').value
        self.step_size = max(1, self.window_size // 2)   # W/2 commit
        self.max_turns = g('max_turns').value
        self.goal_dx = g('goal_dx_cells').value
        self.priority = g('priority').value
        self.pos_tol = g('pos_tol').value
        self.max_lin = g('max_lin').value
        self.max_ang = g('max_ang').value
        self.k_lin = g('k_lin').value
        self.k_ang = g('k_ang').value
        self.align_thresh = g('align_thresh').value
        self.tick_timeout = g('tick_timeout').value
        ctrl_rate = g('control_rate').value

        self.names = [f'robot{i}' for i in range(self.n)]

        # ---- Static obstacle grid --------------------------------------
        # Obstacles as world AABBs (xmin, xmax, ymin, ymax). One wall at x=5.
        self.obstacle_aabbs = [(4.75, 5.25, -5.0, 5.0)]
        self.grid = self._build_grid()
        self.grid_dirty = False     # set True when mark_obstacle() changes the map

        # ---- Pose plumbing ---------------------------------------------
        self.poses = {name: None for name in self.names}   # (x, y, yaw)
        self.cmd_pubs = {}
        for name in self.names:
            self.cmd_pubs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel', 10)
            self.create_subscription(
                PoseArray, f'/{name}/pose',
                lambda msg, n=name: self._pose_cb(n, msg), 10)

        # ---- Run state --------------------------------------------------
        self.running = False
        self.done = False
        self.goals = None
        self.arrived = [False] * self.n
        self.rra_stars = None

        # per-cycle execution state
        self.commit_world = {}       # name -> padded list of (wx, wy) this window
        self.commit_len = 0
        self.cycle_tick = 0
        self.cycle_start = self.get_clock().now()

        # metrics
        self.window_offset = 0       # total committed steps (terminate cap)
        self.executed_ticks = 0      # realised makespan in ticks
        self.replan_count = 0
        self.plan_time_total = 0.0

        self.create_timer(1.0, self._try_start)
        self.create_timer(1.0 / ctrl_rate, self._control_step)

        self.get_logger().info(
            f'MAPF node up (online WHCA*, W={self.window_size}, commit={self.step_size}). '
            f'Waiting for poses from {self.n} robots...')

    # ---------------------------------------------------------------- grid
    def _build_grid(self):
        grid = np.zeros((self.gw, self.gh), dtype=int)
        for cx in range(self.gw):
            for cy in range(self.gh):
                x0 = self.ox + cx * self.res
                x1 = x0 + self.res
                y0 = self.oy + cy * self.res
                y1 = y0 + self.res
                for (axmin, axmax, aymin, aymax) in self.obstacle_aabbs:
                    if x0 < axmax and x1 > axmin and y0 < aymax and y1 > aymin:
                        grid[cx, cy] = 1
                        break
        return grid

    def world_to_cell(self, wx, wy):
        cx = min(max(int((wx - self.ox) / self.res), 0), self.gw - 1)
        cy = min(max(int((wy - self.oy) / self.res), 0), self.gh - 1)
        return (cx, cy)

    def cell_to_world(self, cx, cy):
        return (self.ox + (cx + 0.5) * self.res,
                self.oy + (cy + 0.5) * self.res)

    def mark_obstacle(self, cx, cy):
        """for the obstacle-injection phase. Flips a cell to
        occupied; the next window replans against it. Forces an RRA* rebuild
        because the cached heuristic distances assumed the old map."""
        if 0 <= cx < self.gw and 0 <= cy < self.gh and self.grid[cx, cy] == 0:
            self.grid[cx, cy] = 1
            self.grid_dirty = True
            self.get_logger().info(f'Obstacle marked at cell ({cx},{cy}).')

    # ---------------------------------------------------------------- poses
    def _pose_cb(self, name, msg: PoseArray):
        if not msg.poses:
            return
        p = msg.poses[0]   # PosePublisher (model-pose only) -> single entry
        self.poses[name] = (p.position.x, p.position.y,
                            yaw_from_quat(p.orientation))

    def _current_cells(self):
        return [self.world_to_cell(self.poses[n][0], self.poses[n][1])
                for n in self.names]

    # ------------------------------------------------------------- startup
    def _try_start(self):
        if self.running:
            return
        if any(self.poses[name] is None for name in self.names):
            have = sum(self.poses[n] is not None for n in self.names)
            self.get_logger().info(f'Poses received: {have}/{self.n}')
            return

        # Assign goals once from initial cells: translate +goal_dx in x.
        starts = self._current_cells()
        self.goals = []
        used = set()
        for i, name in enumerate(self.names):
            sc = starts[i]
            if self.grid[sc[0], sc[1]] == 1:
                self.get_logger().warn(f'{name} start cell {sc} is an obstacle.')
            gx = min(sc[0] + self.goal_dx, self.gw - 1)
            gy = sc[1]
            if self.grid[gx, gy] == 1:
                gy = min(gy + 1, self.gh - 1)
            if (gx, gy) in used:
                self.get_logger().warn(f'{name} goal ({gx},{gy}) duplicated.')
            used.add((gx, gy))
            self.goals.append((gx, gy))

        self.get_logger().info(
            f'Starting online WHCA*: {self.n} agents, grid={self.gw}x{self.gh}'
            f'@{self.res}m, W={self.window_size}.')
        self.running = True
        self._plan_cycle()

    # ---------------------------------------------------------- plan window
    def _plan_cycle(self):
        """Plan ONE window from current cells and commit the first W/2 steps."""
        t0 = perf_counter()
        current = self._current_cells()
        self.arrived = [current[i] == self.goals[i] for i in range(self.n)]

        # (Re)build per-agent RRA* heuristics; rebuild if the map changed.
        if self.rra_stars is None or self.grid_dirty:
            self.rra_stars = [
                RRAstar(gx, gy, self.grid) if not self.arrived[i] else None
                for i, (gx, gy) in enumerate(self.goals)]
            self.grid_dirty = False

        # Priority order. 'closest' (nearest-goal-first) 
        if self.priority == 'closest':
            order = sorted(
                range(self.n),
                key=lambda i: (self.rra_stars[i].get_h(*current[i])
                               if self.rra_stars[i] is not None else -1))
        else:
            order = list(range(self.n))
        paths_ordered = plan_window(
            [current[i] for i in order],
            [self.goals[i] for i in order],
            self.grid, self.window_size,
            [self.arrived[i] for i in order],
            [self.rra_stars[i] for i in order])

        paths = [None] * self.n
        for rank, orig in enumerate(order):
            paths[orig] = paths_ordered[rank]

        # Commit the first W/2 moves (window-local t in 1..step_size).
        committed = [[(s.x, s.y) for s in paths[i] if 1 <= s.t <= self.step_size]
                     for i in range(self.n)]
        commit_len = max(1, max(len(c) for c in committed))

        self.commit_world = {}
        for i, name in enumerate(self.names):
            cells = committed[i]
            if cells:
                world = [self.cell_to_world(cx, cy) for (cx, cy) in cells]
                world += [world[-1]] * (commit_len - len(world))   # pad = wait
            else:
                w = self.cell_to_world(*current[i])
                world = [w] * commit_len                            # hold in place
            self.commit_world[name] = world

        self.commit_len = commit_len
        self.cycle_tick = 0
        self.cycle_start = self.get_clock().now()
        self.window_offset += self.step_size
        self.replan_count += 1
        self.plan_time_total += perf_counter() - t0

    def _end_cycle(self):
        self.executed_ticks += self.commit_len
        current = self._current_cells()
        self.arrived = [current[i] == self.goals[i] for i in range(self.n)]
        if all(self.arrived):
            self._finish(reached=True)
        elif self.window_offset >= self.max_turns:
            self.get_logger().warn(
                f'Hit max_turns={self.max_turns} with '
                f'{self.arrived.count(False)} agent(s) short of goal.')
            self._finish(reached=False)
        else:
            self._plan_cycle()

    # ------------------------------------------------------------- control
    def _control_step(self):
        if not self.running or self.done:
            return

        all_arrived = True
        idx = min(self.cycle_tick, self.commit_len - 1)
        for name in self.names:
            tx, ty = self.commit_world[name][idx]
            px, py, yaw = self.poses[name]
            dx, dy = tx - px, ty - py
            dist = math.hypot(dx, dy)

            cmd = Twist()
            if dist < self.pos_tol:
                self.cmd_pubs[name].publish(cmd)     # at target -> hold
                continue

            all_arrived = False
            heading_err = wrap_angle(math.atan2(dy, dx) - yaw)
            cmd.angular.z = max(-self.max_ang, min(self.max_ang,
                                                   self.k_ang * heading_err))
            if abs(heading_err) < self.align_thresh:
                cmd.linear.x = max(-self.max_lin, min(self.max_lin,
                                                      self.k_lin * dist))
            self.cmd_pubs[name].publish(cmd)

        elapsed = (self.get_clock().now() - self.cycle_start).nanoseconds * 1e-9
        if all_arrived or elapsed > self.tick_timeout:
            if elapsed > self.tick_timeout and not all_arrived:
                self.get_logger().warn(
                    f'Step {self.cycle_tick} timed out; force-advancing.')
            self.cycle_tick += 1
            self.cycle_start = self.get_clock().now()
            if self.cycle_tick >= self.commit_len:
                self._end_cycle()

    def _finish(self, reached):
        self.done = True
        stop = Twist()
        for name in self.names:
            self.cmd_pubs[name].publish(stop)
        unreached = self.arrived.count(False)
        self.get_logger().info(
            f'Run complete. reached_all={reached} unreached={unreached}/{self.n} '
            f'| replans={self.replan_count} makespan={self.executed_ticks} ticks '
            f'| total_plan_time={self.plan_time_total:.3f}s '
            f'avg_window={self.plan_time_total / max(1, self.replan_count):.4f}s')


def main():
    rclpy.init()
    node = MAPFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
