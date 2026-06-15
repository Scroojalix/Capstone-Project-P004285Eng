#!/usr/bin/env python3
"""WHCA* Experiment Node — Silver 2005 replication with RViz visualization."""

import csv
import os
import random
import threading
import time
import heapq
from collections import deque
from dataclasses import dataclass
from itertools import groupby

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


# ── Algorithm ─────────────────────────────────────────────────────────────────

@dataclass(frozen=True)
class State:
    x: int
    y: int
    t: int

class ReservationTable:
    """Stores time-extended vertex and edge reservations during windowed planning."""

    def __init__(self):
        self.vertex_reservations = set()
        self.edge_reservations = set()

    def reserve_vertex(self, x, y, t):
        self.vertex_reservations.add((x, y, t))

    def reserve_edge(self, x1, y1, x2, y2, t):
        self.edge_reservations.add((x1, y1, x2, y2, t))

    def is_vertex_reserved(self, x, y, t):
        return (x, y, t) in self.vertex_reservations

    def is_edge_reserved(self, x1, y1, x2, y2, t):
        return (x1, y1, x2, y2, t) in self.edge_reservations

MOVES = [(0, 0), (0, 1), (0, -1), (-1, 0), (1, 0)]


def manhattan_distance(x, y, gx, gy):
    """Return Manhattan distance from (x,y) to goal (gx,gy)."""
    return abs(x - gx) + abs(y - gy)


class RRAstar:
    """
    Reverse Resumable A* heuristic — Silver (2005), Section 3.

    Runs backward Dijkstra from the agent's goal through the static obstacle
    map. When queried for h(x, y), the search resumes until (x, y) is expanded
    and returns the true shortest-path distance, ignoring all other agents.

    One instance per agent per planning window.
    """

    def __init__(self, goal_x: int, goal_y: int, grid: np.ndarray) -> None:
        self.dimx, self.dimy = grid.shape
        self.grid = grid
        self._distances: dict = {}   # closed: (x, y) -> true dist to goal
        self._in_open: dict = {}     # (x, y) -> best g seen in open set
        self._counter: int = 0
        self._open: list = []        # heap: (g, counter, x, y)

        # Seed: the goal itself is distance 0
        heapq.heappush(self._open, (0, 0, goal_x, goal_y))
        self._in_open[(goal_x, goal_y)] = 0

    def get_h(self, x: int, y: int) -> int:
        """
        Return true shortest-path distance from (x, y) to goal.
        Resumes the backward search if (x, y) hasn't been expanded yet.
        Returns 10,000 for unreachable cells.
        """
        if (x, y) in self._distances:
            return self._distances[(x, y)]

        while self._open:
            g, _, px, py = heapq.heappop(self._open)

            if (px, py) in self._distances:
                continue                        # stale entry, skip

            self._distances[(px, py)] = g       # close this node

            if (px, py) == (x, y):
                return g                        # found it

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                nx, ny = px + dx, py + dy
                if not (0 <= nx < self.dimx and 0 <= ny < self.dimy):
                    continue
                if self.grid[nx, ny] == 1:      # static obstacle
                    continue
                if (nx, ny) in self._distances: # already closed
                    continue
                ng = g + 1
                if ng < self._in_open.get((nx, ny), 10**9):
                    self._in_open[(nx, ny)] = ng
                    self._counter += 1
                    heapq.heappush(self._open, (ng, self._counter, nx, ny))

        return 10_000   # unreachable


def reconstruct_path(came_from, current_state, start_state):
    """Reconstruct an A* path from the came_from dictionary."""
    path = [current_state]
    while current_state in came_from:
        current_state = came_from[current_state]
        path.append(current_state)
    path.reverse()
    return path


def windowed_a_star_search(start_state, goal_x, goal_y, window_size, grid, reservation_table, rra_star=None):
    """Perform A* search for a single agent inside the WHCA time window."""
    width, height = grid.shape
    heap_counter = 0
    open_heap = []
    h0 = rra_star.get_h(start_state.x, start_state.y) if rra_star else manhattan_distance(start_state.x, start_state.y, goal_x, goal_y)
    heapq.heappush(open_heap, (start_state.t + h0, heap_counter, start_state))
    came_from = {}
    g_scores = {start_state: 0}

    while open_heap:
        _, _, current_state = heapq.heappop(open_heap)

        if current_state.x == goal_x and current_state.y == goal_y:
            return reconstruct_path(came_from, current_state, start_state)
        if current_state.t >= window_size:
            return reconstruct_path(came_from, current_state, start_state)

        for dx, dy in MOVES:
            next_x = current_state.x + dx
            next_y = current_state.y + dy
            next_t = current_state.t + 1

            if not (0 <= next_x < width and 0 <= next_y < height):
                continue
            if grid[next_x, next_y] == 1:
                continue
            if reservation_table.is_vertex_reserved(next_x, next_y, next_t):
                continue
            if reservation_table.is_edge_reserved(next_x, next_y, current_state.x, current_state.y, current_state.t):
                continue

            neighbor_state = State(next_x, next_y, next_t)
            at_goal_wait = (current_state.x == goal_x and current_state.y == goal_y
                and dx == 0 and dy == 0
                and current_state.t < window_size)
            move_cost = 0 if at_goal_wait else 1
            new_g = g_scores[current_state] + move_cost

            if new_g < g_scores.get(neighbor_state, float("inf")):
                came_from[neighbor_state] = current_state
                g_scores[neighbor_state] = new_g
                heap_counter += 1
                h = rra_star.get_h(next_x, next_y) if rra_star else manhattan_distance(next_x, next_y, goal_x, goal_y)
                heapq.heappush(open_heap, (new_g + h, heap_counter, neighbor_state))

    return None


def plan_window(start_positions, goal_positions, grid, window_size, arrived_flags, rra_stars):
    num_agents = len(start_positions)
    reservation_table = ReservationTable()

    # Keep already-arrived agents parked on their goals for the whole window.
    for agent_index in range(num_agents):
        if not arrived_flags[agent_index]:
            continue
        goal_x, goal_y = goal_positions[agent_index]
        for t in range(window_size + 1):
            reservation_table.reserve_vertex(goal_x, goal_y, t)

    paths = []
    for agent_index in range(num_agents):
        goal_x, goal_y = goal_positions[agent_index]
        if arrived_flags[agent_index]:
            paths.append([State(goal_x, goal_y, 0)])
            continue

        agent_start = State(start_positions[agent_index][0], start_positions[agent_index][1], 0)

        path = windowed_a_star_search(agent_start, goal_x, goal_y, window_size, grid, reservation_table, rra_stars[agent_index])
        if path is None:
            path = [State(agent_start.x, agent_start.y, 0)]

        for state in path:
            reservation_table.reserve_vertex(state.x, state.y, state.t)

        for step_index in range(len(path) - 1):
            state_a = path[step_index]
            state_b = path[step_index + 1]
            reservation_table.reserve_edge(state_a.x, state_a.y, state_b.x, state_b.y, state_a.t)

        final_state = path[-1]
        for t in range(final_state.t + 1, window_size + 1):
            reservation_table.reserve_vertex(final_state.x, final_state.y, t)

        paths.append(path)

    return paths


def run_whca(start_positions, goal_positions, grid, window_size, max_turns=100):
    """Execute the WHCA experiment over multiple windows and collect statistics."""
    num_agents = len(start_positions)
    current_positions = list(start_positions)
    arrived = [start_positions[i] == goal_positions[i] for i in range(num_agents)]
    arrival_times = [0 if arrived[i] else -1 for i in range(num_agents)]
    trajectories = [[(start_positions[i][0], start_positions[i][1])] for i in range(num_agents)]
    elapsed_windows = []
    window_offset = 0
    initial_planning_time = 0.0
    step_size = max(1, window_size // 2)

    # CREATE ONCE — persists across all windows for this trial
    rra_stars = [
        RRAstar(gx, gy, grid) if not arrived[i] else None
        for i, (gx, gy) in enumerate(goal_positions)
    ]

    for window_index in range((max_turns // step_size) + 3):
        if all(arrived) or window_offset >= max_turns:
            break

        """
        # Closest to goal plans first
        priority_order = sorted(
            range(num_agents),
            key=lambda i: rra_stars[i].get_h(*current_positions[i]) if rra_stars[i] else 0,
            reverse=False   
        )
        """
        
        # Static order (agent 0, 1, 2, ...)
        priority_order = list(range(num_agents))


        ordered_starts   = [current_positions[i] for i in priority_order]
        ordered_goals    = [goal_positions[i]    for i in priority_order]
        ordered_arrived  = [arrived[i]           for i in priority_order]
        ordered_rra_stars = [rra_stars[i]        for i in priority_order]  

        start_time = time.perf_counter()
        window_paths_ordered = plan_window(ordered_starts, ordered_goals, grid, window_size,ordered_arrived, ordered_rra_stars
        )  
        elapsed = time.perf_counter() - start_time
        if window_index == 0:
            initial_planning_time = elapsed
        elapsed_windows.append(elapsed)

        # Map results back to original agent indices
        window_paths = [None] * num_agents
        for rank, original_i in enumerate(priority_order):
            window_paths[original_i] = window_paths_ordered[rank]

        for agent_index in range(num_agents):
            if arrived[agent_index]:
                continue
            for state in window_paths[agent_index][1:]:
                if state.t > step_size:        # execute first W/2 steps
                    break
                if window_offset + state.t > max_turns:
                    break
                trajectories[agent_index].append((state.x, state.y))

        window_offset += step_size         # ← advance by W/2

        for agent_index in range(num_agents):
            if arrived[agent_index]:
                continue
            path = window_paths[agent_index]
            last_executed = path[0]
            for state in path[1:]:
                if state.t <= step_size:
                    last_executed = state
                else:
                    break
            current_positions[agent_index] = (last_executed.x, last_executed.y)
            if (last_executed.x, last_executed.y) == goal_positions[agent_index]:
                arrived[agent_index] = True
                arrival_times[agent_index] = window_offset - step_size + last_executed.t

    for agent_index in range(num_agents):
        if not arrived[agent_index]:
            arrival_times[agent_index] = -1

    return arrival_times, trajectories, initial_planning_time, elapsed_windows


# ── Maze ──────────────────────────────────────────────────────────────────────

def generate_maze(size=32, obs=0.20, seed=None):
    """Generate a random binary grid and keep its largest free region."""
    rng = random.Random(seed)
    
    """
    grid = np.zeros((size, size), dtype=np.int8)
    for x in range(size):
        for y in range(size):
            if rng.random() < obs:
                grid[x, y] = 1
    
    free_cells = [(x, y) for x in range(size) for y in range(size) if grid[x, y] == 0]
    
    return grid, free_cells
    """
    
    while True:
        grid = np.zeros((size, size), dtype=np.int8)
        for x in range(size):
            for y in range(size):
                if rng.random() < obs:
                    grid[x, y] = 1

        free_cells = [(x, y) for x in range(size) for y in range(size) if grid[x, y] == 0]
        if not free_cells:
            continue

        visited = set()
        components = []
        for cell in free_cells:
            if cell in visited:
                continue
            region = []
            queue = deque([cell])
            visited.add(cell)
            while queue:
                cx, cy = queue.popleft()
                region.append((cx, cy))
                for ddx, ddy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    neighbor = (cx + ddx, cy + ddy)
                    if (0 <= neighbor[0] < size and 0 <= neighbor[1] < size
                            and grid[neighbor[0], neighbor[1]] == 0
                            and neighbor not in visited):
                        visited.add(neighbor)
                        queue.append(neighbor)
            components.append(region)

        if not components:
            continue

        largest_region = set(max(components, key=len))
        for x, y in free_cells:
            if (x, y) not in largest_region:
                grid[x, y] = 1

        if len(largest_region) >= 50:
            return grid, sorted(largest_region)

def sample_agents(free_cells, agent_count, rng):
    """Randomly sample start and goal positions for each agent."""
    if len(free_cells) < agent_count:
        return None, None
    starts = rng.sample(free_cells, agent_count)
    goals  = rng.sample(free_cells, agent_count)   # independent — may overlap with starts
    # Optional: forbid only start_i == goal_i for the same agent (trivial trial)
    for i in range(agent_count):
        while goals[i] == starts[i]:
            goals[i] = rng.choice(free_cells)
    return [tuple(s) for s in starts], [tuple(g) for g in goals]


# ── Metrics ───────────────────────────────────────────────────────────────────

def metrics(arrival_times, trajectories, max_turns=100):
    """Compute experiment statistics from WHCA results."""
    agent_count = len(arrival_times)
    success_count = sum(1 for t in arrival_times if 0 <= t <= max_turns)
    all_path_lengths = [t if 0 <= t <= max_turns else max_turns for t in arrival_times]

    cycle_counts = []
    for path in trajectories:
        visited = set()
        cycles = 0
        for position in path:
            if position in visited:
                cycles += 1
            visited.add(position)
        cycle_counts.append(cycles)

    successful_cycles = [
        c for c, t in zip(cycle_counts, arrival_times)
        if 0 <= t <= max_turns
    ]

    return {
        "success_rate":   success_count / agent_count * 100,
        "avg_path_len":   float(np.mean(all_path_lengths)),        
        "avg_cycles":     float(np.mean(successful_cycles)) if successful_cycles else 0.0,
    }


# ── Colors ────────────────────────────────────────────────────────────────────

COLORS=[(0.9,0.1,0.1),(0.1,0.1,0.9),(0.1,0.8,0.1),(0.7,0.1,0.9),
        (0.9,0.5,0.0),(0.0,0.8,0.8),(0.9,0.9,0.0),(0.9,0.0,0.5),
        (0.5,0.9,0.5),(0.9,0.5,0.9),(0.5,0.5,0.9),(0.9,0.7,0.3)]


# ── ROS2 Node ─────────────────────────────────────────────────────────────────

class WHCAExperimentNode(Node):
    def __init__(self):
        super().__init__("whca_experiment_node")
        
        # Define experiment parameters and publishers.
        self.window_sizes = [8,16,32]
        self.agent_counts = [10,20,30,40,50,60,70,80,90,100]
        self.n_trials = 10
        self.max_turns = 100
        self.cell_size = 0.5
        self.animate_delay = 8.0
        self.output_csv = str(os.path.expanduser("~/ros2_map/whca_results.csv"))
        self.frame = "map"

        self.get_logger().info(
            f"windows={self.window_sizes}  agents={self.agent_counts}  trials={self.n_trials}"
        )
        
        # Generate maps equal to number of trials to ensure consistent mazes
        # across different window/agent settings for the same trial index.
        self.trial_maps = []
        for trial_idx in range(self.n_trials):
            self.trial_maps.append(generate_maze(seed=trial_idx * 100_000))

        publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        # Publishers for the map, paths, goals, starts, and robot markers.
        self.map_pub = self.create_publisher(OccupancyGrid, "map", publisher_qos)
        self.path_pub = self.create_publisher(MarkerArray, "experiment_paths", publisher_qos)
        self.goal_pub = self.create_publisher(MarkerArray, "goal_markers", publisher_qos)
        self.start_pub = self.create_publisher(MarkerArray, "start_markers", publisher_qos)
        self.robot_pub = self.create_publisher(MarkerArray, "robot_markers", 10)

        self._active_paths = []
        self._animation_frame = 0
        self._animation_length = 0
        self._animation_enabled = False
        self._lock = threading.Lock()
        self.create_timer(0.25, self._anim_tick)
        self._results = []

        # Launch the experiment loop in a separate thread to avoid blocking ROS callbacks.
        #TODO: uncomment 
        threading.Thread(target=self._run_all, daemon=True).start()
        #threading.Thread(target=self._debug, daemon=True).start()
        
    # ── Animation ─────────────────────────────────────────────────────────────

    def _anim_tick(self):
        with self._lock:
            if not self._animation_enabled:
                return
            frame_index = self._animation_frame
            paths = self._active_paths
            self._animation_frame += 1
            if self._animation_frame >= self._animation_length:
                self._animation_enabled = False

        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = self.frame
        delete_marker.ns = "robots"
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, path in enumerate(paths):
            if not path:
                continue
            index = min(frame_index, len(path) - 1)
            wx, wy = self._world_coordinates(path[index][0], path[index][1])
            r, g, b = COLORS[i % len(COLORS)]

            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = now
            marker.ns = "robots"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = wx
            marker.pose.position.y = wy
            marker.pose.position.z = 0.3
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.cell_size * 0.8
            marker.scale.y = self.cell_size * 0.8
            marker.scale.z = self.cell_size * 0.8
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.9
            marker_array.markers.append(marker)

        self.robot_pub.publish(marker_array)

    # ── Experiment loop ────────────────────────────────────────────────────────

    def _debug(self):
        """Run a single trial with animation for debugging purposes."""
        w=8
        a=70
        i=0
        self.get_logger().info(f"Running debug trial with W={w} agents={a}, trial={i}...")
        self._run_single_trial(agent_count=a, window_size=w, trial_index=i, show=True)

    def _run_all(self):
        # time.sleep(8.0)
        total_runs = len(self.window_sizes) * len(self.agent_counts) * self.n_trials
        run_index = 0

        for window_size in self.window_sizes:
            for agent_count in self.agent_counts:
                for trial in range(self.n_trials):
                    run_index += 1
                    self.get_logger().info(
                        f"[{run_index}/{total_runs}] W={window_size} agents={agent_count} trial={trial+1}"
                    )
                    results = self._run_single_trial(agent_count, window_size, trial, show=(trial==0))
                    self._results.append(results)

        self._save()
        self.get_logger().info(f"All done. Results → {self.output_csv}")

    def _run_single_trial(self, agent_count, window_size, trial_index, show=False):
        seed = trial_index * 100_000 + agent_count
        rng = random.Random(seed)
        grid, free_cells = self.trial_maps[trial_index]
        starts, goals = sample_agents(free_cells, agent_count, rng)
        
        # Fix bug where some trials had too few free cells to place all agents' starts and goals.
        # FIXME this needs a proper solution — currently just skipping those trials and marking them in the results.
        if starts is None:
            self.get_logger().info(
                f"Skipping trial with W={window_size} agents={agent_count}, "
                f"trial={trial_index}: not enough free cells for starts/goals."
            )
            return {
                "window_size": window_size,
                "n_agents": agent_count,
                "trial": trial_index,
                "success_rate": 0.0,
                "avg_path_len": 0.0,
                "avg_cycles": 0.0,
                "init_ms": 0.0,
                "max_turn_ms": 0.0,
                "skipped": True,
            }

        self.get_logger().info(f"Running trial with W={window_size} agents={agent_count}, trial={trial_index}...")
        self._publish_map(grid)
        arrival_times, trajectories, initial_time, window_times = run_whca(
            starts, goals, grid, window_size, self.max_turns
        )
        stats = metrics(arrival_times, trajectories, self.max_turns)

        self.get_logger().info(
            f"  success={stats['success_rate']:.0f}%  path={stats['avg_path_len']:.1f}"
            f"  cycles={stats['avg_cycles']:.2f}  init={initial_time*1000:.1f}ms"
        )

        results = {
            "window_size": window_size,
            "n_agents": agent_count,
            "trial": trial_index,
            "success_rate": stats["success_rate"],
            "avg_path_len": stats["avg_path_len"],
            "avg_cycles": stats["avg_cycles"],
            "init_ms": initial_time * 1000,
            "max_turn_ms": (max(window_times) * 1000 if window_times else 0),
            "skipped": False,
        }

        if show:
            self._publish_paths(trajectories)
            self._publish_goals(goals)
            self._publish_starts(starts)
            with self._lock:
                self._active_paths = trajectories
                self._animation_frame = 0
                self._animation_length = max((len(path) for path in trajectories), default=0)
                self._animation_enabled = True
            while True:
                with self._lock:
                    if not self._animation_enabled:
                        break
                time.sleep(0.05)
            time.sleep(1.0)
        
        return results

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_map(self, grid):
        """Publish the occupancy grid used for the current experiment."""
        width, height = grid.shape
        msg = OccupancyGrid()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.cell_size
        msg.info.width = height
        msg.info.height = width
        msg.info.origin.position.x = -(height * self.cell_size) / 2
        msg.info.origin.position.y = -(width * self.cell_size) / 2
        msg.info.origin.orientation.w = 1.0
        msg.data = [100 if grid[x, y] == 1 else 0 for y in range(width) for x in range(height)]
        self.map_pub.publish(msg)

    def _publish_paths(self, trajectories):
        """Publish path line strips for the current agent trajectories."""
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        remove_paths = Marker()
        remove_paths.header.frame_id = self.frame
        remove_paths.ns = "paths"
        remove_paths.action = Marker.DELETEALL
        marker_array.markers.append(remove_paths)

        for agent_index, path in enumerate(trajectories):
            if not path:
                continue
            r, g, b = COLORS[agent_index % len(COLORS)]
            marker = Marker()
            marker.header.frame_id = self.frame
            marker.header.stamp = now
            marker.ns = "paths"
            marker.id = agent_index
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = self.cell_size * 0.15
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.85
            marker.pose.orientation.w = 1.0

            for x, y in path:
                wx, wy = self._world_coordinates(x, y)
                point = Point()
                point.x = wx
                point.y = wy
                point.z = 0.1
                marker.points.append(point)

            marker_array.markers.append(marker)

        self.path_pub.publish(marker_array)

    def _publish_goals(self, goals):
        """Publish goal location markers and labels."""
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        remove_goals = Marker()
        remove_goals.header.frame_id = self.frame
        remove_goals.ns = "goals"
        remove_goals.action = Marker.DELETEALL
        marker_array.markers.append(remove_goals)

        remove_labels = Marker()
        remove_labels.header.frame_id = self.frame
        remove_labels.ns = "glabels"
        remove_labels.action = Marker.DELETEALL
        marker_array.markers.append(remove_labels)

        for agent_index, (goal_x, goal_y) in enumerate(goals):
            r, g, b = COLORS[agent_index % len(COLORS)]
            wx, wy = self._world_coordinates(goal_x, goal_y)

            goal_marker = Marker()
            goal_marker.header.frame_id = self.frame
            goal_marker.header.stamp = now
            goal_marker.ns = "goals"
            goal_marker.id = agent_index
            goal_marker.type = Marker.CYLINDER
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = wx
            goal_marker.pose.position.y = wy
            goal_marker.pose.position.z = 0.05
            goal_marker.pose.orientation.w = 1.0
            goal_marker.scale.x = self.cell_size * 0.9
            goal_marker.scale.y = self.cell_size * 0.9
            goal_marker.scale.z = 0.1
            goal_marker.color.r = r
            goal_marker.color.g = g
            goal_marker.color.b = b
            goal_marker.color.a = 0.5
            marker_array.markers.append(goal_marker)

            label_marker = Marker()
            label_marker.header.frame_id = self.frame
            label_marker.header.stamp = now
            label_marker.ns = "glabels"
            label_marker.id = agent_index
            label_marker.type = Marker.TEXT_VIEW_FACING
            label_marker.action = Marker.ADD
            label_marker.pose.position.x = wx
            label_marker.pose.position.y = wy
            label_marker.pose.position.z = 0.6
            label_marker.pose.orientation.w = 1.0
            label_marker.scale.z = 0.3
            label_marker.color.r = r
            label_marker.color.g = g
            label_marker.color.b = b
            label_marker.color.a = 1.0
            label_marker.text = f"G{agent_index}"
            marker_array.markers.append(label_marker)

        self.goal_pub.publish(marker_array)

    def _publish_starts(self, starts):
        """Publish start location markers for each agent."""
        now = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        remove_starts = Marker()
        remove_starts.header.frame_id = self.frame
        remove_starts.ns = "starts"
        remove_starts.action = Marker.DELETEALL
        marker_array.markers.append(remove_starts)

        for agent_index, (start_x, start_y) in enumerate(starts):
            r, g, b = COLORS[agent_index % len(COLORS)]
            wx, wy = self._world_coordinates(start_x, start_y)

            start_marker = Marker()
            start_marker.header.frame_id = self.frame
            start_marker.header.stamp = now
            start_marker.ns = "starts"
            start_marker.id = agent_index
            start_marker.type = Marker.CUBE
            start_marker.action = Marker.ADD
            start_marker.pose.position.x = wx
            start_marker.pose.position.y = wy
            start_marker.pose.position.z = 0.05
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = self.cell_size * 0.6
            start_marker.scale.y = self.cell_size * 0.6
            start_marker.scale.z = 0.1
            start_marker.color.r = r
            start_marker.color.g = g
            start_marker.color.b = b
            start_marker.color.a = 0.7
            marker_array.markers.append(start_marker)

        self.start_pub.publish(marker_array)

    def _world_coordinates(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates for RViz."""
        origin = -(32 * self.cell_size) / 2
        return origin + (grid_x + 0.5) * self.cell_size, origin + (grid_y + 0.5) * self.cell_size

    # ── CSV ───────────────────────────────────────────────────────────────────

    def _save(self):
        """Write experiment results to CSV and log a summarized report."""
        if not self._results:
            return

        fields = [
            "window_size",
            "n_agents",
            "trial",
            "success_rate",
            "avg_path_len",
            "avg_cycles",
            "init_ms",
            "max_turn_ms",
            "skipped",
        ]
        with open(self.output_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self._results)

        self.get_logger().info("=" * 65)
        self.get_logger().info(
            f"{'W':>4}  {'Agents':>6}  {'Success%':>8}  {'PathLen':>7}  {'Cycles':>6}  {'Init(ms)':>8}"
        )
        self.get_logger().info("=" * 65)

        grouped_results = groupby(
            sorted(self._results, key=lambda r: (r["window_size"], r["n_agents"])),
            key=lambda r: (r["window_size"], r["n_agents"]),
        )
        for (window_size, agent_count), group in grouped_results:
            group = list(group)
            self.get_logger().info(
                f"{window_size:>4}  "
                f"{agent_count:>6}  "
                f"{np.mean([r['success_rate'] for r in group]):>8.1f}  "
                f"{np.mean([r['avg_path_len'] for r in group]):>7.1f}  "
                f"{np.mean([r['avg_cycles'] for r in group]):>6.2f}  "
                f"{np.mean([r['init_ms'] for r in group]):>8.2f}"
            )


def main():
    rclpy.init()
    node = WHCAExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
