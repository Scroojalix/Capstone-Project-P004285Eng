import heapq
from dataclasses import dataclass
import numpy as np

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