import heapq
from dataclasses import dataclass
import numpy as np
import time

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
    current_headings = [-1] * num_agents   # unknown at start; set by first move

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
        ordered_headings  = [current_headings[i] for i in priority_order]

        start_time = time.perf_counter()
        window_paths_ordered = plan_window(ordered_starts, ordered_goals, grid, window_size, ordered_arrived, ordered_rra_stars, ordered_headings)
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
            if last_executed.h != -1:
                current_headings[agent_index] = last_executed.h
            if (last_executed.x, last_executed.y) == goal_positions[agent_index]:
                arrived[agent_index] = True
                arrival_times[agent_index] = window_offset - step_size + last_executed.t

    for agent_index in range(num_agents):
        if not arrived[agent_index]:
            arrival_times[agent_index] = -1

    return arrival_times, trajectories, initial_planning_time, elapsed_windows

@dataclass(frozen=True)
class State:
    x: int
    y: int
    t: int
    h: int = -1   # heading: 0=E(+x), 1=N(+y), 2=W, 3=S; -1 = unspecified

HEADINGS = [(1, 0), (0, 1), (-1, 0), (0, -1)]   # E, N, W, S

class ReservationTable:
    """Stores time-extended vertex and edge reservations during windowed planning."""

    def __init__(self, k=0):
        self.k = max(0, int(k))
        self.vertex_reservations = set()
        self.edge_reservations = set()
        self.history_reservations = {}   # (x, y, t) -> set of agent ids

    def reserve_vertex(self, x, y, t):
        for dt in range(-self.k, self.k + 1):
            self.vertex_reservations.add((x, y, t + dt))

    def reserve_history(self, agent, x, y, t):
        """Where `agent` was t steps ago, Without this the k-band has no memory across window boundaries
        """
        for dt in range(-self.k, self.k + 1):
            self.history_reservations.setdefault((x, y, t + dt), set()).add(agent)
    def reserve_edge(self, x1, y1, x2, y2, t):
        self.edge_reservations.add((x1, y1, x2, y2, t))

    def is_vertex_reserved(self, x, y, t, agent=None):
        if (x, y, t) in self.vertex_reservations:
            return True
        owners = self.history_reservations.get((x, y, t))
        if owners and (agent is None or owners != {agent}):
            return True
        return False

    def is_edge_reserved(self, x1, y1, x2, y2, t):
        return (x1, y1, x2, y2, t) in self.edge_reservations



def manhattan_distance(x, y, gx, gy):
    """Return Manhattan distance from (x,y) to goal (gx,gy)."""
    return abs(x - gx) + abs(y - gy)


class RRAstar:
    """
    Reverse Resumable A* heuristic — Silver (2005).

    Runs backward Dijkstra from the agent's goal through the static obstacle
    map. When queried for h(x, y), the search resumes until (x, y) is expanded
    and returns the true shortest-path distance, ignoring all other agents.

    One instance per agent, reused across all planning windows.
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


def tail_is_free(reservation_table, x, y, t_from, horizon, agent=None):
    """True if (x, y) is unreserved for every step in [t_from, horizon].

    plan_window parks an agent on its final cell for the rest of the window, so
    a path may only *end* on a cell whose remaining window is free. Without this
    check the parked tail can be laid straight over a higher-priority agent's
    reservation, producing a planned vertex conflict.
    """
    for t in range(t_from, horizon + 1):
        if reservation_table.is_vertex_reserved(x, y, t, agent):
            return False
    return True


def reconstruct_path(came_from, current_state, start_state):
    """Reconstruct an A* path from the came_from dictionary."""
    path = [current_state]
    while current_state in came_from:
        current_state = came_from[current_state]
        path.append(current_state)
    path.reverse()
    return path


def windowed_a_star_search(start_state, goal_x, goal_y, window_size, grid,
                           reservation_table, rra_star=None, commit_horizon=None,
                           agent_index=None):
    """Perform A* search for a single agent inside the WHCA time window."""
    width, height = grid.shape
    if commit_horizon is None:
        commit_horizon = window_size
    heap_counter = 0
    open_heap = []
    h0 = rra_star.get_h(start_state.x, start_state.y) if rra_star else manhattan_distance(start_state.x, start_state.y, goal_x, goal_y)
    heapq.heappush(open_heap, (start_state.t + h0, heap_counter, start_state))
    came_from = {}
    g_scores = {start_state: 0}

    while open_heap:
        _, _, current_state = heapq.heappop(open_heap)

        # Only finish on the goal if the agent can actually *stay* there for the
        # rest of the window -- otherwise keep searching (it will wait at the
        # goal until the window ends, or step aside and come back).
        if (current_state.x == goal_x and current_state.y == goal_y
                and tail_is_free(reservation_table, goal_x, goal_y,
                                 current_state.t + 1, commit_horizon, agent_index)):
            return reconstruct_path(came_from, current_state, start_state)
        if current_state.t >= window_size:
            return reconstruct_path(came_from, current_state, start_state)

        # Turn-aware : per timestep an agent
        # may WAIT, ROTATE 90 degrees in place (cell stays occupied -> the vertex
        # reservation naturally holds it through the turn), or MOVE one cell
        # forward in its current heading. 180 degrees = two rotate steps.
        cx, cy, ch, next_t = current_state.x, current_state.y, current_state.h, current_state.t + 1
        successors = [(cx, cy, ch, False)]                       # wait
        if ch == -1:
            # first move sets it, any direction
            for hh, (dx, dy) in enumerate(HEADINGS):
                successors.append((cx + dx, cy + dy, hh, True))
        else:
            successors.append((cx, cy, (ch + 1) % 4, False))     # rotate left
            successors.append((cx, cy, (ch + 3) % 4, False))     # rotate right
            dx, dy = HEADINGS[ch]
            successors.append((cx + dx, cy + dy, ch, True))      # move forward

        for next_x, next_y, next_h, is_move in successors:
            if is_move:
                if not (0 <= next_x < width and 0 <= next_y < height):
                    continue
                if grid[next_x, next_y] == 1:
                    continue
            if reservation_table.is_vertex_reserved(next_x, next_y, next_t, agent_index):
                continue
            if is_move and reservation_table.is_edge_reserved(next_x, next_y, cx, cy, current_state.t):
                continue

            neighbor_state = State(next_x, next_y, next_t, next_h)
            at_goal_wait = (cx == goal_x and cy == goal_y
                and not is_move and next_h == ch
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


def windowed_evade_search(start_state, goal_x, goal_y, window_size, grid,
                         reservation_table, rra_star=None, agent_index=None):
    """Fallback when the goal-directed A* finds nothing: survive the window.

    Breadth-first over space-time from `start_state`, ignoring the goal, keeping
    only reservation-legal successors. Returns (path, reached_window_end). Among
    the deepest reachable states the one closest to the goal is chosen, so the
    agent still makes progress instead of freezing in place.
    """
    width, height = grid.shape
    came_from = {}
    frontier = [start_state]
    seen = {start_state}
    deepest = [start_state]

    for _ in range(window_size):
        if not frontier:
            break
        next_frontier = []
        for current_state in frontier:
            cx, cy, ch = current_state.x, current_state.y, current_state.h
            next_t = current_state.t + 1
            successors = [(cx, cy, ch, False)]
            if ch == -1:
                for hh, (dx, dy) in enumerate(HEADINGS):
                    successors.append((cx + dx, cy + dy, hh, True))
            else:
                successors.append((cx, cy, (ch + 1) % 4, False))
                successors.append((cx, cy, (ch + 3) % 4, False))
                dx, dy = HEADINGS[ch]
                successors.append((cx + dx, cy + dy, ch, True))

            for next_x, next_y, next_h, is_move in successors:
                if is_move:
                    if not (0 <= next_x < width and 0 <= next_y < height):
                        continue
                    if grid[next_x, next_y] == 1:
                        continue
                if reservation_table.is_vertex_reserved(next_x, next_y, next_t, agent_index):
                    continue
                if is_move and reservation_table.is_edge_reserved(next_x, next_y, cx, cy, current_state.t):
                    continue
                neighbor_state = State(next_x, next_y, next_t, next_h)
                if neighbor_state in seen:
                    continue
                seen.add(neighbor_state)
                came_from[neighbor_state] = current_state
                next_frontier.append(neighbor_state)
        if not next_frontier:
            break
        frontier = next_frontier
        deepest = next_frontier

    def _h(state):
        return (rra_star.get_h(state.x, state.y) if rra_star
                else manhattan_distance(state.x, state.y, goal_x, goal_y))

    best = min(deepest, key=_h)
    return reconstruct_path(came_from, best, start_state), best.t >= window_size


def _plan_pass(order, start_positions, goal_positions, grid, window_size,
               arrived_flags, rra_stars, start_headings, commit_horizon,
               k=0, history=None):
    """One prioritised sweep in the given agent order.

    Returns (paths_by_agent_index, hard_failures, astar_failures) where a hard
    failure is an agent that could not stay conflict-free even as far as the
    commit horizon -- i.e. an unavoidable planned collision.
    """
    reservation_table = ReservationTable(k)

    # Carry each agent's last k occupied cells into this window at negative
    # local times, so the k-band spans the window boundary.
    if k:
        # An agent is at its own start cell at t = 0, so under a k-band no other
        # agent may enter that cell before t = k. Start cells are never checked
        # by the search (they are given, not planned), so bind them explicitly.
        for agent_index in range(len(start_positions)):
            sx, sy = start_positions[agent_index]
            reservation_table.reserve_history(agent_index, sx, sy, 0)
        if history is not None:
            for agent_index, cells in enumerate(history):
                for age, cell in enumerate(cells[:k], start=1):
                    if cell is not None:
                        reservation_table.reserve_history(agent_index, cell[0], cell[1], -age)
    for agent_index in order:
        if not arrived_flags[agent_index]:
            continue
        goal_x, goal_y = goal_positions[agent_index]
        for t in range(window_size + 1):
            reservation_table.reserve_vertex(goal_x, goal_y, t)

    paths = {}
    hard_failures, astar_failures = [], 0

    for agent_index in order:
        goal_x, goal_y = goal_positions[agent_index]
        if arrived_flags[agent_index]:
            paths[agent_index] = [State(goal_x, goal_y, 0)]
            continue

        h0 = start_headings[agent_index] if start_headings is not None else -1
        agent_start = State(start_positions[agent_index][0],
                            start_positions[agent_index][1], 0, h0)

        path = windowed_a_star_search(agent_start, goal_x, goal_y, window_size,
                                      grid, reservation_table, rra_stars[agent_index],
                                      commit_horizon, agent_index)
        if path is None:
            # No goal-directed plan exists in this window. Do NOT park on the
            # start cell unconditionally -- it may already be reserved by a
            # higher-priority agent, which is a planned vertex conflict. Find
            # the best legal way to sit the window out instead.
            astar_failures += 1
            path, _complete = windowed_evade_search(
                agent_start, goal_x, goal_y, window_size, grid,
                reservation_table, rra_stars[agent_index], agent_index)
            if path[-1].t < commit_horizon:
                hard_failures.append(agent_index)

        for state in path:
            reservation_table.reserve_vertex(state.x, state.y, state.t)
        for step_index in range(len(path) - 1):
            state_a, state_b = path[step_index], path[step_index + 1]
            if (state_a.x, state_a.y) != (state_b.x, state_b.y):
                reservation_table.reserve_edge(state_a.x, state_a.y,
                                               state_b.x, state_b.y, state_a.t)
        final_state = path[-1]
        for t in range(final_state.t + 1, window_size + 1):
            reservation_table.reserve_vertex(final_state.x, final_state.y, t)

        paths[agent_index] = path

    return paths, hard_failures, astar_failures


def plan_window(start_positions, goal_positions, grid, window_size, arrived_flags,
                rra_stars, start_headings=None, stats=None,
                commit_horizon=None, max_promotions=4, k=0, history=None):
    """Plan one WHCA* window. Agents are given in caller-chosen priority order.

    Only the first `commit_horizon` steps of each path are ever executed (the
    controller commits W//2), so that is the horizon conflict-freedom is
    enforced over.

    If an agent is boxed in by higher-priority reservations and cannot stay
    conflict-free to the commit horizon, the whole window is re-planned with
    that agent promoted to the front of the priority order.
    `stats`, if supplied, is updated in place:
        astar_failures  -- goal-directed A* found nothing, evade search used
        promotions      -- window re-planned with a boxed-in agent promoted
        hard_failures   -- agents still conflicting after all promotions
    """
    num_agents = len(start_positions)
    if commit_horizon is None:
        commit_horizon = max(1, window_size // 2)
    if stats is not None:
        for key in ("astar_failures", "promotions", "hard_failures"):
            stats.setdefault(key, 0)

    order = list(range(num_agents))
    for attempt in range(max_promotions + 1):
        paths, hard_failures, astar_failures = _plan_pass(
            order, start_positions, goal_positions, grid, window_size,
            arrived_flags, rra_stars, start_headings, commit_horizon, k, history)
        if not hard_failures or attempt == max_promotions:
            break
        promoted = set(hard_failures)
        order = hard_failures + [a for a in order if a not in promoted]
        if stats is not None:
            stats["promotions"] += 1

    if stats is not None:
        stats["astar_failures"] += astar_failures
        stats["hard_failures"] += len(hard_failures)

    return [paths[i] for i in range(num_agents)]
