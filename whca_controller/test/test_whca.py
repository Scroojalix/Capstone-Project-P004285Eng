from whca_controller.helpers import *
from whca_controller.whca_functions import *
import numpy as np
import random

def test_single_straight_path():
    """
    0 0 0 0
    s 0 0 g
    0 0 0 0
    """ 
    grid = np.zeros((3, 4))
    start = (1, 0)
    goal = (1, 3)
    rra = RRAstar(goal[0], goal[1], grid)
    
    paths = plan_window([start], [goal], grid, 32, [False], [rra])
    path = paths[0]

    for state in path:
        # Path moves along y direction with time
        assert state.t == state.y

def test_single_blocked_path():
    """
    0 0 0 0 0
    0 0 1 0 0
    s 0 1 0 g
    """
    grid = np.zeros((3, 5))
    grid[1][2] = 1
    grid[2][2] = 1
    # Flip so that (0,0) is in top-left and can index with grid[x][y]
    grid = np.flipud(grid).T
    start = (0, 0)
    goal = (4, 0)
    rra = RRAstar(goal[0], goal[1], grid)
    
    paths = plan_window([start], [goal], grid, 32, [False], [rra], [1])
    path = paths[0]

    # Path should travel:
    # - Reorient to face North
    # - North for 2 time steps
    # - Reorient to face East
    # - East for 4 time steps
    # - Reorient to face South
    # - South for 2 time steps
    # - total time steps: 11
    
    assert len(path) == 11
    
    for state in path:
        # print(state)
        if state.t == 0: # Reorient north
            assert state == State(0, 0, 0, 1)
        elif state.t > 0 and state.t <= 2: # North 2 steps
            assert state.y == state.t
        elif state.t == 3: # Reorient east
            assert state == State(0, 2, 3, 0)
        elif state.t > 3 and state.t <= 7: # East 4 steps
            assert state.x == state.t - 3
        elif state.t == 8:
            assert state == State(4, 2, 8, 3) # Reorient south
        elif state.t < 11:
            assert state.y == 10 - state.t # South 2 steps

            
def test_double_blocked_path():
    """
    0   0   0   0   0
    0   0   1   0   0
    s/g 0   1   0   g/s
    """
    grid = np.zeros((3, 5))
    grid[1][2] = 1
    grid[2][2] = 1
    # Flip so that (0,0) is in top-left and can index with grid[x][y]
    grid = np.flipud(grid).T
    starts = [(0, 0), (4, 0)]
    goals = [(4, 0), (0, 0)]
    rras = [RRAstar(goals[0][0], goals[0][1], grid), RRAstar(goals[1][0], goals[1][1], grid)]
    
    paths = plan_window(starts, goals, grid, 32, [False, False], rras, [1, 1])
    
    max_steps = max(len(path) for path in paths)

    for t in range(max_steps):
        positions = []
        for path in paths:
            if len(path) > t:
                state = path[t]
                pos = (state.x, state.y)
                positions.append(pos)
        
        # Verify that no cell is every occupied by multiple robots at any point in time
        assert len(positions) == len(set(positions))

def test_small_warehouse_many_robots():
    map = load_map('SmallWarehouseOccMap.yaml', 1.0)
    
    start_pos = []
    for x in range(8):
        for y in range(5):
            X = 1 + 8 * x
            Y = 1 + 4 * y
            start_pos.append((X, Y))
            
    random.shuffle(start_pos)
    
    starts = []
    goals = []
    rras = []
    
    num_robot = 6
    for i in range(num_robot):
        s = start_pos[i]
        g = start_pos[len(start_pos) - 1 - i]
        rra = RRAstar(g[0], g[1], map.grid)
        
        starts.append(s)
        goals.append(g)
        rras.append(rra)
    
    paths = plan_window(starts, goals, map.grid, 32, [False] * num_robot, rras, [1]*num_robot)

    print_window(paths)

    max_steps = max(len(path) for path in paths)
    for t in range(max_steps):
        edges = []
        for i, path in enumerate(paths):
            if t < len(path):
                curr = path[t]
                pos = (curr.x, curr.y)
                edges.append(pos)
                            
        # Verify that no cell is every occupied by multiple robots at any point in time
        assert len(edges) == len(set(edges))
