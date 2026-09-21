from whca_controller.helpers import *
from pytest import approx

def test_small_map():
    map: Map = load_map('SmallWarehouse.yaml', 1)
    
    assert map.dimx == 60
    assert map.dimy == 20
    assert map.cell_size == 1
    
    grid = map.grid
    
    # Check that the gaps between shelves are correct widths
    for y in [3,4, 7, 8, 11, 12, 15, 16]:
        for x in range(map.dimx):
            if x not in [1, 2, 3, 28, 29, 30, 31, 56, 57, 58]:
                assert map.grid[x][y] == 1
            else:
                assert map.grid[x][y] == 0
    
    # Check all corners are occupied
    assert grid[0][0] == 1
    assert grid[map.dimx-1][0]
    assert grid[0][map.dimy-1]
    assert grid[map.dimx-1][map.dimy-1] == 1
    
    # Print the grid for visual inspection
    print_map(map)
    
def test_small_map_occupancy():
    map: Map = load_map('SmallWarehouse.yaml', 1)
    
    # Check corners occupied
    assert map.check_world_occupied(-30, -10) == 1
    assert map.check_world_occupied(-30, 10) == 1
    assert map.check_world_occupied(30, -10) == 1
    assert map.check_world_occupied(30, 10) == 1
    
    # Check shelves
    assert map.check_world_occupied(0, 0) == 0
    assert map.check_world_occupied(-15, 0) == 0
    assert map.check_world_occupied(-15, 5) == 1
    
def test_cell_positions():
    map: Map = load_map('SmallWarehouse.yaml', 1)
    
    wx, wy = map.cell_to_world(0, 0)
    assert wx == approx(-29.5)
    assert wy == approx(-9.5)
 
def test_narrow_corrider_map():
    map: Map = load_map('NarrowCorridor.yaml', 1)
    
    assert map.dimx == 30
    assert map.dimy == 21
    
    # Check that the corridors are not occupied
    for x in range(map.dimx):
        if x not in [12, 17]:
            assert map.grid[x][10] == 1
        else:
            assert map.grid[x][10] == 0
    
    print_map(map)

def print_map(map: Map):
    print()
    for x in range(map.dimx):
        print(map.grid[x])