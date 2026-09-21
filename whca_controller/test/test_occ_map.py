from whca_controller.helpers import *
from pytest import approx

def test_small_map():
    map: Map = load_map('SmallWarehouse.yaml', 1)
    
    assert map.dimx == 60
    assert map.dimy == 20
    assert map.cell_size == 1
    
    grid = map.grid
    
    # Print the grid for visual inspection
    print_map(map)
    
    # Check all corners are occupied
    assert grid[0][0] == 1
    assert grid[map.dimx-1][0]
    assert grid[0][map.dimy-1]
    assert grid[map.dimx-1][map.dimy-1] == 1
    
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
    
    print_map(map)

def print_map(map: Map):
    print()
    for x in range(map.dimx):
        print(map.grid[x])