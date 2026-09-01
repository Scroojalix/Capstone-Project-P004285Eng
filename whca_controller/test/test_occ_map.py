from whca_controller.helpers import *

def test_small_map():
    map: Map = load_map('SmallWarehouseOccMap.yaml', 1)
        
    # FIXME: had to manually adjust occupancy map to be exactly 1200x400
    # pixels, when previously it was 1200x399, causing the downscaled
    # grid to be misaligned with real world coordinates
    
    assert map.dimx == 60
    assert map.dimy == 20
    assert map.cell_size == 1
    
    grid = map.grid
    
    # Check all corners are occupied
    assert grid[0][0] == 1
    assert grid[map.dimx-1][0]
    assert grid[0][map.dimy-1]
    assert grid[map.dimx-1][map.dimy-1] == 1
    
def test_small_map_occupancy():
    map: Map = load_map('SmallWarehouseOccMap.yaml', 1)
    
    # Check corners occupied
    assert map.check_world_occupied(-30, -10) == 1
    assert map.check_world_occupied(-30, 10) == 1
    assert map.check_world_occupied(30, -10) == 1
    assert map.check_world_occupied(30, 10) == 1
    
    # Check shelves
    assert map.check_world_occupied(0, 0) == 0
    assert map.check_world_occupied(-15, 0) == 0
    assert map.check_world_occupied(-15, 5) == 1
    
    
    