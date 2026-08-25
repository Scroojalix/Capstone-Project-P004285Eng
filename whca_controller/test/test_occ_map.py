from whca_controller.helpers import *

def test_occ_map():
    map: Map = load_map('SmallWarehouseOccMap.yaml', 1)
    
    assert map.dimx == 60
    assert map.dimy == 20
    assert map.cell_size == 1
    
    grid = map.grid
    
    print()
    for x in range(map.dimx):
        print(grid[x])
        
    
    
    