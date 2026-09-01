import os
import numpy as np
import math
from collections import deque
from PIL import Image
from ament_index_python.packages import get_package_share_directory

class Map:
    def __init__(self, origin_x, origin_y, cell_size, grid):
        self.origin_x: float = origin_x
        self.origin_y: float = origin_y
        self.cell_size: float = cell_size
        self.grid: np.ndarray = grid
        dimx, dimy = grid.shape
        self.dimx: int = dimx
        self.dimy: int = dimy
        
    def check_world_occupied(self, wx, wy):
        (cx, cy) = self.world_to_cell(wx, wy)
        return self.grid[cx][cy]
    
    def world_to_cell(self, wx, wy):
        cx = int(round((wx - self.origin_x) / self.cell_size))
        cy = int(round((wy - self.origin_y) / self.cell_size))
        return (min(max(cx, 0), self.dimx - 1), min(max(cy, 0), self.dimy - 1))

    def cell_to_world(self, cx, cy):
        return (self.origin_x + cx * self.cell_size, self.origin_y + cy * self.cell_size)

    def nearest_free(self, cx, cy, taken=frozenset()):
        """BFS to the closest free cell not in `taken` (returns input if none found)."""
        q, seen = deque([(cx, cy)]), {(cx, cy)}
        while q:
            x, y = q.popleft()
            if self.grid[x, y] == 0 and (x, y) not in taken:
                return (x, y)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (x + dx, y + dy)
                if 0 <= n[0] < self.dimx and 0 <= n[1] < self.dimy and n not in seen:
                    seen.add(n)
                    q.append(n)
        return (cx, cy)

def load_map(yaml_name, cell_size) -> np.ndarray:
    """Load a ROS map (.yaml + image) -> (grid[x, y] 1=blocked, origin, cell)."""
    
    config_path = os.path.join(get_package_share_directory('whca_controller'), 'config')
    
    yaml_path = os.path.join(config_path, yaml_name)
    
    cfg = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.split("#")[0].strip()
            if ":" in line:
                k, v = (p.strip() for p in line.split(":", 1))
                cfg[k] = ([float(x) for x in v.strip("[]").split(",")]
                          if v.startswith("[") else v)
                
    # Extract variables from config, with default values
    img_path = os.path.join(config_path, cfg["image"])
    negate = not bool(int(cfg.get("negate", 0)))
    occ_thresh = float(cfg.get("occupied_thresh", 0.65))
    origin = cfg.get("origin", (0, 0))
    res = float(cfg["resolution"])
       
    arr = np.array(Image.open(img_path).convert("L"), dtype=np.float32) / 255.0
    
    # FIXME: why does negate=false cause inversion
    if negate:
        arr = 1 - arr
    
    occupied = arr > occ_thresh

    # Orient so origin is top left, and can index grid with grid[x][y]
    grid = np.flipud(occupied).T

    f = max(1, int(round(cell_size / res)))

    # downsample: blocked if ANY sub-cell is occupied
    if f > 1:
        w, h = (grid.shape[0] // f) * f, (grid.shape[1] // f) * f
        grid = grid[:w, :h].reshape(w // f, f, h // f, f).any(axis=(1, 3))
    
    # Wrap all map info in own class
    map = Map(origin[0], origin[1], cell_size, grid.astype(int))
    return map

def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_to_heading(yaw):
    """Snap yaw (rad, 0 = +x) to grid heading 0=E, 1=N, 2=W, 3=S."""
    return int(round(yaw / (math.pi / 2))) % 4

def fmt_sched(cells):
    """Compact schedule string: only the timesteps where the cell changes."""
    out, last = [], None
    for t, c in enumerate(cells):
        if c != last:
            out.append(f"t{t}:{c}")
            last = c
    return " ".join(out)