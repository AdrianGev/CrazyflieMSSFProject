import numpy as np
from typing import Tuple, List, Set, Optional


# represents a 2d grid environment for pathfinding
# grid convention:
# - (0,0) is top-left
# - x increases rightward (columns)
# - y increases downward (rows)
# - 0 = free cell, 1 = obstacle
class GridWorld:
    
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=np.int8)
        
    def set_obstacle(self, x: int, y: int):
        # mark cell as obstacle
        if self.in_bounds(x, y):
            self.grid[y, x] = 1
            
    def clear_obstacle(self, x: int, y: int):
        # clear obstacle from cell
        if self.in_bounds(x, y):
            self.grid[y, x] = 0
            
    def is_obstacle(self, x: int, y: int) -> bool:
        # check if cell is obstacle
        if not self.in_bounds(x, y):
            return True
        return self.grid[y, x] == 1
    
    def is_free(self, x: int, y: int) -> bool:
        # check if cell is free
        return self.in_bounds(x, y) and self.grid[y, x] == 0
    
    def in_bounds(self, x: int, y: int) -> bool:
        # check if coordinates are within grid bounds
        return 0 <= x < self.width and 0 <= y < self.height
    
    def get_neighbors(self, x: int, y: int) -> List[Tuple[int, int]]:
        # get valid 4-connected neighbors
        # returns list of (x, y) tuples for free neighboring cells
        neighbors = []
        for dx, dy in [(0, -1), (1, 0), (0, 1), (-1, 0)]:  # up, right, down, left
            nx, ny = x + dx, y + dy
            if self.is_free(nx, ny):
                neighbors.append((nx, ny))
        return neighbors
    
    def get_local_patch(self, x: int, y: int, patch_size: int = 5) -> np.ndarray:
        # extract local patch around position
        # patch_size x patch_size array, padded with 1s outside bounds
        half = patch_size // 2
        patch = np.ones((patch_size, patch_size), dtype=np.int8)
        
        for py in range(patch_size):
            for px in range(patch_size):
                world_x = x + (px - half)
                world_y = y + (py - half)
                if self.in_bounds(world_x, world_y):
                    patch[py, px] = self.grid[world_y, world_x]
                    
        return patch
    
    def manhattan_distance(self, x1: int, y1: int, x2: int, y2: int) -> int:
        # calculate manhattan distance between two points
        return abs(x1 - x2) + abs(y1 - y2)
    
    def copy(self) -> 'GridWorld':
        # create deep copy of grid world
        new_world = GridWorld(self.width, self.height)
        new_world.grid = self.grid.copy()
        return new_world
    
    def clear(self):
        # clear all obstacles
        self.grid.fill(0)
        
    def add_random_obstacles(self, num_obstacles: int, 
                            avoid_positions: Optional[Set[Tuple[int, int]]] = None):
        # add random obstacles to grid
        if avoid_positions is None:
            avoid_positions = set()
            
        added = 0
        max_attempts = num_obstacles * 10
        attempts = 0
        
        while added < num_obstacles and attempts < max_attempts:
            x = np.random.randint(0, self.width)
            y = np.random.randint(0, self.height)
            
            if (x, y) not in avoid_positions and self.grid[y, x] == 0:
                self.set_obstacle(x, y)
                added += 1
                
            attempts += 1
    
    def __repr__(self):
        # string representation of grid
        lines = []
        for row in self.grid:
            line = ''.join(['#' if cell == 1 else '.' for cell in row])
            lines.append(line)
        return '\n'.join(lines)


def label_to_xy(label: str) -> Tuple[int, int]:
    # convert grid label to (x, y) coordinates
    # example: 'A1' -> (0, 0), 'B3' -> (1, 2)
    col_letter = label[0].upper()
    row_num = int(label[1:])
    
    x = ord(col_letter) - ord('A')
    y = row_num - 1
    
    return x, y


def xy_to_label(x: int, y: int) -> str:
    # convert (x, y) coordinates to grid label
    # example: (0, 0) -> 'A1', (1, 2) -> 'B3'
    col_letter = chr(ord('A') + x)
    row_num = y + 1
    
    return f"{col_letter}{row_num}"