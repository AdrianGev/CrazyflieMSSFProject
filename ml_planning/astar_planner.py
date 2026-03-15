import heapq
import time
from typing import Tuple, List, Optional, Dict, Set, Callable
import numpy as np

from ml_planning.grid_world import GridWorld


# node in a* search
class SearchNode:
    def __init__(self, x: int, y: int, g: float, h: float, parent: Optional['SearchNode'] = None):
        self.x = x
        self.y = y
        self.g = g  # cost from start
        self.h = h  # heuristic to goal
        self.parent = parent
    
    def f(self) -> float:
        # total estimated cost
        return self.g + self.h
    
    def __lt__(self, other):
        # for heap ordering
        return self.f() < other.f()
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))


# result from pathfinding search
class SearchResult:
    def __init__(self, path: Optional[List[Tuple[int, int]]], success: bool, 
                 nodes_expanded: int, time_ms: float, hit_deadline: bool,
                 path_cost: float, best_f_score: float):
        self.path = path  # list of (x, y) coordinates
        self.success = success  # whether goal was reached
        self.nodes_expanded = nodes_expanded  # number of nodes expanded
        self.time_ms = time_ms  # time taken in milliseconds
        self.hit_deadline = hit_deadline  # whether deadline was exceeded
        self.path_cost = path_cost  # total path cost
        self.best_f_score = best_f_score  # best f-score when stopped
    
    def path_length(self) -> int:
        # number of steps in path
        return len(self.path) if self.path else 0


# baseline a* planner
# uses manhattan distance heuristic and expands nodes until goal is found
# or search space is exhausted
class AStarPlanner:
    
    def __init__(self, world: GridWorld, cpu_slowdown_factor: float = 1.0):
        self.world = world
        self.cpu_slowdown_factor = cpu_slowdown_factor  # performance multiplier for embedded cpu simulation
        
    def heuristic(self, x: int, y: int, goal_x: int, goal_y: int) -> float:
        # heuristic function (manhattan distance)
        return abs(x - goal_x) + abs(y - goal_y)
    
    def reconstruct_path(self, node: SearchNode) -> List[Tuple[int, int]]:
        # reconstruct path from goal node to start
        path = []
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path
    
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int]) -> SearchResult:
        # find path from start to goal using a*
        start_time = time.perf_counter()
        start_x, start_y = start
        goal_x, goal_y = goal
        
        # initialize
        start_node = SearchNode(
            x=start_x,
            y=start_y,
            g=0,
            h=self.heuristic(start_x, start_y, goal_x, goal_y)
        )
        
        open_set = [start_node]
        closed_set: Set[Tuple[int, int]] = set()
        g_scores: Dict[Tuple[int, int], float] = {(start_x, start_y): 0}
        
        nodes_expanded = 0
        
        while open_set:
            # get node with lowest f-score
            current = heapq.heappop(open_set)
            nodes_expanded += 1
            
            # simulate embedded cpu performance if enabled
            if self.cpu_slowdown_factor > 1.0:
                # busy-wait to simulate slower cpu (more accurate than sleep)
                target_time = time.perf_counter() + (0.0001 * (self.cpu_slowdown_factor - 1.0))
                while time.perf_counter() < target_time:
                    pass
            
            # check if goal reached
            if current.x == goal_x and current.y == goal_y:
                elapsed_ms = (time.perf_counter() - start_time) * 1000
                path = self.reconstruct_path(current)
                return SearchResult(
                    path=path,
                    success=True,
                    nodes_expanded=nodes_expanded,
                    time_ms=elapsed_ms,
                    hit_deadline=False,
                    path_cost=current.g,
                    best_f_score=current.f()
                )
            
            # skip if already processed
            if (current.x, current.y) in closed_set:
                continue
                
            closed_set.add((current.x, current.y))
            
            # expand neighbors
            for nx, ny in self.world.get_neighbors(current.x, current.y):
                if (nx, ny) in closed_set:
                    continue
                
                tentative_g = current.g + 1  # uniform cost
                
                # check if this is a better path
                if (nx, ny) not in g_scores or tentative_g < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = tentative_g
                    
                    neighbor = SearchNode(
                        x=nx,
                        y=ny,
                        g=tentative_g,
                        h=self.heuristic(nx, ny, goal_x, goal_y),
                        parent=current
                    )
                    heapq.heappush(open_set, neighbor)
        
        # no path found
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            path=None,
            success=False,
            nodes_expanded=nodes_expanded,
            time_ms=elapsed_ms,
            hit_deadline=False,
            path_cost=float('inf'),
            best_f_score=float('inf')
        )


# deadline-aware a* planner
# returns best-so-far path if deadline is exceeded
class DeadlineAwarePlanner(AStarPlanner):
    
    def plan(self, start: Tuple[int, int], goal: Tuple[int, int], 
             deadline_ms: float = 50.0) -> SearchResult:
        # find path with deadline constraint
        start_time = time.perf_counter()
        start_x, start_y = start
        goal_x, goal_y = goal
        
        # initialize
        start_node = SearchNode(
            x=start_x,
            y=start_y,
            g=0,
            h=self.heuristic(start_x, start_y, goal_x, goal_y)
        )
        
        open_set = [start_node]
        closed_set: Set[Tuple[int, int]] = set()
        g_scores: Dict[Tuple[int, int], float] = {(start_x, start_y): 0}
        
        nodes_expanded = 0
        best_node = start_node  # track best node found so far
        best_complete_path = None  # track best complete path if found
        
        while open_set:
            # check deadline
            elapsed_ms = (time.perf_counter() - start_time) * 1000
            if elapsed_ms > deadline_ms:
                # deadline exceeded - return best-so-far
                if best_complete_path is not None:
                    # we found at least one complete path
                    return SearchResult(
                        path=best_complete_path,
                        success=True,
                        nodes_expanded=nodes_expanded,
                        time_ms=elapsed_ms,
                        hit_deadline=True,
                        path_cost=len(best_complete_path) - 1,
                        best_f_score=best_node.f()
                    )
                else:
                    # return partial path to most promising node
                    path = self.reconstruct_path(best_node)
                    return SearchResult(
                        path=path,
                        success=False,
                        nodes_expanded=nodes_expanded,
                        time_ms=elapsed_ms,
                        hit_deadline=True,
                        path_cost=best_node.g,
                        best_f_score=best_node.f()
                    )
            
            # get node with lowest f-score
            current = heapq.heappop(open_set)
            nodes_expanded += 1
            
            # simulate embedded cpu performance if enabled
            if self.cpu_slowdown_factor > 1.0:
                # busy-wait to simulate slower cpu (more accurate than sleep)
                target_time = time.perf_counter() + (0.0001 * (self.cpu_slowdown_factor - 1.0))
                while time.perf_counter() < target_time:
                    pass
            
            # update best node (closest to goal by f-score)
            if current.f() < best_node.f():
                best_node = current
            
            # check if goal reached
            if current.x == goal_x and current.y == goal_y:
                elapsed_ms = (time.perf_counter() - start_time) * 1000
                path = self.reconstruct_path(current)
                return SearchResult(
                    path=path,
                    success=True,
                    nodes_expanded=nodes_expanded,
                    time_ms=elapsed_ms,
                    hit_deadline=False,
                    path_cost=current.g,
                    best_f_score=current.f()
                )
            
            # skip if already processed
            if (current.x, current.y) in closed_set:
                continue
                
            closed_set.add((current.x, current.y))
            
            # expand neighbors
            for nx, ny in self.world.get_neighbors(current.x, current.y):
                if (nx, ny) in closed_set:
                    continue
                
                tentative_g = current.g + 1
                
                if (nx, ny) not in g_scores or tentative_g < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = tentative_g
                    
                    neighbor = SearchNode(
                        x=nx,
                        y=ny,
                        g=tentative_g,
                        h=self.heuristic(nx, ny, goal_x, goal_y),
                        parent=current
                    )
                    heapq.heappush(open_set, neighbor)
        
        # no path found and no deadline hit
        elapsed_ms = (time.perf_counter() - start_time) * 1000
        return SearchResult(
            path=None,
            success=False,
            nodes_expanded=nodes_expanded,
            time_ms=elapsed_ms,
            hit_deadline=False,
            path_cost=float('inf'),
            best_f_score=float('inf')
        )


# neural-guided a* planner
# uses learned heuristic to guide search
class NeuralGuidedPlanner(DeadlineAwarePlanner):
    
    def __init__(self, world: GridWorld, neural_ranker: Optional[Callable] = None, cpu_slowdown_factor: float = 1.0):
        super().__init__(world, cpu_slowdown_factor=cpu_slowdown_factor)
        self.neural_ranker = neural_ranker
        
    def heuristic(self, x: int, y: int, goal_x: int, goal_y: int) -> float:
        # hybrid heuristic combining manhattan distance and neural guidance
        base_h = abs(x - goal_x) + abs(y - goal_y)
        
        if self.neural_ranker is None:
            return base_h
        
        # get neural score
        neural_score = self.neural_ranker(x, y, goal_x, goal_y, self.world)
        
        # combine: use neural score as adjustment to base heuristic
        # lower neural score = better node
        return base_h + neural_score