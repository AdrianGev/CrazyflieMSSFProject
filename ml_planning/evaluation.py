import numpy as np
from typing import List, Dict, Tuple, Optional
import time

from ml_planning.grid_world import GridWorld
from ml_planning.astar_planner import AStarPlanner, DeadlineAwarePlanner, NeuralGuidedPlanner, SearchResult


# metrics for evaluating planner performance
class EvaluationMetrics:
    def __init__(self, planner_name: str):
        self.planner_name = planner_name
        self.num_trials = 0
        self.successes = 0
        self.deadline_hits = 0
        self.total_nodes_expanded = 0
        self.total_time_ms = 0.0
        self.path_costs = []
        self.optimal_costs = []
        # detailed per-trial data for graph generation
        self.trial_data = []  # list of dicts with all trial details
    
    def success_rate(self) -> float:
        # fraction of trials that found a path to goal
        return self.successes / self.num_trials if self.num_trials > 0 else 0.0
    
    def deadline_hit_rate(self) -> float:
        # fraction of trials that exceeded deadline
        return self.deadline_hits / self.num_trials if self.num_trials > 0 else 0.0
    
    def avg_nodes_expanded(self) -> float:
        # average nodes expanded per trial
        return self.total_nodes_expanded / self.num_trials if self.num_trials > 0 else 0.0
    
    def avg_time_ms(self) -> float:
        # average time per trial in milliseconds
        return self.total_time_ms / self.num_trials if self.num_trials > 0 else 0.0
    
    def avg_path_inflation(self) -> float:
        # average path inflation: returned_cost / optimal_cost
        # 1.0 = optimal, >1.0 = suboptimal
        if not self.path_costs or not self.optimal_costs:
            return float('inf')
        
        inflations = []
        for path_cost, opt_cost in zip(self.path_costs, self.optimal_costs):
            if opt_cost > 0 and path_cost < float('inf'):
                inflations.append(path_cost / opt_cost)
        
        return np.mean(inflations) if inflations else float('inf')
    
    def summary(self) -> str:
        # generate summary string
        lines = [
            f"\n{'='*60}",
            f"planner: {self.planner_name}",
            f"{'='*60}",
            f"trials:              {self.num_trials}",
            f"success rate:        {self.success_rate():.1%}",
            f"deadline hit rate:   {self.deadline_hit_rate():.1%}",
            f"avg nodes expanded:  {self.avg_nodes_expanded():.1f}",
            f"avg time (ms):       {self.avg_time_ms():.2f}",
            f"avg path inflation:  {self.avg_path_inflation():.3f}",
            f"{'='*60}\n"
        ]
        return '\n'.join(lines)


# evaluates and compares different planners on test scenarios
class PlannerEvaluator:
    
    def __init__(self, grid_width: int = 4, grid_height: int = 7):
        self.grid_width = grid_width
        self.grid_height = grid_height
        
    def generate_test_scenario(self, num_obstacles: int, 
                               min_distance: int = 5) -> Tuple[GridWorld, Tuple[int, int], Tuple[int, int]]:
        # generate single test scenario
        while True:
            # random start and goal
            start_x = np.random.randint(0, self.grid_width)
            start_y = np.random.randint(0, self.grid_height)
            goal_x = np.random.randint(0, self.grid_width)
            goal_y = np.random.randint(0, self.grid_height)
            
            # check distance
            dist = abs(start_x - goal_x) + abs(start_y - goal_y)
            if dist >= min_distance:
                break
        
        # create world with obstacles
        world = GridWorld(self.grid_width, self.grid_height)
        
        avoid = {(start_x, start_y), (goal_x, goal_y)}
        for dx, dy in [(0, -1), (1, 0), (0, 1), (-1, 0)]:
            avoid.add((start_x + dx, start_y + dy))
            avoid.add((goal_x + dx, goal_y + dy))
        
        world.add_random_obstacles(num_obstacles, avoid_positions=avoid)
        
        return world, (start_x, start_y), (goal_x, goal_y)
    
    def evaluate_planner(self, 
                        planner_name: str,
                        planner_factory,
                        num_scenarios: int = 100,
                        obstacle_range: Tuple[int, int] = (5, 20),
                        deadline_ms: Optional[float] = None,
                        verbose: bool = True) -> EvaluationMetrics:
        # evaluate a planner on multiple scenarios
        metrics = EvaluationMetrics(planner_name=planner_name)
        
        if verbose:
            print(f"\nevaluating {planner_name} on {num_scenarios} scenarios...")
        
        for i in range(num_scenarios):
            # generate scenario
            num_obstacles = np.random.randint(obstacle_range[0], obstacle_range[1] + 1)
            world, start, goal = self.generate_test_scenario(num_obstacles)
            
            # get optimal cost for comparison
            optimal_planner = AStarPlanner(world)
            optimal_result = optimal_planner.plan(start, goal)
            
            if not optimal_result.success:
                continue  # skip unsolvable scenarios
            
            optimal_cost = optimal_result.path_cost
            
            # test planner
            planner = planner_factory(world)
            
            if deadline_ms is not None:
                result = planner.plan(start, goal, deadline_ms=deadline_ms)
            else:
                result = planner.plan(start, goal)
            
            # record metrics
            metrics.num_trials += 1
            if result.success:
                metrics.successes += 1
            if result.hit_deadline:
                metrics.deadline_hits += 1
            
            metrics.total_nodes_expanded += result.nodes_expanded
            metrics.total_time_ms += result.time_ms
            
            # only record path costs for successful complete paths
            if result.success and result.path is not None:
                metrics.path_costs.append(result.path_cost)
                metrics.optimal_costs.append(optimal_cost)
            
            # record detailed trial data for graph generation
            # only calculate inflation for successful paths that reached the goal
            path_inflation = None
            if result.success and result.path and optimal_cost > 0:
                path_inflation = result.path_cost / optimal_cost
            
            metrics.trial_data.append({
                'trial_num': metrics.num_trials,
                'num_obstacles': num_obstacles,
                'success': result.success,
                'hit_deadline': result.hit_deadline,
                'nodes_expanded': result.nodes_expanded,
                'time_ms': result.time_ms,
                'path_cost': result.path_cost if result.path else None,
                'optimal_cost': optimal_cost,
                'path_inflation': path_inflation
            })
            
            if verbose and (i + 1) % 20 == 0:
                print(f"  progress: {i+1}/{num_scenarios} scenarios")
        
        if verbose:
            print(metrics.summary())
        
        return metrics
    
    def compare_planners(self,
                        planners: Dict[str, Tuple],
                        num_scenarios: int = 100,
                        obstacle_range: Tuple[int, int] = (5, 20),
                        deadline_ms: float = 50.0) -> Dict[str, EvaluationMetrics]:
        # compare multiple planners on same scenarios
        results = {}
        
        print(f"\n{'='*60}")
        print(f"planner comparison")
        print(f"{'='*60}")
        print(f"scenarios: {num_scenarios}")
        print(f"obstacles: {obstacle_range[0]}-{obstacle_range[1]}")
        print(f"deadline: {deadline_ms} ms")
        
        for planner_name, (factory, use_deadline) in planners.items():
            dl = deadline_ms if use_deadline else None
            metrics = self.evaluate_planner(
                planner_name=planner_name,
                planner_factory=factory,
                num_scenarios=num_scenarios,
                obstacle_range=obstacle_range,
                deadline_ms=dl,
                verbose=True
            )
            results[planner_name] = metrics
        
        # print comparison summary
        print(f"\n{'='*60}")
        print(f"comparison summary")
        print(f"{'='*60}")
        print(f"{'planner':<25} {'success%':<12} {'deadlinehit%':<15} {'avgnodes':<12} {'inflation':<12}")
        print(f"{'-'*60}")
        
        for name, metrics in results.items():
            print(f"{name:<25} {metrics.success_rate()*100:>10.1f}% "
                  f"{metrics.deadline_hit_rate()*100:>13.1f}% "
                  f"{metrics.avg_nodes_expanded():>10.1f}  "
                  f"{metrics.avg_path_inflation():>10.3f}")
        
        print(f"{'='*60}\n")
        
        return results


# evaluates planners in dynamic environments with moving obstacles
class DynamicEnvironmentEvaluator(PlannerEvaluator):
    
    def simulate_dynamic_scenario(self,
                                  planner_factory,
                                  initial_obstacles: int = 10,
                                  num_replans: int = 5,
                                  deadline_ms: float = 50.0) -> Dict:
        # simulate dynamic replanning scenario
        
        # generate initial scenario
        world, start, goal = self.generate_test_scenario(initial_obstacles)
        
        current_pos = start
        total_nodes = 0
        total_time = 0.0
        replans_successful = 0
        path_segments = []
        
        for replan_idx in range(num_replans):
            # plan from current position
            planner = planner_factory(world)
            result = planner.plan(current_pos, goal, deadline_ms=deadline_ms)
            
            total_nodes += result.nodes_expanded
            total_time += result.time_ms
            
            if result.success and result.path:
                replans_successful += 1
                
                # simulate moving along path for a few steps
                steps_to_take = min(3, len(result.path) - 1)
                if steps_to_take > 0:
                    current_pos = result.path[steps_to_take]
                    path_segments.append(result.path[:steps_to_take + 1])
                
                # check if reached goal
                if current_pos == goal:
                    break
                
                # modify environment (add/remove obstacles)
                if replan_idx < num_replans - 1:
                    # remove one obstacle
                    obstacles = [(x, y) for y in range(world.height) 
                                for x in range(world.width) if world.is_obstacle(x, y)]
                    if obstacles:
                        x, y = obstacles[np.random.randint(len(obstacles))]
                        world.clear_obstacle(x, y)
                    
                    # add one obstacle
                    avoid = {current_pos, goal}
                    world.add_random_obstacles(1, avoid_positions=avoid)
            else:
                break
        
        return {
            'success': current_pos == goal,
            'replans_attempted': num_replans,
            'replans_successful': replans_successful,
            'total_nodes_expanded': total_nodes,
            'total_time_ms': total_time,
            'final_position': current_pos,
            'goal': goal
        }