import numpy as np
from typing import List, Tuple, Dict, Set
import pickle
# might as well import cucumber while we're at it lol
from ml_planning.grid_world import GridWorld
from ml_planning.astar_planner import AStarPlanner
# training data generator for neural ranker
# generates grid maps and optimal paths for supervised learning
# single training example for neural ranker
class TrainingExample:
    def __init__(self, node_x: int, node_y: int, goal_x: int, goal_y: int,
                 local_patch: np.ndarray, cost_to_go: float, 
                 on_optimal_path: bool, distance_to_goal: int):
        self.node_x = node_x
        self.node_y = node_y
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.local_patch = local_patch  # 5x5 local obstacle map
        self.cost_to_go = cost_to_go  # true remaining cost to goal
        self.on_optimal_path = on_optimal_path  # whether node is on optimal path
        self.distance_to_goal = distance_to_goal  # manhattan distance to goal


# generates training data by running optimal a* on random maps
# for each map:
# 1. generate random obstacles
# 2. run optimal a* from start to goal
# 3. record which nodes were expanded
# 4. label nodes with true cost-to-go and whether on optimal path
class DataGenerator:
    
    def __init__(self, grid_width: int = 4, grid_height: int = 7, patch_size: int = 5):
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.patch_size = patch_size
        
    def generate_random_map(self, num_obstacles: int, 
                           start: Tuple[int, int], 
                           goal: Tuple[int, int]) -> GridWorld:
        # generate random grid map with obstacles
        world = GridWorld(self.grid_width, self.grid_height)
        
        # avoid start, goal, and their immediate neighbors
        avoid = {start, goal}
        for dx, dy in [(0, -1), (1, 0), (0, 1), (-1, 0)]:
            avoid.add((start[0] + dx, start[1] + dy))
            avoid.add((goal[0] + dx, goal[1] + dy))
        
        world.add_random_obstacles(num_obstacles, avoid_positions=avoid)
        return world
    
    def extract_examples_from_search(self, 
                                     world: GridWorld,
                                     start: Tuple[int, int],
                                     goal: Tuple[int, int]) -> List[TrainingExample]:
        # run optimal a* and extract training examples
        planner = AStarPlanner(world)
        result = planner.plan(start, goal)
        
        if not result.success or result.path is None:
            return []
        
        # build set of nodes on optimal path
        optimal_path_set = set(result.path)
        
        # build map (distance from each node to goal along optimal path)
        cost_to_go_map: Dict[Tuple[int, int], float] = {}
        for i, (x, y) in enumerate(result.path):
            cost_to_go_map[(x, y)] = len(result.path) - 1 - i
        
        # now run a* again to collect all expanded nodes
        # we need to track which nodes were visited during search
        examples = []
        
        # re-run search and collect examples
        start_x, start_y = start
        goal_x, goal_y = goal
        
        from ml_planning.astar_planner import SearchNode
        import heapq
        
        start_node = SearchNode(
            x=start_x,
            y=start_y,
            g=0,
            h=planner.heuristic(start_x, start_y, goal_x, goal_y)
        )
        
        open_set = [start_node]
        closed_set: Set[Tuple[int, int]] = set()
        g_scores: Dict[Tuple[int, int], float] = {(start_x, start_y): 0}
        
        while open_set:
            current = heapq.heappop(open_set)
            
            if (current.x, current.y) in closed_set:
                continue
            
            closed_set.add((current.x, current.y))
            
            # create training example for this node
            node_pos = (current.x, current.y)
            
            # get true cost-to-go
            if node_pos in cost_to_go_map:
                true_cost = cost_to_go_map[node_pos]
            else:
                # node not on optimal path - estimate cost as remaining distance
                true_cost = world.manhattan_distance(current.x, current.y, goal_x, goal_y)
            
            # extract local patch
            local_patch = world.get_local_patch(current.x, current.y, self.patch_size)
            
            example = TrainingExample(
                node_x=current.x,
                node_y=current.y,
                goal_x=goal_x,
                goal_y=goal_y,
                local_patch=local_patch,
                cost_to_go=true_cost,
                on_optimal_path=(node_pos in optimal_path_set),
                distance_to_goal=world.manhattan_distance(current.x, current.y, goal_x, goal_y)
            )
            examples.append(example)
            
            # stop if goal reached
            if current.x == goal_x and current.y == goal_y:
                break
            
            # expand neighbors
            for nx, ny in world.get_neighbors(current.x, current.y):
                if (nx, ny) in closed_set:
                    continue
                
                tentative_g = current.g + 1
                
                if (nx, ny) not in g_scores or tentative_g < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = tentative_g
                    
                    neighbor = SearchNode(
                        x=nx,
                        y=ny,
                        g=tentative_g,
                        h=planner.heuristic(nx, ny, goal_x, goal_y),
                        parent=current
                    )
                    heapq.heappush(open_set, neighbor)
        
        return examples
    
    def generate_dataset(self, 
                        num_maps: int = 1000,
                        min_obstacles: int = 5,
                        max_obstacles: int = 20) -> List[TrainingExample]:
        # generate full training dataset
        all_examples = []
        successful_maps = 0
        
        print(f"generating training data from {num_maps} maps...")
        
        for i in range(num_maps):
            # random start and goal
            start_x = np.random.randint(0, self.grid_width)
            start_y = np.random.randint(0, self.grid_height)
            
            goal_x = np.random.randint(0, self.grid_width)
            goal_y = np.random.randint(0, self.grid_height)
            
            # ensure start and goal are different and reasonably far apart
            if (start_x, start_y) == (goal_x, goal_y):
                continue
            
            manhattan_dist = abs(start_x - goal_x) + abs(start_y - goal_y)
            if manhattan_dist < 4:  # too close
                continue
            
            # random number of obstacles
            num_obstacles = np.random.randint(min_obstacles, max_obstacles + 1)
            
            # generate map
            world = self.generate_random_map(num_obstacles, (start_x, start_y), (goal_x, goal_y))
            
            # extract examples
            examples = self.extract_examples_from_search(world, (start_x, start_y), (goal_x, goal_y))
            
            if examples:
                all_examples.extend(examples)
                successful_maps += 1
            
            if (i + 1) % 100 == 0:
                print(f"  processed {i + 1}/{num_maps} maps, {successful_maps} successful, {len(all_examples)} examples")
        
        print(f"generated {len(all_examples)} training examples from {successful_maps} successful maps")
        return all_examples
    
    def save_dataset(self, examples: List[TrainingExample], filename: str):
        # save dataset to file
        with open(filename, 'wb') as f:
            pickle.dump(examples, f)
        print(f"saved {len(examples)} examples to {filename}")
    
    def load_dataset(self, filename: str) -> List[TrainingExample]:
        # load dataset from file
        with open(filename, 'rb') as f:
            examples = pickle.load(f)
        print(f"loaded {len(examples)} examples from {filename}")
        return examples


def examples_to_arrays(examples: List[TrainingExample]) -> Tuple[np.ndarray, np.ndarray]:
    # convert training examples to numpy arrays for training
    # returns (features, targets) tuple
    # features: (N, feature_dim) array
    # targets: (N,) array of cost-to-go values
    N = len(examples)
    patch_size = examples[0].local_patch.shape[0]
    
    # feature vector: [node_x, node_y, goal_x, goal_y, dx, dy, dist, flattened_patch]
    feature_dim = 7 + (patch_size * patch_size)
    
    features = np.zeros((N, feature_dim), dtype=np.float32)
    targets = np.zeros(N, dtype=np.float32)
    
    for i, ex in enumerate(examples):
        dx = ex.goal_x - ex.node_x
        dy = ex.goal_y - ex.node_y
        
        features[i, 0] = ex.node_x
        features[i, 1] = ex.node_y
        features[i, 2] = ex.goal_x
        features[i, 3] = ex.goal_y
        features[i, 4] = dx
        features[i, 5] = dy
        features[i, 6] = ex.distance_to_goal
        features[i, 7:] = ex.local_patch.flatten()
        
        targets[i] = ex.cost_to_go
    
    return features, targets