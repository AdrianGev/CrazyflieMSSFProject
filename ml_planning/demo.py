# demos training, evaluation, and basic usage
import numpy as np
from ml_planning import (
    GridWorld, 
    AStarPlanner, 
    DeadlineAwarePlanner, 
    NeuralGuidedPlanner,
    DataGenerator,
    NeuralRanker,
    PlannerEvaluator
)


def example_basic_planning():
    # example 1: basic a* planning on a simple grid
    print("\n" + "="*60)
    print("EXAMPLE 1: Basic A* Planning")
    print("="*60)
    
    # create a simple grid world
    world = GridWorld(width=4, height=7)
    
    # add some obstacles
    world.set_obstacle(1, 2)
    world.set_obstacle(1, 3)
    world.set_obstacle(1, 4)
    world.set_obstacle(2, 3)
    
    print("\nGrid world:")
    print(world)
    
    # create planner
    planner = AStarPlanner(world)
    
    # plan from (0,0) to (3,6)
    start = (0, 0)
    goal = (3, 6)
    
    result = planner.plan(start, goal)
    
    print(f"\nPlanning from {start} to {goal}")
    print(f"Success: {result.success}")
    print(f"Path length: {result.path_length}")
    print(f"Path cost: {result.path_cost}")
    print(f"Nodes expanded: {result.nodes_expanded}")
    print(f"Time: {result.time_ms:.2f} ms")
    
    if result.path:
        print(f"Path: {result.path[:5]}..." if len(result.path) > 5 else f"Path: {result.path}")


def example_deadline_aware():
    # example 2: deadline-aware planning
    print("\n" + "="*60)
    print("EXAMPLE 2: Deadline-Aware Planning")
    print("="*60)
    
    # create a complex grid with many obstacles
    world = GridWorld(width=4, height=7)
    world.add_random_obstacles(8, avoid_positions={(0, 0), (3, 6)})
    
    print("\nGrid world with random obstacles:")
    print(world)
    
    # test with different deadlines
    planner = DeadlineAwarePlanner(world)
    start = (0, 0)
    goal = (3, 6)
    
    for deadline_ms in [5.0, 10.0, 50.0]:
        result = planner.plan(start, goal, deadline_ms=deadline_ms)
        
        print(f"\nDeadline: {deadline_ms} ms")
        print(f"  Success: {result.success}")
        print(f"  Hit deadline: {result.hit_deadline}")
        print(f"  Nodes expanded: {result.nodes_expanded}")
        print(f"  Time: {result.time_ms:.2f} ms")
        print(f"  Path length: {result.path_length}")


def example_training_pipeline():
    # example 3: complete training pipeline
    print("\n" + "="*60)
    print("EXAMPLE 3: Training Pipeline")
    print("="*60)
    
    # generate small training dataset
    print("\nGenerating training data...")
    generator = DataGenerator(grid_width=4, grid_height=7)
    
    train_examples = generator.generate_dataset(
        num_maps=100,  # small dataset for demo
        min_obstacles=5,
        max_obstacles=15
    )
    
    val_examples = generator.generate_dataset(
        num_maps=20,
        min_obstacles=5,
        max_obstacles=15
    )
    
    print(f"Training examples: {len(train_examples)}")
    print(f"Validation examples: {len(val_examples)}")
    
    # train neural ranker
    print("\nTraining neural ranker...")
    input_dim = 7 + (5 * 5)
    
    ranker = NeuralRanker(input_dim=input_dim, hidden_dims=[64, 32])
    
    ranker.train(
        train_examples=train_examples,
        val_examples=val_examples,
        epochs=10,  # few epochs for demo
        batch_size=128,
        learning_rate=0.001,
        verbose=True
    )
    
    print("\nTraining complete!")
    
    # test the trained ranker
    world = GridWorld(width=4, height=7)
    world.add_random_obstacles(5, avoid_positions={(0, 0), (3, 6)})
    
    score = ranker.score_node(2, 3, 3, 6, world)
    print(f"\nExample prediction: score_node(2, 3, goal=(3,6)) = {score:.2f}")


def example_planner_comparison():
    # example 4: compare different planners
    print("\n" + "="*60)
    print("EXAMPLE 4: Planner Comparison")
    print("="*60)
    
    # create evaluator
    evaluator = PlannerEvaluator(grid_width=4, grid_height=7)
    
    # define planners to compare
    planners = {
        'Baseline A*': (
            lambda world: AStarPlanner(world),
            False
        ),
        'Deadline-Aware A*': (
            lambda world: DeadlineAwarePlanner(world),
            True
        )
    }
    
    # run comparison
    print("\nRunning comparison on 20 test scenarios...")
    results = evaluator.compare_planners(
        planners=planners,
        num_scenarios=20,
        obstacle_range=(3, 10),
        deadline_ms=50.0
    )
    
    # results are automatically printed by compare_planners


def example_neural_guided():
    # example 5: using neural-guided planner (requires trained model)
    print("\n" + "="*60)
    print("EXAMPLE 5: Neural-Guided Planning")
    print("="*60)
    
    # first train a quick model
    print("\nTraining a simple neural ranker...")
    generator = DataGenerator(grid_width=4, grid_height=7)
    train_examples = generator.generate_dataset(num_maps=50, min_obstacles=3, max_obstacles=10)
    
    input_dim = 7 + (5 * 5)
    ranker = NeuralRanker(input_dim=input_dim, hidden_dims=[64, 32])
    ranker.train(train_examples, epochs=5, batch_size=128, verbose=False)
    
    # create world
    world = GridWorld(width=4, height=7)
    world.add_random_obstacles(5, avoid_positions={(0, 0), (3, 6)})
    
    print("\nGrid world:")
    print(world)
    
    # create neural scorer function
    def neural_scorer(x, y, goal_x, goal_y, world):
        return ranker.score_node(x, y, goal_x, goal_y, world)
    
    # create neural-guided planner
    planner = NeuralGuidedPlanner(world, neural_ranker=neural_scorer)
    
    # plan with deadline
    start = (0, 0)
    goal = (3, 6)
    result = planner.plan(start, goal, deadline_ms=50.0)
    
    print(f"\nNeural-guided planning from {start} to {goal}")
    print(f"Success: {result.success}")
    print(f"Path length: {result.path_length}")
    print(f"Nodes expanded: {result.nodes_expanded}")
    print(f"Time: {result.time_ms:.2f} ms")
    print(f"Hit deadline: {result.hit_deadline}")
    
    # compare with baseline
    baseline = AStarPlanner(world)
    baseline_result = baseline.plan(start, goal)
    
    print(f"\nComparison with baseline A*:")
    print(f"  Baseline nodes: {baseline_result.nodes_expanded}")
    print(f"  Neural nodes: {result.nodes_expanded}")
    print(f"  Efficiency gain: {(1 - result.nodes_expanded/baseline_result.nodes_expanded)*100:.1f}%")


def main():
    # run all examples
    print("\n" + "="*60)
    print("ml pathfinding examples")
    print("="*60)
    
    # set random seed for reproducibility
    np.random.seed(42)
    
    # run examples
    example_basic_planning()
    example_deadline_aware()
    example_planner_comparison()
    example_neural_guided()
    
    # note: example_training_pipeline() takes longer, uncomment to run:
    # example_training_pipeline()
    
    print("\n" + "="*60)
    print("all examples complete")
    print("="*60)
    print("\nnext steps:")
    print("1. train a full model: python -m ml_planning.train_ranker")
    print("2. evaluate planners: python -m ml_planning.evaluate_planners")
    print("3. integrate with your crazyflie system")
    print("="*60 + "\n")


if __name__ == '__main__':
    main()