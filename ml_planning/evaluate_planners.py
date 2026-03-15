import argparse
import os

from ml_planning.grid_world import GridWorld
from ml_planning.astar_planner import AStarPlanner, DeadlineAwarePlanner, NeuralGuidedPlanner
from ml_planning.neural_ranker import NeuralRanker
from ml_planning.evaluation import PlannerEvaluator


def main():
    parser = argparse.ArgumentParser(description='Evaluate and compare pathfinding planners')
    parser.add_argument('--num_scenarios', type=int, default=100,
                       help='Number of test scenarios')
    parser.add_argument('--min_obstacles', type=int, default=5,
                       help='Minimum obstacles per scenario')
    parser.add_argument('--max_obstacles', type=int, default=20,
                       help='Maximum obstacles per scenario')
    parser.add_argument('--deadline_ms', type=float, default=10.0,
                       help='Deadline in milliseconds')
    parser.add_argument('--model_path', type=str, default='ml_planning/models/neural_ranker.pt',
                       help='Path to trained neural ranker model')
    parser.add_argument('--grid_width', type=int, default=4,
                       help='Grid width')
    parser.add_argument('--grid_height', type=int, default=7,
                       help='Grid height')
    parser.add_argument('--planners', type=str, nargs='+', 
                       default=['baseline', 'deadline', 'neural'],
                       help='Planners to evaluate: baseline, deadline, neural')
    parser.add_argument('--export_csv', action='store_true',
                       help='Export evaluation results to CSV')
    parser.add_argument('--export_dir', type=str, default='export_data',
                       help='Directory for CSV exports')
    parser.add_argument('--crazyflie', action='store_true',
                       help='Simulate Crazyflie STM32F405 (168MHz Cortex-M4) performance')
    
    args = parser.parse_args()
    
    # calculate cpu slowdown factor for crazyflie simulation
    # stm32f405: 168mhz cortex-m4 vs typical laptop: ~3-4ghz modern cpu
    # rough estimate: ~20-25x slower for integer ops, ~50-100x for floating point
    # conservative estimate for pathfinding workload: ~30x slowdown
    cpu_slowdown = 30.0 if args.crazyflie else 1.0
    
    print("="*60)
    print("planner evaluation")
    print("="*60)
    print(f"Grid size: {args.grid_width}x{args.grid_height}")
    print(f"Test scenarios: {args.num_scenarios}")
    print(f"Obstacles: {args.min_obstacles}-{args.max_obstacles}")
    print(f"Deadline: {args.deadline_ms} ms")
    if args.crazyflie:
        print(f"CPU simulation: Crazyflie STM32F405 (168MHz Cortex-M4, ~{cpu_slowdown:.0f}x slowdown)")
    print("="*60)
    
    # initialize evaluator
    evaluator = PlannerEvaluator(
        grid_width=args.grid_width,
        grid_height=args.grid_height
    )
    
    # setup planners to compare
    planners = {}
    
    if 'baseline' in args.planners:
        planners['Baseline A*'] = (
            lambda world: AStarPlanner(world, cpu_slowdown_factor=cpu_slowdown),
            False  # no deadline
        )
    
    if 'deadline' in args.planners:
        planners['Deadline-Aware A*'] = (
            lambda world: DeadlineAwarePlanner(world, cpu_slowdown_factor=cpu_slowdown),
            True  # use deadline
        )
    
    if 'neural' in args.planners:
        # load neural ranker
        if os.path.exists(args.model_path):
            print(f"\nLoading neural ranker from {args.model_path}...")
            ranker = NeuralRanker.create_from_file(args.model_path)
            
            # create scoring function
            def neural_scorer(x, y, goal_x, goal_y, world):
                return ranker.score_node(x, y, goal_x, goal_y, world)
            
            planners['Neural-Guided A*'] = (
                lambda world: NeuralGuidedPlanner(world, neural_ranker=neural_scorer, cpu_slowdown_factor=cpu_slowdown),
                True  # use deadline
            )
        else:
            print(f"\nwarning: neural ranker model not found at {args.model_path}")
            print("skipping neural-guided planner evaluation.")
    
    # run comparison
    results = evaluator.compare_planners(
        planners=planners,
        num_scenarios=args.num_scenarios,
        obstacle_range=(args.min_obstacles, args.max_obstacles),
        deadline_ms=args.deadline_ms
    )
    
    # additional analysis
    print("\ndetailed analysis")
    print("="*60)
    
    for name, metrics in results.items():
        print(f"\n{name}:")
        print(f"  Total trials:        {metrics.num_trials}")
        print(f"  Successful:          {metrics.successes} ({metrics.success_rate():.1%})")
        print(f"  Hit deadline:        {metrics.deadline_hits} ({metrics.deadline_hit_rate():.1%})")
        print(f"  Avg nodes expanded:  {metrics.avg_nodes_expanded():.1f}")
        print(f"  Avg time (ms):       {metrics.avg_time_ms():.2f}")
        print(f"  Avg path inflation:  {metrics.avg_path_inflation():.3f}")
        
        if metrics.path_costs:
            print(f"  Min path cost:       {min(metrics.path_costs):.1f}")
            print(f"  Max path cost:       {max(metrics.path_costs):.1f}")
    
    # export results to csv if requested
    if args.export_csv:
        import csv
        
        os.makedirs(args.export_dir, exist_ok=True)
        
        # export 1: summary results (original)
        csv_file = os.path.join(args.export_dir, 'summary.csv')
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'planner', 'trials', 'successes', 'success_rate', 
                'deadline_hits', 'deadline_hit_rate', 'avg_nodes_expanded',
                'avg_time_ms', 'avg_path_inflation', 'min_path_cost', 'max_path_cost'
            ])
            writer.writeheader()
            
            for name, metrics in results.items():
                writer.writerow({
                    'planner': name,
                    'trials': metrics.num_trials,
                    'successes': metrics.successes,
                    'success_rate': metrics.success_rate(),
                    'deadline_hits': metrics.deadline_hits,
                    'deadline_hit_rate': metrics.deadline_hit_rate(),
                    'avg_nodes_expanded': metrics.avg_nodes_expanded(),
                    'avg_time_ms': metrics.avg_time_ms(),
                    'avg_path_inflation': metrics.avg_path_inflation(),
                    'min_path_cost': min(metrics.path_costs) if metrics.path_costs else None,
                    'max_path_cost': max(metrics.path_costs) if metrics.path_costs else None
                })
        
        print(f"\nsummary exported to {csv_file}")
        
        # export 2: per-trial data (for all graphs)
        trial_file = os.path.join(args.export_dir, 'trials.csv')
        with open(trial_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'planner', 'trial_num', 'num_obstacles', 'success', 'hit_deadline',
                'nodes_expanded', 'time_ms', 'path_cost', 'optimal_cost', 'path_inflation'
            ])
            writer.writeheader()
            
            for name, metrics in results.items():
                for trial in metrics.trial_data:
                    writer.writerow({
                        'planner': name,
                        **trial
                    })
        
        print(f"per-trial data exported to {trial_file}")
        
        # export 3: deadline hit-rate by obstacle count (for fig 2)
        deadline_file = os.path.join(args.export_dir, 'fig2_deadline_by_obstacles.csv')
        with open(deadline_file, 'w', newline='') as f:
            # group by obstacle count
            obstacle_groups = {}
            for name, metrics in results.items():
                for trial in metrics.trial_data:
                    obs = trial['num_obstacles']
                    if obs not in obstacle_groups:
                        obstacle_groups[obs] = {}
                    if name not in obstacle_groups[obs]:
                        obstacle_groups[obs][name] = {'total': 0, 'deadline_hits': 0}
                    
                    obstacle_groups[obs][name]['total'] += 1
                    if trial['hit_deadline']:
                        obstacle_groups[obs][name]['deadline_hits'] += 1
            
            # write grouped data
            planner_names = list(results.keys())
            fieldnames = ['num_obstacles'] + [f'{p}_deadline_hit_rate' for p in planner_names]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            
            for obs in sorted(obstacle_groups.keys()):
                row = {'num_obstacles': obs}
                for planner in planner_names:
                    if planner in obstacle_groups[obs]:
                        data = obstacle_groups[obs][planner]
                        rate = (data['deadline_hits'] / data['total']) if data['total'] > 0 else 0
                        row[f'{planner}_deadline_hit_rate'] = rate
                    else:
                        row[f'{planner}_deadline_hit_rate'] = None
                writer.writerow(row)
        
        print(f"fig 2 data (deadline by obstacles) exported to {deadline_file}")
        
        # export 4: nodes expanded data (for fig 3)
        nodes_file = os.path.join(args.export_dir, 'fig3_nodes_expanded.csv')
        with open(nodes_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['planner', 'nodes_expanded'])
            writer.writeheader()
            
            for name, metrics in results.items():
                for trial in metrics.trial_data:
                    if trial['success']:
                        writer.writerow({
                            'planner': name,
                            'nodes_expanded': trial['nodes_expanded']
                        })
        
        print(f"fig 3 data (nodes expanded) exported to {nodes_file}")
        
        # export 5: path inflation scatter data (for fig 4)
        inflation_file = os.path.join(args.export_dir, 'fig4_path_inflation.csv')
        with open(inflation_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['planner', 'trial_num', 'path_inflation'])
            writer.writeheader()
            
            for name, metrics in results.items():
                for trial in metrics.trial_data:
                    if trial['path_inflation'] is not None:
                        writer.writerow({
                            'planner': name,
                            'trial_num': trial['trial_num'],
                            'path_inflation': trial['path_inflation']
                        })
        
        print(f"fig 4 data (path inflation) exported to {inflation_file}")
    
    print("\n" + "="*60)
    print("evaluation complete")
    print("="*60)


if __name__ == '__main__':
    main()