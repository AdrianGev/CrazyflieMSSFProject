import numpy as np
import argparse
import os
# training script for neural ranker
# generates data, trains model, and saves for later use
from ml_planning.data_generator import DataGenerator
from ml_planning.neural_ranker import NeuralRanker

def main():
    parser = argparse.ArgumentParser(description='Train neural ranker for pathfinding')
    parser.add_argument('--num_maps', type=int, default=1000, 
                       help='Number of training maps to generate')
    parser.add_argument('--val_maps', type=int, default=200,
                       help='Number of validation maps')
    parser.add_argument('--min_obstacles', type=int, default=5,
                       help='Minimum obstacles per map')
    parser.add_argument('--max_obstacles', type=int, default=20,
                       help='Maximum obstacles per map')
    parser.add_argument('--epochs', type=int, default=50,
                       help='Training epochs')
    parser.add_argument('--batch_size', type=int, default=256,
                       help='Batch size')
    parser.add_argument('--lr', type=float, default=0.001,
                       help='Learning rate')
    parser.add_argument('--hidden_dims', type=int, nargs='+', default=[128, 64, 32],
                       help='Hidden layer dimensions')
    parser.add_argument('--output_dir', type=str, default='ml_planning/models',
                       help='Output directory for models and data')
    parser.add_argument('--grid_width', type=int, default=4,
                       help='Grid width')
    parser.add_argument('--grid_height', type=int, default=7,
                       help='Grid height')
    parser.add_argument('--load_data', type=str, default=None,
                       help='Load existing training data from file')
    parser.add_argument('--save_data', action='store_true',
                       help='Save generated training data')
    parser.add_argument('--export_csv', action='store_true',
                       help='Export training metrics to CSV')
    parser.add_argument('--export_dir', type=str, default='export_data',
                       help='Directory for CSV exports')
    
    args = parser.parse_args()
    
    # create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    
    print("="*60)
    print("neural ranker training")
    print("="*60)
    print(f"grid size: {args.grid_width}x{args.grid_height}")
    print(f"Training maps: {args.num_maps}")
    print(f"Validation maps: {args.val_maps}")
    print(f"Obstacles: {args.min_obstacles}-{args.max_obstacles}")
    print(f"Epochs: {args.epochs}")
    print(f"Batch size: {args.batch_size}")
    print(f"Learning rate: {args.lr}")
    print(f"Hidden dims: {args.hidden_dims}")
    print("="*60)
    # cool labels lol
    # initialize data generator
    generator = DataGenerator(
        grid_width=args.grid_width,
        grid_height=args.grid_height,
        patch_size=5
    )
    
    # generate or load training data
    if args.load_data:
        print(f"\nLoading training data from {args.load_data}...")
        train_examples = generator.load_dataset(args.load_data)
    else:
        print("\nGenerating training data...")
        train_examples = generator.generate_dataset(
            num_maps=args.num_maps,
            min_obstacles=args.min_obstacles,
            max_obstacles=args.max_obstacles
        )
        
        if args.save_data:
            data_file = os.path.join(args.output_dir, 'training_data.pkl')
            generator.save_dataset(train_examples, data_file)
    
    # generate validation data
    print("\nGenerating validation data...")
    val_examples = generator.generate_dataset(
        num_maps=args.val_maps,
        min_obstacles=args.min_obstacles,
        max_obstacles=args.max_obstacles
    )
    
    # calculate input dimension
    # features: [node_x, node_y, goal_x, goal_y, dx, dy, dist, flattened_5x5_patch]
    input_dim = 7 + (5 * 5)
    
    print(f"\nInput dimension: {input_dim}")
    print(f"Training examples: {len(train_examples)}")
    print(f"Validation examples: {len(val_examples)}")
    
    # initialize neural ranker
    ranker = NeuralRanker(
        input_dim=input_dim,
        hidden_dims=args.hidden_dims
    )
    
    # train
    print("\nStarting training...")
    ranker.train(
        train_examples=train_examples,
        val_examples=val_examples,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        verbose=True,
        export_csv=args.export_csv,
        export_dir=args.export_dir
    )
    
    # save model
    model_file = os.path.join(args.output_dir, 'neural_ranker.pt')
    ranker.save(model_file)
    
    print("\n" + "="*60)
    print("training complete")
    print("="*60)
    print(f"model saved to: {model_file}")
    print("="*60)


if __name__ == '__main__':
    main()