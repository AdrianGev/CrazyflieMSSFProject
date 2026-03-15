# ml planning

deadline-aware a* with neural guidance for the crazyflie drone

## how to train

just run this and it'll generate training data and train the model:
```bash
python -m ml_planning.train_ranker
```

the trained model gets saved to `ml_planning/models/neural_ranker.pt` (this file is gitignored so you gotta train it yourself)

## training with csv export

to save training metrics (loss per epoch) to csv:
```bash
python -m ml_planning.train_ranker --export_csv
```

this creates `export_data/training_history.csv` with epoch-by-epoch training and validation loss

## custom training options

train with more data and epochs:
```bash
python -m ml_planning.train_ranker --num_maps 2000 --epochs 100 --export_csv
```

other options:
- `--num_maps` - number of training maps (default: 1000)
- `--val_maps` - validation maps (default: 200)
- `--min_obstacles` - min obstacles per map (default: 5)
- `--max_obstacles` - max obstacles per map (default: 20)
- `--epochs` - training epochs (default: 50)
- `--batch_size` - batch size (default: 256)
- `--lr` - learning rate (default: 0.001)
- `--hidden_dims` - network layers (default: 128 64 32)
- `--export_csv` - export training metrics to csv
- `--export_dir` - where to save csvs (default: export_data)

## evaluating the model

compare all three planners (baseline, deadline-aware, neural-guided):
```bash
python -m ml_planning.evaluate_planners
```

with csv export for making graphs:
```bash
python -m ml_planning.evaluate_planners --export_csv
```

this creates 5 csv files in `export_data/` (replaces old files each run):
1. `summary.csv` - overall metrics for each planner
2. `trials.csv` - detailed data for every single trial
3. `fig2_deadline_by_obstacles.csv` - deadline hit rate grouped by obstacle count
4. `fig3_nodes_expanded.csv` - nodes expanded per trial (for box plots)
5. `fig4_path_inflation.csv` - path quality scatter data

## evaluation options

test with different settings:
```bash
python -m ml_planning.evaluate_planners --num_scenarios 200 --deadline_ms 10.0 --export_csv
```

options:
- `--num_scenarios` - number of test scenarios (default: 100)
- `--min_obstacles` - min obstacles (default: 5)
- `--max_obstacles` - max obstacles (default: 20)
- `--deadline_ms` - deadline in milliseconds (default: 10.0)
- `--planners` - which to test: baseline deadline neural (default: all three)
- `--export_csv` - export detailed results to csv
- `--export_dir` - where to save csvs (default: export_data)
- `--crazyflie` - simulate crazyflie stm32f405 cpu performance (~30x slowdown)

## simulating crazyflie hardware

to test performance on the actual crazyflie's stm32f405 microcontroller:
```bash
python -m ml_planning.evaluate_planners --export_csv --crazyflie --deadline_ms 10.0
```

this simulates the 168mhz cortex-m4 cpu (~30x slower than laptop) to show:
- realistic planning times on embedded hardware
- which planner meets the deadline under real constraints
- how node efficiency translates to actual time savings

crazyflie specs:
- stm32f405 main mcu (168mhz cortex-m4, 192kb sram, 1mb flash)
- nrf51822 radio (32mhz cortex-m0, 16kb sram, 128kb flash)

## quick examples

run demo scripts showing basic usage:
```bash
python -m ml_planning.demo
```