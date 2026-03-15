# data processing and visualization

automated graph generation from csv exports using matplotlib and pandas.

## requirements

```bash
pip install pandas matplotlib seaborn
```

## usage

### generate all graphs

```bash
python -m data_processing.generate_graphs
```

this creates all figures in `data_processing/figures/`:
- `fig1_summary_comparison.png` - overall planner metrics (4 subplots)
- `fig2_deadline_by_obstacles.png` - deadline performance vs complexity
- `fig3_nodes_expanded.png` - node efficiency box plots
- `fig4_training_loss.png` - neural ranker learning curve
- `fig5_dynamic_results.png` - dynamic environment results (if available)

## workflow

1. **run evaluation** with csv export:
   ```bash
   python -m ml_planning.evaluate_planners --export_csv --crazyflie --deadline_ms 50.0
   ```

2. **optionally run dynamic trials** (~15 for mixed obstacles):
   ```bash
   python -m ml_planning.evaluate_planners --export_csv --crazyflie --deadline_ms 50.0 --dynamic_trials 15
   ```

3. **generate graphs**:
   ```bash
   python -m data_processing.generate_graphs
   ```

4. **view figures** in `data_processing/figures/`

## csv inputs

graphs are generated from:
- `export_data/summary.csv` - aggregate metrics
- `export_data/fig2_deadline_by_obstacles.csv` - deadline vs obstacles
- `export_data/fig3_nodes_expanded.csv` - node counts per trial
- `export_data/training_history.csv` - training loss curves
- `export_data/dynamic_mixed_results.csv` - dynamic environment results (optional)

## customization

edit `generate_graphs.py` to customize:
- colors, styles, fonts
- figure sizes and dpi
- plot types and layouts
- axis ranges and labels