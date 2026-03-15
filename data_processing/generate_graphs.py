import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import os

# set style
sns.set_style("whitegrid")
plt.rcParams['figure.figsize'] = (10, 6)
plt.rcParams['font.size'] = 12

# (blue, orange, green)
COLORS = ['#3498db', '#e67e22', '#27ae60']  # blue, orange, green

# paths
EXPORT_DIR = 'export_data'
OUTPUT_DIR = 'data_processing/figures'

# create output directory
os.makedirs(OUTPUT_DIR, exist_ok=True)


def plot_summary_comparison():
    # fig 1: overall planner comparison table/bar chart
    df = pd.read_csv(os.path.join(EXPORT_DIR, 'summary.csv'))
    
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # success rate
    axes[0, 0].bar(df['planner'], df['success_rate'] * 100, color=COLORS)
    axes[0, 0].set_ylabel('Success Rate (%)')
    axes[0, 0].set_title('Success Rate by Planner')
    axes[0, 0].set_ylim(0, 105)
    axes[0, 0].tick_params(axis='x', rotation=15)
    
    # deadline hit rate
    axes[0, 1].bar(df['planner'], df['deadline_hit_rate'] * 100, color=COLORS)
    axes[0, 1].set_ylabel('Deadline Hit Rate (%)')
    axes[0, 1].set_title('Deadline Hit Rate by Planner')
    axes[0, 1].set_ylim(0, 105)
    axes[0, 1].tick_params(axis='x', rotation=15)
    
    # avg nodes expanded
    axes[1, 0].bar(df['planner'], df['avg_nodes_expanded'], color=COLORS)
    axes[1, 0].set_ylabel('Average Nodes Expanded')
    axes[1, 0].set_title('Computational Efficiency (Fewer is Better)')
    axes[1, 0].tick_params(axis='x', rotation=15)
    
    # avg time
    axes[1, 1].bar(df['planner'], df['avg_time_ms'], color=COLORS)
    axes[1, 1].set_ylabel('Average Time (ms)')
    axes[1, 1].set_title('Execution Time on Crazyflie STM32F405')
    axes[1, 1].tick_params(axis='x', rotation=15)
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, 'fig1_summary_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"saved: {OUTPUT_DIR}/fig1_summary_comparison.png")
    plt.close()


def plot_deadline_by_obstacles():
    # fig 2: deadline hit rate vs number of obstacles
    df = pd.read_csv(os.path.join(EXPORT_DIR, 'fig2_deadline_by_obstacles.csv'))
    
    plt.figure(figsize=(10, 6))
    
    # csv format: num_obstacles, Baseline A*_deadline_hit_rate, Deadline-Aware A*_deadline_hit_rate, etc.
    planners = [col.replace('_deadline_hit_rate', '') for col in df.columns if '_deadline_hit_rate' in col]
    
    for i, planner in enumerate(planners):
        col_name = f'{planner}_deadline_hit_rate'
        plt.plot(df['num_obstacles'], df[col_name] * 100, 
                marker='o', linewidth=2, label=planner, color=COLORS[i % len(COLORS)])
    
    plt.xlabel('Number of Obstacles')
    plt.ylabel('Deadline Hit Rate (%)')
    plt.title('Deadline Performance vs Environment Complexity')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.savefig(os.path.join(OUTPUT_DIR, 'fig2_deadline_by_obstacles.png'), dpi=300, bbox_inches='tight')
    print(f"saved: {OUTPUT_DIR}/fig2_deadline_by_obstacles.png")
    plt.close()


def plot_nodes_expanded():
    # fig 3: box plot of nodes expanded distribution
    df = pd.read_csv(os.path.join(EXPORT_DIR, 'fig3_nodes_expanded.csv'))
    
    plt.figure(figsize=(10, 6))
    
    # create box plot
    planners = df['planner'].unique()
    data = [df[df['planner'] == p]['nodes_expanded'].values for p in planners]
    
    bp = plt.boxplot(data, labels=planners, patch_artist=True,
                     boxprops=dict(facecolor='lightblue', alpha=0.7),
                     medianprops=dict(color='#3498db', linewidth=2),
                     whiskerprops=dict(linewidth=1.5),
                     capprops=dict(linewidth=1.5))
    
    plt.ylabel('Nodes Expanded')
    plt.title('Node Expansion Efficiency')
    plt.grid(True, alpha=0.3, axis='y')
    plt.xticks(rotation=15)
    
    # add mean markers
    for i, planner in enumerate(planners):
        mean_val = df[df['planner'] == planner]['nodes_expanded'].mean()
        plt.plot(i + 1, mean_val, 'D', color='#e67e22', markersize=8, label='Mean' if i == 0 else '')
    
    plt.legend()
    
    plt.savefig(os.path.join(OUTPUT_DIR, 'fig3_nodes_expanded.png'), dpi=300, bbox_inches='tight')
    print(f"saved: {OUTPUT_DIR}/fig3_nodes_expanded.png")
    plt.close()


def plot_efficiency_scatter():
    # fig 4: time vs nodes scatter plot showing efficiency trade-offs
    df = pd.read_csv(os.path.join(EXPORT_DIR, 'trials.csv'))
    
    # only plot successful trials
    df_success = df[df['success'] == True].copy()
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # left plot: time vs nodes
    planners = df_success['planner'].unique()
    
    for i, planner in enumerate(planners):
        planner_data = df_success[df_success['planner'] == planner]
        axes[0].scatter(planner_data['nodes_expanded'], planner_data['time_ms'], 
                       alpha=0.6, s=50, label=planner, color=COLORS[i % len(COLORS)])
    
    axes[0].set_xlabel('Nodes Expanded')
    axes[0].set_ylabel('Execution Time (ms)')
    axes[0].set_title('Computational Efficiency: Time vs Nodes')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # adjust y-axis to spread out the data points
    axes[0].set_ylim(20, 55)
    
    # right plot: success rate vs deadline hit rate
    summary_df = pd.read_csv(os.path.join(EXPORT_DIR, 'summary.csv'))
    
    x = summary_df['success_rate'] * 100
    y = summary_df['deadline_hit_rate'] * 100
    
    for i, planner in enumerate(summary_df['planner']):
        axes[1].scatter(x.iloc[i], y.iloc[i], s=200, alpha=0.7, 
                       color=COLORS[i % len(COLORS)], label=planner)
        axes[1].annotate(planner, (x.iloc[i], y.iloc[i]), 
                        xytext=(5, 5), textcoords='offset points', fontsize=9)
    
    axes[1].set_xlabel('Success Rate (%)')
    axes[1].set_ylabel('Deadline Hit Rate (%)')
    axes[1].set_title('Reliability vs Real-Time Performance')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlim(80, 105)
    axes[1].set_ylim(-2, max(y) + 5)
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, 'fig4_efficiency_analysis.png'), dpi=300, bbox_inches='tight')
    print(f"saved: {OUTPUT_DIR}/fig4_efficiency_analysis.png")
    plt.close()


def plot_dynamic_results():
    # fig 5: dynamic environment results (if available)
    dynamic_file = os.path.join(EXPORT_DIR, 'dynamic_mixed_results.csv')
    
    if not os.path.exists(dynamic_file):
        print("no dynamic results found, skipping fig 5")
        return
    
    df = pd.read_csv(dynamic_file)
    
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    
    # success rate
    axes[0].bar(df['planner'], df['success_rate'] * 100, color=COLORS)
    axes[0].set_ylabel('Success Rate (%)')
    axes[0].set_title('Dynamic Environment Success Rate')
    axes[0].set_ylim(0, 105)
    axes[0].tick_params(axis='x', rotation=15)
    
    # avg nodes
    axes[1].bar(df['planner'], df['avg_nodes_expanded'], color=COLORS)
    axes[1].set_ylabel('Average Nodes Expanded')
    axes[1].set_title('Total Nodes (5 Replans)')
    axes[1].tick_params(axis='x', rotation=15)
    
    # avg replans
    axes[2].bar(df['planner'], df['avg_replans'], color=COLORS)
    axes[2].set_ylabel('Average Successful Replans')
    axes[2].set_title('Replanning Success')
    axes[2].set_ylim(0, 3)
    axes[2].tick_params(axis='x', rotation=15)
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, 'fig5_dynamic_results.png'), dpi=300, bbox_inches='tight')
    print(f"saved: {OUTPUT_DIR}/fig5_dynamic_results.png")
    plt.close()


def generate_all_graphs():
    # generate all graphs from csv data
    print("="*60)
    print("generating graphs from csv data")
    print("="*60)
    
    plot_summary_comparison()
    plot_deadline_by_obstacles()
    plot_nodes_expanded()
    plot_efficiency_scatter()
    plot_dynamic_results()
    
    print("="*60)
    print(f"all graphs saved to {OUTPUT_DIR}/")
    print("="*60)


if __name__ == '__main__':
    generate_all_graphs()