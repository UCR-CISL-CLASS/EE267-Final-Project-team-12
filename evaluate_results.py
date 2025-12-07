"""
Evaluation and Visualization Script
Generates plots and analysis from experiment results
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend


def load_results(results_dir='results'):
    """
    Load all experiment results from JSON files.
    
    Args:
        results_dir (str): Directory containing result files
        
    Returns:
        list: List of metrics dictionaries
    """
    results = []
    
    # Try to load combined results first
    combined_file = os.path.join(results_dir, 'all_experiments.json')
    if os.path.exists(combined_file):
        with open(combined_file, 'r') as f:
            results = json.load(f)
        print(f"Loaded {len(results)} experiments from {combined_file}")
        return results
    
    # Otherwise load individual files
    for filename in os.listdir(results_dir):
        if filename.endswith('.json') and filename != 'all_experiments.json':
            filepath = os.path.join(results_dir, filename)
            with open(filepath, 'r') as f:
                results.append(json.load(f))
    
    print(f"Loaded {len(results)} experiments from individual files")
    return results


def plot_lateral_error_comparison(results, output_dir='plots'):
    """
    Plot lateral error over time for all experiments.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # Plot Pure Pursuit experiments
    pp_colors = ['#1f77b4', '#2ca02c', '#d62728']
    pp_idx = 0
    for result in results:
        if 'PurePursuit' in result['experiment_name']:
            ax1.plot(result['timestamps'], result['lateral_errors'], 
                    label=result['experiment_name'], 
                    color=pp_colors[pp_idx], linewidth=1.5)
            pp_idx += 1
    
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Lateral Error (m)', fontsize=12)
    ax1.set_title('Pure Pursuit: Lateral Error vs Time', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot Stanley experiments
    stanley_colors = ['#ff7f0e', '#9467bd', '#8c564b']
    stanley_idx = 0
    for result in results:
        if 'Stanley' in result['experiment_name']:
            ax2.plot(result['timestamps'], result['lateral_errors'], 
                    label=result['experiment_name'], 
                    color=stanley_colors[stanley_idx], linewidth=1.5)
            stanley_idx += 1
    
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Lateral Error (m)', fontsize=12)
    ax2.set_title('Stanley: Lateral Error vs Time', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'lateral_error_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/lateral_error_comparison.png")
    plt.close()


def plot_heading_error_comparison(results, output_dir='plots'):
    """
    Plot heading error over time for all experiments.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # Plot Pure Pursuit experiments
    pp_colors = ['#1f77b4', '#2ca02c', '#d62728']
    pp_idx = 0
    for result in results:
        if 'PurePursuit' in result['experiment_name']:
            ax1.plot(result['timestamps'], result['heading_errors'], 
                    label=result['experiment_name'], 
                    color=pp_colors[pp_idx], linewidth=1.5, alpha=0.7)
            pp_idx += 1
    
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Heading Error (degrees)', fontsize=12)
    ax1.set_title('Pure Pursuit: Heading Error vs Time', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
    
    # Plot Stanley experiments
    stanley_colors = ['#ff7f0e', '#9467bd', '#8c564b']
    stanley_idx = 0
    for result in results:
        if 'Stanley' in result['experiment_name']:
            ax2.plot(result['timestamps'], result['heading_errors'], 
                    label=result['experiment_name'], 
                    color=stanley_colors[stanley_idx], linewidth=1.5, alpha=0.7)
            stanley_idx += 1
    
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Heading Error (degrees)', fontsize=12)
    ax2.set_title('Stanley: Heading Error vs Time', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=0.8, alpha=0.5)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'heading_error_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/heading_error_comparison.png")
    plt.close()


def plot_steering_smoothness(results, output_dir='plots'):
    """
    Plot steering angle over time to show smoothness.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # Plot Pure Pursuit experiments
    pp_colors = ['#1f77b4', '#2ca02c', '#d62728']
    pp_idx = 0
    for result in results:
        if 'PurePursuit' in result['experiment_name']:
            ax1.plot(result['timestamps'], result['steering_angles'], 
                    label=result['experiment_name'], 
                    color=pp_colors[pp_idx], linewidth=1.2, alpha=0.8)
            pp_idx += 1
    
    ax1.set_xlabel('Time (s)', fontsize=12)
    ax1.set_ylabel('Steering Angle (normalized)', fontsize=12)
    ax1.set_title('Pure Pursuit: Steering Angle vs Time', fontsize=14, fontweight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([-1.1, 1.1])
    
    # Plot Stanley experiments
    stanley_colors = ['#ff7f0e', '#9467bd', '#8c564b']
    stanley_idx = 0
    for result in results:
        if 'Stanley' in result['experiment_name']:
            ax2.plot(result['timestamps'], result['steering_angles'], 
                    label=result['experiment_name'], 
                    color=stanley_colors[stanley_idx], linewidth=1.2, alpha=0.8)
            stanley_idx += 1
    
    ax2.set_xlabel('Time (s)', fontsize=12)
    ax2.set_ylabel('Steering Angle (normalized)', fontsize=12)
    ax2.set_title('Stanley: Steering Angle vs Time', fontsize=14, fontweight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-1.1, 1.1])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'steering_smoothness.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/steering_smoothness.png")
    plt.close()


def plot_summary_bar_charts(results, output_dir='plots'):
    """
    Create bar charts comparing summary metrics across experiments.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Extract summary data
    experiment_names = []
    mean_lateral_errors = []
    mean_heading_errors = []
    steering_smoothness = []
    
    for result in results:
        experiment_names.append(result['experiment_name'])
        mean_lateral_errors.append(result['summary']['mean_lateral_error'])
        mean_heading_errors.append(result['summary']['mean_abs_heading_error'])
        steering_smoothness.append(result['summary']['steering_smoothness'])
    
    # Create bar charts
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    
    # Colors for each controller type
    colors = []
    for name in experiment_names:
        if 'PurePursuit' in name:
            colors.append('#2ca02c')
        else:
            colors.append('#ff7f0e')
    
    # Mean Lateral Error
    axes[0].bar(range(len(experiment_names)), mean_lateral_errors, color=colors, alpha=0.8)
    axes[0].set_xlabel('Experiment', fontsize=12)
    axes[0].set_ylabel('Mean Lateral Error (m)', fontsize=12)
    axes[0].set_title('Mean Lateral Error Comparison', fontsize=14, fontweight='bold')
    axes[0].set_xticks(range(len(experiment_names)))
    axes[0].set_xticklabels(experiment_names, rotation=45, ha='right')
    axes[0].grid(True, alpha=0.3, axis='y')
    
    # Mean Absolute Heading Error
    axes[1].bar(range(len(experiment_names)), mean_heading_errors, color=colors, alpha=0.8)
    axes[1].set_xlabel('Experiment', fontsize=12)
    axes[1].set_ylabel('Mean Abs Heading Error (degrees)', fontsize=12)
    axes[1].set_title('Mean Heading Error Comparison', fontsize=14, fontweight='bold')
    axes[1].set_xticks(range(len(experiment_names)))
    axes[1].set_xticklabels(experiment_names, rotation=45, ha='right')
    axes[1].grid(True, alpha=0.3, axis='y')
    
    # Steering Smoothness (lower is better)
    axes[2].bar(range(len(experiment_names)), steering_smoothness, color=colors, alpha=0.8)
    axes[2].set_xlabel('Experiment', fontsize=12)
    axes[2].set_ylabel('Steering Smoothness (std of Δθ)', fontsize=12)
    axes[2].set_title('Steering Smoothness Comparison', fontsize=14, fontweight='bold')
    axes[2].set_xticks(range(len(experiment_names)))
    axes[2].set_xticklabels(experiment_names, rotation=45, ha='right')
    axes[2].grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'summary_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/summary_comparison.png")
    plt.close()


def generate_summary_table(results, output_dir='plots'):
    """
    Generate a summary table of all metrics.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save table
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Create table
    headers = ['Experiment', 'Mean Lateral\nError (m)', 'Max Lateral\nError (m)', 
               'Mean Heading\nError (°)', 'Steering\nSmoothness']
    
    table_data = []
    for result in results:
        row = [
            result['experiment_name'],
            f"{result['summary']['mean_lateral_error']:.3f}",
            f"{result['summary']['max_lateral_error']:.3f}",
            f"{result['summary']['mean_abs_heading_error']:.3f}",
            f"{result['summary']['steering_smoothness']:.4f}"
        ]
        table_data.append(row)
    
    # Create figure
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.axis('tight')
    ax.axis('off')
    
    table = ax.table(cellText=table_data, colLabels=headers, 
                     cellLoc='center', loc='center',
                     colWidths=[0.25, 0.15, 0.15, 0.15, 0.15])
    
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1, 2)
    
    # Color header
    for i in range(len(headers)):
        table[(0, i)].set_facecolor('#40466e')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    # Alternate row colors
    for i in range(1, len(table_data) + 1):
        for j in range(len(headers)):
            if i % 2 == 0:
                table[(i, j)].set_facecolor('#f0f0f0')
    
    plt.savefig(os.path.join(output_dir, 'summary_table.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/summary_table.png")
    plt.close()
    
    # Also save as text
    with open(os.path.join(output_dir, 'summary_table.txt'), 'w') as f:
        # Write headers
        f.write(f"{'Experiment':<20} {'Mean Lat Err':<15} {'Max Lat Err':<15} "
                f"{'Mean Heading':<15} {'Steering Smooth':<15}\n")
        f.write("=" * 80 + "\n")
        
        # Write data
        for result in results:
            f.write(f"{result['experiment_name']:<20} "
                   f"{result['summary']['mean_lateral_error']:<15.3f} "
                   f"{result['summary']['max_lateral_error']:<15.3f} "
                   f"{result['summary']['mean_abs_heading_error']:<15.3f} "
                   f"{result['summary']['steering_smoothness']:<15.4f}\n")
    
    print(f"Saved: {output_dir}/summary_table.txt")


def main():
    """
    Main function to generate all plots and analysis.
    """
    print("Loading experiment results...")
    results = load_results('results')
    
    if not results:
        print("No results found! Run experiment_runner.py first.")
        return
    
    print(f"\nGenerating plots for {len(results)} experiments...")
    
    # Generate all plots
    plot_lateral_error_comparison(results)
    plot_heading_error_comparison(results)
    plot_steering_smoothness(results)
    plot_summary_bar_charts(results)
    generate_summary_table(results)
    
    print("\n" + "="*60)
    print("EVALUATION COMPLETE")
    print("="*60)
    print("All plots saved in './plots/' directory:")
    print("  - lateral_error_comparison.png")
    print("  - heading_error_comparison.png")
    print("  - steering_smoothness.png")
    print("  - summary_comparison.png")
    print("  - summary_table.png")
    print("  - summary_table.txt")


if __name__ == '__main__':
    main()
