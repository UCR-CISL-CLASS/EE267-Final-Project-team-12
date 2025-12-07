"""
Extended Evaluation and Visualization Script
Analyzes results including hybrid controller performance
"""

import os
import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')


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
    combined_file = os.path.join(results_dir, 'all_experiments_extended.json')
    if os.path.exists(combined_file):
        with open(combined_file, 'r') as f:
            results = json.load(f)
        print(f"Loaded {len(results)} experiments from {combined_file}")
        return results
    
    # Otherwise load individual files
    for filename in os.listdir(results_dir):
        if filename.endswith('.json') and 'all_experiments' not in filename:
            filepath = os.path.join(results_dir, filename)
            with open(filepath, 'r') as f:
                results.append(json.load(f))
    
    print(f"Loaded {len(results)} experiments from individual files")
    return results


def plot_all_controllers_comparison(results, output_dir='plots'):
    """
    Plot comprehensive comparison including hybrid controllers.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # Define colors for each controller type
    colors = {
        'PurePursuit': '#2ca02c',
        'Stanley': '#ff7f0e',
        'Hybrid': '#d62728'
    }
    
    # Plot 1: Lateral Error Time Series
    ax = axes[0, 0]
    for result in results:
        name = result['experiment_name']
        if 'Hybrid' in name:
            color = colors['Hybrid']
            linewidth = 2.5
            alpha = 0.9
        elif 'Stanley' in name:
            color = colors['Stanley']
            linewidth = 1.5
            alpha = 0.6
        else:
            color = colors['PurePursuit']
            linewidth = 1.5
            alpha = 0.6
        
        ax.plot(result['timestamps'], result['lateral_errors'],
                label=name, color=color, linewidth=linewidth, alpha=alpha)
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Lateral Error (m)', fontsize=12)
    ax.set_title('Lateral Error Comparison (All Controllers)', fontsize=14, fontweight='bold')
    ax.legend(fontsize=8, loc='upper right')
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Mean Lateral Error Bar Chart
    ax = axes[0, 1]
    names = [r['experiment_name'] for r in results]
    errors = [r['summary']['mean_lateral_error'] for r in results]
    bar_colors = []
    for name in names:
        if 'Hybrid' in name:
            bar_colors.append(colors['Hybrid'])
        elif 'Stanley' in name:
            bar_colors.append(colors['Stanley'])
        else:
            bar_colors.append(colors['PurePursuit'])
    
    bars = ax.bar(range(len(names)), errors, color=bar_colors, alpha=0.8)
    ax.set_xlabel('Experiment', fontsize=12)
    ax.set_ylabel('Mean Lateral Error (m)', fontsize=12)
    ax.set_title('Mean Lateral Error Comparison', fontsize=14, fontweight='bold')
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Highlight best performer
    best_idx = np.argmin(errors)
    bars[best_idx].set_edgecolor('black')
    bars[best_idx].set_linewidth(3)
    
    # Plot 3: Steering Smoothness
    ax = axes[1, 0]
    smoothness = [r['summary']['steering_smoothness'] for r in results]
    ax.bar(range(len(names)), smoothness, color=bar_colors, alpha=0.8)
    ax.set_xlabel('Experiment', fontsize=12)
    ax.set_ylabel('Steering Smoothness (std)', fontsize=12)
    ax.set_title('Steering Smoothness Comparison (Lower is Better)', fontsize=14, fontweight='bold')
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    
    # Plot 4: Heading Error
    ax = axes[1, 1]
    heading_errors = [r['summary']['mean_abs_heading_error'] for r in results]
    ax.bar(range(len(names)), heading_errors, color=bar_colors, alpha=0.8)
    ax.set_xlabel('Experiment', fontsize=12)
    ax.set_ylabel('Mean Heading Error (°)', fontsize=12)
    ax.set_title('Mean Heading Error Comparison', fontsize=14, fontweight='bold')
    ax.set_xticks(range(len(names)))
    ax.set_xticklabels(names, rotation=45, ha='right', fontsize=8)
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'comprehensive_comparison.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/comprehensive_comparison.png")
    plt.close()


def plot_hybrid_controller_analysis(results, output_dir='plots'):
    """
    Detailed analysis of hybrid controller behavior.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save plots
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Filter for hybrid results
    hybrid_results = [r for r in results if 'Hybrid' in r['experiment_name']]
    
    if not hybrid_results:
        print("No hybrid controller results found")
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    colors = ['#d62728', '#9467bd', '#8c564b']
    
    # Plot 1: Curvature over time
    ax = axes[0, 0]
    for i, result in enumerate(hybrid_results):
        if 'curvatures' in result and result['curvatures']:
            ax.plot(result['timestamps'], result['curvatures'],
                   label=result['experiment_name'], color=colors[i], linewidth=2)
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Path Curvature (1/m)', fontsize=12)
    ax.set_title('Path Curvature Detection', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Blend weight for blending mode
    ax = axes[0, 1]
    for i, result in enumerate(hybrid_results):
        if 'blending' in result['experiment_name'].lower():
            if 'blend_weights' in result and result['blend_weights']:
                ax.plot(result['timestamps'], result['blend_weights'],
                       label=result['experiment_name'], color=colors[i], linewidth=2)
                ax.axhline(y=0.5, color='gray', linestyle='--', alpha=0.5, label='Equal blend')
                ax.fill_between(result['timestamps'], 0, result['blend_weights'],
                               alpha=0.3, label='Stanley dominance')
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Blend Weight (0=PP, 1=Stanley)', fontsize=12)
    ax.set_title('Controller Blending Weight Over Time', fontsize=14, fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim([-0.1, 1.1])
    
    # Plot 3: Lateral error comparison (Hybrid vs Best Individual)
    ax = axes[1, 0]
    
    # Find best Pure Pursuit and Stanley
    pp_results = [r for r in results if 'PurePursuit' in r['experiment_name']]
    stanley_results = [r for r in results if 'Stanley' in r['experiment_name'] and 'Hybrid' not in r['experiment_name']]
    
    if pp_results:
        best_pp = min(pp_results, key=lambda x: x['summary']['mean_lateral_error'])
        ax.plot(best_pp['timestamps'], best_pp['lateral_errors'],
               label=f"Best PP: {best_pp['experiment_name']}", 
               color='#2ca02c', linewidth=2, alpha=0.7)
    
    if stanley_results:
        best_stanley = min(stanley_results, key=lambda x: x['summary']['mean_lateral_error'])
        ax.plot(best_stanley['timestamps'], best_stanley['lateral_errors'],
               label=f"Best Stanley: {best_stanley['experiment_name']}", 
               color='#ff7f0e', linewidth=2, alpha=0.7)
    
    # Plot hybrid
    for i, result in enumerate(hybrid_results):
        ax.plot(result['timestamps'], result['lateral_errors'],
               label=result['experiment_name'], color=colors[i], linewidth=2.5)
    
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Lateral Error (m)', fontsize=12)
    ax.set_title('Hybrid vs Best Individual Controllers', fontsize=14, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Performance metrics radar chart
    ax = axes[1, 1]
    
    # Calculate relative performance scores (lower is better, so invert)
    all_lat_errors = [r['summary']['mean_lateral_error'] for r in results]
    all_heading_errors = [r['summary']['mean_abs_heading_error'] for r in results]
    all_smoothness = [r['summary']['steering_smoothness'] for r in results]
    
    max_lat = max(all_lat_errors)
    max_heading = max(all_heading_errors)
    max_smooth = max(all_smoothness)
    
    # Create table comparing metrics
    table_data = []
    for result in hybrid_results:
        row = [
            result['experiment_name'],
            f"{result['summary']['mean_lateral_error']:.3f}",
            f"{result['summary']['mean_abs_heading_error']:.3f}",
            f"{result['summary']['steering_smoothness']:.4f}"
        ]
        table_data.append(row)
    
    # Add best individual controllers
    if pp_results:
        best_pp = min(pp_results, key=lambda x: x['summary']['mean_lateral_error'])
        table_data.append([
            f"Best PP: {best_pp['experiment_name']}",
            f"{best_pp['summary']['mean_lateral_error']:.3f}",
            f"{best_pp['summary']['mean_abs_heading_error']:.3f}",
            f"{best_pp['summary']['steering_smoothness']:.4f}"
        ])
    
    if stanley_results:
        best_stanley = min(stanley_results, key=lambda x: x['summary']['mean_lateral_error'])
        table_data.append([
            f"Best Stanley: {best_stanley['experiment_name']}",
            f"{best_stanley['summary']['mean_lateral_error']:.3f}",
            f"{best_stanley['summary']['mean_abs_heading_error']:.3f}",
            f"{best_stanley['summary']['steering_smoothness']:.4f}"
        ])
    
    ax.axis('tight')
    ax.axis('off')
    
    headers = ['Controller', 'Lateral\nError (m)', 'Heading\nError (°)', 'Steering\nSmooth']
    table = ax.table(cellText=table_data, colLabels=headers,
                    cellLoc='center', loc='center',
                    colWidths=[0.35, 0.2, 0.2, 0.2])
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1, 2.5)
    
    # Color header
    for i in range(len(headers)):
        table[(0, i)].set_facecolor('#d62728')
        table[(0, i)].set_text_props(weight='bold', color='white')
    
    # Highlight hybrid rows
    for i in range(1, len(hybrid_results) + 1):
        for j in range(len(headers)):
            table[(i, j)].set_facecolor('#ffe6e6')
    
    ax.set_title('Hybrid Controller Performance Summary', fontsize=14, fontweight='bold', pad=20)
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'hybrid_analysis.png'), dpi=300, bbox_inches='tight')
    print(f"Saved: {output_dir}/hybrid_analysis.png")
    plt.close()


def generate_extended_summary_table(results, output_dir='plots'):
    """
    Generate comprehensive summary table including hybrid controllers.
    
    Args:
        results (list): List of experiment metrics
        output_dir (str): Directory to save table
    """
    os.makedirs(output_dir, exist_ok=True)
    
    # Create detailed table
    with open(os.path.join(output_dir, 'extended_summary.txt'), 'w') as f:
        f.write("="*100 + "\n")
        f.write("COMPREHENSIVE CONTROLLER COMPARISON\n")
        f.write("="*100 + "\n\n")
        
        f.write(f"{'Controller':<25} {'Lat Err (m)':<12} {'Max Lat (m)':<12} "
                f"{'Head Err (°)':<12} {'Smooth':<12} {'Type':<15}\n")
        f.write("-"*100 + "\n")
        
        # Group by controller type
        pp_results = [r for r in results if 'PurePursuit' in r['experiment_name']]
        stanley_results = [r for r in results if 'Stanley' in r['experiment_name'] and 'Hybrid' not in r['experiment_name']]
        hybrid_results = [r for r in results if 'Hybrid' in r['experiment_name']]
        
        # Write Pure Pursuit results
        if pp_results:
            f.write("\nPURE PURSUIT CONTROLLERS:\n")
            f.write("-"*100 + "\n")
            for r in pp_results:
                f.write(f"{r['experiment_name']:<25} "
                       f"{r['summary']['mean_lateral_error']:<12.3f} "
                       f"{r['summary']['max_lateral_error']:<12.3f} "
                       f"{r['summary']['mean_abs_heading_error']:<12.3f} "
                       f"{r['summary']['steering_smoothness']:<12.4f} "
                       f"{'Geometric':<15}\n")
        
        # Write Stanley results
        if stanley_results:
            f.write("\nSTANLEY CONTROLLERS:\n")
            f.write("-"*100 + "\n")
            for r in stanley_results:
                f.write(f"{r['experiment_name']:<25} "
                       f"{r['summary']['mean_lateral_error']:<12.3f} "
                       f"{r['summary']['max_lateral_error']:<12.3f} "
                       f"{r['summary']['mean_abs_heading_error']:<12.3f} "
                       f"{r['summary']['steering_smoothness']:<12.4f} "
                       f"{'Error-based':<15}\n")
        
        # Write Hybrid results
        if hybrid_results:
            f.write("\nHYBRID CONTROLLERS:\n")
            f.write("-"*100 + "\n")
            for r in hybrid_results:
                f.write(f"{r['experiment_name']:<25} "
                       f"{r['summary']['mean_lateral_error']:<12.3f} "
                       f"{r['summary']['max_lateral_error']:<12.3f} "
                       f"{r['summary']['mean_abs_heading_error']:<12.3f} "
                       f"{r['summary']['steering_smoothness']:<12.4f} "
                       f"{'Combined':<15}\n")
        
        # Find and highlight best performers
        f.write("\n" + "="*100 + "\n")
        f.write("BEST PERFORMERS:\n")
        f.write("="*100 + "\n")
        
        best_overall = min(results, key=lambda x: x['summary']['mean_lateral_error'])
        f.write(f"Best Lateral Error:    {best_overall['experiment_name']:<30} "
               f"{best_overall['summary']['mean_lateral_error']:.3f} m\n")
        
        best_smooth = min(results, key=lambda x: x['summary']['steering_smoothness'])
        f.write(f"Best Smoothness:       {best_smooth['experiment_name']:<30} "
               f"{best_smooth['summary']['steering_smoothness']:.4f}\n")
        
        best_heading = min(results, key=lambda x: x['summary']['mean_abs_heading_error'])
        f.write(f"Best Heading Control:  {best_heading['experiment_name']:<30} "
               f"{best_heading['summary']['mean_abs_heading_error']:.3f}°\n")
        
        # Performance improvement
        if hybrid_results and (pp_results or stanley_results):
            f.write("\n" + "="*100 + "\n")
            f.write("HYBRID CONTROLLER IMPROVEMENT:\n")
            f.write("="*100 + "\n")
            
            best_hybrid = min(hybrid_results, key=lambda x: x['summary']['mean_lateral_error'])
            
            if pp_results:
                best_pp = min(pp_results, key=lambda x: x['summary']['mean_lateral_error'])
                improvement_pp = ((best_pp['summary']['mean_lateral_error'] - 
                                 best_hybrid['summary']['mean_lateral_error']) / 
                                best_pp['summary']['mean_lateral_error'] * 100)
                f.write(f"vs Best Pure Pursuit:  {improvement_pp:+.1f}% improvement\n")
            
            if stanley_results:
                best_stanley = min(stanley_results, key=lambda x: x['summary']['mean_lateral_error'])
                improvement_stanley = ((best_stanley['summary']['mean_lateral_error'] - 
                                      best_hybrid['summary']['mean_lateral_error']) / 
                                     best_stanley['summary']['mean_lateral_error'] * 100)
                f.write(f"vs Best Stanley:       {improvement_stanley:+.1f}% improvement\n")
    
    print(f"Saved: {output_dir}/extended_summary.txt")


def main():
    """
    Main function to generate all extended plots and analysis.
    """
    print("Loading experiment results...")
    results = load_results('results')
    
    if not results:
        print("No results found! Run experiment_runner_extended.py first.")
        return
    
    print(f"\nGenerating extended analysis for {len(results)} experiments...")
    
    # Generate all plots
    plot_all_controllers_comparison(results)
    plot_hybrid_controller_analysis(results)
    generate_extended_summary_table(results)
    
    print("\n" + "="*60)
    print("EXTENDED EVALUATION COMPLETE")
    print("="*60)
    print("Generated plots:")
    print("  - comprehensive_comparison.png (all controllers)")
    print("  - hybrid_analysis.png (detailed hybrid analysis)")
    print("  - extended_summary.txt (comprehensive metrics)")


if __name__ == '__main__':
    main()
