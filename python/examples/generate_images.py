"""
Script to generate static images of the visualizations for documentation.
"""

import sys
from pathlib import Path

# Add python directory to path
python_dir = Path(__file__).parent.parent
sys.path.insert(0, str(python_dir))

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import data_loader
import visualize_attitude


def main():
    # Path to the CSV file
    csv_file = Path(__file__).parent.parent.parent / 'build' / 'mission_data.csv'
    
    if not csv_file.exists():
        print(f"ERROR: {csv_file} not found!")
        return
    
    output_dir = Path(__file__).parent.parent.parent / 'python'
    output_dir.mkdir(exist_ok=True)
    
    print(f"Loading data from: {csv_file}")
    df = data_loader.load_simulation_data(csv_file)
    print(f"Loaded {len(df)} data points")
    
    # Generate time series visualization
    print("\nGenerating time series plot...")
    fig1, axes1 = visualize_attitude.plot_attitude_timeseries(str(csv_file))
    output_file1 = output_dir / 'attitude_timeseries.png'
    fig1.savefig(output_file1, dpi=150, bbox_inches='tight')
    print(f"  Saved to: {output_file1}")
    plt.close(fig1)
    
    # Generate 3D visualization at end
    print("\nGenerating 3D visualization (final state)...")
    fig2, ax2 = visualize_attitude.visualize_attitude_static(str(csv_file), time_index=-1)
    output_file2 = output_dir / 'attitude_3d_final.png'
    fig2.savefig(output_file2, dpi=150, bbox_inches='tight')
    print(f"  Saved to: {output_file2}")
    plt.close(fig2)
    
    # Generate 3D visualization at detumble completion
    print("\nGenerating 3D visualization (after detumble)...")
    # Find approximate time when mode switched to NOMINAL
    nominal_idx = np.where(df['mode'] == 1)[0][0]
    fig3, ax3 = visualize_attitude.visualize_attitude_static(str(csv_file), time_index=nominal_idx)
    output_file3 = output_dir / 'attitude_3d_nominal.png'
    fig3.savefig(output_file3, dpi=150, bbox_inches='tight')
    print(f"  Saved to: {output_file3}")
    plt.close(fig3)
    
    print("\n✓ All visualizations saved successfully!")


if __name__ == '__main__':
    main()
