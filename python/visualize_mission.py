import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

def plot_mission_data(csv_path='mission_data.csv'):
    if not os.path.exists(csv_path):
        print(f"Error: {csv_path} not found. Run the simulation first.")
        return

    df = pd.read_csv(csv_path)

    # Set professional style
    plt.style.use('seaborn-v0_8-muted')
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    
    # 1. Attitude (Quaternions)
    axes[0].plot(df['t'], df['qw'], label='qw', linewidth=1.5)
    axes[0].plot(df['t'], df['qx'], label='qx', linestyle='--')
    axes[0].plot(df['t'], df['qy'], label='qy', linestyle=':')
    axes[0].plot(df['t'], df['qz'], label='qz', linestyle='-.')
    axes[0].set_ylabel('Quaternion')
    axes[0].set_title('Satellite Attitude History', fontsize=14, fontweight='bold')
    axes[0].legend(loc='upper right', frameon=True)
    axes[0].grid(True, alpha=0.3)

    # 2. Angular Rates
    axes[1].plot(df['t'], df['wx'], label='wx', color='red')
    axes[1].plot(df['t'], df['wy'], label='wy', color='green')
    axes[1].plot(df['t'], df['wz'], label='wz', color='blue')
    axes[1].set_ylabel('Rate (rad/s)')
    axes[1].set_title('Body Angular Rates', fontsize=12)
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)

    # 3. Pointing Error
    axes[2].plot(df['t'], df['error_deg'], color='purple', linewidth=2)
    axes[2].fill_between(df['t'], df['error_deg'], color='purple', alpha=0.1)
    axes[2].set_ylabel('Error (deg)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_title('Pointing Error (Sun-Pointing)', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    
    # Add a horizontal line for requirements (e.g. 5 deg)
    axes[2].axhline(y=5.0, color='gray', linestyle='--', alpha=0.5, label='Requirement (5 deg)')

    plt.tight_layout()
    output_png = 'mission_summary.png'
    plt.savefig(output_png, dpi=300)
    print(f"Professional plot saved to {output_png}")
    plt.show()

if __name__ == "__main__":
    plot_mission_data()
