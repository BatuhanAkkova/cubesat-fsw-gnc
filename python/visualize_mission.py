"""
Unified Mission Visualization Dashboard

Comprehensive visualization tool for CubeSat mission data from mission_test.cpp.
Displays trajectory, ground track, orbital elements, attitude dynamics, GNC performance,
and mission progress in a single integrated dashboard.

Usage:
    python visualize_mission.py <path_to_mission_data.csv>
    python visualize_mission.py build/mission_data.csv --save
"""

import sys
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from mpl_toolkits.mplot3d import Axes3D
import data_loader

def get_mode_color(mode_int):
    """Get color for mission mode visualization."""
    color_map = {
        0: '#FF6B6B',  # SAFE - Red
        1: '#4ECDC4',  # NOMINAL - Cyan
        2: '#45B7D1',  # SCIENCE - Blue
        3: '#96CEB4',  # DOWNLINK - Green
        4: '#FFEAA7',  # MAINTENANCE - Yellow
    }
    return color_map.get(int(mode_int), '#95A5A6')  # Gray for unknown

def create_comprehensive_dashboard(csv_file, save_path=None):
    """
    Create comprehensive mission visualization dashboard.
    
    Args:
        csv_file: Path to mission CSV data
        save_path: Optional path to save figure as PNG
        
    Returns:
        matplotlib Figure
    """
    print(f"Loading mission data from: {csv_file}")
    df = data_loader.load_simulation_data(csv_file)
    
    # Extract all data
    times = df['time'].values
    time_hours = times / 3600.0
    
    # State data
    quaternions = data_loader.extract_quaternions(df)
    rates = data_loader.extract_angular_rates(df)
    positions = data_loader.extract_position(df)
    velocities = data_loader.extract_velocity(df)
    modes = df['mode'].values if 'mode' in df.columns else None
    
    # GNC data
    torque_cmd, torque_ext = data_loader.extract_control_torques(df)
    q_target = data_loader.extract_target_quaternion(df)
    pointing_error = data_loader.extract_pointing_error(df)
    momentum = data_loader.extract_momentum(df)
    
    # Mission progress
    data_collected, data_downlinked = data_loader.extract_mission_progress(df)
    
    # Computed data
    euler = data_loader.quaternion_to_euler(quaternions, seq='ZYX')
    lats, lons = data_loader.eci_to_latlon(positions, times)
    elements = data_loader.compute_orbital_elements_from_state(positions, velocities)
    transitions = data_loader.get_mode_transitions(df)
    
    print(f"Loaded {len(df)} data points spanning {time_hours[-1]:.2f} hours")
    print(f"Detected {len(transitions)} mode transitions")
    
    # Create figure with 4x3 grid
    fig = plt.figure(figsize=(20, 14))
    fig.suptitle('CubeSat Mission Dashboard - Complete Timeline', fontsize=20, fontweight='bold')
    gs = GridSpec(4, 3, figure=fig, hspace=0.35, wspace=0.30)
    
    # ===== ROW 1: ORBITAL DYNAMICS =====
    
    # 1.1: 3D Orbit Trajectory
    ax_orbit = fig.add_subplot(gs[0, 0], projection='3d')
    pos_km = positions / 1000.0
    
    # Plot trajectory with mode-based colors
    if modes is not None:
        for i in range(len(pos_km) - 1):
            color = get_mode_color(modes[i])
            ax_orbit.plot(pos_km[i:i+2, 0], pos_km[i:i+2, 1], pos_km[i:i+2, 2],
                         color=color, linewidth=1.5, alpha=0.7)
    else:
        ax_orbit.plot(pos_km[:, 0], pos_km[:, 1], pos_km[:, 2], 'b-', linewidth=1.5)
    
    # Earth sphere
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    R_EARTH_km = 6378.137
    x_earth = R_EARTH_km * np.outer(np.cos(u), np.sin(v))
    y_earth = R_EARTH_km * np.outer(np.sin(u), np.sin(v))
    z_earth = R_EARTH_km * np.outer(np.ones(np.size(u)), np.cos(v))
    ax_orbit.plot_surface(x_earth, y_earth, z_earth, color='lightblue', alpha=0.3, edgecolor='none')
    
    ax_orbit.set_xlabel('X [km]', fontsize=9)
    ax_orbit.set_ylabel('Y [km]', fontsize=9)
    ax_orbit.set_zlabel('Z [km]', fontsize=9)
    ax_orbit.set_title('3D Orbit Trajectory', fontsize=11, fontweight='bold')
    max_range = np.max(np.abs(pos_km)) * 1.1
    ax_orbit.set_xlim([-max_range, max_range])
    ax_orbit.set_ylim([-max_range, max_range])
    ax_orbit.set_zlim([-max_range, max_range])
    
    # 1.2: Ground Track
    ax_ground = fig.add_subplot(gs[0, 1])
    if modes is not None:
        for i in range(len(lons)):
            ax_ground.scatter(lons[i], lats[i], c=get_mode_color(modes[i]), s=4, alpha=0.6)
    else:
        ax_ground.scatter(lons, lats, c=time_hours, cmap='viridis', s=4, alpha=0.6)
    
    ax_ground.set_xlabel('Longitude [deg]', fontsize=9)
    ax_ground.set_ylabel('Latitude [deg]', fontsize=9)
    ax_ground.set_title('Ground Track', fontsize=11, fontweight='bold')
    ax_ground.set_xlim([-180, 180])
    ax_ground.set_ylim([-90, 90])
    ax_ground.grid(True, alpha=0.3, linestyle='--')
    ax_ground.axhline(0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    ax_ground.axvline(0, color='gray', linestyle='--', linewidth=0.5, alpha=0.5)
    
    # 1.3: Orbital Elements
    ax_elem = fig.add_subplot(gs[0, 2])
    ax_elem.plot(time_hours, elements['a'] / 1000.0, 'b-', linewidth=1.5, label='SMA [km]', alpha=0.8)
    ax_elem.set_xlabel('Time [hours]', fontsize=9)
    ax_elem.set_ylabel('Semi-Major Axis [km]', fontsize=9)
    ax_elem.set_title('Orbital Elements', fontsize=11, fontweight='bold')
    ax_elem.grid(True, alpha=0.3)
    ax_elem.legend(fontsize=8)
    
    # Add inclination on twin axis
    ax_elem2 = ax_elem.twinx()
    ax_elem2.plot(time_hours, np.degrees(elements['i']), 'r-', linewidth=1.5, label='Inclination [deg]', alpha=0.8)
    ax_elem2.set_ylabel('Inclination [deg]', fontsize=9, color='r')
    ax_elem2.tick_params(axis='y', labelcolor='r')
    ax_elem2.legend(fontsize=8, loc='upper right')
    
    # ===== ROW 2: ATTITUDE DYNAMICS =====
    
    # 2.1: 3D Attitude Visualization
    ax_att = fig.add_subplot(gs[1, 0], projection='3d')
    R_final = data_loader.quaternion_to_rotation_matrix(quaternions[-1])
    
    # Plot body frame axes
    scale = 1.0
    origin = np.zeros(3)
    ax_att.quiver(*origin, *R_final[:, 0], color='r', arrow_length_ratio=0.15, linewidth=2.5, label='X (Roll)')
    ax_att.quiver(*origin, *R_final[:, 1], color='g', arrow_length_ratio=0.15, linewidth=2.5, label='Y (Pitch)')
    ax_att.quiver(*origin, *R_final[:, 2], color='b', arrow_length_ratio=0.15, linewidth=2.5, label='Z (Yaw)')
    
    ax_att.set_xlabel('X'); ax_att.set_ylabel('Y'); ax_att.set_zlabel('Z')
    ax_att.set_title(f'Attitude (Final) @ t={times[-1]:.1f}s', fontsize=11, fontweight='bold')
    ax_att.set_xlim([-1.2, 1.2]); ax_att.set_ylim([-1.2, 1.2]); ax_att.set_zlim([-1.2, 1.2])
    ax_att.legend(fontsize=8)
    
    # 2.2: Quaternion Components
    ax_quat = fig.add_subplot(gs[1, 1])
    ax_quat.plot(time_hours, quaternions[:, 0], 'k-', linewidth=1.5, label='qw', alpha=0.8)
    ax_quat.plot(time_hours, quaternions[:, 1], 'r-', linewidth=1.5, label='qx', alpha=0.8)
    ax_quat.plot(time_hours, quaternions[:, 2], 'g-', linewidth=1.5, label='qy', alpha=0.8)
    ax_quat.plot(time_hours, quaternions[:, 3], 'b-', linewidth=1.5, label='qz', alpha=0.8)
    ax_quat.set_xlabel('Time [hours]', fontsize=9)
    ax_quat.set_ylabel('Quaternion Components', fontsize=9)
    ax_quat.set_title('Attitude Quaternion', fontsize=11, fontweight='bold')
    ax_quat.legend(fontsize=8, ncol=2)
    ax_quat.grid(True, alpha=0.3)
    
    # 2.3: Angular Rates
    ax_rates = fig.add_subplot(gs[1, 2])
    ax_rates.plot(time_hours, rates[:, 0], 'r-', linewidth=1.5, label='ωx', alpha=0.8)
    ax_rates.plot(time_hours, rates[:, 1], 'g-', linewidth=1.5, label='ωy', alpha=0.8)
    ax_rates.plot(time_hours, rates[:, 2], 'b-', linewidth=1.5, label='ωz', alpha=0.8)
    ax_rates.plot(time_hours, np.linalg.norm(rates, axis=1), 'k--', linewidth=2, label='|ω|', alpha=0.8)
    ax_rates.set_xlabel('Time [hours]', fontsize=9)
    ax_rates.set_ylabel('Angular Rate [rad/s]', fontsize=9)
    ax_rates.set_title('Angular Rates', fontsize=11, fontweight='bold')
    ax_rates.legend(fontsize=8, ncol=2)
    ax_rates.grid(True, alpha=0.3)
    
    # ===== ROW 3: GNC PERFORMANCE =====
    
    # 3.1: Pointing Error
    ax_pointing = fig.add_subplot(gs[2, 0])
    if pointing_error is not None:
        if modes is not None:
            # Plot with mode colors
            for i in range(len(time_hours) - 1):
                ax_pointing.plot(time_hours[i:i+2], pointing_error[i:i+2], 
                               color=get_mode_color(modes[i]), linewidth=1.5, alpha=0.8)
        else:
            ax_pointing.plot(time_hours, pointing_error, 'b-', linewidth=1.5)
        
        # Add mode transition markers
        for t_trans, old_mode, new_mode in transitions:
            ax_pointing.axvline(t_trans/3600.0, color='gray', linestyle='--', linewidth=1, alpha=0.5)
        
        ax_pointing.set_xlabel('Time [hours]', fontsize=9)
        ax_pointing.set_ylabel('Pointing Error [deg]', fontsize=9)
        ax_pointing.set_title('Pointing Error', fontsize=11, fontweight='bold')
        ax_pointing.grid(True, alpha=0.3)
        ax_pointing.set_ylim(bottom=0)
    else:
        ax_pointing.text(0.5, 0.5, 'No pointing error data', ha='center', va='center', transform=ax_pointing.transAxes)
        ax_pointing.set_title('Pointing Error', fontsize=11, fontweight='bold')
    
    # 3.2: Control Torques
    ax_torque = fig.add_subplot(gs[2, 1])
    if torque_cmd is not None:
        ax_torque.plot(time_hours, torque_cmd[:, 0], 'r-', linewidth=1.5, label='Tcmd_x', alpha=0.7)
        ax_torque.plot(time_hours, torque_cmd[:, 1], 'g-', linewidth=1.5, label='Tcmd_y', alpha=0.7)
        ax_torque.plot(time_hours, torque_cmd[:, 2], 'b-', linewidth=1.5, label='Tcmd_z', alpha=0.7)
        
        if torque_ext is not None:
            # Plot magnitude of external torque
            torque_ext_mag = np.linalg.norm(torque_ext, axis=1)
            ax_torque.plot(time_hours, torque_ext_mag, 'k--', linewidth=2, label='|Text| (Bdot)', alpha=0.8)
        
        ax_torque.set_xlabel('Time [hours]', fontsize=9)
        ax_torque.set_ylabel('Torque [Nm]', fontsize=9)
        ax_torque.set_title('Control Torques', fontsize=11, fontweight='bold')
        ax_torque.legend(fontsize=8, ncol=2)
        ax_torque.grid(True, alpha=0.3)
    else:
        ax_torque.text(0.5, 0.5, 'No torque data', ha='center', va='center', transform=ax_torque.transAxes)
        ax_torque.set_title('Control Torques', fontsize=11, fontweight='bold')
    
    # 3.3: Reaction Wheel Momentum
    ax_momentum = fig.add_subplot(gs[2, 2])
    if momentum is not None:
        ax_momentum.plot(time_hours, momentum[:, 0], 'r-', linewidth=1.5, label='hx', alpha=0.8)
        ax_momentum.plot(time_hours, momentum[:, 1], 'g-', linewidth=1.5, label='hy', alpha=0.8)
        ax_momentum.plot(time_hours, momentum[:, 2], 'b-', linewidth=1.5, label='hz', alpha=0.8)
        ax_momentum.plot(time_hours, np.linalg.norm(momentum, axis=1), 'k--', linewidth=2, label='|h|', alpha=0.8)
        ax_momentum.set_xlabel('Time [hours]', fontsize=9)
        ax_momentum.set_ylabel('Momentum [Nms]', fontsize=9)
        ax_momentum.set_title('Reaction Wheel Momentum', fontsize=11, fontweight='bold')
        ax_momentum.legend(fontsize=8, ncol=2)
        ax_momentum.grid(True, alpha=0.3)
    else:
        ax_momentum.text(0.5, 0.5, 'No momentum data', ha='center', va='center', transform=ax_momentum.transAxes)
        ax_momentum.set_title('Reaction Wheel Momentum', fontsize=11, fontweight='bold')
    
    # ===== ROW 4: MISSION PROGRESS & HEALTH =====
    
    # 4.1: Mode Timeline
    ax_mode = fig.add_subplot(gs[3, 0])
    if modes is not None:
        # Create colored bands for modes
        mode_changes = [0] + [i for i in range(1, len(modes)) if modes[i] != modes[i-1]] + [len(modes)-1]
        
        for i in range(len(mode_changes) - 1):
            start_idx = mode_changes[i]
            end_idx = mode_changes[i+1]
            mode = modes[start_idx]
            color = get_mode_color(mode)
            ax_mode.axvspan(time_hours[start_idx], time_hours[end_idx], 
                          color=color, alpha=0.5, label=data_loader.get_mode_name(mode))
        
        # Remove duplicate labels
        handles, labels = ax_mode.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax_mode.legend(by_label.values(), by_label.keys(), fontsize=8, loc='upper left')
        
        ax_mode.set_xlabel('Time [hours]', fontsize=9)
        ax_mode.set_ylabel('Mission Mode', fontsize=9)
        ax_mode.set_title('Mode Timeline', fontsize=11, fontweight='bold')
        ax_mode.set_ylim([0, 1])
        ax_mode.set_xlim([0, time_hours[-1]])
        ax_mode.set_yticks([])
        ax_mode.grid(True, alpha=0.3, axis='x')
    else:
        ax_mode.text(0.5, 0.5, 'No mode data', ha='center', va='center', transform=ax_mode.transAxes)
        ax_mode.set_title('Mode Timeline', fontsize=11, fontweight='bold')
    
    # 4.2: Mission Progress (Data Collection)
    ax_data = fig.add_subplot(gs[3, 1])
    if data_collected is not None and data_downlinked is not None:
        ax_data.plot(time_hours, data_collected, 'b-', linewidth=2, label='Data Collected', alpha=0.8)
        ax_data.plot(time_hours, data_downlinked, 'g-', linewidth=2, label='Data Downlinked', alpha=0.8)
        ax_data.fill_between(time_hours, 0, data_collected, alpha=0.2, color='blue')
        ax_data.fill_between(time_hours, 0, data_downlinked, alpha=0.2, color='green')
        ax_data.set_xlabel('Time [hours]', fontsize=9)
        ax_data.set_ylabel('Data [units]', fontsize=9)
        ax_data.set_title('Mission Data Budget', fontsize=11, fontweight='bold')
        ax_data.legend(fontsize=8)
        ax_data.grid(True, alpha=0.3)
        ax_data.set_ylim(bottom=0)
    else:
        ax_data.text(0.5, 0.5, 'No mission progress data', ha='center', va='center', transform=ax_data.transAxes)
        ax_data.set_title('Mission Data Budget', fontsize=11, fontweight='bold')
    
    # 4.3: Mission Statistics Summary
    ax_stats = fig.add_subplot(gs[3, 2])
    ax_stats.axis('off')
    
    # Calculate final statistics
    altitude_final = (np.linalg.norm(positions[-1]) - 6378137.0) / 1000.0
    velocity_final = np.linalg.norm(velocities[-1]) / 1000.0
    rate_final = np.linalg.norm(rates[-1])
    
    stats_text = f"""Mission Summary
    
Duration: {time_hours[-1]:.2f} hours ({times[-1]/60:.1f} min)

Final Orbit:
• Altitude: {altitude_final:.1f} km
• Velocity: {velocity_final:.2f} km/s
• SMA: {elements['a'][-1]/1000.0:.1f} km
• Eccentricity: {elements['e'][-1]:.6f}
• Inclination: {np.degrees(elements['i'][-1]):.2f}°

Final Attitude:
• Rate: {rate_final*1000:.2f} mrad/s
  ({np.degrees(rate_final):.3f} deg/s)"""
    
    if pointing_error is not None:
        stats_text += f"\n• Pointing Error: {pointing_error[-1]:.3f}°"
    
    if modes is not None:
        stats_text += f"\n• Final Mode: {data_loader.get_mode_name(modes[-1])}"
    
    if data_downlinked is not None:
        stats_text += f"\n\nMission Data:\n• Downlinked: {data_downlinked[-1]:.2f} units"
    
    if len(transitions) > 0:
        stats_text += f"\n• Mode Transitions: {len(transitions)}"
    
    ax_stats.text(0.05, 0.5, stats_text, fontsize=9, 
                 verticalalignment='center', family='monospace',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.3))
    ax_stats.set_title('Mission Statistics', fontsize=11, fontweight='bold')
    
    plt.tight_layout()
    
    if save_path:
        print(f"Saving figure to: {save_path}")
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print("Figure saved successfully")
    
    return fig

def main():
    """Main entry point for visualization script."""
    if len(sys.argv) < 2:
        print("Usage: python visualize_mission.py <mission_data.csv> [--save]")
        print("\nExample:")
        print("  python visualize_mission.py build/mission_data.csv")
        print("  python visualize_mission.py build/mission_data.csv --save")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    save_output = '--save' in sys.argv
    
    csv_path = Path(csv_file)
    if not csv_path.exists():
        print(f"Error: File not found: {csv_file}")
        print("\nPlease run mission_test first:")
        print("  cd build")
        print("  .\\tests\\Release\\mission_test.exe --gtest_filter=MissionTest.FullMissionSimulation")
        sys.exit(1)
    
    print("=" * 60)
    print("CubeSat Mission Visualization Dashboard")
    print("=" * 60)
    print(f"Data file: {csv_file}\n")
    
    try:
        save_path = csv_path.parent / "mission_dashboard.png" if save_output else None
        fig = create_comprehensive_dashboard(csv_file, save_path)
        
        print("\nDashboard created successfully!")
        if not save_output:
            print("\nClose the window to exit.")
            plt.show()
        
    except Exception as e:
        print(f"\nError creating dashboard: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
