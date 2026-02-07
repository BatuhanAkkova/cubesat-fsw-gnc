"""
3D Attitude Visualization for CubeSat GNC.

Provides interactive 3D visualization of spacecraft attitude using quaternion data.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import data_loader


def plot_body_frame(ax, rotation_matrix, origin=np.zeros(3), scale=1.0, alpha=1.0):
    """
    Plot spacecraft body frame axes.
    
    Args:
        ax: matplotlib 3D axis
        rotation_matrix: 3x3 rotation matrix from inertial to body
        origin: Origin point for the frame
        scale: Length of axis arrows
        alpha: Transparency
    """
    # Body frame axes in body coordinates
    x_body = np.array([1, 0, 0])
    y_body = np.array([0, 1, 0])
    z_body = np.array([0, 0, 1])
    
    # Rotate to inertial frame
    x_inertial = rotation_matrix @ x_body
    y_inertial = rotation_matrix @ y_body
    z_inertial = rotation_matrix @ z_body
    
    # Plot arrows
    ax.quiver(*origin, *x_inertial*scale, color='r', alpha=alpha, arrow_length_ratio=0.15, linewidth=2, label='X (Roll)')
    ax.quiver(*origin, *y_inertial*scale, color='g', alpha=alpha, arrow_length_ratio=0.15, linewidth=2, label='Y (Pitch)')
    ax.quiver(*origin, *z_inertial*scale, color='b', alpha=alpha, arrow_length_ratio=0.15, linewidth=2, label='Z (Yaw)')


def plot_cubesat_body(ax, rotation_matrix, size=0.5, alpha=0.3):
    """
    Plot a simple cube representing the CubeSat body.
    
    Args:
        ax: matplotlib 3D axis
        rotation_matrix: 3x3 rotation matrix
        size: Cube size
        alpha: Transparency
    """
    # Define cube vertices in body frame
    vertices = np.array([
        [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],  # Bottom face
        [-1, -1,  1], [1, -1,  1], [1,  1,  1], [-1,  1,  1]   # Top face
    ]) * size / 2
    
    # Rotate vertices to inertial frame
    rotated_vertices = (rotation_matrix @ vertices.T).T
    
    # Define cube faces
    faces = [
        [rotated_vertices[j] for j in [0, 1, 2, 3]],  # Bottom
        [rotated_vertices[j] for j in [4, 5, 6, 7]],  # Top
        [rotated_vertices[j] for j in [0, 1, 5, 4]],  # Front
        [rotated_vertices[j] for j in [2, 3, 7, 6]],  # Back
        [rotated_vertices[j] for j in [0, 3, 7, 4]],  # Left
        [rotated_vertices[j] for j in [1, 2, 6, 5]]   # Right
    ]
    
    # Plot faces
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    face_collection = Poly3DCollection(faces, alpha=alpha, facecolors='cyan', edgecolors='k', linewidths=0.5)
    ax.add_collection3d(face_collection)


def visualize_attitude_static(csv_file, time_index=-1):
    """
    Create static 3D visualization of attitude at a specific time.
    
    Args:
        csv_file: Path to mission data CSV
        time_index: Index of time point to visualize (-1 for last)
    """
    # Load data
    df = data_loader.load_simulation_data(csv_file)
    quats = data_loader.extract_quaternions(df)
    
    # Get quaternion at specified time
    quat = quats[time_index]
    rot_mat = data_loader.quaternion_to_rotation_matrix(quat)
    
    # Create figure
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot inertial frame (for reference)
    scale = 1.5
    ax.quiver(0, 0, 0, scale, 0, 0, color='r', alpha=0.3, arrow_length_ratio=0.1, linestyle='--', label='X_inertial')
    ax.quiver(0, 0, 0, 0, scale, 0, color='g', alpha=0.3, arrow_length_ratio=0.1, linestyle='--', label='Y_inertial')
    ax.quiver(0, 0, 0, 0, 0, scale, color='b', alpha=0.3, arrow_length_ratio=0.1, linestyle='--', label='Z_inertial')
    
    # Plot body frame
    plot_body_frame(ax, rot_mat, scale=1.0, alpha=1.0)
    plot_cubesat_body(ax, rot_mat, size=0.6, alpha=0.4)
    
    # Set axis properties
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(f'CubeSat Attitude at t={df["time"].iloc[time_index]:.1f}s')
    
    # Set equal aspect ratio
    max_range = scale * 1.2
    ax.set_xlim([-max_range, max_range])
    ax.set_ylim([-max_range, max_range])
    ax.set_zlim([-max_range, max_range])
    
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig, ax


def animate_attitude(csv_file, output_file=None, fps=10, skip_frames=5):
    """
    Create animated 3D visualization of attitude over time.
    
    Args:
        csv_file: Path to mission data CSV
        output_file: Optional path to save animation (e.g., 'attitude.gif' or 'attitude.mp4')
        fps: Frames per second for animation
        skip_frames: Skip this many data points between frames
    """
    # Load data
    df = data_loader.load_simulation_data(csv_file)
    quats = data_loader.extract_quaternions(df)
    times = df['time'].values
    
    # Subsample data for smoother animation
    indices = np.arange(0, len(quats), skip_frames)
    quats = quats[indices]
    times = times[indices]
    
    # Create figure
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Initialize plot elements
    scale = 1.5
    
    def init():
        ax.clear()
        # Plot inertial frame
        ax.quiver(0, 0, 0, scale, 0, 0, color='r', alpha=0.2, arrow_length_ratio=0.1, linestyle='--')
        ax.quiver(0, 0, 0, 0, scale, 0, color='g', alpha=0.2, arrow_length_ratio=0.1, linestyle='--')
        ax.quiver(0, 0, 0, 0, 0, scale, color='b', alpha=0.2, arrow_length_ratio=0.1, linestyle='--')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        
        max_range = scale * 1.2
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])
        ax.grid(True, alpha=0.3)
        
        return []
    
    def update(frame):
        ax.clear()
        
        # Plot inertial frame
        ax.quiver(0, 0, 0, scale, 0, 0, color='r', alpha=0.2, arrow_length_ratio=0.1, linestyle='--')
        ax.quiver(0, 0, 0, 0, scale, 0, color='g', alpha=0.2, arrow_length_ratio=0.1, linestyle='--')
        ax.quiver(0, 0, 0, 0, 0, scale, color='b', alpha=0.2, arrow_length_ratio=0.1, linestyle='--')
        
        # Get current quaternion and rotation matrix
        quat = quats[frame]
        rot_mat = data_loader.quaternion_to_rotation_matrix(quat)
        
        # Plot spacecraft
        plot_body_frame(ax, rot_mat, scale=1.0, alpha=1.0)
        plot_cubesat_body(ax, rot_mat, size=0.6, alpha=0.5)
        
        # Set title with current time
        ax.set_title(f'CubeSat Attitude - Time: {times[frame]:.1f}s', fontsize=14, fontweight='bold')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        
        max_range = scale * 1.2
        ax.set_xlim([-max_range, max_range])
        ax.set_ylim([-max_range, max_range])
        ax.set_zlim([-max_range, max_range])
        ax.grid(True, alpha=0.3)
        
        return []
    
    # Create animation
    anim = FuncAnimation(fig, update, init_func=init, frames=len(quats), 
                        interval=1000/fps, blit=False, repeat=True)
    
    # Save or show
    if output_file:
        print(f"Saving animation to {output_file}...")
        if output_file.endswith('.gif'):
            anim.save(output_file, writer='pillow', fps=fps)
        elif output_file.endswith('.mp4'):
            anim.save(output_file, writer='ffmpeg', fps=fps)
        print("Animation saved!")
    else:
        plt.show()
    
    return fig, anim


def plot_attitude_timeseries(csv_file):
    """
    Plot attitude quaternion and angular rates vs time.
    
    Args:
        csv_file: Path to mission data CSV
    """
    # Load data
    df = data_loader.load_simulation_data(csv_file)
    times = df['time'].values
    quats = data_loader.extract_quaternions(df)
    rates = data_loader.extract_angular_rates(df)
    
    # Convert to Euler angles for easier interpretation
    euler = data_loader.quaternion_to_euler(quats, seq='ZYX')
    
    # Create subplots
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    
    # Plot quaternions
    axes[0].plot(times, quats[:, 0], 'k-', label='qw', linewidth=1.5)
    axes[0].plot(times, quats[:, 1], 'r-', label='qx', alpha=0.7)
    axes[0].plot(times, quats[:, 2], 'g-', label='qy', alpha=0.7)
    axes[0].plot(times, quats[:, 3], 'b-', label='qz', alpha=0.7)
    axes[0].set_ylabel('Quaternion')
    axes[0].set_title('Attitude Quaternion vs Time')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    
    # Plot Euler angles
    axes[1].plot(times, np.rad2deg(euler[:, 0]), 'r-', label='Roll (Z)', linewidth=1.5)
    axes[1].plot(times, np.rad2deg(euler[:, 1]), 'g-', label='Pitch (Y)', linewidth=1.5)
    axes[1].plot(times, np.rad2deg(euler[:, 2]), 'b-', label='Yaw (X)', linewidth=1.5)
    axes[1].set_ylabel('Euler Angles (deg)')
    axes[1].set_title('Euler Angles (ZYX) vs Time')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    
    # Plot angular rates
    axes[2].plot(times, rates[:, 0], 'r-', label='ωx', linewidth=1.5)
    axes[2].plot(times, rates[:, 1], 'g-', label='ωy', linewidth=1.5)
    axes[2].plot(times, rates[:, 2], 'b-', label='ωz', linewidth=1.5)
    axes[2].set_xlabel('Time (s)')
    axes[2].set_ylabel('Angular Rate (rad/s)')
    axes[2].set_title('Angular Rates vs Time')
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig, axes


if __name__ == '__main__':
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python visualize_attitude.py <csv_file> [--animate] [--save <output_file>]")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    if '--animate' in sys.argv:
        output_file = None
        if '--save' in sys.argv:
            save_idx = sys.argv.index('--save')
            if save_idx + 1 < len(sys.argv):
                output_file = sys.argv[save_idx + 1]
        
        animate_attitude(csv_file, output_file=output_file, fps=10, skip_frames=10)
    else:
        # Show static visualization and time series
        visualize_attitude_static(csv_file)
        plot_attitude_timeseries(csv_file)
        plt.show()
