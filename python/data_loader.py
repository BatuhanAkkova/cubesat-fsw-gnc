"""
Data loader for C++ simulation CSV output files.

This module provides utilities to load and parse CSV files generated
by the C++ simulation with DataLogger.
"""

import pandas as pd
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation as R


def load_simulation_data(filepath):
    """
    Load simulation data from CSV file.
    
    Args:
        filepath: Path to CSV file
        
    Returns:
        pandas.DataFrame with simulation data
    """
    filepath = Path(filepath)
    if not filepath.exists():
        raise FileNotFoundError(f"Data file not found: {filepath}")
    
    df = pd.read_csv(filepath)
    
    # Validate required columns
    if 'time' not in df.columns:
        raise ValueError("CSV must contain 'time' column")
    
    return df


def extract_quaternions(df):
    """
    Extract quaternion data from DataFrame.
    
    Args:
        df: DataFrame with columns qw, qx, qy, qz
        
    Returns:
        numpy array of shape (N, 4) with [w, x, y, z] quaternions
    """
    required_cols = ['qw', 'qx', 'qy', 'qz']
    if not all(col in df.columns for col in required_cols):
        raise ValueError(f"DataFrame must contain quaternion columns: {required_cols}")
    
    return df[required_cols].values


def extract_angular_rates(df):
    """
    Extract angular rate data from DataFrame.
    
    Args:
        df: DataFrame with columns wx, wy, wz
        
    Returns:
        numpy array of shape (N, 3) with [wx, wy, wz] rates in rad/s
    """
    required_cols = ['wx', 'wy', 'wz']
    if not all(col in df.columns for col in required_cols):
        raise ValueError(f"DataFrame must contain rate columns: {required_cols}")
    
    return df[required_cols].values


def extract_position(df):
    """
    Extract position data from DataFrame.
    
    Args:
        df: DataFrame with columns rx, ry, rz
        
    Returns:
        numpy array of shape (N, 3) with [rx, ry, rz] position in meters
    """
    required_cols = ['rx', 'ry', 'rz']
    if not all(col in df.columns for col in required_cols):
        raise ValueError(f"DataFrame must contain position columns: {required_cols}")
    
    return df[required_cols].values


def quaternion_to_rotation_matrix(quat):
    """
    Convert quaternion(s) to rotation matrix/matrices.
    
    Args:
        quat: Quaternion as [w, x, y, z] or array of shape (N, 4)
        
    Returns:
        Rotation matrix (3x3) or array of matrices (Nx3x3)
    """
    quat = np.atleast_2d(quat)
    
    # scipy expects [x, y, z, w], we have [w, x, y, z]
    quat_scipy = quat[:, [1, 2, 3, 0]]
    
    r = R.from_quat(quat_scipy)
    matrices = r.as_matrix()
    
    if matrices.shape[0] == 1:
        return matrices[0]
    return matrices


def quaternion_to_euler(quat, seq='ZYX'):
    """
    Convert quaternion(s) to Euler angles.
    
    Args:
        quat: Quaternion as [w, x, y, z] or array of shape (N, 4)
        seq: Euler angle sequence (default 'ZYX' for roll-pitch-yaw)
        
    Returns:
        Euler angles in radians, shape (3,) or (N, 3)
    """
    quat = np.atleast_2d(quat)
    
    # scipy expects [x, y, z, w], we have [w, x, y, z]
    quat_scipy = quat[:, [1, 2, 3, 0]]
    
    r = R.from_quat(quat_scipy)
    euler = r.as_euler(seq, degrees=False)
    
    if euler.shape[0] == 1:
        return euler[0]
    return euler


def get_mode_name(mode_int):
    """
    Convert mission mode integer to string name.
    
    Args:
        mode_int: Mode as integer
        
    Returns:
        Mode name string
    """
    mode_map = {
        0: 'SAFE',
        1: 'NOMINAL',
        2: 'SCIENCE',
        3: 'DOWNLINK',
        4: 'MAINTENANCE'
    }
    return mode_map.get(int(mode_int), 'UNKNOWN')


def compute_quaternion_error(q_current, q_target):
    """
    Compute quaternion error between current and target.
    
    Args:
        q_current: Current quaternion [w, x, y, z]
        q_target: Target quaternion [w, x, y, z]
        
    Returns:
        Angular error in radians
    """
    # Ensure quaternions are normalized
    q_current = q_current / np.linalg.norm(q_current)
    q_target = q_target / np.linalg.norm(q_target)
    
    # Compute dot product
    dot_product = np.dot(q_current, q_target)
    
    # Clamp to valid range
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # Compute angular distance
    return 2.0 * np.arccos(np.abs(dot_product))


def extract_velocity(df):
    """
    Extract velocity data from DataFrame.
    
    Args:
        df: DataFrame with columns vx, vy, vz
        
    Returns:
        numpy array of shape (N, 3) with [vx, vy, vz] velocities in m/s
    """
    required_cols = ['vx', 'vy', 'vz']
    if not all(col in df.columns for col in required_cols):
        raise ValueError(f"DataFrame must contain velocity columns: {required_cols}")
    
    return df[required_cols].values


def eci_to_latlon(pos_eci, time_seconds=None):
    """
    Convert ECI position to geodetic latitude and longitude.
    
    Args:
        pos_eci: Position in ECI frame [x, y, z] in meters, or array of shape (N, 3)
        time_seconds: Time since epoch in seconds (for Earth rotation)
                     If None, assumes Earth rotation matches data index
        
    Returns:
        lat, lon in degrees (or arrays of shape (N,) if input is (N, 3))
    """
    pos_eci = np.atleast_2d(pos_eci)
    
    # Earth rotation rate (rad/s)
    omega_earth = 7.2921159e-5  # rad/s
    
    # Constants
    R_EARTH = 6378137.0  # m
    
    latitudes = []
    longitudes = []
    
    for i, pos in enumerate(pos_eci):
        # Compute geocentric latitude (approximate geodetic for visualization)
        r_xy = np.sqrt(pos[0]**2 + pos[1]**2)
        lat = np.arctan2(pos[2], r_xy)
        
        # Compute longitude in ECI frame
        lon_eci = np.arctan2(pos[1], pos[0])
        
        # Apply Earth rotation if time is provided
        if time_seconds is not None:
            if isinstance(time_seconds, (list, np.ndarray)):
                t = time_seconds[i]
            else:
                t = time_seconds
            # Subtract Earth rotation angle (ECEF rotates with Earth)
            theta_earth = omega_earth * t
            lon = lon_eci - theta_earth
        else:
            lon = lon_eci
        
        # Normalize longitude to [-pi, pi]
        lon = np.arctan2(np.sin(lon), np.cos(lon))
        
        latitudes.append(np.degrees(lat))
        longitudes.append(np.degrees(lon))
    
    lat_array = np.array(latitudes)
    lon_array = np.array(longitudes)
    
    if len(lat_array) == 1:
        return lat_array[0], lon_array[0]
    return lat_array, lon_array


def compute_orbital_elements_from_state(pos, vel):
    """
    Compute classical orbital elements from position and velocity vectors.
    
    Args:
        pos: Position vector [x, y, z] in meters (or array of shape (N, 3))
        vel: Velocity vector [vx, vy, vz] in m/s (or array of shape (N, 3))
        
    Returns:
        dict with keys: 'a' (semi-major axis, m), 'e' (eccentricity),
                       'i' (inclination, rad), 'raan' (rad), 'argp' (rad)
    """
    MU_EARTH = 3.986004418e14  # m^3/s^2
    
    pos = np.atleast_2d(pos)
    vel = np.atleast_2d(vel)
    
    elements = {
        'a': [],
        'e': [],
        'i': [],
        'raan': [],
        'argp': []
    }
    
    for p, v in zip(pos, vel):
        r = np.linalg.norm(p)
        v_mag = np.linalg.norm(v)
        
        # Specific orbital energy
        energy = 0.5 * v_mag**2 - MU_EARTH / r
        
        # Semi-major axis
        a = -MU_EARTH / (2.0 * energy)
        
        # Angular momentum vector
        h = np.cross(p, v)
        h_mag = np.linalg.norm(h)
        
        # Eccentricity vector
        e_vec = np.cross(v, h) / MU_EARTH - p / r
        e = np.linalg.norm(e_vec)
        
        # Inclination
        i = np.arccos(h[2] / h_mag)
        
        # Node vector
        k = np.array([0, 0, 1])
        n = np.cross(k, h)
        n_mag = np.linalg.norm(n)
        
        # RAAN
        if n_mag > 1e-10:
            raan = np.arctan2(n[1], n[0])
        else:
            raan = 0.0
        
        # Argument of perigee
        if e > 1e-10 and n_mag > 1e-10:
            cos_argp = np.dot(n, e_vec) / (n_mag * e)
            argp = np.arccos(np.clip(cos_argp, -1.0, 1.0))
            if e_vec[2] < 0:
                argp = 2.0 * np.pi - argp
        else:
            argp = 0.0
        
        elements['a'].append(a)
        elements['e'].append(e)
        elements['i'].append(i)
        elements['raan'].append(raan)
        elements['argp'].append(argp)
    
    # Convert lists to arrays
    for key in elements:
        elements[key] = np.array(elements[key])
        if len(elements[key]) == 1:
            elements[key] = elements[key][0]
    
    return elements


def extract_control_torques(df):
    """
    Extract commanded and external control torques from DataFrame.
    
    Args:
        df: DataFrame with columns torque_cmd_x/y/z and torque_ext_x/y/z
        
    Returns:
        Tuple of (torque_cmd, torque_ext) as numpy arrays of shape (N, 3)
    """
    cmd_cols = ['torque_cmd_x', 'torque_cmd_y', 'torque_cmd_z']
    ext_cols = ['torque_ext_x', 'torque_ext_y', 'torque_ext_z']
    
    torque_cmd = None
    torque_ext = None
    
    if all(col in df.columns for col in cmd_cols):
        torque_cmd = df[cmd_cols].values
    
    if all(col in df.columns for col in ext_cols):
        torque_ext = df[ext_cols].values
    
    return torque_cmd, torque_ext


def extract_target_quaternion(df):
    """
    Extract target quaternion data from DataFrame.
    
    Args:
        df: DataFrame with columns qw_target, qx_target, qy_target, qz_target
        
    Returns:
        numpy array of shape (N, 4) with [w, x, y, z] quaternions or None
    """
    required_cols = ['qw_target', 'qx_target', 'qy_target', 'qz_target']
    
    if not all(col in df.columns for col in required_cols):
        return None
    
    return df[required_cols].values


def extract_pointing_error(df):
    """
    Extract pointing error time series from DataFrame.
    
    Args:
        df: DataFrame with column pointing_error
        
    Returns:
        numpy array of pointing errors in degrees or None
    """
    if 'pointing_error' not in df.columns:
        return None
    
    return df['pointing_error'].values


def extract_momentum(df):
    """
    Extract reaction wheel momentum from DataFrame.
    
    Args:
        df: DataFrame with columns momentum_x, momentum_y, momentum_z
        
    Returns:
        numpy array of shape (N, 3) with momentum values or None
    """
    required_cols = ['momentum_x', 'momentum_y', 'momentum_z']
    
    if not all(col in df.columns for col in required_cols):
        return None
    
    return df[required_cols].values


def extract_mission_progress(df):
    """
    Extract mission progress data from DataFrame.
    
    Args:
        df: DataFrame with columns data_collected, data_downlinked
        
    Returns:
        Tuple of (data_collected, data_downlinked) as numpy arrays or (None, None)
    """
    if 'data_collected' not in df.columns or 'data_downlinked' not in df.columns:
        return None, None
    
    return df['data_collected'].values, df['data_downlinked'].values


def get_mode_transitions(df):
    """
    Detect mode transitions in the mission timeline.
    
    Args:
        df: DataFrame with 'mode' and 'time' columns
        
    Returns:
        List of tuples: (time, old_mode, new_mode)
    """
    if 'mode' not in df.columns or 'time' not in df.columns:
        return []
    
    transitions = []
    modes = df['mode'].values
    times = df['time'].values
    
    for i in range(1, len(modes)):
        if modes[i] != modes[i-1]:
            transitions.append((
                times[i],
                get_mode_name(modes[i-1]),
                get_mode_name(modes[i])
            ))
    
    return transitions

