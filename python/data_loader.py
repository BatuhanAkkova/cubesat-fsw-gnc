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
