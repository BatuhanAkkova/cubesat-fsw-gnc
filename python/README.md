# Python Visualization Tools - Usage Guide

This directory contains Python tools for visualizing CubeSat GNC simulation data exported from `mission_test.cpp`.

## Overview

The visualization system provides comprehensive analysis of the full mission timeline, including:
- **Orbital Dynamics**: 3D trajectory, ground track, orbital elements
- **Attitude Dynamics**: Quaternions, Euler angles, angular rates  
- **GNC Performance**: Pointing errors, control torques, reaction wheel momentum
- **Mission Progress**: Mode timeline, data collection/downlink metrics

## Quick Start

### 1. Install Dependencies

```powershell
cd c:\Users\batuh\OneDrive\Desktop\cubesat-fsw-gnc
python -m pip install -r python/requirements.txt
```

### 2. Generate Mission Data

```powershell
cd build
.\tests\Release\mission_test.exe --gtest_filter=MissionTest.FullMissionSimulation
```

This generates `mission_data.csv` with comprehensive telemetry (~30 columns).

### 3. Visualize the Mission

```powershell
# From project root
cd c:\Users\batuh\OneDrive\Desktop\cubesat-fsw-gnc

# Create interactive dashboard
python python/visualize_mission.py build/mission_data.csv

# Save dashboard as PNG
python python/visualize_mission.py build/mission_data.csv --save
```

## Files

### Core Modules

#### `data_loader.py`
Utilities for loading and parsing CSV data exported by C++ `DataLogger`.

**Key Functions:**
- `load_simulation_data(filepath)` - Load CSV data into DataFrame
- `extract_quaternions(df)` - Extract attitude quaternions [w,x,y,z]
- `extract_angular_rates(df)` - Extract angular rates [wx,wy,wz]
- `extract_position(df)` - Extract ECI position [rx,ry,rz]
- `extract_velocity(df)` - Extract ECI velocity [vx,vy,vz]
- `extract_control_torques(df)` - Extract commanded and external torques
- `extract_target_quaternion(df)` - Extract target attitude
- `extract_pointing_error(df)` - Extract pointing error time series
- `extract_momentum(df)` - Extract reaction wheel momentum
- `extract_mission_progress(df)` - Extract data collection metrics
- `get_mode_transitions(df)` - Detect mode transition events
- `quaternion_to_rotation_matrix(quat)` - Convert quaternion to rotation matrix
- `quaternion_to_euler(quat)` - Convert quaternion to Euler angles
- `eci_to_latlon(pos, times)` - Convert ECI to lat/lon for ground track
- `compute_orbital_elements_from_state(pos, vel)` - Compute SMA, ecc, inc, RAAN, argp

#### `visualize_mission.py` **Main Visualization Script**
Comprehensive dashboard showing all mission aspects in a single 12-panel view.

**Dashboard Layout (4 rows × 3 columns):**

**Row 1: Orbital Dynamics**
- 3D orbit trajectory with Earth (mode-colored)
- Ground track (lat/lon projection)
- Orbital elements evolution (SMA, inclination)

**Row 2: Attitude Dynamics**
- 3D attitude visualization (final state)
- Quaternion components time series
- Angular rates (ωx, ωy, ωz, |ω|)

**Row 3: GNC Performance**
- Pointing error vs time (mode-colored, with transitions)
- Control torques (commanded + external/Bdot)
- Reaction wheel momentum buildup

**Row 4: Mission Progress & Health**
- Mode timeline (color-coded bands)
- Data collection/downlink progress
- Mission statistics summary

**Features:**
- Mode-based color coding throughout
- Mode transition event markers
- Automatic layout and scaling
- Rich statistical summary
- PNG export capability



## Mission Data CSV Format

The `mission_data.csv` file contains the following columns:

### Basic State (14 columns)
- `time` - Mission elapsed time [s]
- `qw, qx, qy, qz` - Attitude quaternion (scalar-first)
- `wx, wy, wz` - Angular rates [rad/s]
- `rx, ry, rz` - ECI position [m]
- `vx, vy, vz` - ECI velocity [m/s]
- `mode` - Mission mode (0=SAFE, 1=NOMINAL, 2=SCIENCE, 3=DOWNLINK)

### GNC Control Data (13 columns)
- `torque_cmd_x/y/z` - Commanded control torques [Nm]
- `torque_ext_x/y/z` - External torques (magnetorquer) [Nm]
- `qw_target, qx_target, qy_target, qz_target` - Target quaternion
- `pointing_error` - Angular pointing error [deg]
- `momentum_x/y/z` - Reaction wheel momentum [Nms]

### Mission Progress (2 columns)
- `data_collected` - Science data collected [units]
- `data_downlinked` - Data downlinked [units]

**Total: 29 columns**

## Mission Phases

The visualization automatically detects and color-codes mission phases:

1. **SAFE Mode** (Red) - Initial detumbling with B-dot control
2. **NOMINAL Mode** (Cyan) - Sun-pointing for power generation
3. **SCIENCE Mode** (Blue) - Target tracking and data collection
4. **DOWNLINK Mode** (Green) - Ground station communication

Mode transitions are marked with vertical lines on time-series plots.

## Usage Examples

### Basic Usage
```powershell
python python/visualize_mission.py build/mission_data.csv
```

### Save Dashboard
```powershell
python python/visualize_mission.py build/mission_data.csv --save
# Saves to: build/mission_dashboard.png
```

### Programmatic Use
```python
import sys
sys.path.insert(0, 'python')
import data_loader
import visualize_mission

# Load data
df = data_loader.load_simulation_data('build/mission_data.csv')

# Get mode transitions
transitions = data_loader.get_mode_transitions(df)
print(f"Detected {len(transitions)} mode transitions:")
for t, old_mode, new_mode in transitions:
    print(f"  {t/60:.1f} min: {old_mode} -> {new_mode}")

# Create dashboard
fig = visualize_mission.create_comprehensive_dashboard('build/mission_data.csv')
import matplotlib.pyplot as plt
plt.show()
```

## Common Issues

### "FileNotFoundError: Data file not found"
**Solution:** Always run from project root and use relative paths:
```powershell
cd c:\Users\batuh\OneDrive\Desktop\cubesat-fsw-gnc
python python/visualize_mission.py build/mission_data.csv
```

### Missing CSV file
**Solution:** Run the C++ test first:
```powershell
cd build
.\tests\Release\mission_test.exe --gtest_filter=MissionTest.FullMissionSimulation
```

### Module import errors
**Solution:** Run from project root, not from `python/` directory.

## Performance

- **Data loading**: < 1 second for typical mission (1500s, 300 samples)
- **Dashboard rendering**: 3-5 seconds
- **Memory usage**: < 200 MB
- **PNG export**: ~2 MB at 150 DPI

## Tips

1. **Always run from project root** (`cubesat-fsw-gnc/` directory)
2. **Use `--save` flag** for non-interactive batch processing
3. **Check mode transitions** to understand mission timeline
4. **Look for momentum saturation** in Row 3, Panel 3
5. **Verify pointing convergence** in Row 3, Panel 1

## Dependencies

```
numpy >= 1.20.0
scipy >= 1.7.0
matplotlib >= 3.4.0
pandas >= 1.3.0
```

Install all dependencies:
```powershell
python -m pip install -r python/requirements.txt
```