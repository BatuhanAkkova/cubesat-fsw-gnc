# CubeSat Flight Software & Guidance, Navigation, and Control (GNC)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A high-performance C++ Flight Software (FSW) and GNC simulation framework for CubeSats. This project implements a modular architecture with comprehensive visualization capabilities, designed for transitioning from pure software simulation to hardware-in-the-loop (HIL) readiness.

## Mission Showcase

The system demonstrates a complete autonomous CubeSat mission from deployment through science operations to ground communication. The visualization below shows a full **60-minute mission timeline** with real-time telemetry across all subsystems.

### Mission Overview
![Mission Dashboard](docs/mission_dashboard.png)

### 1. Orbital Dynamics
![Orbit Visualization](docs/orbit_visualization.png)
The simulation propagates a realistic **Low Earth Orbit (LEO)** trajectory:
- **Altitude**: ~400 km (International Space Station orbit)
- **Inclination**: 51.6°
- **Perturbations**: J2 zonal harmonic effects included
- **Ground Track**: Shows global coverage over the 1-hour mission duration

### 2. Attitude Control & Detumbling
![Attitude Dynamics](docs/attitude_dynamics.png)
**Deployment Phase (0-2 min)**:
- Initial tumble rate: **0.12 rad/s** (~7 deg/s)
- **B-Dot Controller** (Red trace) rapidly detumbles the spacecraft using magnetorquers
- Stabilization achieved in **<2 minutes**

### 3. GNC Performance
![GNC Performance](docs/gnc_performance.png)
**Pointing Accuracy**:
- **NOMINAL Mode** (Cyan): Sun-pointing with <1° error
- **SCIENCE Mode** (Blue): Precise target tracking with <0.5° error
- **DOWNLINK Mode** (Green): Ground station pointing
- **Reaction Wheels**: Momentum is managed comfortably within limits (no saturation)

### 4. Autonomous Mission Timeline
![Mission Timeline](docs/mission_timeline.png)
The **Mode Manager** autonomously drives the mission state machine:
1. **SAFE Mode** (Red): Detumble and checkout
2. **NOMINAL Mode** (Cyan): Power positive, charge batteries
3. **SCIENCE Mode** (Blue): Perform payload operations (maximize data collection)
4. **DOWNLINK Mode** (Green): Transmit data to ground stations

**Mission Statistics (1 Hour Run)**:
- **Duration**: 60 minutes
- **Data Collected**: >10 units
- **Data Downlinked**: >10 units (100% throughput)
- **Mode Transitions**: 3 autonomous transitions verified

## Key Features

-   **Attitude Determination**: Multiplicative Extended Kalman Filter (MEKF) fusing Magnetometer, Sun Sensor, and Star Tracker data.
-   **Control Algorithms**:
    -   **B-Dot**: Magnetic detumbling for safe-mode operations.
    -   **PID & LQG**: Precision 3-axis pointing control using Reaction Wheels.
    -   **Wheel Desaturation**: Momentum management using magnetorquers.
-   **Simulation Engine**:
    -   High-fidelity rigid body dynamics (Euler's equations).
    -   Runge-Kutta 4 (RK4) integration.
    -   J2 Perturbation orbit propagation.
    -   Realistic sensor/actuator models with noise and latency.
-   **Mission Visualization**:
    -   **Comprehensive Dashboard**: 12-panel unified view of all mission aspects
    -   **3D Trajectory**: Interactive orbit visualization with Earth sphere.
    -   **Ground Track**: Latitude/longitude projection on 2D world map.
    -   **Orbital Elements**: Time-series analysis of Keplerian elements (SMA, eccentricity, inclination, RAAN, argument of perigee).
    -   **GNC Performance**: Real-time pointing error, control torques, momentum tracking
    -   **Mission Progress**: Mode transitions, data collection/downlink metrics
-   **FDIR (Fault Detection, Isolation, and Recovery)**: Sensor health monitoring and automatic mode switching.
-   **Communication & Command**:
    -   **CCSDS Telemetry**: Packet encoding for attitude, orbit, and health data.
    -   **Command Handling**: Robust command parsing and execution (e.g., "Slew to Nadir", "Set PID Gains").
    -   **Ground Station Simulation**: Simulated ground segment for commanding and telemetry monitoring.
-   **Optimization**: Genetic algorithms for automatic controller gain tuning and Monte Carlo robustness analysis.
-   **Autonomous Mission**: 
    -   **Full Timeline Simulation**: Deployment → Detumble → Science → Downlink
    -   **Auto Mode Transitions**: Rate-based and pointing-based state machine
    -   **Performance Profiling**: Real-time overhead measurement

## Project Structure

```text
├── src/
│   ├── common/         # Common math types, time, profiler and logging utilities
│   ├── fsw/            # Flight Software core
│   │   ├── core/       # DataStore, ModeManager, TaskScheduler, Command Handling
│   │   ├── gnc/        # MEKF, PID, B-Dot, Pointing Strategies
│   │   ├── fdir/       # Fault Detection and Recovery
│   │   └── telemetry/  # CCSDS Telemetry Encoding
│   ├── hal/            # Hardware Abstraction Layer interfaces
│   ├── sim/            # Simulation engine, hardware models, and Ground Station
│   └── opt/            # Optimization tools (Genetic Algorithms)
├── tests/              # Unit tests and integration/mission demos
├── python/             # Visualization and analysis tools
│   ├── visualize_mission.py    # Unified mission dashboard
│   ├── data_loader.py          # Data extraction utilities
│   └── README.md               # Detailed usage guide
```

## Getting Started

### Prerequisites

-   CMake (>= 3.16)
-   C++17 Compiler (GCC/Clang/MSVC)
-   Python 3.8+ (for visualization)
-   Dependencies (Automated via CMake): Eigen, spdlog, googletest

### Build Instructions

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### Running the Mission Simulation

```powershell
cd build
.\tests\Release\mission_test.exe --gtest_filter=MissionTest.FullMissionSimulation
```

**Output**: Generates `mission_data.csv` with 31 columns of telemetry:
- Basic state: time, quaternion, rates, position, velocity, mode
- GNC control: commanded/external torques, target attitude, pointing error, momentum
- Mission progress: data collected, data downlinked

### Visualizing Mission Results

```powershell
# From project root
# Install Python dependencies (first time only)
python -m pip install -r python/requirements.txt

# Create interactive dashboard
python python/visualize_mission.py build/mission_data.csv

# Or save to PNG
python python/visualize_mission.py build/mission_data.csv --save
```

**Output**: `build/mission_dashboard.png` - Comprehensive 12-panel visualization

See [`python/README.md`](python/README.md) for detailed visualization documentation.

## Understanding the Dashboard

The mission dashboard provides complete observability:

**Orbital Dynamics (Row 1)**:
- Left: 3D trajectory colored by mission mode
- Center: Ground track showing satellite path over Earth
- Right: Orbital elements (SMA and inclination vs time)

**Attitude Dynamics (Row 2)**:
- Left: 3D body frame visualization (final state)
- Center: Quaternion components showing smooth attitude evolution
- Right: Angular rates with detumble visible in first 80 seconds

**GNC Performance (Row 3)**:
- Left: Pointing error - spikes during slew maneuvers, convergence in each mode
- Center: Control torques - B-dot (external) in SAFE, PID (commanded) in other modes
- Right: Reaction wheel momentum buildup from control activity

**Mission Progress (Row 4)**:
- Left: Mode timeline with color-coded phases
- Center: Data collection during SCIENCE, downlink during DOWNLINK
- Right: Mission statistics summary (orbit, attitude, data metrics)

## Future Roadmap (Upcoming Phases)

### Extended Mission Scenarios
-   [ ] **Constellation Support**: Multi-satellite coordination.
-   [ ] **Eclipse Modeling**: Power-constrained operations.
-   [ ] **Advanced FDIR**: Sensor failure recovery demonstrations.

### Virtual HIL
-   [ ] **Virtual Bus Interface**: Implement a virtual I2C/SPI interface.
-   [ ] **Virtual Sensors**: Implement a virtual sensor interface with quantization.
-   [ ] **Virtual Driver**: Implement a driver for FSW, virtual MPU6050.

*Developed for advanced CubeSat mission modeling and flight software development.*
---

## License

MIT License. See [LICENSE](LICENSE) for details.

## Author

**Batuhan Akkova**  
[Email](mailto:batuhanakkova1@gmail.com)
