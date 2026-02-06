# CubeSat Flight Software & Guidance, Navigation, and Control (GNC)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A high-performance C++ Flight Software (FSW) and GNC simulation framework for CubeSats. This project implements a modular architecture, transitioning from pure software simulation to hardware-in-the-loop (HIL) readiness.

## Overview

This repository contains a complete FSW stack including attitude determination, control logic, failure detection (FDIR), and a high-fidelity simulation engine. The system is designed with a "Single Source of Truth" approach using a central DataStore for inter-module communication.

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
-   **FDIR (Fault Detection, Isolation, and Recovery)**: Sensor health monitoring and automatic mode switching.
-   **Communication & Command**:
    -   **CCSDS Telemetry**: Packet encoding for attitude, orbit, and health data.
    -   **Command Handling**: Robust command parsing and execution (e.g., "Slew to Nadir", "Set PID Gains").
    -   **Ground Station Simulation**: Simulated ground segment for commanding and telemetry monitoring.
-   **Optimization**: Genetic algorithms for automatic controller gain tuning and Monte Carlo robustness analysis.

## Project Structure

```text
├── src/
│   ├── common/         # Common math types, time, and logging utilities
│   ├── fsw/            # Flight Software core
│   │   ├── core/       # DataStore, ModeManager, TaskScheduler, Command Handling
│   │   ├── gnc/        # MEKF, PID, B-Dot, Pointing Strategies
│   │   ├── fdir/       # Fault Detection and Recovery
│   │   └── telemetry/  # CCSDS Telemetry Encoding
│   ├── hal/            # Hardware Abstraction Layer interfaces
│   ├── sim/            # Simulation engine, hardware models, and Ground Station
│   └── opt/            # Optimization tools (Genetic Algorithms)
├── tests/              # Unit tests and integration/mission demos
```

## Getting Started

### Prerequisites

-   CMake (>= 3.16)
-   C++17 Compiler (GCC/Clang/MSVC)
-   Dependencies (Automated via CMake): Eigen, spdlog, googletest

### Build Instructions

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
```

### Running Tests

```powershell
cd build
ctest -C Release --output-on-failure
```

## Future Roadmap (Upcoming Phases)

The following phases are planned for upcoming development cycles:

### Mission Operations & Ground Segment
-   [ ]**Mission Scripting**: Full mission timeline simulation (Deployment -> Science -> Downlink).
-   [ ]**Performance profiling**: Measure actual overhead in full mission simulation.

### Simulation and Visualization in Python
-   [ ]Implement a Python-based simulation environment for rapid prototyping.
-   [ ]Create 3D visualizations for attitude and orbit verification.

### Example Simulations
-   [ ]**Detumble Simulation**: Verify B-Dot performance from high tip-off rates.
-   [ ]**Pointing Simulation**: Verify PID/LQG slew performance and settling time.
-   [ ]**Full Mission Simulation**: End-to-end verification (Deploy -> Detumble -> Pointing).

### Virtual HIL
-   [ ]**Virtual Bus Interface**: Implement a virtual I2C/SPI interface.
-   [ ]**Virtual Sensors**: Implement a virtual sensor interface with quantization.
-   [ ]**Virtual Driver**: Implement a driver for FSW, virtual MPU6050.

*Developed for advanced CubeSat mission modeling and flight software development.*
---

## License

MIT License. See [LICENSE](LICENSE) for details.

## Author

**Batuhan Akkova**
[Email](mailto:batuhanakkova1@gmail.com)
