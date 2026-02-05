# Enhanced CubeSat FSW & GNC Project Structure

Based on the initial `init.md`, this document outlines a more comprehensive project structure, advanced principles, and specific algorithms suitable for a professional-grade CubeSat Flight Software (FSW) and Guidance, Navigation, and Control (GNC) system.

## 1. key Principles (Expanded)

In addition to the core principles (High-perf C++, Determinism, Pseudo-Hardware), we add:

*   **Safety & Fault Tolerance**:
    *   **Watchdogs**: Hardware and Software watchdogs to reset the system if it hangs.
    *   **Safe Mode**: A minimal operational mode (tumbling, basic beacon) that the system reverts to upon critical failure.
    *   **FDIR (Fault Detection, Isolation, and Recovery)**: A dedicated module to monitor component health and switch to redundant sensors/actuators.

*   **Observability**:
    *   **Telemetry (TLM)**: Structured packet generation (CCSDS format recommended) for downlink. All critical states must be logged.
    *   **Logging**: On-board storage of high-frequency data for post-mission analysis (if SD card available).

*   **Testability**:
    *   **Unit Tests**: GoogleTest for logic classes (GNC math, FSM transitions).
    *   **SIL (Software-In-The-Loop)**: Running the FSW against the Physics Simulator on a PC.
    *   **HIL (Hardware-In-The-Loop)**: Running the FSW on the actual flight microcontroller, interfacing with real sensors or hardware emulators.

## 2. Enhanced Directory Structure

```text
project_root/
├── CMakeLists.txt              # Main build script
├── README.md                   # Documentation entry point
├── LICENSE
├── external/                   # Third-party dependencies (fetchcontent or submodules)
│   ├── eigen/
│   ├── googletest/
│   ├── spdlog/                 # Logging
│   └── nlohmann_json/          # Config parsing
├── config/                     # Configuration files
│   ├── production/             # Flight parameters
│   │   └── gnc_gains.json
│   └── simulation/             # Simulation-only parameters
│       ├── orbit_init.json
│       └── noise_models.json
├── tools/                      # Helper scripts
│   ├── vis/                    # Python/Matlab scripts to visualize telemetry/logs
│   ├── deploy/                 # Flashing/Deployment scripts
│   └── sim_runner.py           # Script to run batch SIL scenarios
├── src/
│   ├── common/                 # Shared utilities
│   │   ├── types.hpp           # Common math types (Vector3, Quat)
│   │   ├── time.hpp            # SpacecraftTime class
│   │   ├── constants.hpp       # Physics constants
│   │   └── logger.hpp          # Logging interface
│   │
│   ├── hal/                    # Hardware Abstraction Layer
│   │   ├── interfaces/         # Abstract Base Classes (Pure Virtual)
│   │   │   ├── IGyro.hpp
│   │   │   ├── IMagnetometer.hpp
│   │   │   ├── IStarTracker.hpp
│   │   │   ├── IRW.hpp         # Reaction Wheel
│   │   │   └── ITorquer.hpp    # Magnetorquer
│   │   └── mcu/                # Low-level MCU drivers (GPIO, I2C, SPI) - *Platform Specific*
│   │       ├── I2C.hpp
│   │       └── SPI.hpp
│   │
│   ├── fsw/                    # Flight Software Core
│   │   ├── core/
│   │   │   ├── ModeManager.cpp # Main FSM (Safe, Detumble, Pointing)
│   │   │   ├── TaskScheduler.cpp # Real-time task scheduling
│   │   │   └── DataStore.hpp   # Central data exchange (Pub/Sub)
│   │   ├── comms/              # Communication Handling
│   │   │   ├── CmdHandler.cpp  # Telecommand decoding/execution
│   │   │   └── TlmGenerator.cpp# Telemetry packet creation
│   │   ├── gnc/
│   │   │   ├── ekf/            # Estimation
│   │   │   │   ├── MEKF.cpp    # Multiplicative EKF for Attitude
│   │   │   │   └── OrbitEst.cpp# SGP4 or Orbit Determinator
│   │   │   ├── control/
│   │   │   │   ├── PID.cpp
│   │   │   │   ├── Bdot.cpp    # Magnetic Detumbling
│   │   │   │   └── Controller.cpp # High-level control logic
│   │   │   └── guidance/
│   │   │       ├── PointingStrategies.cpp (Nadir, Sun, Target)
│   │   │       └── Trajectory.cpp
│   │   └── drivers/            # Concrete Drivers implementing HAL
│   │       ├── BNO055.cpp
│   │       └── GomSpaceWheel.cpp
│   │
│   ├── sim/                    # Simulation Infrastructure (PC Only)
│   │   ├── engine/
│   │   │   ├── Integrator.hpp  # RK4 / RK45 implementations
│   │   │   └── World.cpp       # Physics step manager
│   │   ├── dynamics/
│   │   │   ├── RigidBody.cpp   # Euler Equations
│   │   │   ├── Orbit.cpp       # Keplerian/J2 propagators
│   │   │   └── Environment.cpp # Magnetic field (IGRF), Sun vector, Eclipse
│   │   └── models/             # Simulated Hardware (inherits HAL interfaces)
│   │       ├── SimGyro.cpp     # Adds bias, noise, saturation
│   │       └── SimRW.cpp       # Adds friction, jitter, limits
│   │
│   └── main_fsw.cpp            # Entry point for Flight (FreeRTOS task or bare metal loop)
│
└── test/
    ├── unit/                   # Unit tests for GNC algo, FSM logic
    └── sil/                    # Integration tests (Sim + FSW)
        └── main_sil.cpp        # Entry point for PC Simulation
```

## 3. Recommended Algorithms

### Navigation (Attitude Determination)
*   **MEKF (Multiplicative Extended Kalman Filter)**: The standard for spacecraft attitude.
    *   **State**: Quaternion (error state) + Gyro Bias.
    *   **Measurement Update**: Magnetometer, Sun Sensors, Star Tracker.
    *   **Propagation**: Gyro integration.
*   **TRIAD / QUEST**: For coarse, static initialization if lost at sea.

### Navigation (Orbit)
*   **SGP4**: If relying on TLEs uplinked from ground.
*   **On-board Propagator**: J2 or J4 perturbations for short-term propagation between GPS fixes (if GPS available).

### Control
*   **B-Dot**: Magnetic detumbling. Essential for initial deployment and safe mode.
    *   Law: $M = -k \cdot \dot{B}$
*   **PID**: Simple pointing control.
    *   Outer Loop: Target Orientation -> Desired Rate.
    *   Inner Loop: Desired Rate -> Torque.
*   **Reaction Wheel Management**:
    *   **Desaturation (Momentum Dumping)**: Using magnetorquers to lower wheel speeds when they saturate.

### Guidance
*   **Modes**:
    *   **Idle**: Do nothing.
    *   **Detumble**: Reduce angular rates < X deg/s.
    *   **Sun Pointing**: Maximize power generation.
    *   **Nadir Pointing**: Payload operations (Earth facing).
    *   **Target Pointing**: Slew to specific inertial or ground coordinates.

## 4. Architecture Patterns

### The "Data Store" (Pub/Sub)
Avoid spaghetti coupling between GNC and Comms. Use a central data exchange.
*   **Publishers**: Drivers (update sensor data), GNC (publish state estimate).
*   **Subscribers**: ModeManager (checks state), Controllers (read state), Telemetry (reads everything).
*   *Implementation*: A thread-safe `Blackboard` or a lightweight `MessageBus` using topics.

### The Interface Adapter (HAL)
Crucial for testing.
*   **Flight**: `IO_Wrapper -> HAL_Interface`
*   **Sim**: `Sim_Model -> HAL_Interface`
*   **Benefit**: FSW simply calls `gyro->read()` and doesn't care if it's a BNO055 or a Gaussian Random Number Generator.

### The Orchestrator (Deterministic Loop)
Decouple "Simulation Time" from "Wall Clock Time".
*   In **Flight**: Run at fixed hardware timer interrupts (e.g., 10Hz).
*   In **Sim**: Run as fast as CPU allows, but advance internal logic by fixed `dt`.
    *   Step Physics ($t \to t+dt$)
    *   Step GNC ($t \to t+dt$)
    *   Repeat.
