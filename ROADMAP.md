# Roadmap for Enhanced CubeSat FSW & GNC Implementation

This roadmap outlines the step-by-step implementation of the algorithms and project structure defined in `enhanced_project_structure.md`. The approach is iterative, starting from the core infrastructure and moving towards complex GNC algorithms.

## Phase 1: Foundation & Core Infrastructure
**Goal**: Establish the build system, common math types, and the central data exchange mechanism.

1.  **Project Scaffolding**: Setup `CMakeLists.txt` and directory structure (`src/`, `config/`, `tools/`).
2.  **External Dependencies**: specific versions of Eigen, spdlog, and googletest.
3.  **Common Utilities**:
    *   Implement `Vector3` and `Quaternion` wrappers (or typedefs) in `src/common/types.hpp`.
    *   Implement `SpacecraftTime` in `src/common/time.hpp`.
    *   Setup `Logger` wrapper using spdlog.
4.  **Data Store (Pub/Sub)**:
    *   Implement the `DataStore` class in `src/fsw/core/DataStore.hpp`.
    *   Define the message topics and data structures (e.g., `GyroMsg`, `ControlCmd`).
    *   *Verification*: Unit test ensuring multiple threads can publish/subscribe safely.

## Phase 2: Hardware Abstraction & Simulation Environment
**Goal**: Create the "World" where code runs and the Interfaces it talks to.

1.  **HAL Interfaces**:
    *   Define pure virtual classes: `IGyro`, `IMagnetometer`, `ITorquer`, `IRW` in `src/hal/interfaces/`.
2.  **Simulation Engine**:
    *   Implement `Integrator` (RK4) in `src/sim/engine/`.
    *   Implement `RigidBody` dynamics (Euler's equations) in `src/sim/dynamics/`.
    *   Implement basic `Orbit` propagator (Keplerian) to get position/velocity.
3.  **Simulated Hardware**:
    *   Create `SimGyro` and `SimMagnetometer` inheriting from HAL interfaces.
    *   Feed ground truth dynamics from `RigidBody` into these sensors (adding noise is optional for now).

## Phase 3: Basic GNC - The "Safe Mode" (B-Dot)
**Goal**: Get the satellite to stable state (Detumble) using the simplest algorithm.

1.  **B-Dot Controller**:
    *   Implement `Bdot` class in `src/fsw/gnc/control/`.
    *   Algorithm: $M = -k \cdot \dot{B}$ (requires finite difference of magnetometer readings).
2.  **Magnetorquer Driver**:
    *   Implement `SimTorquer` in `src/sim/models/`.
3.  **Control Loop Integration**:
    *   Create a simple loop that reads Mag -> Runs B-Dot -> Commands Torquers.
4.  **Verification (SIL)**:
    *   Initialize `RigidBody` with high initial angular rates.
    *   Run simulation and verify rates assume zero (or near zero) over time.

## Phase 4: Attitude Determination (MEKF)
**Goal**: Know *where* we are pointing.

1.  **MEKF Implementation**:
    *   Create `MEKF` class in `src/fsw/gnc/ekf/`.
    *   **Prediction Step**: Integrate Gyro measurements (Quaternion kinematics).
    *   **Update Step**: Fuse Magnetometer and Sun Sensor/Star Tracker data.
2.  **Simulated Star Tracker**:
    *   Implement `IStarTracker` and `SimStarTracker` to provide orientation measurements.
3.  **Verification**:
    *   Run simulation with known attitude.
    *   Compare MEKF estimate vs Ground Truth.

## Phase 5: 3-Axis Pointing Control (PID)
**Goal**: Point at something specific.

1.  **PID Controller**:
    *   Implement `PID` class in `src/fsw/gnc/control/`.
    *   Create `Controller` class that uses PID for attitude errors.
2.  **Reaction Wheels**:
    *   Implement `IRW` and `SimRW`.
    *   Update `RigidBody` dynamics to include wheel angular momentum exchange.
3.  **Pointing Logic**:
    *   Implement `PointingStrategies` (e.g., calculations to find target quaternion from inertial vectors).
4.  **Verification**:
    *   Command a 90-degree slew.
    *   Verify settling time and overshoot.

## Phase 6: Scheduler & Mode Management
**Goal**: Automate the switching between behaviors.

1.  **Mode Manager**:
    *   Implement Finite State Machine: `Safe` (B-Dot) <-> `Nominal` (Pointing).
2.  **Task Scheduler**:
    *   Implement `TaskScheduler` to run GNC loops at fixed rates (e.g., 10Hz).
3.  **Full Mission Simulation**:
    *   Simulate: Deploy -> High Rate -> Detumble -> Sun Pointing.

## Phase 7: Refinement
1.  **Orbit Propagator**: Upgrade from Keplerian to J2 (SGP4-like) in `src/fsw/gnc/ekf/OrbitEst.cpp`.
2.  **Wheel Desaturation**: Implement logic to fire Torquers when Wheel speed > Limit.
3.  **FDIR**: meaningful checks (e.g., "If Gyro reading stuck, reset").
4.  **B-field Model**: Implement time-varying B-field model (IGRF + orbital motion).