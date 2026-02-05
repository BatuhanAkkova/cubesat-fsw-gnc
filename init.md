## Description
Creating a flight software and GNC system in C++ for a CubeSat.

## Principles
- **High-performance C++**: Achieved by using efficient linear algebra libraries (like Eigen) and avoiding runtime polymorphism in tight loops where possible (though virtual functions in HAL are negligible for 10-100Hz GNC).
- **Deterministic Synchronization**: Achieved by the Orchestrator Loop that decouples simulation time from CPU execution time.
- **Pseudo-Hardware**: Achieved by the sim/models classes that inject noise and delay.
- **Seamless Transition**: Achieved by the HAL (src/hal/interfaces).

## Structure
Example structure:
project_root/
├── CMakeLists.txt
├── external/                # Third-party libs (Eigen, SPDLog, GoogleTest)
├── config/                  # JSON/YAML files (Orbit params, PID gains, Noise std_dev)
├── src/
│   ├── common/              # Types shared by FSW and SIM (e.g., math, constants)
│   │   ├── types.hpp        # Vector3, Quaternion definitions
│   │   └── time.hpp         # Custom time class (essential for determinism)
│   │
│   ├── hal/                 # Hardware Abstraction Layer (The most important part)
│   │   ├── interfaces/      # Pure virtual classes
│   │   │   ├── IGyro.hpp
│   │   │   ├── IReactionWheel.hpp
│   │   │   └── IStarTracker.hpp
│   │
│   ├── fsw/                 # FLIGHT SOFTWARE (The logic to be tested)
│   │   ├── core/
│   │   │   ├── ModeManager.cpp  # FSM (Detumble, Pointing, Idle)
│   │   │   └── DataBus.hpp      # Pub/Sub or Blackboard for sharing data
│   │   ├── gnc/
│   │   │   ├── ekf/             # Estimation algorithms
│   │   │   └── control/         # Control laws (PID, LQR)
│   │   └── drivers/             # IMPLEMENTATIONS of HAL for Real Hardware
│   │       └── RealGyro_BNO055.cpp (Not used in SIL, but lives here)
│   │
│   └── sim/                 # SIMULATION (The "Pseudo-Hardware" & Physics)
│       ├── dynamics/        # The "Truth" models
│       │   ├── OrbitPropagator.cpp
│       │   └── RigidBody.cpp    # Euler's equations
│       ├── models/          # "Pseudo-Hardware" IMPLEMENTATIONS of HAL
│       │   ├── SimGyro.cpp      # Adds bias/noise to Truth
│       │   └── SimWheel.cpp     # Adds latency/jitter to commands
│       └── environment/     # Disturbance models
│           ├── Gravity.cpp      # J2, spherical harmonics
│           └── MagField.cpp     # IGRF dipoles
│
└── test/                    # Unit and Integration tests
    └── sil_scenarios/       # Full mission scenario runners

## Must-Have Features
**Hardware Abstraction Layer (HAL):**
Example script:
// src/hal/interfaces/IGyro.hpp
class IGyro {
public:
    virtual ~IGyro() = default;
    
    // FSW calls this. In SIL, it returns simulated noisy data. 
    // On the testbed, it returns real I2C/SPI data.
    virtual Vector3d getAngularVelocity() = 0; 
    
    // Useful for health checks
    virtual bool isHealthy() = 0;
};

**Psuedo-Hardware Implementation:**
Example script for Sensor Noise:
// src/sim/models/SimGyro.cpp
Vector3d SimGyro::getAngularVelocity() {
    // 1. Get Truth from Dynamics Engine
    Vector3d true_omega = dynamics_->getOmega(); 

    // 2. Add Bias (Random Walk) + White Noise
    return true_omega + current_bias_ + GaussianNoise(0.0, sigma_);
}
Example script for Actuator Latency:
// src/sim/models/SimReactionWheel.cpp
void SimReactionWheel::setTorqueCommand(double cmd) {
    // Push command to a queue with a timestamp
    command_queue_.push({sim_time_ + latency_ms_, cmd});
}

void SimReactionWheel::step() {
    // Only apply torque if the timestamp has passed
    if (command_queue_.front().time <= sim_time_) {
         current_torque_ = command_queue_.front().value;
         command_queue_.pop();
    }
}

**Deterministtic Scheduler:**
Example script for Orchestrator Loop:
double t = 0.0;
const double dt = 0.1; // 10Hz GNC loop

while (t < MAX_SIM_TIME) {
    // 1. Step Physics (High fidelity, maybe 100Hz or RK4 steps)
    plant_dynamics.step(dt); 

    // 2. Update Sensors (Generate "measurements" from new physics state)
    sim_sensors.update(plant_dynamics.state, t);

    // 3. Step Flight Software (The code you are testing)
    // Pass the virtual time 't', not system time!
    fsw_system.step(sim_sensors, t);

    // 4. Update Actuators (Apply FSW commands to Plant with delay)
    sim_actuators.update(fsw_system.commands, t);

    t += dt;
}

**Factory/Composition Root:**
Example scripts:
// main_sil.cpp
auto gyro = std::make_shared<SimGyro>(config);
auto fsw = std::make_unique<FlightController>(gyro); // Inject Sim
fsw->run();

// main_hardware.cpp
auto gyro = std::make_shared<BNO055_Driver>(i2c_bus);
auto fsw = std::make_unique<FlightController>(gyro); // Inject Real
fsw->run();