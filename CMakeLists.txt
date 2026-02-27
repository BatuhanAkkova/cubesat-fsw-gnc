cmake_minimum_required(VERSION 3.16)
project(CubeSatSim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(ENABLE_COVERAGE "Enable code coverage reporting" OFF)
if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(--coverage)
    add_link_options(--coverage)
endif()

# Modules for dependencies
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Dependencies
include(GetEigen)
include(GetSpdlog)
include(GetGTest)

# Core Library
add_library(core_lib INTERFACE)
target_include_directories(core_lib INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(core_lib INTERFACE
    Eigen3::Eigen
    spdlog::spdlog
)

# --- Common ---
add_library(common INTERFACE)
target_sources(common INTERFACE
    src/common/types.hpp
    src/common/time.hpp
    src/common/logger.hpp
)
target_link_libraries(common INTERFACE core_lib)

# --- HAL Interfaces ---
add_library(hal_interfaces INTERFACE)
target_sources(hal_interfaces INTERFACE
    src/hal/interfaces/IGyro.hpp
    src/hal/interfaces/IMagnetometer.hpp
    src/hal/interfaces/ITorquer.hpp
    src/hal/interfaces/IRW.hpp
    src/hal/interfaces/IStarTracker.hpp
)
target_link_libraries(hal_interfaces INTERFACE common)

# --- Sim Engine ---
add_library(sim_engine STATIC
    src/sim/dynamics/RigidBody.cpp
    src/sim/dynamics/Orbit.cpp
    src/sim/Simulation.cpp
    src/sim/GroundStation.cpp
    src/sim/models/SimGyro.cpp
    src/sim/models/SimMagnetometer.cpp
    src/sim/models/SimTorquer.cpp
    src/sim/models/SimStarTracker.cpp
    src/sim/models/SimRW.cpp
)

target_sources(sim_engine PUBLIC
    src/sim/engine/Integrator.hpp
    src/sim/dynamics/RigidBody.hpp
    src/sim/dynamics/Orbit.hpp
    src/sim/Simulation.hpp
    src/sim/GroundStation.hpp
    src/sim/models/SimGyro.hpp
    src/sim/models/SimMagnetometer.hpp
    src/sim/models/SimTorquer.hpp
    src/sim/models/SimStarTracker.hpp
    src/sim/models/SimRW.hpp
)
target_include_directories(sim_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(sim_engine PUBLIC common hal_interfaces)

# --- Flight Software (Merged) ---
add_library(fsw_lib STATIC
    src/fsw/gnc/control/Bdot.cpp
    src/fsw/gnc/ekf/MEKF.cpp
    src/fsw/gnc/control/WheelDesaturation.cpp
    src/fsw/fdir/FDIRManager.cpp
    src/fsw/telemetry/TelemetryService.cpp
    src/fsw/telemetry/TelemetryManager.cpp
    src/fsw/core/ModeManager.cpp
    src/fsw/core/TaskScheduler.cpp
    src/fsw/core/CommandManager.cpp
    src/fsw/core/CommandParser.cpp
    src/fsw/FlightSoftware.cpp
)

target_sources(fsw_lib PUBLIC
    src/fsw/gnc/control/Bdot.hpp
    src/fsw/gnc/ekf/MEKF.hpp
    src/fsw/gnc/control/PID.hpp
    src/fsw/gnc/control/AttitudeController.hpp
    src/fsw/gnc/control/WheelDesaturation.hpp
    src/fsw/gnc/guidance/PointingStrategies.hpp
    src/fsw/fdir/FDIRConfig.hpp
    src/fsw/fdir/SensorHealthMonitor.hpp
    src/fsw/fdir/FDIRManager.hpp
    src/fsw/core/DataStore.hpp
    src/fsw/core/ModeManager.hpp
    src/fsw/core/TaskScheduler.hpp
    src/fsw/core/Command.hpp
    src/fsw/core/CommandManager.hpp
    src/fsw/core/CommandParser.hpp
    src/fsw/core/commands/SlewToNadirCommand.hpp
    src/fsw/telemetry/CommandProtocol.hpp
    src/fsw/FlightSoftware.hpp
)

target_include_directories(fsw_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(fsw_lib PUBLIC common)

# --- Optimization & Testing Tools ---
add_library(optimization STATIC
    src/opt/GeneticOptimizer.cpp
)
target_sources(optimization PUBLIC
    src/opt/FitnessEvaluator.hpp
    src/opt/GeneticOptimizer.hpp
)
target_include_directories(optimization PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(optimization PUBLIC fsw_lib sim_engine common)

add_library(mc_runner STATIC
    src/test/MonteCarloRunner.cpp
)
target_sources(mc_runner PUBLIC
    src/test/MonteCarloRunner.hpp
)
target_include_directories(mc_runner PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(mc_runner PUBLIC fsw_lib sim_engine common)

# --- Tests ---
enable_testing()
add_subdirectory(tests)
