cmake_minimum_required(VERSION 3.16)
project(CubeSatSim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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

# Source subdirectories (We'll add files here later, for now just setting up structure)
# For now, since we haven't created the source files yet, we won't add_subdirectory(src) 
# until we populate it to avoid CMake errors about empty targets if we were to make them there.
# Instead, we will define the library targets as we create the files.

# --- Common ---
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
)
# Integrator.hpp is a template header, so it doesn't need to be compiled, 
# but we can list it for IDEs.
target_sources(sim_engine PUBLIC
    src/sim/engine/Integrator.hpp
    src/sim/dynamics/RigidBody.hpp
    src/sim/dynamics/Orbit.hpp
)
target_include_directories(sim_engine PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(sim_engine PUBLIC common)

# --- Sim Models ---
add_library(sim_models STATIC
    src/sim/models/SimGyro.cpp
    src/sim/models/SimMagnetometer.cpp
    src/sim/models/SimTorquer.cpp
    src/sim/models/SimStarTracker.cpp
)
target_sources(sim_models PUBLIC
    src/sim/models/SimGyro.hpp
    src/sim/models/SimMagnetometer.hpp
    src/sim/models/SimTorquer.hpp
    src/sim/models/SimStarTracker.hpp
)
target_include_directories(sim_models PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(sim_models PUBLIC hal_interfaces sim_engine common)

# --- FSW Core ---
add_library(fsw_core INTERFACE)
target_sources(fsw_core INTERFACE
    src/fsw/core/DataStore.hpp
)
target_link_libraries(fsw_core INTERFACE common)

# --- FSW GNC ---
add_library(fsw_gnc STATIC
    src/fsw/gnc/control/Bdot.cpp
    src/fsw/gnc/ekf/MEKF.cpp
)
target_sources(fsw_gnc PUBLIC
    src/fsw/gnc/control/Bdot.hpp
    src/fsw/gnc/ekf/MEKF.hpp
)
target_include_directories(fsw_gnc PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(fsw_gnc PUBLIC common)

# --- Tests ---
enable_testing()
add_subdirectory(tests)
