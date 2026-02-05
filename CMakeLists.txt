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

# --- FSW Core ---
add_library(fsw_core INTERFACE)
target_sources(fsw_core INTERFACE
    src/fsw/core/DataStore.hpp
)
target_link_libraries(fsw_core INTERFACE common)

# --- Tests ---
enable_testing()
add_subdirectory(tests)
