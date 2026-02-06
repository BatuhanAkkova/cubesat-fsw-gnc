include(GoogleTest)

# Common library tests
add_executable(test_math test_math.cpp)
target_link_libraries(test_math PRIVATE common gtest_main)
gtest_discover_tests(test_math)

# DataStore tests
add_executable(test_datastore test_datastore.cpp)
target_link_libraries(test_datastore PRIVATE fsw_core gtest_main)
gtest_discover_tests(test_datastore)

# B-Dot Test
add_executable(bdot_test bdot_test.cpp)
target_link_libraries(bdot_test PRIVATE sim_engine sim_models fsw_gnc common gtest_main)
gtest_discover_tests(bdot_test)

# MEKF Test
add_executable(mekf_test fsw/gnc/ekf/MEKF_test.cpp)
target_link_libraries(mekf_test PRIVATE fsw_gnc common gtest_main)
gtest_discover_tests(mekf_test)

# Pointing Test
add_executable(pointing_test pointing_test.cpp)
target_link_libraries(pointing_test PRIVATE sim_engine sim_models fsw_gnc common gtest_main)
gtest_discover_tests(pointing_test)

# Mode Manager Test
add_executable(mode_manager_test mode_manager_test.cpp)
target_link_libraries(mode_manager_test PRIVATE fsw_core sim_engine sim_models fsw_gnc common gtest_main)
gtest_discover_tests(mode_manager_test)

# Task Scheduler Test
add_executable(task_scheduler_test task_scheduler_test.cpp)
target_link_libraries(task_scheduler_test PRIVATE fsw_core common gtest_main)
gtest_discover_tests(task_scheduler_test)

# Mission Test
add_executable(mission_test mission_test.cpp)
target_link_libraries(mission_test PRIVATE fsw_core sim_engine sim_models fsw_gnc common gtest_main)
gtest_discover_tests(mission_test)

# Orbit J2 Test
add_executable(orbit_j2_test orbit_j2_test.cpp)
target_link_libraries(orbit_j2_test PRIVATE sim_engine common gtest_main)
gtest_discover_tests(orbit_j2_test)

# Integration Test
add_executable(integration_test integration_test.cpp)
target_link_libraries(integration_test PRIVATE sim_engine sim_models fsw_gnc common gtest_main)
gtest_discover_tests(integration_test)

# Wheel Desaturation Demo
add_executable(wheel_desaturation_demo wheel_desaturation_demo.cpp)
target_link_libraries(wheel_desaturation_demo PRIVATE sim_engine sim_models fsw_gnc common)

# LQG Controller Tests
add_executable(lqg_controller_test fsw/lqg_controller_test.cpp)
target_link_libraries(lqg_controller_test PRIVATE fsw_gnc common gtest_main)
gtest_discover_tests(lqg_controller_test)

add_executable(lqg_performance_demo fsw/lqg_performance_demo.cpp)
target_link_libraries(lqg_performance_demo PRIVATE sim_engine sim_models fsw_gnc common gtest_main)
gtest_discover_tests(lqg_performance_demo)

# FDIR Tests
add_executable(sensor_health_test fdir/sensor_health_test.cpp)
target_link_libraries(sensor_health_test PRIVATE fsw_fdir common gtest_main)
gtest_discover_tests(sensor_health_test)

add_executable(fdir_integration_test fdir/fdir_integration_test.cpp)
target_link_libraries(fdir_integration_test PRIVATE fsw_fdir fsw_core sim_engine sim_models fsw_gnc common gtest_main)
gtest_discover_tests(fdir_integration_test)

# Controller Gain Test
add_executable(controller_gain_test controller_gain_test.cpp)
target_link_libraries(controller_gain_test PRIVATE optimization mc_runner fsw_gnc common gtest_main)
gtest_discover_tests(controller_gain_test)

# Telemetry Test
add_executable(telemetry_test telemetry_test.cpp)
target_link_libraries(telemetry_test PRIVATE fsw_fdir common gtest_main)
gtest_discover_tests(telemetry_test)

# Command Handling Test
add_executable(command_parser_test command_parser_test.cpp)
target_link_libraries(command_parser_test PRIVATE fsw_core common gtest_main)
gtest_discover_tests(command_parser_test)

# Command Integration Test
add_executable(command_integration_test command_integration_test.cpp)
target_link_libraries(command_integration_test PRIVATE fsw_core sim_engine sim_models common gtest_main)
gtest_discover_tests(command_integration_test)

# Ground Station Test
add_executable(ground_station_test ground_station_test.cpp)
target_link_libraries(ground_station_test PRIVATE sim_engine fsw_fdir common gtest_main)
gtest_discover_tests(ground_station_test)