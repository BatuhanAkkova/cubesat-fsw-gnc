# Common library tests
add_executable(test_math test_math.cpp)
target_link_libraries(test_math PRIVATE common gtest_main)

# DataStore tests
add_executable(test_datastore test_datastore.cpp)
target_link_libraries(test_datastore PRIVATE fsw_core gtest_main)

# B-Dot Test
add_executable(phase3_bdot_test phase3_bdot_test.cpp)
target_link_libraries(phase3_bdot_test PRIVATE sim_engine sim_models fsw_gnc common gtest_main)

# MEKF Test
add_executable(mekf_test fsw/gnc/ekf/MEKF_test.cpp)
target_link_libraries(mekf_test PRIVATE fsw_gnc common gtest_main)

# Pointing Test
add_executable(phase5_pointing_test phase5_pointing_test.cpp)
target_link_libraries(phase5_pointing_test PRIVATE sim_engine sim_models fsw_gnc common gtest_main)

# Mode Manager Test
add_executable(mode_manager_test mode_manager_test.cpp)
target_link_libraries(mode_manager_test PRIVATE fsw_core sim_engine sim_models fsw_gnc common gtest_main)

# Task Scheduler Test
add_executable(task_scheduler_test task_scheduler_test.cpp)
target_link_libraries(task_scheduler_test PRIVATE fsw_core common gtest_main)

# Full Mission Test
add_executable(phase6_mission_test phase6_mission_test.cpp)
target_link_libraries(phase6_mission_test PRIVATE fsw_core sim_engine sim_models fsw_gnc common gtest_main)

# Orbit J2 Test
add_executable(orbit_j2_test orbit_j2_test.cpp)
target_link_libraries(orbit_j2_test PRIVATE sim_engine common gtest_main)

# Integration Test
add_executable(phase7_integration_test phase7_integration_test.cpp)
target_link_libraries(phase7_integration_test PRIVATE sim_engine sim_models fsw_gnc common gtest_main)

# Wheel Desaturation Demo
add_executable(wheel_desaturation_demo wheel_desaturation_demo.cpp)
target_link_libraries(wheel_desaturation_demo PRIVATE sim_engine sim_models fsw_gnc common)

include(GoogleTest)
gtest_discover_tests(test_math)
gtest_discover_tests(test_datastore)
gtest_discover_tests(phase3_bdot_test)
gtest_discover_tests(mekf_test)
gtest_discover_tests(phase5_pointing_test)
gtest_discover_tests(orbit_j2_test)
gtest_discover_tests(phase7_integration_test)
gtest_discover_tests(mode_manager_test)
gtest_discover_tests(task_scheduler_test)
gtest_discover_tests(phase6_mission_test)

