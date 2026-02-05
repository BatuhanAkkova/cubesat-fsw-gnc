# Common library tests
add_executable(test_math test_math.cpp)
target_link_libraries(test_math PRIVATE common gtest_main)

# DataStore tests
add_executable(test_datastore test_datastore.cpp)
target_link_libraries(test_datastore PRIVATE fsw_core gtest_main)

# Phase 3: B-Dot Test
add_executable(phase3_bdot_test phase3_bdot_test.cpp)
target_link_libraries(phase3_bdot_test PRIVATE sim_engine sim_models fsw_gnc common gtest_main)

include(GoogleTest)
gtest_discover_tests(test_math)
gtest_discover_tests(test_datastore)
gtest_discover_tests(phase3_bdot_test)
