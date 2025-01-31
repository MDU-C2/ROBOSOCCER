#===============================================================================
# Includes

# GTest
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

enable_testing()

#===============================================================================
# Setup gtest with main_test

add_executable(main_test_exe main_test.cc)

# !!! Add libraries that your tests use here (link in the necessary 
# includes) !!!
target_link_libraries(
  main_test_exe
  GTest::gtest_main
  path_planning_lib
  state_lib
  ball_lib
  supporting_lib
)

#===============================================================================
# Dependencies

# !!! This is where you add your source files which contain tests !!!
target_sources(main_test_exe PRIVATE 
  main_test.cc 
  individual-robot-behaviour-test/path_planning_test.cc
  individual-robot-behaviour-test/state_test.cc
  individual-robot-behaviour-test/ball_test.cc
  individual-robot-behaviour-test/supporting_test.cc
  )

# !!! Add packages that must be included here !!!
ament_target_dependencies(main_test_exe rclcpp dwb_core nav_msgs tf2)

#===============================================================================
# Must be last (I think)

gtest_discover_tests(main_test_exe)

#===============================================================================
