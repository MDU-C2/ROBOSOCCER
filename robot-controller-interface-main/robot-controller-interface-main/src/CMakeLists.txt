#===============================================================================
# Add which modules/directories contains stuff that should be compiled

add_executable(main_exe main.cc)

# Where to find source code for libraries etc
add_subdirectory(individual-robot-behaviour)

#===============================================================================
# Dependencies

# !!! Add any libraries which is used by main here (link in necessary 
# includes) !!!
target_link_libraries(
  main_exe 
  path_planning_lib 
  state_lib 
  ball_lib 
  supporting_lib
)

# !!! Add packages that must be included here !!!
ament_target_dependencies(main_exe rclcpp)

#===============================================================================
