#===============================================================================
# Path planning

add_library(path_planning_lib path_planning.cc)
ament_target_dependencies(path_planning_lib rclcpp dwb_core nav_msgs)

# Link in their includes
target_link_libraries(path_planning_lib state_lib)

#===============================================================================
# State

add_library(state_lib state.cc)
ament_target_dependencies(state_lib rclcpp nav_msgs tf2 tf2_ros 
  tf2_geometry_msgs)

#===============================================================================
# Ball

add_library(ball_lib ball.cc)

# Link in their includes
target_link_libraries(ball_lib state_lib path_planning_lib)

#===============================================================================
# Supporting

add_library(supporting_lib supporting.cc)

# Link in their includes
target_link_libraries(supporting_lib state_lib)

#===============================================================================
