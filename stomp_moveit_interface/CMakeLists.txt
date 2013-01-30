cmake_minimum_required(VERSION 2.8.3)
project(stomp_moveit_interface)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS stomp moveit_core pluginlib)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES stomp_moveit_interface
  CATKIN-DEPENDS stomp moveit_core pluginlib
  DEPENDS 
)

add_library(stomp_moveit_interface
  src/feature_set.cpp
  src/cost_features/stomp_cost_feature.cpp
  src/stomp_planner.cpp
  src/stomp_trajectory.cpp
  src/cost_features/exact_collision_feature.cpp
  src/stomp_optimization_task.cpp
#  src/cost_features/cartesian_orientation_feature.cpp
#  src/cost_features/cartesian_vel_acc_feature.cpp
#  src/cost_features/collision_feature.cpp
#  src/cost_features/joint_vel_acc_feature.cpp
#  src/stomp_collision_point.cpp
#  src/stomp_collision_space.cpp
#  src/stomp_cost_function_input.cpp
#  src/stomp_robot_model.cpp
#  src/treefksolverjointposaxis_partial.cpp
)

add_executable(test_stomp_moveit_interface
  test/test_stomp_moveit_interface.cpp
)
target_link_libraries(test_stomp_moveit_interface stomp_moveit_interface)

#add_executable(display_robot_model
#  src/display_robot_model.cpp
#)

#target_link_libraries(display_robot_model stomp_moveit_interface)

#add_executable(stomp_node
#  src/stomp_node.cpp
#)

#target_link_libraries(stomp_node stomp_moveit_interface)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(stomp_moveit_interface
  ${catkin_LIBRARIES})
