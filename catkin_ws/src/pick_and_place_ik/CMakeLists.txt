cmake_minimum_required(VERSION 3.0.2)
project(pick_and_place_ik)

add_compile_options(-std=c++11)

## Declare catkin packages required
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp moveit_ros_planning_interface tf2_geometry_msgs trajectory_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ur10_pick_and_place_node src/ur10_pick_place.cpp)

target_link_libraries(ur10_pick_and_place_node
  ${catkin_LIBRARIES}
)


