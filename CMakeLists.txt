cmake_minimum_required(VERSION 3.0.2)
project(rrt_path_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rrt_path_planner
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf2_geometry_msgs tf2_ros visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rrt_node src/rrt.cpp)

target_link_libraries(rrt_node
  ${catkin_LIBRARIES}
)

