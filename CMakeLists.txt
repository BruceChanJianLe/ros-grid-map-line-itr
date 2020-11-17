cmake_minimum_required(VERSION 3.0.2)
project(ros-grid-map-line-itr)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  grid_map_ros
  grid_map_visualization
  roscpp
  rospy
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ros-grid-map-line-itr
#  CATKIN_DEPENDS geometry_msgs grid_map_ros grid_map_visualization roscpp rospy
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(line_node
src/line_node.cpp
src/line.cpp
)
target_link_libraries(line_node ${catkin_LIBRARIES})