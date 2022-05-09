cmake_minimum_required(VERSION 3.0.2)
project(detect_action)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  visualization_msgs
  tf
  jsk_rviz_plugins
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(detect_action_node src/detect_action_node.cpp src/detect_action.cpp)
target_link_libraries(detect_action_node
  ${catkin_LIBRARIES}
)
