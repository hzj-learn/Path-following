cmake_minimum_required(VERSION 3.0.2)
project(global_waypoints_generator)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  visualization_msgs
  jsk_gui_msgs
  std_msgs
  autoware_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(global_waypoints_generator_node src/global_waypoints_generator_node.cpp src/global_waypoints_generator.cpp)
target_link_libraries(global_waypoints_generator_node
  ${catkin_LIBRARIES}
)
