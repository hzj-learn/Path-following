cmake_minimum_required(VERSION 3.0.2)
project(vis_global_path)

add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  visualization_msgs
  jsk_gui_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(vis_global_path_node src/vis_global_path_node.cpp src/vis_global_path.cpp)
target_link_libraries(vis_global_path_node
  ${catkin_LIBRARIES}
)
