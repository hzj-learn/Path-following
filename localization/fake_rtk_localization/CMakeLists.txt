cmake_minimum_required(VERSION 3.0.2)
project(fake_rtk_localization)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  visualization_msgs
  gazebo_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(fake_rtk_localization_node src/fake_rtk_localization_node.cpp src/fake_rtk_localization.cpp)
target_link_libraries(fake_rtk_localization_node
  ${catkin_LIBRARIES}
)
