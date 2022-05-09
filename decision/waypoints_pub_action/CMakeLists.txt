cmake_minimum_required(VERSION 2.8.3)
project(waypoints_pub_action)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  tf
  geometry_msgs
  std_msgs
)

find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES WaypointsPub.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(WaypointsPub_client src/WaypointsPub_client.cpp)
target_link_libraries( WaypointsPub_client ${catkin_LIBRARIES})
add_dependencies(WaypointsPub_client ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(WaypointsPub_server src/WaypointsPub_server.cpp)
target_link_libraries( WaypointsPub_server ${catkin_LIBRARIES})
add_dependencies(WaypointsPub_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
