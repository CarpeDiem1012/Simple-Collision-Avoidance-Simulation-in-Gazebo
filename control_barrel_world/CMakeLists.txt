cmake_minimum_required(VERSION 3.0.2)
project(control_barrel_world)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  vision_msgs
  geometry_msgs
  prius_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES control_barrel_world
#  CATKIN_DEPENDS prius_msgs roscpp sensor_msgs std_msgs vision_msgs
#  DEPENDS system_lib
)

include_directories(
  include/${PROJECT_NAME} 
  ${catkin_INCLUDE_DIRS}
)

add_executable(control_barrel_world_node src/control_barrel_world.cpp src/control_barrel_world_node.cpp)
add_dependencies(control_barrel_world_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_barrel_world_node ${catkin_LIBRARIES})
