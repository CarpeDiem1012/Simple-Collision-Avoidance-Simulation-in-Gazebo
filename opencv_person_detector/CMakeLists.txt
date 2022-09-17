cmake_minimum_required(VERSION 3.0.2)
project(opencv_person_detector)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_person_detector
#  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

find_package(OpenCV)
include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_executable(opencv_person_detector_node src/opencv_person_detector.cpp src/opencv_person_detector_node.cpp)
target_link_libraries(opencv_person_detector_node ${OpenCV_LIBS})
target_link_libraries(opencv_person_detector_node ${catkin_LIBRARIES})
