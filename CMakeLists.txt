cmake_minimum_required(VERSION 2.8.3)
project(colored_ball_detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cv_tutorial
  CATKIN_DEPENDS cv_bridge image_transport roscpp sensor_msgs std_msgs geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(colored_ball_detector src/colored_ball_detector.cpp)

target_link_libraries(colored_ball_detector
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
