cmake_minimum_required(VERSION 2.8.3)
project(square_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(blue_square_detection_node src/blue_square_detection_node.cpp)
target_link_libraries(blue_square_detection_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

