cmake_minimum_required(VERSION 2.8.3)
project(bioloid_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  bioloid_msg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs

)

catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(bioloid_interface src/bioloid_interface.cpp)
add_library(bioloid_lib include/dynamixel.c include/dxl_hal.c)
target_link_libraries(bioloid_interface bioloid_lib)
target_link_libraries(bioloid_interface ${catkin_LIBRARIES})
