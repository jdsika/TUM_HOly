cmake_minimum_required(VERSION 2.8.3)
project(holy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  bioloid_interface
  moveit_ros_planning_interface
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})


## Declare a cpp executable
 add_executable(turtle_house src/test.cpp)
 add_executable(wave src/wave.cpp)
 add_executable(moveit_controller src/moveit.cpp)
 add_executable(walk src/walk_main.cpp src/core.cpp src/walk.cpp src/poses.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries( turtle_house ${catkin_LIBRARIES} )
 target_link_libraries( wave ${catkin_LIBRARIES} )
 target_link_libraries( moveit_controller ${catkin_LIBRARIES} )
 target_link_libraries( walk ${catkin_LIBRARIES} )

