cmake_minimum_required(VERSION 2.8.3)
project(holy)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
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
add_executable(holy
    src/holy_fsm_defines
    src/holy_main.cpp
    src/core.cpp
    src/walk.cpp
    src/fight.cpp
    src/kinect.cpp
    src/stairs.cpp
    src/poses/poses.cpp
    src/poses/limbpose.cpp
    src/poses/robopose.cpp
    src/poses/parser.cpp
    src/CSV/csvrow.cpp
    src/CSV/csviterator.cpp
)

## Specify libraries to link a library or executable target against
 target_link_libraries( holy ${catkin_LIBRARIES} )

