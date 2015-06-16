cmake_minimum_required(VERSION 2.8.3)
project(holy_cog)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})


## Declare a cpp executable
add_executable(cog
    src/cog.cpp
)

## Specify libraries to link a library or executable target against
 target_link_libraries( cog ${catkin_LIBRARIES} )
