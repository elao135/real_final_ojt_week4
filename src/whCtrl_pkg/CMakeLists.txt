cmake_minimum_required(VERSION 3.0.2)
project(whCtrl_pkg)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES whCtrl_pkg
#  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs
#  DEPENDS system_lib
)

###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(cmd_pub src/cmd_pub.cpp)
target_link_libraries(cmd_pub ${catkin_LIBRARIES})