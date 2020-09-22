cmake_minimum_required(VERSION 2.8.3)
project(jetson_can_interface)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sgtdv_msgs
)

catkin_package(
  CATKIN_DEPENDS sgtdv_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/JetsonCanInterfaceNODE.cpp src/JetsonCanInterface.cpp )

target_link_libraries(jetson_can_interface
  ${catkin_LIBRARIES}
)


