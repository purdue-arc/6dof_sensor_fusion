cmake_minimum_required(VERSION 2.8.3)
project(m7_sensor_fusion)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  cmake_modules
  tf
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include include/m7_sensor_fusion
#  LIBRARIES m7_sensor_fusion
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


add_library(ekf include/m7_sensor_fusion/EKF.cpp
				include/m7_sensor_fusion/Types.h
				include/m7_sensor_fusion/Parameters.h)
target_link_libraries(ekf ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

add_executable(sensor_fusion src/sensor_fusion.cpp)
target_link_libraries(sensor_fusion ${catkin_LIBRARIES} ekf)


