cmake_minimum_required(VERSION 2.8.3)
project(imu_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imu_driver
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  modules/BMI160_driver
)

option(DEFINE_DEBUG
  "Build the project with debug logs enabled"
  OFF)
if(DEFINE_DEBUG)
  add_definitions(-DDEBUG)
endif()

add_executable(${PROJECT_NAME}_node src/imu_driver_node.cpp src/ImuDeviceBmi160.cpp src/spi.cpp modules/BMI160_driver/bmi160.c)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

