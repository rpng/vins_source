cmake_minimum_required(VERSION 2.8.3)
project(microstrain_3dm_gx3_45)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  geometry_msgs
  geodesy
  geographic_msgs
)


catkin_package(
 CATKIN_DEPENDS 
  roscpp
  sensor_msgs
  tf
  geometry_msgs
  geodesy
  geographic_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
)

add_library(3dmgx3_45 src/driver.cpp)
add_executable(imu_driver src/node.cpp)
target_link_libraries(imu_driver 3dmgx3_45 ${catkin_LIBRARIES})

