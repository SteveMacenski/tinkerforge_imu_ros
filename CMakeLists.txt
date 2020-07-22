cmake_minimum_required(VERSION 2.8.3)
project(tinkerforge_imu_ros)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(tinkerforge_imu_ros src/tinkerforge_imu_ros.cpp
                                 src/brick_imu_v2.cpp
                                 src/ip_connection.cpp)
target_link_libraries(tinkerforge_imu_ros ${catkin_LIBRARIES})

install(TARGETS tinkerforge_imu_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

