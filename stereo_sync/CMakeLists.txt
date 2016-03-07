cmake_minimum_required(VERSION 2.8.3)
project(stereo_sync)



find_package(catkin REQUIRED COMPONENTS
  image_transport
  camera_info_manager
  roscpp
  sensor_msgs
  geometry_msgs
  std_msgs
  tf
)


SET(LIB_TYPE STATIC) 


include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)



ADD_EXECUTABLE(stereo_sync src/cameras_timesync.cpp)


target_link_libraries(stereo_sync
    ${catkin_LIBRARIES}
)

