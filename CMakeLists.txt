cmake_minimum_required(VERSION 2.8.3)
project(zlab_drone)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  joy 
  message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  FILES
  Tak_land.msg
  Circles.msg
  Circle.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  zlab_drone
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

add_executable(vdlist src/videoFeedListener.cpp)
target_link_libraries(vdlist ${catkin_LIBRARIES})

add_executable(vdstab src/vidStab.cpp)
target_link_libraries(vdstab ${catkin_LIBRARIES})

add_executable(fly src/fly_from_joy.cpp)
target_link_libraries(fly ${catkin_LIBRARIES})
