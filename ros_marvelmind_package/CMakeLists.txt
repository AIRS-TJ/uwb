cmake_minimum_required(VERSION 2.8.3)
project(marvelmind_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  visualization_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  hedge_pos.msg
  hedge_pos_a.msg
  hedge_pos_ang.msg
  beacon_pos_a.msg
  hedge_imu_raw.msg
  hedge_imu_fusion.msg
  beacon_distance.msg
  hedge_telemetry.msg
  hedge_quality.msg
  marvelmind_waypoint.msg
)

generate_messages(
   DEPENDENCIES
  std_msgs
)

catkin_package(
INCLUDE_DIRS include/${PROJECT_NAME}/
#  LIBRARIES marvelmind_nav
CATKIN_DEPENDS message_runtime
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(include ${roscpp_INCLUDE_DIRS})

## Declare a C++ library
add_library(marvelmind_nav
   src/marvelmind_hedge.c
)


## Declare a C++ executable
add_executable(hedge_rcv_bin 
src/hedge_rcv_bin.cpp 
src/marvelmind_hedge.c
)

add_dependencies(hedge_rcv_bin marvelmind_nav_generate_messages_cpp)

target_link_libraries(hedge_rcv_bin
 ${catkin_LIBRARIES}
)

#####################

add_executable(subscriber_test 
src/subscriber_test.cpp
)

add_dependencies(subscriber_test marvelmind_nav_generate_messages_cpp)

target_link_libraries(subscriber_test
 ${catkin_LIBRARIES}
)

install(TARGETS hedge_rcv_bin subscriber_test
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

