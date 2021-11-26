cmake_minimum_required(VERSION 2.8.3)
project(odom)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
  marvelmind_nav
)

catkin_package(
CATKIN_DEPENDS 
std_msgs
nav_msgs
tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(hedge_odom  src/hedge_odom.cpp)
target_link_libraries(hedge_odom  ${catkin_LIBRARIES})

add_executable(udp_recv_demo  src/udp_recv_demo.cpp)
target_link_libraries(udp_recv_demo  ${catkin_LIBRARIES})


