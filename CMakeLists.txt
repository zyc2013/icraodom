cmake_minimum_required(VERSION 2.8.3)
project(rikirobot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  riki_msgs
  std_msgs
  tf
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rikirobot
#  CATKIN_DEPENDS geometry_msgs ros_arduino_msgs roscpp rospy rosserial_python sensor_msgs std_msgs tf tf
#  DEPENDS system_lib
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_executable(bringup src/riki_base_node_l.cpp)
target_link_libraries(bringup ${catkin_LIBRARIES} )
add_executable(riki_base_node src/riki_base_node.cpp)
target_link_libraries(riki_base_node ${catkin_LIBRARIES})
add_executable(bringup_icra src/riki_base_node_icra.cpp)
target_link_libraries(bringup_icra ${catkin_LIBRARIES} )
add_executable(text src/text.cpp)
target_link_libraries(text ${catkin_LIBRARIES} )

add_executable(bringup_pose_icra src/icra_tdt_pose.cpp)
target_link_libraries(bringup_pose_icra ${catkin_LIBRARIES} )


