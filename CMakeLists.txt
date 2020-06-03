cmake_minimum_required(VERSION 3.0.2)
project(trajectory_error)

find_package(catkin REQUIRED COMPONENTS
  rosbag
  roscpp
  rospy
  std_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_error
#  CATKIN_DEPENDS rosbag roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

#############
## Testing ##
#############

add_executable(error src/Calc_Errors.cpp)
target_link_libraries(error ${catkin_LIBRARIES})