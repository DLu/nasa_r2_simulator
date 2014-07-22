cmake_minimum_required(VERSION 2.8.3)
project(r2_gazebo)

find_package(catkin REQUIRED COMPONENTS gazebo_msgs kdl_parser r2_msgs roscpp rospy sensor_msgs std_msgs tf urdf)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES r2_gazebo
  CATKIN_DEPENDS r2_msgs
#  DEPENDS system_lib
)

execute_process(COMMAND setup.sh ${CMAKE_SOURCE_DIR}/)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY Media config urdf robots launch gazebo
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
