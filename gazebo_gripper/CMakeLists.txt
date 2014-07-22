cmake_minimum_required(VERSION 2.8.3)
project(gazebo_gripper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)
find_package(SDFormat REQUIRED)

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES gazebo_gripper
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include(FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS} 
)

## Declare a cpp library
add_library(gazebo_gripper src/GazeboGripper.cpp include/gazebo_gripper/GazeboGripper.h)

## Specify libraries to link a library or executable target against
target_link_libraries(gazebo_gripper ${SDFormat_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(gazebo_gripper PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_gripper PROPERTIES COMPILE_FLAGS "${cxx_flags}")

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS gazebo_gripper 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
