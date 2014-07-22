cmake_minimum_required(VERSION 2.8.3)
project(gazebo_taskboard)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_runtime message_generation actionlib geometry_msgs pluginlib roscpp rospy rostest std_msgs)

## System dependencies are found with CMake's conventions
find_package(PkgConfig)
find_package(Boost REQUIRED COMPONENTS system)
find_package(SDFormat REQUIRED)
find_package(gazebo REQUIRED)

# parse gazebo flags
set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()
set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  TaskboardPanelA.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ManipulateNumPad.srv
  ManipulatePowerCover.srv
  ManipulatePowerSwitch.srv
  ManipulateRockerSwitch.srv
  ManipulateSafeToggle.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES gazebo_taskboard
   CATKIN_DEPENDS message_runtime message_generation actionlib geometry_msgs pluginlib roscpp rospy rostest std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

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
add_library(gazebo_taskboard src/gazebo_taskboard/GazeboTaskboard.cpp include/gazebo_taskboard/GazeboTaskboard.h)
target_link_libraries(gazebo_taskboard ${SDFormat_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
set_target_properties(gazebo_taskboard PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(gazebo_taskboard PROPERTIES COMPILE_FLAGS "${cxx_flags}")
add_dependencies(gazebo_taskboard ${PROJECT_NAME}_gencpp)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS gazebo_taskboard
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

install(DIRECTORY Media 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
