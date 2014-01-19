cmake_minimum_required(VERSION 2.8.3)
project(gazebo_interface)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS r2_msgs sensor_msgs)

## System dependencies are found with CMake's conventions
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

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES gazebo_interface
  CATKIN_DEPENDS r2_msgs sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS} 
)

## Declare a cpp library
add_library(robot_controller
  src/RobotController.cpp include/${PROJECT_NAME}/RobotController.h
)
add_dependencies(robot_controller r2_msgs_gencpp)

add_library(gazebo_interface
  src/GazeboInterface.cpp include/${PROJECT_NAME}/GazeboInterface.h
  src/JointController.cpp include/${PROJECT_NAME}/JointController.h
)
add_dependencies(gazebo_interface r2_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gazebo_interface
  robot_controller
  ${SDFormat_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS gazebo_interface 
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

