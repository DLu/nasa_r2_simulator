cmake_minimum_required(VERSION 2.8.3)
project(gazebo_gripper)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)

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
#  LIBRARIES r2_gazebo
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
#//////////////////////////////////////////////////////////***Gazebo
include(FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# Declare a cpp library
add_library(gazebo_gripper
  src/${PROJECT_NAME}/GazeboGripper.cpp
)

## Declare a cpp executable
# add_executable(r2_gazebo_node src/r2_gazebo_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(r2_gazebo_node r2_gazebo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(r2_gazebo_node
#   ${catkin_LIBRARIES}
# )


