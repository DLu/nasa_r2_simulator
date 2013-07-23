cmake_minimum_required(VERSION 2.8.3)
project(r2_controllers_gazebo)

# system dependency, is this needed?
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS r2_msgs controller_interface control_msgs control_toolbox realtime_tools urdf)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(r2_gaze_controller src/r2_gaze_controller.cpp src/r2_gaze_ik.cpp)
target_link_libraries(r2_gaze_controller ${Boost_LIBRARIES})

add_library(kdl_treeid_solver src/treeidsolver_recursive_newton_euler.cpp include/r2_controllers_gazebo/treeidsolver_recursive_newton_euler.hpp)
target_link_libraries(kdl_treeid_solver ${catkin_LIBRARIES})

add_library(r2_controller_lib
  src/r2_impedance_controller.cpp include/r2_controllers_gazebo/r2_impedance_controller.h
  src/r2_joint_trajectory_action_controller.cpp include/r2_controllers_gazebo/r2_joint_trajectory_action_controller.h)
target_link_libraries(r2_controller_lib kdl_treeid_solver)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS r2_msgs controller_interface control_msgs control_toolbox realtime_tools urdf
  INCLUDE_DIRS include
  LIBRARIES kdl_treeid_solver r2_controller_lib
  )

# Install
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS r2_gaze_controller
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS kdl_treeid_solver
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(TARGETS r2_controller_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES r2_controllers_gazebo_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
