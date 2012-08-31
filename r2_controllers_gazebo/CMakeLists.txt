cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

#common commands for building c++ executables and libraries
rosbuild_add_library(kdl_treeid_solver src/treeidsolver_recursive_newton_euler.cpp)
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)


rosbuild_add_library(kdl_treeid_solver src/treeidsolver_recursive_newton_euler.cpp)


rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(r2_gaze_controller src/r2_gaze_controller.cpp src/r2_gaze_ik.cpp)
rosbuild_link_boost(r2_gaze_controller signals)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(r2_controller_lib src/r2_impedance_controller.cpp)
rosbuild_add_library(r2_controller_lib src/r2_joint_trajectory_action_controller.cpp)
target_link_libraries(r2_controller_lib kdl_treeid_solver)
