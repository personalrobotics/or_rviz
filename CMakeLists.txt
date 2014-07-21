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

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")

add_definitions(--std=c++0x)

rosbuild_add_library("${PROJECT_NAME}"
    src/or_interactivemarker.cpp
    src/or_conversions.cpp
    src/KinBodyMarker.cpp
    src/KinBodyLinkMarker.cpp
    src/KinBodyJointMarker.cpp
    src/LinkMarker.cpp
    src/JointMarker.cpp
    src/ManipulatorMarker.cpp
)
rosbuild_add_library("${PROJECT_NAME}_plugin" SHARED
    src/or_interactivemarker_plugin.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin" "${PROJECT_NAME}")
