cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    std_msgs
    visualization_msgs
    openrave_catkin
)
catkin_package()

include_directories(
    ${OpenRAVE_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${OpenRAVE_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

add_definitions(--std=c++0x)

openrave_plugin("${PROJECT_NAME}_plugin"
    src/or_interactivemarker_plugin.cpp
    src/or_interactivemarker.cpp
    src/or_conversions.cpp
    src/KinBodyMarker.cpp
    src/KinBodyLinkMarker.cpp
    src/KinBodyJointMarker.cpp
    src/LinkMarker.cpp
    src/JointMarker.cpp
    src/ManipulatorMarker.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin"
    ${catkin_LIBRARIES}
)
