cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
    interactive_markers
    std_msgs
    visualization_msgs
)
catkin_package()

find_package(OpenRAVE REQUIRED)

include_directories(
    ${OpenRAVE_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${OpenRAVE_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

add_definitions(--std=c++0x)

add_library("${PROJECT_NAME}"
    src/or_interactivemarker.cpp
    src/or_conversions.cpp
    src/KinBodyMarker.cpp
    src/KinBodyLinkMarker.cpp
    src/KinBodyJointMarker.cpp
    src/LinkMarker.cpp
    src/JointMarker.cpp
    src/ManipulatorMarker.cpp
)
add_library("${PROJECT_NAME}_plugin" SHARED
    src/or_interactivemarker_plugin.cpp
)
target_link_libraries("${PROJECT_NAME}_plugin" "${PROJECT_NAME}")
set_target_properties("${PROJECT_NAME}_plugin" PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)

install(TARGETS "${PROJECT_NAME}"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
)
install(TARGETS "${PROJECT_NAME}_plugin"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)
