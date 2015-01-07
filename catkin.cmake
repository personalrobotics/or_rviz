cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    interactive_markers
    openrave_catkin
    std_msgs
    visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        geometry_msgs
        interactive_markers
        std_msgs
        visualization_msgs
    DEPENDS
        boost
        openrave
)

include_directories(
    include/${PROJECT_NAME}
    ${OpenRAVE_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)
link_directories(
    ${OpenRAVE_LIBRARY_DIRS}
    ${catkin_LIBRARY_DIRS}
)

add_definitions(--std=c++0x)

# Helper library that implements core functionality. This includes the OpenRAVE
# viewer plugins.
add_library(${PROJECT_NAME} SHARED
    src/or_interactivemarker.cpp
    src/markers/JointMarker.cpp
    src/markers/KinBodyJointMarker.cpp
    src/markers/KinBodyLinkMarker.cpp
    src/markers/KinBodyMarker.cpp
    src/markers/LinkMarker.cpp
    src/markers/ManipulatorMarker.cpp
    src/util/or_conversions.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

# Stub library that registers the plugins with OpenRAVE.
openrave_plugin(${PROJECT_NAME}_plugin
    src/or_interactivemarker_plugin.cpp
)
target_link_libraries(${PROJECT_NAME}_plugin
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
