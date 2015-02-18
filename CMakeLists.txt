cmake_minimum_required(VERSION 2.4.6)
project(or_rviz)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
    rviz
    geometry_msgs
    interactive_markers
    openrave_catkin
    std_msgs
    visualization_msgs
)
include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${OpenRAVE_INCLUDE_DIRS}
)
link_directories(
    ${catkin_LIBRARY_DIRS}
    ${OpenRAVE_LIBRARY_DIRS}
)

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(OGRE REQUIRED)
include_directories(SYSTEM ${OGRE_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS})

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        ${PROJECT_NAME}
        ${PROJECT_NAME}_markers
    CATKIN_DEPENDS
        geometry_msgs
        interactive_markers
        std_msgs
        visualization_msgs
    DEPENDS
        boost
        openrave
)

include_directories(include/${PROJECT_NAME})

include(DetectCXX11Flags)

# Helper library that implements core functionality.
add_library(${PROJECT_NAME}_markers SHARED
    src/markers/JointMarker.cpp
    src/markers/KinBodyJointMarker.cpp
    src/markers/KinBodyLinkMarker.cpp
    src/markers/KinBodyMarker.cpp
    src/markers/LinkMarker.cpp
    src/markers/ManipulatorMarker.cpp
    src/util/ScopedConnection.cpp
    src/util/InteractiveMarkerGraphHandle.cpp
    src/util/ogre_conversions.cpp
    src/util/ros_conversions.cpp
)
target_link_libraries(${PROJECT_NAME}_markers
    ${catkin_LIBRARIES}
)

# RViz viewer plugins.
qt4_wrap_cpp(RVIZ_MOC
    include/${PROJECT_NAME}/rviz/EnvironmentDisplay.h
)

add_library(${PROJECT_NAME}_rviz SHARED
    src/rviz/EnvironmentDisplay.cpp
    ${RVIZ_MOC}
)

# OpenRAVE viewer classes.
qt4_wrap_cpp(VIEWER_MOC
    include/${PROJECT_NAME}/RVizViewer.h
)

add_library(${PROJECT_NAME} SHARED
    src/InteractiveMarkerViewer.cpp
    src/RVizViewer.cpp
    ${VIEWER_MOC}
)
target_link_libraries(${PROJECT_NAME}
    ${PROJECT_NAME}_markers
    ${PROJECT_NAME}_rviz
    ${catkin_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${QT_LIBRARIES}
)

# Stub library that registers the plugins with OpenRAVE.
openrave_plugin(${PROJECT_NAME}_plugin
    src/plugin.cpp
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
