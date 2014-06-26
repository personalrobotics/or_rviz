cmake_minimum_required(VERSION 2.8.3)
project(or_rviz)

find_package(catkin REQUIRED COMPONENTS rviz)
catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# OpenRAVE
find_package(OpenRAVE REQUIRED)
include_directories(${OpenRAVE_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS})

# OGRE
include(FindPkgConfig)
pkg_check_modules(OGRE REQUIRED OGRE)
include_directories(${OGRE_INCLUDE_DIRS})

# Eigen
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
link_directories(${EIGEN_DEFINITIONS})

# QT
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
add_definitions(-DQT_NO_KEYWORDS)
include(${QT_USE_FILE})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

# Build the RViz plugin.
qt4_wrap_cpp(MOC_FILES 
    src/OpenRaveRviz.h 
    src/Plugins/EnvironmentDisplay.h
    src/Plugins/KinBodyVisual.h
)
add_library("${PROJECT_NAME}_rvizplugin" SHARED
    src/Plugins/EnvironmentDisplay.cpp
    src/Plugins/KinBodyVisual.cpp
    src/Plugins/LinkVisual.cpp
    ${RVIZ_PLUGIN_FILES}
)
target_link_libraries("${PROJECT_NAME}_rvizplugin"
    ${OpenRAVE_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${catkin_LIBRARIES}
)

# Build the OpenRAVE viewer plugin.
add_library("${PROJECT_NAME}_plugin" SHARED
    src/OpenRaveRviz.cpp
    ${MOC_FILES}
)
target_link_libraries("${PROJECT_NAME}_plugin"
    "${PROJECT_NAME}_rvizplugin"
    ${QT_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
    ${OGRE_LIBRARIES}
    ${catkin_LIBRARIES}
)
set_target_properties("${PROJECT_NAME}_plugin" PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)

install(TARGETS "${PROJECT_NAME}_rvizplugin"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
)
install(TARGETS "${PROJECT_NAME}_plugin"
    LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}/openrave-${OpenRAVE_LIBRARY_SUFFIX}"
)
