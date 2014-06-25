cmake_minimum_required(VERSION 2.8.3)
include(FindPkgConfig)

# TODO: We might be missing dependencies here.
find_package(catkin REQUIRED)
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS rviz openrave
    DEPENDS Boost
)
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# OpenRAVE
find_package(OpenRAVE REQUIRED)
include_directories(${OPENRAVE_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS})

# OGRE
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
             src/Plugins/KinBodyVisual.h)
add_library(${PROJECT_NAME}_rvizplugin SHARED
    src/Plugins/EnvironmentDisplay.cpp
    src/Plugins/KinBodyVisual.cpp
    src/Plugins/LinkVisual.cpp
    ${RVIZ_PLUGIN_FILES}
)

# Build the OpenRAVE viewer plugin.
add_library(${PROJECT_NAME}_plugin SHARED
    src/OpenRaveRviz.cpp
    ${MOC_FILES})
target_link_libraries(${PROJECT_NAME}_plugin
    "${PROJECT_NAME}_rvizplugin"
    ${QT_LIBRARIES}
    ${OpenRAVE_LIBRARIES}
)
set_target_properties(${PROJECT_NAME}_plugin PROPERTIES
    PREFIX ""
    COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}"
    LINK_FLAGS "${OpenRAVE_LINK_FLAGS}"
)

# Build a test program.
#add_executable(test_or_rviz src/TestOrRviz.cpp)
#target_link_libraries(test_or_rviz
#    "${PROJECT_NAME}_plugin"
#    ${Boost_LIBRARIES}
#    ${OpenRAVE_LIBRARIES}
#)

#install(TARGETS or_ompl
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
#install(TARGETS or_ompl_plugin
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/openrave0.9)
#install(DIRECTORY include/${PROJECT_NAME}/
#    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#    PATTERN ".svn" EXCLUDE)
