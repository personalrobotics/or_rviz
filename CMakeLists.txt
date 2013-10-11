cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")



#QT Stuff
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

#Eigen stuff
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

qt4_wrap_cpp(MOC_FILES
  src/OpenRaveRviz.h
)


set(SOURCE_FILES
  src/OpenRaveRviz.cpp
  src/Plugins/KinBodyDisplay.cpp
  src/Plugins/KinBodyVisual.cpp
  src/Plugins/LinkVisual.cpp
  ${MOC_FILES})

#Building the viewer
#link_directories(${OPENRAVE_LINK_DIRS})
rosbuild_add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
#set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(${PROJECT_NAME}  ${QT_LIBRARIES} default_plugin)

rosbuild_add_executable(test_or_rviz src/TestOrRviz.cpp)
target_link_libraries(test_or_rviz ${PROJECT_NAME})
#install(TARGETS ${PROJECT_NAME} DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-qtosgrave)
#Ensure we are linking to OPENGL
#target_link_libraries(${PROJECT_NAME} GL)


