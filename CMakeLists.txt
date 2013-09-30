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
SET(QT_USE_QTOPENGL TRUE)
include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)

#Eigen stuff
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

qt4_wrap_cpp(MOC_FILES
  src/SuperViewer.h
)


set(SOURCE_FILES
  src/SuperViewer.cpp
  src/Plugins/KinBodyDisplay.cpp
  src/Plugins/KinBodyVisual.cpp
  src/Plugins/LinkVisual.cpp
  ${MOC_FILES})

#Building the viewer
rosbuild_add_library(${PROJECT_NAME} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} default_plugin)

rosbuild_add_executable(test_superviewer src/TestSuperviewer.cpp)
target_link_libraries(test_superviewer ${PROJECT_NAME})

#Ensure we are linking to OPENGL
target_link_libraries(${PROJECT_NAME} GL)


