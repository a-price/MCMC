# Project Description
cmake_minimum_required(VERSION 2.6)
project(MCMC CXX C)
set(CMAKE_CXX_FLAGS "-std=c++0x")

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#execute_process(COMMAND export "ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ROS_PACKAGE_PATH")
rosbuild_init() #export ROS_PACKAGE_PATH=/path/to/proj:$ROS_PACKAGE_PATH
link_libraries(roscpp roscpp_serialization rosconsole rostime roslib tf cv_bridge image_transport rosbag message_filters)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/find/")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Include Threading
find_package (Threads)
link_libraries("${CMAKE_THREAD_LIBS_INIT}")

# Include Boost
find_package(Boost REQUIRED)# COMPONENTS system filesystem thread)
include_directories(${BOOST_INCLUDE_DIRS})
link_libraries(${BOOST_LIBRARIES})
link_libraries(boost_system boost_thread boost_signals) #???

# Include the OpenCV library
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

# Include the PCL library
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})
message("Found PCL:" ${PCL_INCLUDE_DIRS})

# Include the Eigen library
#set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(Eigen REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Include OpenGL, GLUT and GLU
find_package (OpenGL REQUIRED)
find_package (GLUT REQUIRED)
find_package (GLU REQUIRED)
include_directories(${GLUT_INCLUDE_DIR} ${GLU_INCLUDE_DIR})
link_libraries(${GLUT_LIBRARY} ${GLU_LIBRARY} glui)

# Include the PNG library
include_directories("/usr/include/libpng12")
link_libraries("-lpng12")

# Include gtsam
find_package(GTSAM REQUIRED)
include_directories(AFTER ${GTSAM_INCLUDE_DIRS})
link_libraries(gtsam-shared)

# Include CppUnit
find_package(CppUnit)
include_directories(${CPPUNIT_INCLUDE_DIR})
link_libraries(${CPPUNIT_LIBRARIES})

# Include the project files
include_directories(include)

# Include the Hubo_Tech components
#include_directories(modules/hubo-motion-rt/include)
#include_directories(modules/hubo-ach/include)

  
# Collect the source, script and test files
file(GLOB lib_source "src/libraries/*.cpp")
LIST(SORT lib_source)
file(GLOB tests_source "src/tests/*.cpp")
LIST(SORT tests_source)
file(GLOB unit_tests_source "src/tests/unit/*.cpp")
LIST(SORT unit_tests_source)
file(GLOB system_tests_source "src/tests/system/*.cpp")
LIST(SORT system_tests_source)
file(GLOB scripts_source "src/exe/*.cpp")
LIST(SORT scripts_source)

# Build library
add_library(${PROJECT_NAME} SHARED ${lib_source})
#target_link_libraries(${PROJECT_NAME} freenect) 



# Build executables
message(STATUS "\n-- EXECUTABLES: ")
foreach(script_src_file ${scripts_source})
    get_filename_component(script_base ${script_src_file} NAME_WE)
    message(STATUS "Adding executable ${script_base}" )
    add_executable(${script_base} ${script_src_file})
    target_link_libraries(${script_base} ${PROJECT_NAME})
endforeach(script_src_file)

