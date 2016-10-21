cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Pfilter)
include_directories(include)

find_package(Eigen REQUIRED)
include_directories(EIGEN3_INCLUDE_DIR)

file(GLOB SOURCES "src/*.cpp" "src/*.c")
add_executable (Pfilter ${SOURCES})


#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#target_link_libraries (Pfilter ${PCL_LIBRARIES})
