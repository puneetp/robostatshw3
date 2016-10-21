cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Pfilter)
include_directories(include)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
#find_package(Eigen REQUIRED)
include_directories("/usr/include/eigen3/")

file(GLOB SOURCES "src/*.cpp" "src/*.c")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable (Pfilter ${SOURCES})


#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#target_link_libraries (Pfilter ${PCL_LIBRARIES})
