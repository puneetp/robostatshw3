cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Pfilter)
file(GLOB_RECURSE PfilterFiles "main.cpp" "bee-map.c" "bee-map.h")
add_executable (Pfilter ${PfilterFiles})


#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#target_link_libraries (Pfilter ${PCL_LIBRARIES})