cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(Pfilter)
include_directories(include)

#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})
#find_package(Eigen REQUIRED)
include_directories("/usr/include/eigen3/")
find_package(OpenCV REQUIRED)

file(GLOB run_sources "src/*.cpp" "src/*.c")
file(GLOB test_sources "src/*.cpp" "src/*.c")

list(REMOVE_ITEM run_sources "${CMAKE_CURRENT_SOURCE_DIR}/src/getsensormodel.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/probabilityDensityFunction.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/test_main.cpp")

list(REMOVE_ITEM test_sources "${CMAKE_CURRENT_SOURCE_DIR}/src/getsensormodel.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/probabilityDensityFunction.cpp" "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp")

#Print out the value of source value of source.
#message("${sources}")

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_executable (Pfilter ${run_sources})
target_link_libraries(Pfilter ${OpenCV_LIBS} )
add_executable (TestPfilter ${test_sources})
target_link_libraries(TestPfilter ${OpenCV_LIBS} )


#find_package(PCL 1.2 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#target_link_libraries (pfilter ${PCL_LIBRARIES})
