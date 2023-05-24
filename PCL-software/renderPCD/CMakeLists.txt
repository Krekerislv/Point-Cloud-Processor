cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(renderPCD)

find_package(PCL 1.2 REQUIRED)
find_package(VTK 7.1 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


add_executable (renderPCD src/renderPCD.cpp src/ProcessPCDs.cpp)
target_link_libraries (renderPCD ${PCL_LIBRARIES})



