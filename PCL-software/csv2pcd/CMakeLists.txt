cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
add_definitions(-std=c++14)

project(csv2pcd)

find_package(PCL 1.2 REQUIRED)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (csv2pcd src/main_csv2pcd.cpp src/CSV2PCD.cpp)
target_link_libraries (csv2pcd ${PCL_LIBRARIES})

