cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(change_detection)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (change_detection change_detection.cpp)
target_link_libraries (change_detection ${PCL_LIBRARIES})
