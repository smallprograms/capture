cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(capture)
find_package(PCL 1.0 REQUIRED COMPONENTS io visualization)
set(BIN_NAME "capture")

include_directories(/usr/include/eigen3)
include_directories(/usr/include/openni/)
include_directories(/usr/include/pcl-1.6/)

add_definitions(${PCL_DEFINITIONS})


add_executable(${BIN_NAME} src/capture.cpp)
target_link_libraries(${BIN_NAME} OpenNI)
target_link_libraries(${BIN_NAME} pcl_io)
target_link_libraries(${BIN_NAME} pcl_visualization)
target_link_libraries(${BIN_NAME} pcl_registration)
target_link_libraries(${BIN_NAME} pcl_common)
target_link_libraries(${BIN_NAME} pcl_kdtree)
target_link_libraries(${BIN_NAME} pcl_octree)
target_link_libraries(${BIN_NAME} pcl_filters)



