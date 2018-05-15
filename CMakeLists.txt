cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(dds-gazebo-plugins)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/cmake")

find_package(RTIConnextDDS)
find_package(gazebo)

set (CMAKE_CXX_STANDARD 11)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/laser_scan")
