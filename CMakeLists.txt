# Copyright 2018 Real-Time Innovations, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(gazebo-dds-plugins)

set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/cmake")

find_package(RTIConnextDDS COMPONENTS messaging_api)
find_package(gazebo)

set (CMAKE_CXX_STANDARD 11)

add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/resources/idl")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/laser_scan")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/imu_scan")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/diff_drive")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/elevator")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/bumper_scan")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/skid_steer_drive")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/camera")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/stereo_camera")
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/src/api_plugin")
