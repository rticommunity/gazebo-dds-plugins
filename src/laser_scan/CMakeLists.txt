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

find_library(RayPlugin 
    NAMES RayPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsLaserPlugin MODULE 
    LaserScan.cxx 
    LaserScan.h
)

target_link_libraries(DdsLaserPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
    ${RayPlugin}
    rosddstypes
)

target_include_directories(DdsLaserPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS} 
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsLaserPlugin PRIVATE 
    ${CONNEXTDDS_COMPILE_DEFINITIONS}
)

install(TARGETS DdsLaserPlugin
    LIBRARY DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
)
