# # 2.8.8 required to use PROTOBUF_IMPORT_DIRS
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(testing)
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/cmake")
message(${CMAKE_MODULE_PATH})
FIND_PACKAGE(Boost 1.40 COMPONENTS system REQUIRED )
FIND_PACKAGE(RTIConnextDDS)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")
#
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

add_definitions("${CONNEXTDDS_DEFINITIONS}")
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${SDF_INCLUDE_DIRS}
  ${CONNEXTDDS_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS} $ENV{LD_LIBRARY_PATH})

add_library(GazeboDDSLaser SHARED src/gazebo-dds-laser/gazebo-dds-laser.cxx src/gazebo-dds-laser/laserScanMsg.cxx src/gazebo-dds-laser/laserScanMsg.hpp src/gazebo-dds-laser/laserScanMsgImpl.cxx src/gazebo-dds-laser/laserScanMsgImpl.h src/gazebo-dds-laser/laserScanMsgImplPlugin.cxx src/gazebo-dds-laser/laserScanMsgImplPlugin.h )
target_link_libraries(GazeboDDSLaser ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${SDF_LIBRARIES} ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
${CONNEXTDDS_EXTERNAL_LIBS} RayPlugin)
