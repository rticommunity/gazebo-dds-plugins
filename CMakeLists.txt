cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(dds-gazebo-plugins)
set(CMAKE_MODULE_PATH
    ${CMAKE_MODULE_PATH}
    "${CMAKE_CURRENT_SOURCE_DIR}/resources/cmake")

include(ConnextDdsCodegen)
find_package(RTIConnextDDS)

set (CMAKE_CXX_FLAGS "-g -Wall -std=c++11")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

link_directories(${GAZEBO_LIBRARY_DIRS} $ENV{LD_LIBRARY_PATH})

connextdds_rtiddsgen_run(
  LANG C++11
  OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated"
  IDL_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/laserScanMsg.idl"
  UNBOUNDED
)

add_library(GazeboDDSLaser MODULE src/gazebo_dds_laser.cxx ${laserScanMsg_CXX11_SOURCES})

target_link_libraries(GazeboDDSLaser 
  ${Boost_LIBRARIES} 
  ${GAZEBO_LIBRARIES} 
  ${SDF_LIBRARIES} 
  ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
  ${CONNEXTDDS_EXTERNAL_LIBS} 
  RayPlugin
)

target_include_directories(GazeboDDSLaser PRIVATE 
  ${CONNEXTDDS_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${SDF_INCLUDE_DIRS}
)

target_compile_definitions(GazeboDDSLaser PRIVATE ${CONNEXTDDS_DEFINITIONS})

