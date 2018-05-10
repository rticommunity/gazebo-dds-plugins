    cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

    project(dds-gazebo-plugins)

    set(CMAKE_MODULE_PATH
        ${CMAKE_MODULE_PATH}
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/cmake")

    find_package(RTIConnextDDS)
    find_package(gazebo)

    include(FindGazeboPlugin)
    include(CompileIdl)

    set (CMAKE_CXX_STANDARD 11)

    add_library(GazeboDDSLaser MODULE src/laser_scan/LaserScan.cxx ${LaserScanMsg_CXX11_SOURCES})

    target_link_libraries(GazeboDDSLaser 
        ${GAZEBO_LIBRARIES}  
        ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
        ${CONNEXTDDS_EXTERNAL_LIBS} 
        ${RayPlugin}
    )

    target_include_directories(GazeboDDSLaser PRIVATE 
        ${CONNEXTDDS_INCLUDE_DIRS} 
        ${GAZEBO_INCLUDE_DIRS} 
        ${SDF_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}/generated
        ${CMAKE_CURRENT_SOURCE_DIR}/src/common
    )

    target_compile_definitions(GazeboDDSLaser PRIVATE ${CONNEXTDDS_DEFINITIONS})
