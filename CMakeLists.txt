    cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

    project(dds-gazebo-plugins)
    set(CMAKE_MODULE_PATH
        ${CMAKE_MODULE_PATH}
        "${CMAKE_CURRENT_SOURCE_DIR}/resources/cmake")

    include(ConnextDdsCodegen)
    find_package(RTIConnextDDS)

    set (CMAKE_CXX_STANDARD 11)

    include (FindPkgConfig)
    if (PKG_CONFIG_FOUND)
        pkg_check_modules(GAZEBO gazebo)
        pkg_check_modules(SDF sdformat)
    endif()

    connextdds_rtiddsgen_run(
        LANG C++11
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated"
        IDL_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/LaserScanMsg.idl"
        UNBOUNDED
    )
    
    add_library(GazeboDDSLaser MODULE src/laser_scan/LaserScan.cxx ${LaserScanMsg_CXX11_SOURCES})

    target_link_libraries(GazeboDDSLaser 
        ${GAZEBO_LIBRARIES}  
        ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
        ${CONNEXTDDS_EXTERNAL_LIBS} 
        ${GAZEBO_LIBRARY_DIRS}/libRayPlugin.so
    )

    target_include_directories(GazeboDDSLaser PRIVATE 
        ${CONNEXTDDS_INCLUDE_DIRS} 
        ${GAZEBO_INCLUDE_DIRS} 
        ${SDF_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}/generated
        ${CMAKE_CURRENT_SOURCE_DIR}/src/common
    )

    target_compile_definitions(GazeboDDSLaser PRIVATE ${CONNEXTDDS_DEFINITIONS})
