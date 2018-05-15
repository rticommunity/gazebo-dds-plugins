include(ConnextDdsCodegen)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    IDL_FILE "${CMAKE_CURRENT_SOURCE_DIR}/LaserScanMsg.idl"
    UNBOUNDED
)

find_library(RayPlugin 
    NAMES RayPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsLaserPlugin MODULE 
    LaserScan.cxx 
    LaserScan.h 
    ${LaserScanMsg_CXX11_SOURCES})

target_link_libraries(DdsLaserPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
    ${RayPlugin}
)

target_include_directories(DdsLaserPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_definitions(DdsLaserPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
