find_library(RayPlugin 
    NAMES RayPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsLaserPlugin MODULE 
    LaserScan.cxx 
    LaserScan.h
    $<TARGET_OBJECTS:TimeTypeLibrary>
    $<TARGET_OBJECTS:HeaderTypeLibrary>
    $<TARGET_OBJECTS:LaserScanTypeLibrary>
)

target_link_libraries(DdsLaserPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
    ${RayPlugin}
)

target_include_directories(DdsLaserPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/src/common"
)

target_compile_definitions(DdsLaserPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
