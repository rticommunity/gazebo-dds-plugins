find_library(CameraPlugin 
    NAMES CameraPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsCameraPlugin MODULE 
    Camera.cxx 
    Camera.h
    $<TARGET_OBJECTS:TimeTypeLibrary>
    $<TARGET_OBJECTS:HeaderTypeLibrary>
    $<TARGET_OBJECTS:RegionOfInterestTypeLibrary>
    $<TARGET_OBJECTS:CameraInfoTypeLibrary>
    $<TARGET_OBJECTS:ImageTypeLibrary>
)

target_link_libraries(DdsCameraPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    ${CameraPlugin}
)

target_include_directories(DdsCameraPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/src/common"
)

target_compile_definitions(DdsCameraPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
    