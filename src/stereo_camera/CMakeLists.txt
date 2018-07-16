add_library(DdsStereoCameraPlugin MODULE 
    StereoCamera.cxx 
    StereoCamera.h
    $<TARGET_OBJECTS:TimeTypeLibrary>
    $<TARGET_OBJECTS:HeaderTypeLibrary>
    $<TARGET_OBJECTS:RegionOfInterestTypeLibrary>
    $<TARGET_OBJECTS:CameraInfoTypeLibrary>
    $<TARGET_OBJECTS:ImageTypeLibrary>
)

target_link_libraries(DdsStereoCameraPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
)

target_include_directories(DdsStereoCameraPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/src/common"
)

target_compile_definitions(DdsStereoCameraPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
