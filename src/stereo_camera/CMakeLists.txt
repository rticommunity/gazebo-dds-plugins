add_library(DdsStereoCameraPlugin MODULE 
    StereoCamera.cxx 
    StereoCamera.h
)

target_link_libraries(DdsStereoCameraPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    rosddstypes
)

target_include_directories(DdsStereoCameraPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsStereoCameraPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
