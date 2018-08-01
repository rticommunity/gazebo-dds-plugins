add_library(DdsImuPlugin MODULE 
    ImuScan.cxx 
    ImuScan.h 
)

target_link_libraries(DdsImuPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
    rosddstypes
)

target_include_directories(DdsImuPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsImuPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
