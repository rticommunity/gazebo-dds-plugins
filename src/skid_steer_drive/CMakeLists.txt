add_library(DdsSkidSteerDrivePlugin MODULE 
    SkidSteerDrive.cxx
    SkidSteerDrive.h
)

target_link_libraries(DdsSkidSteerDrivePlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    rosddstypes
)

target_include_directories(DdsSkidSteerDrivePlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsSkidSteerDrivePlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
