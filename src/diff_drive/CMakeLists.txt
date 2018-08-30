add_library(DdsDiffDrivePlugin MODULE 
    DiffDrive.cxx
    DiffDrive.h
)

target_link_libraries(DdsDiffDrivePlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    rosddstypes
)

target_include_directories(DdsDiffDrivePlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsDiffDrivePlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS})

add_executable (diffdrivepublisher
    DiffDrivePublisher.cxx 
)

target_link_libraries(diffdrivepublisher 
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}    
    rosddstypes
)

target_include_directories(diffdrivepublisher PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(diffdrivepublisher PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
