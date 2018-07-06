add_library(DdsDiffDrivePlugin MODULE 
    DiffDrive.cxx
    DiffDrive.h
    $<TARGET_OBJECTS:TimeTypeLibrary>
    $<TARGET_OBJECTS:HeaderTypeLibrary>
    $<TARGET_OBJECTS:Pose2DTypeLibrary>
    $<TARGET_OBJECTS:JointStateTypeLibrary>
    $<TARGET_OBJECTS:PointTypeLibrary>
    $<TARGET_OBJECTS:QuaternionTypeLibrary>
    $<TARGET_OBJECTS:PoseTypeLibrary>
    $<TARGET_OBJECTS:PoseWithCovarianceTypeLibrary>
    $<TARGET_OBJECTS:Vector3TypeLibrary>
    $<TARGET_OBJECTS:TwistTypeLibrary>
    $<TARGET_OBJECTS:TwistWithCovarianceTypeLibrary>
    $<TARGET_OBJECTS:OdometryTypeLibrary>
)

target_link_libraries(DdsDiffDrivePlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
)

target_include_directories(DdsDiffDrivePlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/src/common"
)

target_compile_definitions(DdsDiffDrivePlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS})

add_executable (diffdrivepublisher
    DiffDrivePublisher.cxx 
    $<TARGET_OBJECTS:Vector3TypeLibrary>
    $<TARGET_OBJECTS:TwistTypeLibrary>
)

target_link_libraries(diffdrivepublisher 
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}    
)
target_include_directories(diffdrivepublisher PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/src/common"
)

target_compile_definitions(diffdrivepublisher PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
