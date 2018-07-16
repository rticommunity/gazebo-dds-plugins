add_library(DdsImuPlugin MODULE 
    ImuScan.cxx 
    ImuScan.h 
    $<TARGET_OBJECTS:QuaternionTypeLibrary>
    $<TARGET_OBJECTS:Vector3TypeLibrary>
    $<TARGET_OBJECTS:TimeTypeLibrary>
    $<TARGET_OBJECTS:HeaderTypeLibrary>
    $<TARGET_OBJECTS:ImuTypeLibrary>
)

target_link_libraries(DdsImuPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
)

target_include_directories(DdsImuPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/src/common"
)

target_compile_definitions(DdsImuPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
