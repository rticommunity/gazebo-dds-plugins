include(ConnextDdsCodegen)

connextdds_rtiddsgen_run(
        LANG C++11
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
        IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Time.idl"
    )

connextdds_rtiddsgen_run(
        LANG C++11
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
        IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Header.idl"
    )

connextdds_rtiddsgen_run(
        LANG C++11
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/LaserScanMsg.idl"
        INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl"
        UNBOUNDED
    )

find_library(RayPlugin 
    NAMES RayPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsLaserPlugin MODULE 
    LaserScan.cxx 
    LaserScan.h 
    ${Time_CXX11_SOURCES}
    ${Header_CXX11_SOURCES}
    ${LaserScanMsg_CXX11_SOURCES}
)

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
    ${CONNEXTDDS_DEFINITIONS})
