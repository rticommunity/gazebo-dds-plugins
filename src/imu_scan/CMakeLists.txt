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
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
        IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Quaternion.idl"
    )

connextdds_rtiddsgen_run(
        LANG C++11
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
        IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Vector3.idl"
    )

connextdds_rtiddsgen_run(
        LANG C++11
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/Imu.idl"
        INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
    )

add_library(DdsImuPlugin MODULE 
    ImuScan.cxx 
    ImuScan.h 
    ${Quaternion_CXX11_SOURCES}
    ${Vector3_CXX11_SOURCES}
    ${Time_CXX11_SOURCES}
    ${Header_CXX11_SOURCES}
    ${Imu_CXX11_SOURCES}
)

target_link_libraries(DdsImuPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
)

target_include_directories(DdsImuPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_definitions(DdsImuPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS})
