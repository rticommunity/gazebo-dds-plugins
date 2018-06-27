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
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/RegionOfInterest.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/CameraInfo.idl"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/Image.idl"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
    UNBOUNDED
)

add_library(DdsStereoCameraPlugin MODULE 
    StereoCamera.cxx 
    StereoCamera.h
    ${Time_CXX11_SOURCES}
    ${Header_CXX11_SOURCES}
    ${RegionOfInterest_CXX11_SOURCES}
    ${CameraInfo_CXX11_SOURCES}
    ${Image_CXX11_SOURCES}
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
)

target_compile_definitions(DdsStereoCameraPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
