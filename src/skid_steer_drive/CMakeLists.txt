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
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Pose2D.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Vector3.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Twist.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Point.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Quaternion.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Pose.idl"
)    

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/PoseWithCovariance.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/TwistWithCovariance.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/nav_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/nav_msgs/msg/Odometry.idl"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/JointState.idl"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

add_library(DdsSkidSteerDrivePlugin MODULE 
    SkidSteerDrive.cxx
    SkidSteerDrive.h
    ${Time_CXX11_SOURCES}
    ${Header_CXX11_SOURCES}
    ${Pose2D_CXX11_SOURCES}
    ${JointState_CXX11_SOURCES}
    ${Point_CXX11_SOURCES}
    ${Quaternion_CXX11_SOURCES}
    ${Pose_CXX11_SOURCES}
    ${PoseWithCovariance_CXX11_SOURCES}
    ${Vector3_CXX11_SOURCES}
    ${Twist_CXX11_SOURCES}
    ${TwistWithCovariance_CXX11_SOURCES}
    ${Odometry_CXX11_SOURCES}
)

target_link_libraries(DdsSkidSteerDrivePlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
)

target_include_directories(DdsSkidSteerDrivePlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsSkidSteerDrivePlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
