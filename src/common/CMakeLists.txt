include(ConnextDdsCodegen)
include(ConnectDdsTypeObjectLibrary)

set (CMAKE_CXX_FLAGS "-fPIC")

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Time.idl"
    OBJECT_LIBRARY_NAME "TimeTypeLibrary"
    SOURCES "Time_CXX11_SOURCES"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Header.idl"
    OBJECT_LIBRARY_NAME "HeaderTypeLibrary"
    SOURCES "Header_CXX11_SOURCES"
)

add_dependencies(HeaderTypeLibrary TimeTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/RegionOfInterest.idl"
    OBJECT_LIBRARY_NAME "RegionOfInterestTypeLibrary"
    SOURCES "RegionOfInterest_CXX11_SOURCES"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/CameraInfo.idl"
    OBJECT_LIBRARY_NAME "CameraInfoTypeLibrary"
    SOURCES "CameraInfo_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

add_dependencies(CameraInfoTypeLibrary HeaderTypeLibrary RegionOfInterestTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/Image.idl"
    OBJECT_LIBRARY_NAME "ImageTypeLibrary"
    SOURCES "Image_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
    UNBOUNDED
)

add_dependencies(ImageTypeLibrary HeaderTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/LaserScan.idl"
    OBJECT_LIBRARY_NAME "LaserScanTypeLibrary"
    SOURCES "LaserScan_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl"
    UNBOUNDED
)

add_dependencies(LaserScanTypeLibrary HeaderTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Quaternion.idl"
    OBJECT_LIBRARY_NAME "QuaternionTypeLibrary"
    SOURCES "Quaternion_CXX11_SOURCES"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Vector3.idl"
    OBJECT_LIBRARY_NAME "Vector3TypeLibrary"
    SOURCES "Vector3_CXX11_SOURCES"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/Imu.idl"
    OBJECT_LIBRARY_NAME "ImuTypeLibrary"
    SOURCES "Imu_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

add_dependencies(ImuTypeLibrary HeaderTypeLibrary Vector3TypeLibrary QuaternionTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Int32.idl"
    OBJECT_LIBRARY_NAME "Int32TypeLibrary"
    SOURCES "Int32_CXX11_SOURCES"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Wrench.idl"
    OBJECT_LIBRARY_NAME "WrenchTypeLibrary"
    SOURCES "Wrench_CXX11_SOURCES"
)

add_dependencies(WrenchTypeLibrary Vector3TypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/gazebo_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/gazebo_msgs/msg/ContactState.idl"
    OBJECT_LIBRARY_NAME "ContactStateTypeLibrary"
    SOURCES "ContactState_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

add_dependencies(ContactStateTypeLibrary Vector3TypeLibrary WrenchTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/gazebo_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/gazebo_msgs/msg/ContactsState.idl"
    OBJECT_LIBRARY_NAME "ContactsStateTypeLibrary"
    SOURCES "ContactsState_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

add_dependencies(ContactsStateTypeLibrary HeaderTypeLibrary ContactStateTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Pose2D.idl"
    OBJECT_LIBRARY_NAME "Pose2DTypeLibrary"
    SOURCES "Pose2D_CXX11_SOURCES"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Twist.idl"
    OBJECT_LIBRARY_NAME "TwistTypeLibrary"
    SOURCES "Twist_CXX11_SOURCES"
)

add_dependencies(TwistTypeLibrary Vector3TypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Point.idl"
    OBJECT_LIBRARY_NAME "PointTypeLibrary"
    SOURCES "Point_CXX11_SOURCES"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Pose.idl"
    OBJECT_LIBRARY_NAME "PoseTypeLibrary"
    SOURCES "Pose_CXX11_SOURCES"
)    

add_dependencies(PoseTypeLibrary PointTypeLibrary QuaternionTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/PoseWithCovariance.idl"
    OBJECT_LIBRARY_NAME "PoseWithCovarianceTypeLibrary"
    SOURCES "PoseWithCovariance_CXX11_SOURCES"
)

add_dependencies(PoseWithCovarianceTypeLibrary PoseTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/TwistWithCovariance.idl"
    OBJECT_LIBRARY_NAME "TwistWithCovarianceTypeLibrary"
    SOURCES "TwistWithCovariance_CXX11_SOURCES"
)

add_dependencies(TwistWithCovarianceTypeLibrary TwistTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/nav_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/nav_msgs/msg/Odometry.idl"
    OBJECT_LIBRARY_NAME "OdometryTypeLibrary"
    SOURCES "Odometry_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

add_dependencies(OdometryTypeLibrary HeaderTypeLibrary TwistWithCovarianceTypeLibrary PoseWithCovarianceTypeLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/JointState.idl"
    OBJECT_LIBRARY_NAME "JointStateTypeLibrary"
    SOURCES "JointState_CXX11_SOURCES"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

add_dependencies(JointStateTypeLibrary HeaderTypeLibrary)
