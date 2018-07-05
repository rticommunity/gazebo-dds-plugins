include(ConnextDdsCodegen)
include(ConnectDdsTypeObjectLibrary)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Time.idl"
    OBJECT_LIBRARY_NAME "TimeTypeLibrary"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Header.idl"
    OBJECT_LIBRARY_NAME "HeaderTypeLibrary"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/RegionOfInterest.idl"
    OBJECT_LIBRARY_NAME "RegionOfInterestTypeLibrary"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/CameraInfo.idl"
    OBJECT_LIBRARY_NAME "CameraInfoTypeLibrary"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

connectdds_type_object_library(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sensor_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/sensor_msgs/msg/Image.idl"
    OBJECT_LIBRARY_NAME "ImageTypeLibrary"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
    UNBOUNDED
)
