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
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Vector3.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/geometry_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/geometry_msgs/msg/Wrench.idl"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/gazebo_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/gazebo_msgs/msg/ContactState.idl"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/gazebo_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/gazebo_msgs/msg/ContactsState.idl"
    INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
)

find_library(ContactPlugin 
    NAMES ContactPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsBumperScanPlugin MODULE 
    BumperScan.cxx
    BumperScan.h
    ${Time_CXX11_SOURCES}
    ${Header_CXX11_SOURCES}
    ${Vector3_CXX11_SOURCES}
    ${Wrench_CXX11_SOURCES}
    ${ContactState_CXX11_SOURCES}
    ${ContactsState_CXX11_SOURCES}
)

target_link_libraries(DdsBumperScanPlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_STATIC}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    ${ContactPlugin}
)

target_include_directories(DdsBumperScanPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsBumperScanPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
