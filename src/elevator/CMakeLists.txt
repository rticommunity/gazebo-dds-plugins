include(ConnextDdsCodegen)

connextdds_rtiddsgen_run(
    LANG C++11
    OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/std_msgs/msg"
    IDL_FILE "${CMAKE_SOURCE_DIR}/resources/idl/std_msgs/msg/Int32.idl"
)

find_library(ElevatorPlugin 
    NAMES ElevatorPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsElevatorPlugin MODULE 
    Elevator.cxx
    Elevator.h
    ${Int32_CXX11_SOURCES}
)

target_link_libraries(DdsElevatorPlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    ${ElevatorPlugin}
)

target_include_directories(DdsElevatorPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsElevatorPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS})

add_executable (elevatorpublisher 
    ElevatorPublisher.cxx 
    ${Int32_CXX11_SOURCES} 
)

target_link_libraries(elevatorpublisher 
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
)

target_include_directories(elevatorpublisher PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(elevatorpublisher PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
