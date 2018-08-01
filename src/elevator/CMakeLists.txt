find_library(ElevatorPlugin 
    NAMES ElevatorPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsElevatorPlugin MODULE 
    Elevator.cxx
    Elevator.h
)

target_link_libraries(DdsElevatorPlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    ${ElevatorPlugin}
    rosddstypes
)

target_include_directories(DdsElevatorPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsElevatorPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)

add_executable (elevatorpublisher 
    ElevatorPublisher.cxx 
)

target_link_libraries(elevatorpublisher 
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
    rosddstypes
)

target_include_directories(elevatorpublisher PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(elevatorpublisher PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
