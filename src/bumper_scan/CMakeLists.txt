find_library(ContactPlugin 
    NAMES ContactPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsBumperScanPlugin MODULE 
    BumperScan.cxx
    BumperScan.h
)

target_link_libraries(DdsBumperScanPlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    ${ContactPlugin}
    rosddstypes
)

target_include_directories(DdsBumperScanPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsBumperScanPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
