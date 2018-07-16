include(ConnextDdsCodegen)

find_library(ContactPlugin 
    NAMES ContactPlugin
    PATHS ${GAZEBO_LIBRARY_DIRS}
)

add_library(DdsBumperScanPlugin MODULE 
    BumperScan.cxx
    BumperScan.h
    $<TARGET_OBJECTS:TimeTypeLibrary>
    $<TARGET_OBJECTS:HeaderTypeLibrary>
    $<TARGET_OBJECTS:Vector3TypeLibrary>
    $<TARGET_OBJECTS:WrenchTypeLibrary>
    $<TARGET_OBJECTS:ContactStateTypeLibrary>
    $<TARGET_OBJECTS:ContactsStateTypeLibrary>
)

target_link_libraries(DdsBumperScanPlugin 
    ${GAZEBO_LIBRARIES}
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}
    ${ContactPlugin}
)

target_include_directories(DdsBumperScanPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${SDF_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_BINARY_DIR}/src/common"
)

target_compile_definitions(DdsBumperScanPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
