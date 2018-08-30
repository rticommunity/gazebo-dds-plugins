add_library(DdsApiPlugin MODULE 
    ApiPlugin.cxx 
    ApiPlugin.h 
)

target_link_libraries(DdsApiPlugin 
    ${GAZEBO_LIBRARIES}  
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${MESSAGING_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS} 
    rosddstypes
)

target_include_directories(DdsApiPlugin PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS} 
    ${GAZEBO_INCLUDE_DIRS} 
    ${SDF_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(DdsApiPlugin PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)

add_executable (apipublisher
    ApiPublisher.cxx 
    ApiParametersManager.cxx
)

target_link_libraries(apipublisher 
    ${CONNEXTDDS_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${MESSAGING_CPP2_API_LIBRARIES_RELEASE_SHARED}
    ${CONNEXTDDS_EXTERNAL_LIBS}    
    rosddstypes
)
target_include_directories(apipublisher PRIVATE 
    ${CONNEXTDDS_INCLUDE_DIRS}
    "${CMAKE_SOURCE_DIR}/src"
)

target_compile_definitions(apipublisher PRIVATE 
    ${CONNEXTDDS_DEFINITIONS}
)
