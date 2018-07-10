include(CMakeParseArguments)
include(ConnextDdsArgumentChecks)
include(ConnextDdsCodegen)

macro(connectdds_type_object_library)
    set(options UNBOUNDED)
    set(single_value_args LANG OUTPUT_DIRECTORY IDL_FILE 
        OBJECT_LIBRARY_NAME SOURCES 
    )
    set(multi_value_args INCLUDE_DIRS DEPENDENCIES)

    cmake_parse_arguments(_OBJLIB
        "${options}"
        "${single_value_args}"
        "${multi_value_args}"
        ${ARGN}
    )

    set(unbounded "")
    if(_OBJLIB_UNBOUNDED)
        set(unbounded "UNBOUNDED")
    endif()

    connextdds_rtiddsgen_run(
        LANG ${_OBJLIB_LANG}
        OUTPUT_DIRECTORY ${_OBJLIB_OUTPUT_DIRECTORY}
        IDL_FILE ${_OBJLIB_IDL_FILE}
        ${unbounded}
        INCLUDE_DIRS ${_OBJLIB_INCLUDE_DIRS}
    )
    
    add_library( ${_OBJLIB_OBJECT_LIBRARY_NAME} OBJECT
        ${${_OBJLIB_SOURCES}}
    )

    set_property(TARGET ${_OBJLIB_OBJECT_LIBRARY_NAME} PROPERTY 
        POSITION_INDEPENDENT_CODE ON
    )

    target_include_directories(${_OBJLIB_OBJECT_LIBRARY_NAME} PRIVATE 
        ${CONNEXTDDS_INCLUDE_DIRS}
        "${CMAKE_BINARY_DIR}/src/common"
    )

    target_compile_definitions(${_OBJLIB_OBJECT_LIBRARY_NAME} PRIVATE 
        ${CONNEXTDDS_DEFINITIONS}
    )

    if(_OBJLIB_DEPENDENCIES)
        add_dependencies(${_OBJLIB_OBJECT_LIBRARY_NAME} ${_OBJLIB_DEPENDENCIES})
    endif()

endmacro()
