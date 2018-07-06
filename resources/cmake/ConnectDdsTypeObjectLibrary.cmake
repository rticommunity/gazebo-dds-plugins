include(CMakeParseArguments)
include(ConnextDdsArgumentChecks)
include(ConnextDdsCodegen)

macro(connectdds_type_object_library)
    set(options
        NOT_REPLACE UNBOUNDED IGNORE_ALIGNMENT USE42_ALIGNMENT
        OPTIMIZE_ALIGNMENT NO_TYPECODE DISABLE_PREPROCESSOR STL STANDALONE
    )
    set(single_value_args LANG OUTPUT_DIRECTORY IDL_FILE VAR PACKAGE OBJECT_LIBRARY_NAME SOURCES)
    set(multi_value_args TYPE_NAMES INCLUDE_DIRS DEFINES EXTRA_ARGS)

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

    set(include_dirs "")
    if(_OBJLIB_INCLUDE_DIRS)
        set(include_dirs "INCLUDE_DIRS")
    endif()

    connextdds_rtiddsgen_run(
        LANG ${_OBJLIB_LANG}
        OUTPUT_DIRECTORY ${_OBJLIB_OUTPUT_DIRECTORY}
        IDL_FILE ${_OBJLIB_IDL_FILE}
        ${unbounded}
        # ${include_dirs}
        INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/resources/idl/"
    )

    add_library( ${_OBJLIB_OBJECT_LIBRARY_NAME} OBJECT
        ${${_OBJLIB_SOURCES}}
    )

    target_include_directories(${_OBJLIB_OBJECT_LIBRARY_NAME} PRIVATE 
        ${CONNEXTDDS_INCLUDE_DIRS}
        "${CMAKE_BINARY_DIR}/src/common"
    )

    target_compile_definitions(${_OBJLIB_OBJECT_LIBRARY_NAME} PRIVATE 
        ${CONNEXTDDS_DEFINITIONS}
    )

endmacro()