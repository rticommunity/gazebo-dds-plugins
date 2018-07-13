include(CMakeParseArguments)
include(ConnextDdsArgumentChecks)
include(ConnextDdsCodegen)

#[[

ConnextDdsAddTypeObjectLibrary
-----------------

Generate a object file using connextdds_rtiddsgen_run to generate the code of the idl file::

    connextdds_add_type_object_library(
        LANG <language>
        OUTPUT_DIRECTORY <dir>
        IDL_FILE <idl file path>
        OBJECT_LIBRARY_NAME <library_name>
        SOURCES <sources>
        [DEPENDENCIES ...]
        [INCLUDE_DIRS ...]
        [UNBOUNDED]
    )

Arguments:

``IDL_FILE`` (mandatory)
    The IDL filename that will be used to generate code

``OUTPUT_DIRECTORY`` (mandatory)
    The directory where to put generated files

``LANG`` (mandatory)
    The language to generate source files for. Expected values are:
    C, C++, C++03, C++11, C++/CLI, C# and Java.

``OBJECT_LIBRARY_NAME`` (mandatory)
    The name of the object file that will be generated

``SOURCES`` (mandatory)
    The sources of the object file 

``DEPENDENCIES`` (optional)
    The dependencies of the idl

``INCLUDE_DIRS`` (optional)
    List of include directories passed to Codegen (-I flag)

``UNBOUNDED`` (optional)
    Generate type files with unbounded support (``-unboundedSupport``) flag.

Output values:

``<OBJECT_LIBRARY_NAME>``
    The object file that was generated 

#]]

macro(connextdds_add_type_object_library)
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
