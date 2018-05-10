
#####################################################################
# Compile idl files                                                 #
#####################################################################
include(ConnextDdsCodegen)

connextdds_rtiddsgen_run(
        LANG C++11
        OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/generated"
        IDL_FILE "${CMAKE_CURRENT_SOURCE_DIR}/src/LaserScanMsg.idl"
        UNBOUNDED
    )
