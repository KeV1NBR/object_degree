
find_path(DARKNET_INCLUDE_DIRS yolo_v2_class.hpp
    "/home/kevin/api/darknet/include"
    )
find_library(DARKNET_LIBRARIES libdarknet.so
    "/home/kevin/api/darknet"
    )
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(darknet
    FOUND_VAR darknet_FOUND
    REQUIRED_VARS DARKNET_INCLUDE_DIRS DARKNET_LIBRARIES
    )

