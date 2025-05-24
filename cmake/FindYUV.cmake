# Try to find yuv libraries
# Sets the following variables
#
# YUV_FOUND - system has yuv libraries
# YUV_INCLUDE_DIRS - yuv include directories
# YUV_LIBRARIES - yuv libraries

# libyuv
find_path(YUV_INCLUDE_DIR NAMES libyuv.h
    HINTS
    /usr/local/include
    /usr/local/include/yuv
    /usr/include
    /usr/include/yuv
)
find_library(YUV_LIBRARY NAMES yuv
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(YUV REQUIRED_VARS
    YUV_INCLUDE_DIR
    YUV_LIBRARY)

if(YUV_FOUND)
    set(YUV_INCLUDE_DIRS ${YUV_INCLUDE_DIR})
    set(YUV_LIBRARIES
        ${YUV_LIBRARY})
    if(NOT TARGET yuv::yuv)
        add_library(yuv::yuv UNKNOWN IMPORTED)
        set_target_properties(yuv::yuv PROPERTIES
            IMPORTED_LOCATION "${YUV_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${YUV_INCLUDE_DIR}")
    endif()
endif()

mark_as_advanced(YUV_INCLUDE_DIRS YUV_LIBRARIES)
