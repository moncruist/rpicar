# Try to find ffmpeg libraries
# Sets the following variables
#
# ffmpeg_FOUND - system has ffmpeg libraries
# ffmpeg_INCLUDE_DIRS - ffmpeg include directories
# ffmpeg_LIBRARIES - ffmpeg libraries

# libavcodec
find_path(LIBAVCODEC_INCLUDE_DIR NAMES libavcodec/avcodec.h
    HINTS
    /usr/local/include
    /usr/local/include/ffmpeg
    /usr/include
    /usr/include/ffmpeg
)
find_library(AVCODEC_LIBRARY NAMES avcodec libavcodec
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

# libavformat
find_path(LIBAVFORMAT_INCLUDE_DIR NAMES libavformat/avformat.h
    HINTS
    /usr/local/include
    /usr/local/include/ffmpeg
    /usr/include
    /usr/include/ffmpeg
)
find_library(AVFORMAT_LIBRARY NAMES avformat libavformat
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

#libavutil
find_path(LIBAVUTIL_INCLUDE_DIR NAMES libavutil/avutil.h
    HINTS
    /usr/local/include
    /usr/local/include/ffmpeg
    /usr/include
    /usr/include/ffmpeg
)
find_library(AVUTIL_LIBRARY NAMES avutil libavutil
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

#libswscale
find_path(LIBSWSCALE_INCLUDE_DIR NAMES libswscale/swscale.h
    HINTS
    /usr/local/include
    /usr/local/include/ffmpeg
    /usr/include
    /usr/include/ffmpeg
)
find_library(LIBSWSCALE_LIBRARY NAMES swscale libswscale
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

#libswresample
find_path(LIBSWRESAMPLE_INCLUDE_DIR NAMES libswresample/swresample.h
    HINTS
    /usr/local/include
    /usr/local/include/ffmpeg
    /usr/include
    /usr/include/ffmpeg
)
find_library(LIBSWRESAMPLE_LIBRARY NAMES swresample libswresample
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ffmpeg REQUIRED_VARS
    LIBAVCODEC_INCLUDE_DIR
    AVCODEC_LIBRARY

    LIBAVFORMAT_INCLUDE_DIR
    AVFORMAT_LIBRARY

    LIBAVUTIL_INCLUDE_DIR
    AVUTIL_LIBRARY

    LIBSWSCALE_INCLUDE_DIR
    LIBSWSCALE_LIBRARY

    LIBSWRESAMPLE_INCLUDE_DIR
    LIBSWRESAMPLE_LIBRARY)

if(ffmpeg_FOUND)
    set(ffmpeg_INCLUDE_DIRS ${LIBAVCODEC_INCLUDE_DIR})
    set(ffmpeg_LIBRARIES
        ${AVCODEC_LIBRARY}
        ${AVFORMAT_LIBRARY}
        ${AVUTIL_LIBRARY}
        ${LIBSWSCALE_LIBRARY}
        ${LIBSWRESAMPLE_LIBRARY})
    if(NOT TARGET ffmpeg::ffmpeg)
        add_library(ffmpeg::avcodec UNKNOWN IMPORTED)
        set_target_properties(ffmpeg::avcodec PROPERTIES
            IMPORTED_LOCATION "${AVCODEC_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBAVCODEC_INCLUDE_DIR}")

        add_library(ffmpeg::avformat UNKNOWN IMPORTED)
        set_target_properties(ffmpeg::avformat PROPERTIES
            IMPORTED_LOCATION "${AVFORMAT_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBAVFORMAT_INCLUDE_DIR}")

        add_library(ffmpeg::avutil UNKNOWN IMPORTED)
        set_target_properties(ffmpeg::avutil PROPERTIES
            IMPORTED_LOCATION "${AVUTIL_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBAVUTIL_INCLUDE_DIR}")

        add_library(ffmpeg::swscale UNKNOWN IMPORTED)
        set_target_properties(ffmpeg::swscale PROPERTIES
            IMPORTED_LOCATION "${LIBSWSCALE_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBSWSCALE_INCLUDE_DIR}")

        add_library(ffmpeg::swresample UNKNOWN IMPORTED)
        set_target_properties(ffmpeg::swresample PROPERTIES
            IMPORTED_LOCATION "${LIBSWRESAMPLE_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBSWRESAMPLE_INCLUDE_DIR}")

        add_library(ffmpegAll INTERFACE)
        target_link_libraries(ffmpegAll
            INTERFACE
            ffmpeg::avcodec
            ffmpeg::avformat
            ffmpeg::avutil
            ffmpeg::swscale
            ffmpeg::swresample)
        add_library(ffmpeg::ffmpeg ALIAS ffmpegAll)
    endif()
endif()

mark_as_advanced(ffmpeg_INCLUDE_DIRS ffmpeg_LIBRARIES)
