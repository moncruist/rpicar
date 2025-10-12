# Try to find libcamera library
# Sets the following variables
#
# camera_FOUND - system has camera_ library
# camera_INCLUDE_DIRS - camera_ include directories
# camera_LIBRARIES - camera_ library

# libcamera
find_path(LIBCAMERA_INCLUDE_DIR NAMES libcamera/camera.h
    HINTS
    /usr/local/include
    /usr/local/include/libcamera
    /usr/include/libcamera
    /usr/include)
find_library(LIBCAMERA_LIBRARY NAMES camera libcamera
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

find_library(LIBCAMERA_BASE_LIBRARY NAMES camera-base libcamera-base
    HINTS
    /usr/local/lib
    /usr/local/lib64
    /usr/lib
    /usr/lib64)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(camera REQUIRED_VARS
    LIBCAMERA_INCLUDE_DIR
    LIBCAMERA_LIBRARY
    LIBCAMERA_BASE_LIBRARY)

if(camera_FOUND)
    set(camera_INCLUDE_DIRS ${LIBCAMERA_INCLUDE_DIR})
    set(camera_LIBRARIES
        ${LIBCAMERA_LIBRARY}
        ${LIBCAMERA_BASE_LIBRARY})
    if(NOT TARGET camera::camera)
        add_library(camera::_camera UNKNOWN IMPORTED)
        set_target_properties(camera::_camera PROPERTIES
            IMPORTED_LOCATION "${LIBCAMERA_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBCAMERA_INCLUDE_DIR}")

        add_library(camera::_camerabase UNKNOWN IMPORTED)
        set_target_properties(camera::_camerabase PROPERTIES
            IMPORTED_LOCATION "${LIBCAMERA_BASE_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBCAMERA_INCLUDE_DIR}")

        add_library(CameraAll INTERFACE)
        target_link_libraries(CameraAll
                INTERFACE
                camera::_camera
                camera::_camerabase)
        add_library(camera::camera ALIAS CameraAll)
    endif()
endif()

mark_as_advanced(camera_INCLUDE_DIRS camera_LIBRARIES)