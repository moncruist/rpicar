# Try to find libcamera library
# Sets the following variables
#
# Camera_FOUND - system has camera_ library
# Camera_INCLUDE_DIRS - camera_ include directories
# Camera_LIBRARIES - camera_ library

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
find_package_handle_standard_args(Camera REQUIRED_VARS
    LIBCAMERA_INCLUDE_DIR
    LIBCAMERA_LIBRARY
    LIBCAMERA_BASE_LIBRARY)

if(Camera_FOUND)
    set(Camera_INCLUDE_DIRS ${LIBCAMERA_INCLUDE_DIR})
    set(Camera_LIBRARIES
        ${LIBCAMERA_LIBRARY}
        ${LIBCAMERA_BASE_LIBRARY})
    if(NOT TARGET Camera::Camera)
        add_library(Camera::_Camera UNKNOWN IMPORTED)
        set_target_properties(Camera::_Camera PROPERTIES
            IMPORTED_LOCATION "${LIBCAMERA_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBCAMERA_INCLUDE_DIR}")

        add_library(Camera::_CameraBase UNKNOWN IMPORTED)
        set_target_properties(Camera::_CameraBase PROPERTIES
            IMPORTED_LOCATION "${LIBCAMERA_BASE_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${LIBCAMERA_INCLUDE_DIR}")

        add_library(CameraAll INTERFACE)
        target_link_libraries(CameraAll
                INTERFACE
                Camera::_Camera
                Camera::_CameraBase)
        add_library(Camera::Camera ALIAS CameraAll)
    endif()
endif()

mark_as_advanced(Camera_INCLUDE_DIRS Camera_LIBRARIES)