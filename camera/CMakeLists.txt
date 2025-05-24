add_library(camera STATIC
    camera.h
    rpi_camera.cc
    rpi_camera.h
    transcoder.h
    transcoder.cc
    v4l2_camera.h
    v4l2_camera.cc)

target_link_libraries(camera PUBLIC utils ffmpeg::ffmpeg rclcpp::rclcpp Camera::Camera)
target_include_directories(camera
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
