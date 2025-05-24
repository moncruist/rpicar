add_library(camera_node STATIC
    camera_node_config.h
    camera_node.cc
    camera_node.h)

target_link_libraries(camera_node PUBLIC
    camera
    utils
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${cpp_typesupport_target}
    yuv::yuv)

target_include_directories(camera_node
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
