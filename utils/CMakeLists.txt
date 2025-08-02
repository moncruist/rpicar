add_library(utils STATIC
    compat.h
    error_utils.cc
    error_utils.h
    logging.h)

target_include_directories(utils
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

target_link_libraries(utils PUBLIC rclcpp::rclcpp ${COMPAT_FMT})
