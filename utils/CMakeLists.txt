add_library(utils STATIC
    assert.h
    compat.h
    error_utils.cc
    error_utils.h
    logging.h)

target_include_directories(utils
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(STACKTRACE_LIB stdc++exp)
endif()

target_link_libraries(utils PUBLIC ${STACKTRACE_LIB} rclcpp::rclcpp ${COMPAT_FMT})
