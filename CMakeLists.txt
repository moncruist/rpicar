cmake_minimum_required(VERSION 3.25)
project(rpicar)

option(CLANG_TIDY_CHECK "Run clang-tidy checks" OFF)
option(DEBUG_ASSERTIONS "Enable debug assertions" OFF)
message(STATUS "Clang-tidy checks enabled: ${CLANG_TIDY_CHECK}")

set(CMAKE_CXX_STANDARD 23)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(DEBUG_ASSERTIONS OR CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(DEBUG_ASSERTIONS)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CLANG_TIDY_CHECK)
  set(CMAKE_CXX_CLANG_TIDY
    clang-tidy;
    -format-style='file';
    -header-filter=${CMAKE_CURRENT_SOURCE_DIR};
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ffmpeg REQUIRED)
find_package(YUV REQUIRED)
find_package(camera REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(image_transport REQUIRED)
find_package(Boost CONFIG REQUIRED COMPONENTS headers)

set(COMPAT_FMT "")
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 13)
    find_package(fmt REQUIRED)
    set(COMPAT_FMT fmt::fmt)
  endif()
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(MSG_FILES msg/CameraImageFrame.msg)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs sensor_msgs
)

rosidl_get_typesupport_target(rpicar_cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# suppress clang-tidy checks for generated files
foreach(typesupport
    "rosidl_typesupport_cpp"
    "rosidl_typesupport_c"
    "rosidl_typesupport_fastrtps_cpp"
    "rosidl_typesupport_fastrtps_c"
    "rosidl_typesupport_introspection_cpp"
    "rosidl_generator_cpp"
)
  rosidl_get_typesupport_target(target "${PROJECT_NAME}" "${typesupport}")
  get_target_property(sources ${target} SOURCES)
  set_source_files_properties(${sources} PROPERTIES SKIP_LINTING ON)
endforeach()

ament_export_dependencies(rosidl_default_runtime)

add_subdirectory(utils)
add_subdirectory(camera)
add_subdirectory(nodes)

add_executable(rpicar_start main.cc)
target_link_libraries(rpicar_start rclcpp::rclcpp camera_node)

install(TARGETS rpicar_start
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()