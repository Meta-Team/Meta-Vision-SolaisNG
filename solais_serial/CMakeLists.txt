cmake_minimum_required(VERSION 3.8)
project(solais_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_library(solais_serial_legacy SHARED
  src/solais_serial_legacy.cpp
  src/crc.cpp
)
target_include_directories(solais_serial_legacy PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rclcpp_components_register_node(solais_serial_legacy
  PLUGIN "solais_serial::SerialNodeLegacy"
  EXECUTABLE solais_serial_legacy_node
)

ament_target_dependencies(solais_serial_legacy
  rclcpp
  rclcpp_components
  serial_driver
  auto_aim_interfaces
  rmoss_projectile_motion
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
)


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

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS solais_serial_legacy
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
