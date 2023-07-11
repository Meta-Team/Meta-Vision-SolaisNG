cmake_minimum_required(VERSION 3.8)
project(solais_auto_aim)

## Use C++14
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(magic_enum CONFIG REQUIRED)
ament_auto_find_build_dependencies()

# Armor Detector
add_library(solais_armor_detector SHARED
  src/detector/solais_armor_detector.cpp
  src/detector/solais_armor_detector_node.cpp
  src/detector/solais_pos_solver.cpp
  src/detector/solais_armor_lenet.cpp
)
rclcpp_components_register_node(solais_armor_detector
  PLUGIN "solais_auto_aim::ArmorDetectorNode"
  EXECUTABLE solais_armor_detector_node)
target_include_directories(solais_armor_detector PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(solais_armor_detector 
  rclcpp
  rclcpp_components
  cv_bridge
  OpenCV
  solais_camera
  solais_interfaces
  magic_enum
  tf2_geometry_msgs
)
target_link_libraries(solais_armor_detector magic_enum::magic_enum)

# Armor Tracker
# add_library(solais_armor_tracker_component SHARED
#   src/tracker/solais_armor_tracker.cpp
# )
# rclcpp_components_register_node(solais_armor_tracker_component
# PLUGIN "solais_auto_aim::ArmorDetectorNode"
# EXECUTABLE solais_armor_tracker_node)
# target_include_directories(solais_armor_tracker_component PRIVATE
# $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# $<INSTALL_INTERFACE:include>)



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

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS solais_armor_detector
    EXPORT export_${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

ament_package()
