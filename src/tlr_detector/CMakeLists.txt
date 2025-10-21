cmake_minimum_required(VERSION 3.8)
project(tlr_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ultralytics_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Add the include directory
include_directories(
  include
)

# Add tlr_detector_node executable
add_executable(tlr_detector_node
  src/tlr_detector_node.cpp
)
ament_target_dependencies(tlr_detector_node
  rclcpp
  vision_msgs
  ultralytics_ros
  std_msgs
)
install(TARGETS
  tlr_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()