cmake_minimum_required(VERSION 3.5)
project(image_subscriber)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in further dependencies
# manually. find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
# set(OpenCV_DIR /home/workspace/Library/opencv_build) # Directory include
# OpenCVConfig.cmake
find_package(OpenCV 4 REQUIRED # PATHS /opt/OpenCV4/ # Path OpenCV installed
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights uncomment
  # the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE) the following line skips cpplint (only
  # works in a git repo) uncomment the line when this package is not in a git
  # repo set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Package includes
include_directories(include ${OpenCV_INCLUDE_DIRS} "/usr/include/eigen3")

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# =============
# Camera node
# =============

add_library(cam_node SHARED src/cam_node.cpp)
ament_target_dependencies(cam_node rclcpp OpenCV)

add_library(optical_flow SHARED src/optical_flow.cpp)
ament_target_dependencies(optical_flow rclcpp OpenCV)

# =============
# Camera main
# =============

add_executable(cam_main src/cam_main.cpp)
target_link_libraries(cam_main cam_node optical_flow)
ament_target_dependencies(cam_main rclcpp)

# =============
# Export Best practice, see
# https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
# =============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_cam_node export_optical_flow)

ament_export_libraries(export_cam_node export_optical_flow)

ament_package()

# =============
# Install
# =============

# Install nodes
install(
  TARGETS cam_node
  EXPORT export_cam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  TARGETS optical_flow
  EXPORT export_optical_flow
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# Install executables
install(TARGETS cam_main DESTINATION lib/${PROJECT_NAME})

# Install various directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
