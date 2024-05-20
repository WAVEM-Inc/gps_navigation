cmake_minimum_required(VERSION 3.8)
project(brake_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(route_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)

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


# Add executable
add_executable(drive_break_publisher src/main.cpp)

# Include directories
include_directories(include)

# Link against libraries
ament_target_dependencies(drive_break_publisher rclcpp route_msgs)

# Install
install(TARGETS
  drive_break_publisher
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_export_dependencies(rclcpp route_msgs)


ament_package()
