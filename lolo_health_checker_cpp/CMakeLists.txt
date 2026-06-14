cmake_minimum_required(VERSION 3.8)
project(lolo_health_checker_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(PkgConfig)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lolo_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)

add_executable(lolo_health_checker_cpp src/lolo_health_checker.cpp)
# target_include_directories(lolo_health_checker_cpp PUBLIC ./include/${GeographicLib_INCLUDE_DIRS})
target_link_libraries(lolo_health_checker_cpp
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${smarc_msgs_TARGETS}
  ${lolo_msgs_TARGETS}
  )

install(TARGETS
  lolo_health_checker_cpp
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
