cmake_minimum_required(VERSION 3.8)
project(lolo_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lpthread -lboost_thread)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
#find_package(Boost REQUIRED COMPONENTS asio)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lolo_msgs REQUIRED)
find_package(smarc_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)


include_directories(
  ${PROJECT_SOURCE_DIR}/lib/
)

set (UDP_INTERFACE_SRC
  lib/udpinterface/CaptainInterFace.cpp
  lib/udpinterface/UDPInterface.cpp
)

set (MINICAP_SRC
  lib/MiniCapHelpers/helper.cpp
)

add_executable(lolo_interface 
  src/interface.cpp
  src/view/view.cpp 
  src/controller/controller.cpp 
  ${UDP_INTERFACE_SRC}
  ${MINICAP_SRC})
  
ament_target_dependencies(lolo_interface 
  rclcpp 
  std_msgs
  lolo_msgs
  smarc_msgs
  diagnostic_msgs)


install(TARGETS
  lolo_interface
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
