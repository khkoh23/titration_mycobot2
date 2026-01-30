cmake_minimum_required(VERSION 3.8)
project(titration_robot_syringe)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(titration_robot_interfaces REQUIRED)

include_directories(include)

add_executable(delta_ph_server src/delta_ph_server.cpp)
ament_target_dependencies(delta_ph_server
  rclcpp
  std_msgs
  titration_robot_interfaces)

add_executable(derivative_ph_volume_server src/derivative_ph_volume_server.cpp)
ament_target_dependencies(derivative_ph_volume_server
  rclcpp
  std_msgs
  titration_robot_interfaces)

add_executable(syringe_equivalence_executor src/syringe_equivalence_executor.cpp)
ament_target_dependencies(syringe_equivalence_executor 
  behaviortree_cpp_v3
  rclcpp
  rclcpp_action
  sensor_msgs
  titration_robot_interfaces)

add_executable(syringe_server src/syringe_server.cpp)
ament_target_dependencies(syringe_server
  rclcpp
  std_msgs
  titration_robot_interfaces)

install(TARGETS 
  delta_ph_server
  derivative_ph_volume_server
  syringe_equivalence_executor
  syringe_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  tree
  DESTINATION share/${PROJECT_NAME})

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
