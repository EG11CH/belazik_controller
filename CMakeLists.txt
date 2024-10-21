cmake_minimum_required(VERSION 3.8)
project(rack_pinion_controller)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(ackermann_steering_controller REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/test.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  control_toolbox
  ackermann_steering_controller
  controller_interface
  hardware_interface
  pluginlib
  urdf
)

pluginlib_export_plugin_description_file(controller_interface plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  control_toolbox
  ackermann_steering_controller
  controller_interface
  hardware_interface
  pluginlib
  urdf
)

ament_package()
