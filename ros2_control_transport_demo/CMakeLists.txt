cmake_minimum_required(VERSION 3.16)
project(ros2_control_transport_demo LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transport_interface REQUIRED)

add_library(transport_demo SHARED
  src/transport_demo_actuator.cpp
)
target_include_directories(transport_demo PRIVATE src)
target_link_libraries(transport_demo
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  transport_interface::transport_interface
)

pluginlib_export_plugin_description_file(hardware_interface transport_demo_plugins.xml)

install(
  DIRECTORY description config launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS transport_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
