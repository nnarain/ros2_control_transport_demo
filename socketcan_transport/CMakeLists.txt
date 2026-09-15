cmake_minimum_required(VERSION 3.16)
project(socketcan_transport LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transport_interface REQUIRED)

add_library(socketcan_transport SHARED
  src/socketcan_transport.cpp
)
target_include_directories(socketcan_transport PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/socketcan_transport>
)
target_link_libraries(socketcan_transport PUBLIC
  pluginlib::pluginlib
  rclcpp::rclcpp
  transport_interface::transport_interface
)

pluginlib_export_plugin_description_file(socketcan_transport plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include/socketcan_transport
)
install(
  TARGETS socketcan_transport
  EXPORT export_socketcan_transport
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_targets(export_socketcan_transport HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib rclcpp transport_interface)
ament_package()
