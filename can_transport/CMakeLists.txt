cmake_minimum_required(VERSION 3.16)
project(can_transport LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(transport_interface REQUIRED)

add_library(can_transport SHARED
  src/mock_can_transport.cpp
)
target_include_directories(can_transport PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(can_transport PUBLIC
  pluginlib::pluginlib
  transport_interface::transport_interface
)

pluginlib_export_plugin_description_file(transport_interface plugins.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS can_transport
  EXPORT export_can_transport
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_include_directories(include)
ament_export_targets(export_can_transport HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib transport_interface)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(rclcpp REQUIRED)
  ament_add_gtest(test_transport test/test_transport.cpp)
  target_link_libraries(test_transport can_transport)

  ament_add_gmock(test_resource_manager_transport test/test_resource_manager_transport.cpp)
  ament_target_dependencies(test_resource_manager_transport hardware_interface rclcpp)
  target_link_libraries(
    test_resource_manager_transport hardware_interface::hardware_interface can_transport)

  add_library(test_hardware_components SHARED
    test/test_hardware_components/test_transport_consumer_actuator.cpp
  )
  ament_target_dependencies(test_hardware_components hardware_interface rclcpp)
  target_link_libraries(test_hardware_components hardware_interface::hardware_interface can_transport)
  install(TARGETS test_hardware_components DESTINATION lib)
  pluginlib_export_plugin_description_file(
    hardware_interface test/test_hardware_components/test_hardware_components.xml
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
