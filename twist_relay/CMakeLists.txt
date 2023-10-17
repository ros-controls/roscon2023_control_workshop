cmake_minimum_required(VERSION 3.8)
project(twist_relay_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  geometry_msgs
  realtime_tools
  pluginlib
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(twist_relay_controller_parameters
  src/twist_relay_controller_parameter.yaml
)

add_library(
  twist_relay_controller
  SHARED
  src/twist_relay_controller.cpp
)
target_compile_features(twist_relay_controller PUBLIC cxx_std_17)
target_include_directories(twist_relay_controller PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(twist_relay_controller PUBLIC twist_relay_controller_parameters)
ament_target_dependencies(twist_relay_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(
  controller_interface controller_plugins.xml)

## INSTALL

install(
  DIRECTORY include/
  DESTINATION include/twist_relay_controller
)

install(
  TARGETS twist_relay_controller twist_relay_controller_parameters
  EXPORT export_twist_relay_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_twist_relay_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
