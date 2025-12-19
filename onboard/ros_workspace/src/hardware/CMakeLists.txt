cmake_minimum_required(VERSION 3.8)
project(hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# Only one library: hardware
add_library(hardware SHARED 
  src/mecanumdrive_esp.cpp 
  src/wheel.cpp 
  src/esp_comms.cpp
)

target_include_directories(
  hardware
  PRIVATE
  include
)

ament_target_dependencies(
  hardware
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

# Plugin export - Use the correct file name: robot_hardware.xml
pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

# Installation
install(
  TARGETS hardware
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  FILES robot_hardware.xml  # Install the XML file
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_libraries(hardware)
ament_export_include_directories(include)
ament_export_dependencies(
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

ament_package()