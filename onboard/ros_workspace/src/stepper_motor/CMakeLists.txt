cmake_minimum_required(VERSION 3.5)
project(stepper_motor)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Stepper Motor Controller Library
add_library(stepper_motor_controller 
  src/stepper_motor_controller.cpp
)
target_include_directories(stepper_motor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stepper_motor_controller rclcpp std_msgs)

# Stepper Motor Node
add_executable(stepper_motor_node 
  src/stepper_motor_node.cpp
)
target_include_directories(stepper_motor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(stepper_motor_node rclcpp std_msgs sensor_msgs)
target_link_libraries(stepper_motor_node stepper_motor_controller)

# Stepper Motor Teleop
add_executable(stepper_motor_teleop src/stepper_motor_teleop.cpp)
ament_target_dependencies(stepper_motor_teleop rclcpp std_msgs)

# Install library
install(TARGETS stepper_motor_controller
  EXPORT stepper_motor_controller_export
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
  stepper_motor_node
  stepper_motor_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export for downstream packages
ament_export_targets(stepper_motor_controller_export HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs)

ament_package()