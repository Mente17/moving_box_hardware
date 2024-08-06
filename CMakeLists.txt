cmake_minimum_required(VERSION 3.8)
project(moving_box_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  moving_box_hardware
  SHARED
  src/moving_box_hardware_interface.cpp
)
target_include_directories(
  moving_box_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  moving_box_hardware
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  std_msgs
)
# prevent pluginlib from using boost
target_compile_definitions(moving_box_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface moving_box_hardware.xml)

install(
  TARGETS
  moving_box_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  PROGRAMS moving_box_hardware/gps_sender.py
  DESTINATION lib/moving_box_hardware
)

install(
  PROGRAMS moving_box_hardware/test_sub.py
  DESTINATION lib/moving_box_hardware
)

install(
  DIRECTORY description/ros2_control description/urdf
#  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/moving_box_hardware
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/moving_box_hardware
)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(hardware_interface REQUIRED)
  ament_add_gmock(test_moving_box_hardware_interface test/test_moving_box_hardware_interface.cpp)
  target_include_directories(test_moving_box_hardware_interface PRIVATE include)
  ament_target_dependencies(
    test_moving_box_hardware_interface
    hardware_interface
    pluginlib
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  moving_box_hardware
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
