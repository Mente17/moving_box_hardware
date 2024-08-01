// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOVING_BOX_HARDWARE__MOVING_BOX_HARDWARE_INTERFACE_HPP_
#define MOVING_BOX_HARDWARE__MOVING_BOX_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>

#include "moving_box_hardware/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int32.hpp"

#define TICKS_PER_ROTATION 90

namespace moving_box_hardware
{

enum
{
  left_wheel,
  right_wheel
};

class MovingBoxRaspberryNode : public rclcpp::Node
{
public:
  MovingBoxRaspberryNode();
  void publish_cmd_left(int16_t message);
  void publish_cmd_right(int16_t message);

  mutable int32_t posLeft;
  mutable int32_t posRight;
  
private:
  void left_wheel_callback (const std_msgs::msg::Int32 & msg) const;
  void right_wheel_callback (const std_msgs::msg::Int32 & msg) const;
  
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr left_wheel_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr right_wheel_publisher_;
  
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr left_wheel_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr right_wheel_subscription_;
};

class MovingBoxHardware : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
  std::shared_ptr<MovingBoxRaspberryNode> raspi_node; // make the Raspberry node a member

private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace moving_box_hardware

#endif  // MOVING_BOX_HARDWARE__MOVING_BOX_HARDWARE_INTERFACE_HPP_
