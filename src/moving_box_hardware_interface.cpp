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

#include <limits>
#include <vector>
#include <cmath>

#include "moving_box_hardware/moving_box_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace moving_box_hardware
{

using std::placeholders::_1;

MovingBoxRaspberryNode::MovingBoxRaspberryNode() : Node("moving_box_raspberry_node")
{
  posLeft = 0;
  posRight = 0;
  
  left_wheel_publisher_ = this->create_publisher<std_msgs::msg::Int16>("cmd_left", 10);
  right_wheel_publisher_ = this->create_publisher<std_msgs::msg::Int16>("cmd_right", 10);
  
  left_wheel_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "pos_left", 10, std::bind(&MovingBoxRaspberryNode::left_wheel_callback, this, _1));
  right_wheel_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
      "pos_right", 10, std::bind(&MovingBoxRaspberryNode::right_wheel_callback, this, _1));
}

void MovingBoxRaspberryNode::publish_cmd_left(int16_t message)
{
  std_msgs::msg::Int16 cmd;
  cmd.data = message;
  left_wheel_publisher_->publish(cmd);
}

void MovingBoxRaspberryNode::publish_cmd_right(int16_t message)
{
  std_msgs::msg::Int16 cmd;
  cmd.data = message;
  right_wheel_publisher_->publish(cmd);
}

void MovingBoxRaspberryNode::left_wheel_callback (const std_msgs::msg::Int32 & msg) const
{
//  RCLCPP_INFO(rclcpp::get_logger("MovingBoxHardware"), "I heard posLeft: '%d'", msg.data);
  posLeft = msg.data;
}

void MovingBoxRaspberryNode::right_wheel_callback (const std_msgs::msg::Int32 & msg) const
{
//  RCLCPP_INFO(this->get_logger(), "I heard posRight: '%d'", msg.data);
  posRight = msg.data;
}

hardware_interface::CallbackReturn MovingBoxHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // TODO(anyone): read parameters and initialize the hardware
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // MovingBox has exactly one state and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MovingBoxHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MovingBoxHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MovingBoxHardware"),
        "Joint '%s' has %zu state interfaces found. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MovingBoxHardware"),
        "Joint '%s' have '%s' state interfaces found. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  
  raspi_node = std::make_shared<MovingBoxRaspberryNode>(); // fire up the Raspberry node

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovingBoxHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to be ready for read calls and write calls of some interfaces

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MovingBoxHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MovingBoxHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn MovingBoxHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_states_.size(); i++)
  {
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MovingBoxHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // TODO(anyone): prepare the robot to stop receiving commands

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MovingBoxHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (rclcpp::ok())
  {
    rclcpp::spin_some(raspi_node);
  }
  hw_states_[left_wheel] = (double) raspi_node->posLeft * 2.0 * M_PI / (double)TICKS_PER_ROTATION;
  hw_states_[right_wheel] = (double) raspi_node->posRight * 2.0 * M_PI / (double)TICKS_PER_ROTATION;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MovingBoxHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO(anyone): write robot's commands'
  raspi_node->publish_cmd_left((int16_t) (hw_commands_[left_wheel] * 9.5493)); //* 9.5493 -> conversion from rad/s to rpm
  raspi_node->publish_cmd_right((int16_t) (hw_commands_[right_wheel] * 9.5493));

  return hardware_interface::return_type::OK;
}

}  // namespace moving_box_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  moving_box_hardware::MovingBoxHardware, hardware_interface::SystemInterface)
