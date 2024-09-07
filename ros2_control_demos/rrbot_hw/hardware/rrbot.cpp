// Copyright 2020 ros2_control Development Team
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

#include "rrbot_hw/rrbot.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rrbot_hw
{
hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_effort_  .resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_speed_ .resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Loop through {command, state} interfaces and make sure they are the right number and type
  for (const hardware_interface::ComponentInfo & joint : info_.joints){
    const char interface_classes[][] = {"command", "state"}; // Just for debug output
    hardware_interface::ComponentInfo interface_infos[][] = {joint.command_interfaces, joint.state_interfaces}
    for (int i=0; i<sizeof(interface_types)/sizeof(interface_types)[0]; i++){
      const int len = sizeof(command_interface_types[i])/sizeof(command_interface_types[i][0]);

      if (interface_infos[i].size() != len){
        RCLCPP_FATAL(
          rclcpp::get_logger("RRBotSystemEffortOnlyHardware"),
          "Joint '%s' has %zu %s interfaces. %zu expected.", joint.name.c_str(),
          interface_infos[i].size(), interface_classes[i], len);
        return hardware_interface::CallbackReturn::ERROR;
      }

      for(int j=0; j<len; j++)
        if (interface_infos[i][j].name != command_interface_types[i][j]){
          RCLCPP_FATAL(
            rclcpp::get_logger("RRBotSystemEffortOnlyHardware"),
            "Joint '%s' has %s %s interfaces. '%s' expected.", joint.name.c_str(),
            interface_infos[i][i].name.c_str(), interface_classes[i], interface_types[i][j]);
          return hardware_interface::CallbackReturn::ERROR;
        }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Configuring ...please wait...");

  // reset state and command values before connecting
  for (uint i = 0; i < hw_states_[0].size(); i++){
    for (uint j = 0; j < num_command; j++)
      hw_commands_[j][i] = 0;
    for (uint j = 0; j < num_state; j++)
      hw_states_[j][i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RRBotSystemEffortOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
    for (const char[] s : interface_types[InterfaceClass::STATE])
      state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, s, &hw_states_[j][i]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemEffortOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
    for (const char[] s : interface_types[InterfaceClass::COMMAND])
      command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, s, &hw_commands_[j][i]));

  return command_interfaces;
}

hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RRBotSystemEffortOnlyHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RRBotSystemEffortOnlyHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Reading...");

  for (uint i = 0; i < hw_states_position_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[2][i] = hw_commands_[1][i];
    hw_states_[1][i] += (hw_commands_[1][i]*0.5 - hw_states_[1][i])/2;
    hw_states_[0][i] += hw_states_[1][i]*0.5;
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemEffortOnlyHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RRBotSystemEffortOnlyHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return hardware_interface::return_type::OK;
}

}  // namespace rrbot_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  rrbot_hw::RRBotSystemEffortOnlyHardware, hardware_interface::SystemInterface)
