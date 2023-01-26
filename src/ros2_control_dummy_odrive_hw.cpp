/*
* Copyright (C) 2021 Alexander Junk <dev@junk.technology>
* 
* This program is free software: you can redistribute it and/or modify it 
* under the terms of the GNU Lesser General Public License as published 
* by the Free Software Foundation, either version 3 of the License, or 
* (at your option) any later version.
* 
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; 
* without even the implied warranty of MERCHANTABILITY or FITNESS
* FOR A PARTICULAR PURPOSE. 
* See the GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License 
* along with this program. If not, see <https://www.gnu.org/licenses/>. 
*
*/

#include "ros2_control_odrive_hw/ros2_control_dummy_odrive_hw.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_odrive_hw
{

CallbackReturn DummyODriveSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
{
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    hw_states_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_states_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    control_level_.resize(info_.joints.size());



    for (const hardware_interface::ComponentInfo& joint : info_.joints)
    {

        if (joint.command_interfaces.size() < 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("ODriveSystemHardware"),
                "Joint '%s' has %zu command interfaces. 1 expected.", joint.name.c_str(),
                  joint.command_interfaces.size());

            return CallbackReturn::ERROR;
        }

        
        if (joint.state_interfaces.size() < 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("ODriveSystemHardware"),
                "!Joint '%s'has %zu state interfaces. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());

            return CallbackReturn::ERROR;
        }

        if ( !(joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY) )
        {
            RCLCPP_FATAL(
                rclcpp::get_logger("ODriveSystemHardware"),
                "Joint '%s' has %s state interface. Expected %s.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);

            return CallbackReturn::ERROR;
        }

    }

    RCLCPP_INFO(
    rclcpp::get_logger("DummyODriveSystemHardware"), "DONE INITIALIZING ");

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DummyODriveSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));

  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DummyODriveSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

CallbackReturn DummyODriveSystemHardware::on_configure( const rclcpp_lifecycle::State & /*previous_state*/)
{

  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyODriveSystemHardware::on_activate( const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("DummyODriveSystemHardware"), "ACTIVATING %u",
    control_level_[0]);
 
  // Set some default values
  for (std::size_t i = 0; i < hw_states_velocities_.size(); i++)
  {
    if (std::isnan(hw_states_velocities_[i]))
    {
      hw_states_velocities_[i] = 0;
    }
    if (std::isnan(hw_commands_velocities_[i]))
    {
      hw_commands_velocities_[i] = 0;
    }
    if (std::isnan(hw_states_positions_[i]))
    {
      hw_states_positions_[i] = 0;
    }
    
    control_level_[i] = integration_level_t::VELOCITY;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("DummyODriveSystemHardware"), "System successfully started! %u",
    control_level_[0]);

  return CallbackReturn::SUCCESS;
}

CallbackReturn DummyODriveSystemHardware::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("DummyODriveSystemHardware"), "Stopping... please wait...");


  RCLCPP_INFO(rclcpp::get_logger("DummyODriveSystemHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type DummyODriveSystemHardware::read()
{    
    hw_states_velocities_.at(0) = hw_commands_velocities_.at(0);
    hw_states_velocities_.at(1) = hw_commands_velocities_.at(1);

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type DummyODriveSystemHardware::write()
{
  return hardware_interface::return_type::OK;
}


//namespace end
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_odrive_hw::DummyODriveSystemHardware, hardware_interface::SystemInterface)