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

#include "ros2_control_odrive_hw/ros2_control_odrive_hw.hpp"

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

CallbackReturn ODriveSystemHardware::on_init(const hardware_interface::HardwareInfo& info)
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

    /*
    
        INITIALIZE ODRIVE DATA HERE!
    
    */

    if(info_.hardware_parameters.find("serial_number") != info_.hardware_parameters.end()){
      odrive_parameter_sn_ = info_.hardware_parameters["serial_number"];
    }else{
      odrive_parameter_sn_ = "";
    }

    if (odrive_parameter_sn_.empty())
    {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemHardware"), "No serial number provided. Can not initialize ODRIVE node!");
      return CallbackReturn::ERROR;
    }else{
      RCLCPP_INFO(rclcpp::get_logger("ODriveSystemHardware"), "Configured for ODrive with SN: %s", odrive_parameter_sn_.c_str());
    }
    

    if(info_.hardware_parameters.find("json_descriptor_path") != info_.hardware_parameters.end()){
      odrive_parameter_cfg_ = info_.hardware_parameters["json_descriptor_path"];
    }

    std::string param;

    if(info_.hardware_parameters.find("read_config") != info_.hardware_parameters.end()){
      param = info_.hardware_parameters["read_config"];
      if(param == "true"){
        odrive_parameter_read_cfg_ = true;
      }else if (param == "false"){
        odrive_parameter_read_cfg_ = false;
      }else{
        RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemHardware"), "Error, %s is no valid value for parameter %s.", param.c_str(), "read_config");
      }
    }

    if(info_.hardware_parameters.find("write_config") != info_.hardware_parameters.end()){
      param = info_.hardware_parameters["write_config"];
      if(param == "true"){
        odrive_parameter_write_cfg = true;
      }else if (param == "false"){
        odrive_parameter_write_cfg = false;
      }else{
        RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemHardware"), "Error, %s is no valid value for parameter %s.", param.c_str(), "write_config");
      }
    }

   // --------------------------

    RCLCPP_INFO(
    rclcpp::get_logger("ODriveSystemHardware"), "DONE INITIALIZING ");

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ODriveSystemHardware::export_state_interfaces()
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
ODriveSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

CallbackReturn ODriveSystemHardware::on_configure( const rclcpp_lifecycle::State & /*previous_state*/)
{
  //Init ODRIVE communication
  odrive_endpoint_ = new odrive_endpoint();

  if (odrive_endpoint_->init(odrive_parameter_sn_, odrive_parameter_read_cfg_, odrive_parameter_write_cfg))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemHardware"), "Could not open USB endpoint!");
    delete odrive_endpoint_;
    odrive_endpoint_ = nullptr;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveSystemHardware::on_activate( const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ODriveSystemHardware"), "ACTIVATING %u",
    control_level_[0]);

  if(odrive_endpoint_ && odrive_endpoint_->getDescriptor()){
    RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemHardware"), "Failed to retreive JSON description from ODRIVE!");
  }else if(!odrive_endpoint_){
    RCLCPP_ERROR(rclcpp::get_logger("ODriveSystemHardware"), "Empty odrive handle!");
  }else{
    RCLCPP_INFO(rclcpp::get_logger("ODriveSystemHardware"), "Got endpoint desciption.");
    // Process configuration file
    float speed = 0;
    cmd_set_vel<0>(odrive_endpoint_, speed);
    cmd_set_vel<1>(odrive_endpoint_, speed);

    //ToDo: Handle configuration via ROS-Parameters
  }
 
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
    rclcpp::get_logger("ODriveSystemHardware"), "System successfully started! %u",
    control_level_[0]);
  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveSystemHardware::on_deactivate( const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ODriveSystemHardware"), "Stopping... please wait...");


  RCLCPP_INFO(rclcpp::get_logger("ODriveSystemHardware"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveSystemHardware::read()
{
    //Read from odrive hardware
 
    uint32_t error_0 = 0, error_1 = 0;
    odrive_endpoint_->readOdriveData(std::string("axis0.error"), error_0);   
    odrive_endpoint_->readOdriveData(std::string("axis1.error"), error_1);
    
    if(error_0 || error_1){
      //return hardware_interface::return_type::ERROR;
    }


    float fval = 0.0;
    if(odrive_endpoint_->readOdriveData(std::string("axis0.encoder.vel_estimate"), fval) == ODRIVE_OK){
      hw_states_velocities_.at(0) = fval;
    }else{
      hw_states_velocities_.at(0) = 0.0;
    }

    if(odrive_endpoint_->readOdriveData(std::string("axis1.encoder.vel_estimate"), fval) == ODRIVE_OK){
      hw_states_velocities_.at(1) = fval;
    }else{
      hw_states_velocities_.at(1) = 0.0;
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ODriveSystemHardware::write()
{
    //WRITE TO ODRIVE HW
    
    cmd_set_vel<0>(this->odrive_endpoint_, hw_commands_velocities_.at(0) );
    cmd_set_vel<1>(this->odrive_endpoint_, hw_commands_velocities_.at(1) );

    for (std::size_t i = 0; i < hw_commands_velocities_.size(); i++)
    {
       //Send velocities to HW
       return hardware_interface::return_type::OK;
    }

  return hardware_interface::return_type::OK;
}


//namespace end
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_odrive_hw::ODriveSystemHardware, hardware_interface::SystemInterface)