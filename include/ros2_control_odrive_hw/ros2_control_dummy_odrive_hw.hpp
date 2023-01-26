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

#ifndef ROS2_CONTROL_ODRIVE_HW_INCLUDED
#define ROS2_CONTROL_ODRIVE_HW_INCLUDED

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ros2_odrive/odrive_endpoint.hpp"
#include "ros2_odrive/odrive_utils.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ros2_control_odrive_hw
{


class DummyODriveSystemHardware: public hardware_interface::SystemInterface
{
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(DummyODriveSystemHardware)

        CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
        

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read() override;

        hardware_interface::return_type write() override;

    private:
        // Parameters for the  simulation
        double hw_start_sec_;
        double hw_stop_sec_;

        // Store the command for the simulated robot
        std::vector<double> hw_commands_velocities_;
        std::vector<double> hw_states_velocities_;

        std::vector<double> hw_states_positions_;


        // Enum defining at which control level we are
        // Dumb way of maintaining the command_interface type per joint.
        enum integration_level_t : std::uint8_t
        {
            UNDEFINED = 0,
            POSITION = 1,
            VELOCITY = 2,
            ACCELERATION = 3
        };

        // Active control mode for each actuator
        std::vector<integration_level_t> control_level_;

        //ODrive hardware endpoint and associated variables
        odrive_endpoint* odrive_endpoint_{nullptr};
        Json::Value odrive_json_;
        std::string odrive_cfg_;

        std::string odrive_parameter_sn_;
        std::string odrive_parameter_cfg_;
        bool odrive_parameter_read_cfg_{false};
        bool odrive_parameter_write_cfg{false};



};






}

#endif