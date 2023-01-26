// Copyright (c) 2022 Alexander Junk
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
//
// Author: Alexander Junk


#include <gmock/gmock.h>

#include <cmath>
#include <string>
#include <unordered_map>

#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "ros2_control_test_assets/components_urdfs.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

class TestODriveSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    hardware_system_odrive =
      R"(
      <ros2_control name="odrive_system" type="system">
        <hardware>
          <plugin>ros2_control_odrive_hw/ODriveSystemHardware</plugin>
          <param name="serial_number">000000</param>
        </hardware>

        <joint name="axis0">
          <command_interface name="velocity"/>
          <state_interface name="velocity"/>
        </joint>
        <joint name="axis1">
          <command_interface name="velocity"/>
          <state_interface name="velocity"/>
      </joint>
      </ros2_control>
    )";

  }

  std::string hardware_system_odrive;
};

TEST_F(TestODriveSystem, load_odrive_system)
{
  auto urdf = ros2_control_test_assets::urdf_head + hardware_system_odrive +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(urdf));
}
