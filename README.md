# ros2_control_odrive_hw

ROS2_control hardware abstraction for ODrive motor driver boards.

## Overview
This repository provides a ros2_control hardware component for working with ODrive driver boards.

It uses the ros2_odrive driver for communication and configuration.

One instance is bound to one driver board.

A dummy implementation is provided for use in simulation/without physical hardware.

## ROS2_control Hardware Parameters

| Parameter |  Type  | Description |
|:-----|:--------:|:---|
| `serial_number` | hex | Serial number of the ODrive board to connect to in hexadecimal notation. No leading '0x'. |
| `json_descriptor_path` | string | File path in which to read/write the API-descriptor used by/for the ODrive. |
| `read_config` | bool | Set to true if API descriptor should be read from file instead of board. |
| `write_config` | bool | Set to true if API descriptor should be written to file. |

