#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=["ros2", "run", "control_as", "autonomous_controller.py"],
            shell=True,
            output="screen",
            cwd="/UnderWaterVehicle/ws/src/control_as/scripts"
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "control_as", "move_to_action_server.py"],
            shell=True,
            output="screen",
            cwd="/UnderWaterVehicle/ws/src/control_as/scripts"
        ),
        ExecuteProcess(
            cmd=["ros2", "run", "control_as", "autonomous_rov_controller.py"],
            shell=True,
            output="screen",
            cwd="/UnderWaterVehicle/ws/src/control_as/scripts"
        )
    ])
