# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():


    ugv_commander_node = Node(
        package='ugv_commander', executable='striker3_manager.py', name='striker3_manager', output='screen'
        )

    striker_lyve_reporter_node = Node(
            package="ugv_commander",
            executable="striker_lyve_reporter.py",
            output="both"
    )  

    # single_delivery_task_generator
    striker3_single_delivery_task_generator_node = Node(
            package="ugv_commander",
            executable="striker3_single_delivery_task_generator.py",
            output="both"
    )  

    # as_idle
    as_idle_node = Node(
            package="ugv_commander",
            executable="as_idle.py",
            output="both"
    ) 

    # as_pickup
    as_pickup_node = Node(
            package="ugv_commander",
            executable="as_pickup.py",
            output="both"
    )

    # as_dropoff
    as_dropoff_node = Node(
            package="ugv_commander",
            executable="as_dropoff.py",
            output="both"
    ) 

    # as_cross_road
    as_cross_road_node = Node(
            package="ugv_commander",
            executable="as_cross_road.py",
            output="both"
    ) 

    # Create the launch description and populate
    ld = LaunchDescription()

    #striker3 utils
    ld.add_action(striker3_single_delivery_task_generator_node)
    ld.add_action(as_idle_node)
    ld.add_action(as_pickup_node)
    ld.add_action(as_dropoff_node)
    ld.add_action(as_cross_road_node)
    ld.add_action(ugv_commander_node)
    ld.add_action(striker_lyve_reporter_node)
    return ld
