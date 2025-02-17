#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node  # Import Node for bridging
import os

def generate_launch_description():
    # Set environment variables for Gazebo
    os.environ['GZ_SIM_RESOURCE_PATH'] = '/UnderWaterVehicle/ws/src/bluerov2_gz/models:/UnderWaterVehicle/ws/src/bluerov2_gz/worlds'
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = '/home/bluerov2/ardupilot_gazebo/build'

    # Gazebo execution command
    gazebo_process = ExecuteProcess(
        cmd=[
            'gz', 'sim', '-v', '3', '-r',
            '/UnderWaterVehicle/ws/src/bluerov2_gz/worlds/bluerov2_heavy_underwater.world'
        ],
        output='screen'
    )

    # ROS-Gazebo Bridge Node Now inside the function
    bridge_node = Node(
        package="ros_gz_bridge",  #  Correct package for Gazebo Fortress+
        executable="parameter_bridge",
        parameters=[{'use_sim_time': True, 'expand_gz_topic_names': True}],
        arguments=[
            #"/FL_depth_camera_sensor@sensor_msgs/msg/Image@gz.msgs.Image",
            #"/FR_depth_camera_sensor@sensor_msgs/msg/Image@gz.msgs.Image",
            #"/FR_depth_camera_rgb/rgb@sensor_msgs/msg/Image@gz.msgs.Image",
            #"/FL_depth_camera_rgb/rgb@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rgb_camera@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        output="screen",
    )

    # Return both processes inside LaunchDescription
    return LaunchDescription([
        gazebo_process,
        bridge_node
    ])

