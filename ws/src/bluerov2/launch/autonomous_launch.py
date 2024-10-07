from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Bluerov2',
            executable='arm_and_move_client',
            name='arm_and_move_client',
            output='screen'
        )
    ])

