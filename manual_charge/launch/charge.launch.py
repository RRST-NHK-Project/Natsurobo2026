import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manual_charge',
            executable='charge',
            name='charge_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        )
    ])