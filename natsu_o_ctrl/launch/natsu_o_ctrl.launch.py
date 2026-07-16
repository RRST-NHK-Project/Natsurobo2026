from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # PS4コントローラの入力を /joy に流す（ros-jazzy-joy が必要）
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "deadzone": 0.05,
                "autorepeat_rate": 0.0,
            }],
        ),

        # □ボタン -> ODrive axis0 速度指令
        Node(
            package="natsu_o_ctrl",
            executable="natsu_o_ctrl",
            name="natsu_o_ctrl",
            output="screen",
        ),
    ])
