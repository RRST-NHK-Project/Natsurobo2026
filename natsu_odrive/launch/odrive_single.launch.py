from launch import LaunchDescription
from launch_ros.actions import Node

# 1台だけ (board A) を起動する。今回はモータ1つなので axis0 のみ使う。
BOARD_A_SERIAL = "336C355A3033"


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="natsu_odrive",
            executable="odrive_node",
            name="odrive_node",
            namespace="odrv_a",
            output="screen",
            parameters=[{
                "serial_number": BOARD_A_SERIAL,
                "vel_limit": 220.0,
                "current_limit": 20.0,
                "input_mode": "passthrough",
                "vel_ramp_rate": 200.0,
                "vel_gain": 0.0,
                "vel_integrator_gain": 0.0,
                "cmd_timeout": 0.5,
                "publish_rate": 20.0,
                "use_axis0": True,
                "use_axis1": False,
            }],
        ),
    ])
