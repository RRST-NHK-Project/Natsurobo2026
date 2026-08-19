
#ros2 launch manual_control_natsu26 manual_launch_natsu_2026.launch.py
#ros2 launch manual_control_natsu26 manual_launch_natsu_2026.launch.py use_ir:=false   # IR受信ESP32が無い時
#ros2 launch manual_control_natsu26 manual_launch_natsu_2026.launch.py use_joy:=false  # joy_nodeを別で立てている時


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_joy = LaunchConfiguration("use_joy")
    use_ir = LaunchConfiguration("use_ir")

    return LaunchDescription([

        DeclareLaunchArgument("use_joy", default_value="true",
                              description="PS4コントローラ入力(/joy)を出す joy_node を起動"),
        DeclareLaunchArgument("use_ir", default_value="true",
                              description="IR受信の ir_node を起動(/dev/natsurobo_ir が必要)"),

        # PS4コントローラ -> /joy。これが無いと操縦系は一切動かない(ros-jazzy-joy が必要)。
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "deadzone": 0.05,
                "autorepeat_rate": 0.0,
            }],
            condition=IfCondition(use_joy),
        ),

        Node(
            # このexeは HardWareControl と SwitchInput の2ノードを1プロセスで起動する。
            # name= で上書きすると両方が同名になり /mc_2026 が重複するため指定しない
            # (各ノードはコード側の hardware_control_<id> / switch_input_<id> を使う)。
            package="manual_control_natsu26",
            executable="hardware_control_3",
            output="screen"
        ),
        Node(
            package="natsu_drive_v26",
            executable="omni_drive",
            name="zakiomni",
            output="screen"
        ),
        Node(
            package="upper_control_natsu26",
            executable="unaginobori2026",
            name="t101mm",
            output="screen"
        ),
        Node(
            package='manual_charge',
            executable='charge',
            name='charge_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}]
        ),

        Node(
            package="natsu_ir",
            executable="ir_node",
            name="ir_node",
            output="screen",
            condition=IfCondition(use_ir),
        ),

        Node(
            package="natsu_ir",
            executable="ir_led_policy",
            name="ir_led_policy",
            output="screen"
        ),

    ])
