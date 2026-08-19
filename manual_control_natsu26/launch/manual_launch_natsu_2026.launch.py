from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      
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
        
    ])