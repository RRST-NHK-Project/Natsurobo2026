"""
wall_detection_only.launch.py

LiDARドライバは別途起動済みの前提で、wall_detectionノードだけ起動する。

使い方:
    ros2 launch wall_detection wall_detection_only.launch.py

照準モードの切り替え(ランタイム):
    ros2 topic pub /wall_detection/aiming_mode std_msgs/msg/Bool "data: true"
    ros2 topic pub /wall_detection/aiming_mode std_msgs/msg/Bool "data: false"

キャリブレーション確認:
    ros2 topic echo /wall_detection/angle
    ros2 topic echo /wall_detection/distance
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # パラメータは src/wall_detection_node.cpp の declare_parameter デフォルトに
    # 一元化した。調整はソース側を編集して colcon build すること(yamlは廃止)。
    wall_detection_node = Node(
        package="wall_detection",
        executable="wall_detection_node",
        name="wall_detection_node",
        output="screen",
        # LiDARトピック名がドライバによって違う場合はここでremapする
        # 例: STL-19Pのドライバが /stl19p/scan を出す場合
        # remappings=[("/scan", "/stl19p/scan")],
        remappings=[("/scan", "/ldlidar_node/scan")],
    )

    return LaunchDescription([wall_detection_node])
