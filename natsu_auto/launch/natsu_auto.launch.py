"""
    # シーケンス開始
    ros2 topic pub -1 /auto/start std_msgs/msg/Bool "data: true"
    # 手動回収完了(GUI相当)
    ros2 topic pub -1 /auto/collect_done std_msgs/msg/Bool "data: true"
    # 緊急中断
    ros2 topic pub -1 /auto/abort std_msgs/msg/Bool "data: true"

状態監視:
    ros2 topic echo /auto/state
    ros2 topic echo /auto/arbitration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("natsu_auto"),
        "config", "natsu_auto.yaml")

    auto_node = Node(
        package="natsu_auto",
        executable="natsu_auto_node",
        name="natsu_auto_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([auto_node])
