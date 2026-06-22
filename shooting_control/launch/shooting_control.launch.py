"""
shooting_control.launch.py

wall_detection・cage_detection が既に起動している前提で
shooting_control_node だけ起動する。

発射テスト（FSMなしで単体テスト）:
    # 発射許可を手動送信
    ros2 topic pub -1 /shooter/fire_request std_msgs/msg/Bool "data: true"

    # コマンド確認
    ros2 topic echo /shooter/command

    # 準備完了確認
    ros2 topic echo /shooter/ready

キャリブレーション手順:
    1. 固定距離（1m, 2m, 3m）でテスト射出
    2. ros2 topic echo /shooter/command でelev/rpmを記録
    3. 実際の着弾を確認してrpm_per_v0を調整
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("shooting_control"),
        "config",
        "shooting_control.yaml",
    )

    shooting_node = Node(
        package="shooting_control",
        executable="shooting_control_node",
        name="shooting_control_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([shooting_node])
