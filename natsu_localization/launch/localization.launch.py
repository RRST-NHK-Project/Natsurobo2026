"""
natsu_localization — scan-to-known-map ICP 自己位置推定の起動ファイル

  起動するもの:
    - scan_matcher_localizer : /scan + odom を既知壁マップにマッチングし
                               /localization/pose と TF(map->odom) を出力
    - static_transform_publisher : base_link -> ldlidar_link (LiDAR取付)
                                   ※実機の取付値に合わせて x/y/yaw を更新すること

  前提: summer2026_odometry(/odom) と LD19(/scan) が別途起動済み。
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('natsu_localization'), 'config', 'field_map.yaml')

    return LaunchDescription([
        # 自己位置推定ノード
        Node(
            package='natsu_localization',
            executable='scan_matcher_localizer',
            name='scan_matcher_localizer',
            output='screen',
            parameters=[cfg],
        ),

        # LiDAR 取付 TF (base_link -> ldlidar_link)。実機の固定値に合わせる。
        # 引数: --x --y --z --yaw --pitch --roll --frame-id --child-frame-id
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_ldlidar',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0',
                       '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0',
                       '--frame-id', 'base_link', '--child-frame-id', 'ldlidar_link'],
        ),
    ])
