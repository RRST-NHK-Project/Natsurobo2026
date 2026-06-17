"""
sensor_test.launch.py

足回りセンサ調整用の一括起動スクリプト。

起動するノード:
  - ldlidar_component  (LD19 LiDAR, composable container)
  - wt901_node         (WT901C IMU)
  - wall_detection_node
  - sensor_visualizer
  - RViz2

センサの取り付けオフセットは static TF の引数で調整すること。
現在値はすべて (0, 0, 0) の仮置き。
  base_link → ldlidar_link : --x --y --z --yaw --pitch --roll
  base_link → imu_link     : 同上
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_sensor_viz = get_package_share_directory('sensor_viz')
    pkg_wall       = get_package_share_directory('wall_detection')

    rviz_cfg = os.path.join(pkg_sensor_viz, 'rviz', 'sensor_test.rviz')
    wall_cfg = os.path.join(pkg_wall, 'config', 'wall_detection.yaml')

    imu_port = LaunchConfiguration('imu_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_port', default_value='/dev/ttyUSB1',
            description='WT901C のシリアルポート'),
        DeclareLaunchArgument(
            'use_imu', default_value='false',
            description='WT901C IMU を起動するか'),

        # ── LiDAR（LD19）─ ldlidar_component をコンテナで直接起動 ────
        ComposableNodeContainer(
            name='ldlidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_isolated',
            composable_node_descriptions=[
                ComposableNode(
                    package='ldlidar_component',
                    plugin='ldlidar::LdLidarComponent',
                    name='ld19_lidar',
                    parameters=[{
                        'general.debug_mode': False,
                        'comm.serial_port': '/dev/ldlidar',
                        'comm.baudrate': 230400,
                        'comm.timeout_msec': 1000,
                        'lidar.model': 'LD19',
                        'lidar.rot_verse': 'CCW',
                        'lidar.units': 'M',
                        'lidar.frame_id': 'ldlidar_link',
                        'lidar.bins': 455,
                        'lidar.range_min': 0.03,
                        'lidar.range_max': 15.0,
                        'lidar.enable_angle_crop': False,
                    }],
                    remappings=[('~/scan', '/scan')],
                ),
            ],
            output='screen',
        ),

        # LiDAR lifecycle manager: コンテナのロード後 (3s) に起動して configure → activate を行う
        TimerAction(period=3.0, actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lidar_lifecycle_manager',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['ld19_lidar'],
                    'bond_timeout': 5.0,
                    'attempt_respawn_reconnection': True,
                    'bond_respawn_max_duration': 10.0,
                }],
            ),
        ]),

        # ── IMU（WT901C） ───────────────────────────────────
        Node(
            package='wt901_node',
            executable='wt901_node',
            name='wt901_node',
            output='screen',
            parameters=[{'port': imu_port}],
            condition=IfCondition(LaunchConfiguration('use_imu')),
        ),

        # ── 静的 TF ─────────────────────────────────────────
        # LiDAR 取り付け位置（計測後に数値を更新すること）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_lidar',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.10',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'ldlidar_link',
            ],
        ),
        # IMU 取り付け位置
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_imu',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.05',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'imu_link',
            ],
        ),
        # odom → base_link の仮 TF（summer2026_odometry 未起動時のフォールバック）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--yaw', '0', '--pitch', '0', '--roll', '0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link',
            ],
        ),

        # ── wall_detection ──────────────────────────────────
        Node(
            package='wall_detection',
            executable='wall_detection_node',
            name='wall_detection_node',
            output='screen',
            parameters=[wall_cfg],
        ),

        # ── sensor_visualizer ───────────────────────────────
        Node(
            package='sensor_viz',
            executable='sensor_visualizer',
            name='sensor_visualizer',
            output='screen',
        ),

        # ── RViz2 ───────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_cfg],
            output='screen',
        ),
    ])
