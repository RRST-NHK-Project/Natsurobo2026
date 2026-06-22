"""
cage_detection_viz.launch.py

cage_detection_node + cage_visualizer + RViz2 を一括起動する。

使い方:
    # カメラなし（デスクで動作確認）
    ros2 launch cage_detection cage_detection_viz.launch.py

    # カメラあり
    ros2 launch cage_detection cage_detection_viz.launch.py with_camera:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('cage_detection')
    config   = os.path.join(pkg, 'config', 'cage_detection.yaml')
    rviz_cfg = os.path.join(pkg, 'rviz', 'cage_detection.rviz')

    with_camera = LaunchConfiguration('with_camera')

    return LaunchDescription([
        DeclareLaunchArgument(
            'with_camera', default_value='false',
            description='true でリアルセンスドライバも起動する'),

        # ── RealSense (オプション) ──────────────────────────
        # Jazzy 版の realsense2_camera はパラメータ名が変わっている
        #   depth_module.profile  → depth_module.depth_profile / depth_module.color_profile
        #   rgb_camera.profile    → rgb_camera.color_profile
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch', 'rs_launch.py')),
            launch_arguments={
                'depth_module.depth_profile': '640x480x30',
                'depth_module.color_profile': '640x480x30',
                'rgb_camera.color_profile':   '640x480x30',
                'align_depth.enable':         'true',
                'pointcloud.enable':          'false',
                'enable_gyro':                'false',
                'enable_accel':               'false',
            }.items(),
            condition=IfCondition(with_camera),
        ),

        # ── カメラなし時: static TF を仮置きして RViz2 がフレームを解決できるようにする
        # camera_color_optical_frame → world (前方=Z, 右=X, 下=Y のまま)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_tf_stub',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'camera_color_optical_frame'],
            condition=UnlessCondition(with_camera),
        ),

        # ── cage_detection_node ─────────────────────────────
        Node(
            package='cage_detection',
            executable='cage_detection_node',
            name='cage_detection_node',
            output='screen',
            parameters=[config],
        ),

        # ── cage_visualizer ─────────────────────────────────
        Node(
            package='cage_detection',
            executable='cage_visualizer',
            name='cage_visualizer',
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
