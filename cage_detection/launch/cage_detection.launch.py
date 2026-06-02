"""
cage_detection.launch.py

D456ドライバ + cage_detection_node を一括起動する。

前提:
    sudo apt install ros-humble-realsense2-camera

使い方:
    ros2 launch cage_detection cage_detection.launch.py

検出ON/OFF:
    ros2 topic pub /cage_detection/enable std_msgs/msg/Bool "data: true"
    ros2 topic pub /cage_detection/enable std_msgs/msg/Bool "data: false"

デバッグ画像確認:
    ros2 run rqt_image_view rqt_image_view /cage_detection/debug_image

カゴ位置確認:
    ros2 topic echo /cage_detection/target
    ros2 topic echo /cage_detection/cages
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("cage_detection"),
        "config",
        "cage_detection.yaml",
    )

    # ── D456 ドライバ ─────────────────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "depth_module.profile":  "640x480x30",
            "rgb_camera.profile":    "640x480x30",
            "align_depth.enable":    "true",   # RGB-深度アライメント
            "pointcloud.enable":     "false",  # 点群は不要（軽量化）
            "enable_gyro":           "false",
            "enable_accel":          "false",
        }.items(),
    )

    # ── cage_detection_node ───────────────────────────────
    cage_node = Node(
        package="cage_detection",
        executable="cage_detection_node",
        name="cage_detection_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([
        realsense_launch,
        cage_node,
    ])
