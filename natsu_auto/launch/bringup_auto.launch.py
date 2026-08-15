"""自動走行 一括起動 (bringup)

起動順(安全側): serial_bridge → 知覚(lidar+IMU+wall) → omni_drive → natsu_auto(頭脳)
頭脳は足回り・センサが立ってから遅延起動する。

  ros2 launch natsu_auto bringup_auto.launch.py
  ros2 launch natsu_auto bringup_auto.launch.py use_mechanisms:=true use_rviz:=true
  ros2 launch natsu_auto bringup_auto.launch.py use_perception:=false   # ブラインド駆動確認

  # 起動後、開始/中断は手動で:
  ros2 topic pub -1 /auto/start std_msgs/msg/Bool "data: true"
  ros2 topic pub -1 /auto/abort std_msgs/msg/Bool "data: true"

注意:
- serial_bridge は別ワークスペース(/home/ubuntu/serial_can_bridge)。source 済みであること。
- 安全のため /auto/start は自動発行しない(このlaunchは待機状態で止まる)。
"""

import os

from ament_index_python.packages import (PackageNotFoundError,
                                          get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _include(pkg, rel, *, condition, launch_arguments=None):
    """パッケージが見つからなければ include せず警告だけ出す(部分bringup対応)。"""
    try:
        path = os.path.join(get_package_share_directory(pkg), *rel)
    except PackageNotFoundError:
        return LogInfo(msg=f"[bringup_auto] package '{pkg}' 未検出 -> スキップ"
                           f"(該当ワークスペースを source 済みか確認)")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=(launch_arguments or {}).items() or None,
        condition=condition,
    )


def generate_launch_description():
    use_hardware = LaunchConfiguration("use_hardware")
    use_perception = LaunchConfiguration("use_perception")
    use_mechanisms = LaunchConfiguration("use_mechanisms")
    use_rviz = LaunchConfiguration("use_rviz")
    lidar_port = LaunchConfiguration("lidar_port")
    imu_port = LaunchConfiguration("imu_port")
    start_auto_delay = LaunchConfiguration("start_auto_delay")

    args = [
        DeclareLaunchArgument("use_hardware", default_value="true",
                              description="serial_bridge(モータ基板橋渡し)を起動"),
        DeclareLaunchArgument("use_perception", default_value="true",
                              description="lidar+IMU+wall_detection を起動(sensor_test)"),
        DeclareLaunchArgument("use_mechanisms", default_value="false",
                              description="登り/回収/射出/オドメトリの機構ノードを起動"),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="RViz2 を出す(実機はfalse推奨)"),
        DeclareLaunchArgument("lidar_port", default_value="/dev/ldlidar"),
        DeclareLaunchArgument("imu_port", default_value="/dev/wt901"),
        DeclareLaunchArgument("start_auto_delay", default_value="4.0",
                              description="頭脳(natsu_auto)を遅延起動する秒数"),
    ]

    auto_cfg = os.path.join(
        get_package_share_directory("natsu_auto"), "config", "natsu_auto.yaml")

    # ── ① ハード橋渡し(別ws) ─────────────────────────────
    serial_bridge = _include(
        "serial_bridge", ("launch", "serial_bridge.launch.py"),
        condition=IfCondition(use_hardware))

    # ── ② 知覚: lidar + WT901C IMU + wall_detection + TF ──
    perception = _include(
        "sensor_viz", ("launch", "sensor_test.launch.py"),
        condition=IfCondition(use_perception),
        launch_arguments={
            "lidar_port": lidar_port,
            "imu_port": imu_port,
            "use_imu": "true",
            "use_rviz": use_rviz,
        })

    # ── ③ 駆動: /cmd_vel → serial_tx ─────────────────────
    omni_drive = Node(
        package="natsu_drive_v26", executable="omni_drive",
        name="omni_drive", output="screen",
    )

    # ── ④ 機構(任意) ────────────────────────────────────
    mechanisms = [
        Node(package="upper_control_natsu26", executable="unaginobori2026",
             name="upper_control", output="screen",
             condition=IfCondition(use_mechanisms)),
        Node(package="manual_control_natsu26", executable="hardware_control_3",
             name="collect_control", output="screen",
             condition=IfCondition(use_mechanisms)),
        Node(package="shooting_control", executable="shooting_control_node",
             name="shooting_control", output="screen",
             condition=IfCondition(use_mechanisms)),
        Node(package="natsu_metry26", executable="guess_position_26",
             name="odometry", output="screen",
             condition=IfCondition(use_mechanisms)),
    ]

    # ── ⑤ 頭脳: 足回り・センサが立ってから遅延起動 ────────
    brain = TimerAction(
        period=PythonExpression([start_auto_delay]),
        actions=[Node(
            package="natsu_auto", executable="natsu_auto_node",
            name="natsu_auto_node", output="screen",
            parameters=[auto_cfg],
        )],
    )

    return LaunchDescription(
        args + [serial_bridge, perception, omni_drive, *mechanisms, brain])
