"""
「昇降まで」自律シーケンス launch。

構成:
  natsu_auto_node       = 司令塔(FSM)。状態遷移を管理し /auto/state を出す。
  natsu_climb_seq_node  = 実行ノード。/auto/state を見て各出入り口へ司令を出し、
                          LiDAR距離/IMU旋回/IMU乗り上げで完了判定 → /auto/phase_done を返す。

前提: 走行/昇降/センサ側は別launchで起動しておくこと。
  - zakiomni(omni_drive) : /cmd_vel を受け /auto/arbitration=auto の時だけ効く
  - unaginobori          : /climb/start を受ける
  - wall_detection       : /wall_detection/distance, /wall_detection/angle を出す
  - imu_publisher / wt901c_publisher : /imu を出す

使い方:
  ros2 launch natsu_auto natsu_climb_seq.launch.py
  ros2 topic pub -1 /auto/start std_msgs/msg/Bool "data: true"   # シーケンス開始
  ros2 topic echo /auto/state          # 状態監視
  ros2 topic echo /auto/phase_done     # フェーズ完了監視
  ros2 topic pub -1 /auto/abort std_msgs/msg/Bool "data: true"   # 緊急中断
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share = get_package_share_directory("natsu_auto")
    fsm_cfg = os.path.join(share, "config", "natsu_auto.yaml")
    seq_cfg = os.path.join(share, "config", "natsu_climb_seq.yaml")

    fsm_node = Node(
        package="natsu_auto",
        executable="natsu_auto_node",
        name="natsu_auto_node",
        output="screen",
        parameters=[fsm_cfg],
    )

    seq_node = Node(
        package="natsu_auto",
        executable="natsu_climb_seq_node",
        name="natsu_climb_seq_node",
        output="screen",
        parameters=[seq_cfg],
    )

    return LaunchDescription([fsm_node, seq_node])
