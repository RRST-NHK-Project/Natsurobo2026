from launch import LaunchDescription
from launch_ros.actions import Node

# 2台の ODrive (= 4モータ) を一括起動する。
# ★ serial_number は odrivetool や configure_odrive.py の出力で確認して置き換える。
BOARD_A_SERIAL = "336C355A3033"          # 確認済み
BOARD_B_SERIAL = "REPLACE_WITH_SERIAL_B"  # ★2台目が分かったら書き換える

COMMON = {
    "vel_limit": 20.0,
    "current_limit": 20.0,
    "vel_ramp_rate": 10.0,
    "cmd_timeout": 0.5,
    "publish_rate": 20.0,
    "use_axis0": True,
    "use_axis1": True,
}


def make(ns, serial):
    params = dict(COMMON)
    params["serial_number"] = serial
    return Node(
        package="natsu_odrive",
        executable="odrive_node",
        name="odrive_node",
        namespace=ns,
        output="screen",
        parameters=[params],
    )


def generate_launch_description():
    return LaunchDescription([
        make("odrv_a", BOARD_A_SERIAL),
        make("odrv_b", BOARD_B_SERIAL),
    ])
