from launch import LaunchDescription
from launch_ros.actions import Node

# 2台の ODrive で 3モータを起動する。
#   odrv_a (336C355A3033): axis0 + axis1  ← 2モータ
#   odrv_b (345F36533334): axis0 のみ      ← 1モータ
BOARD_A_SERIAL = "336C355A3033"          # 確認済み
BOARD_B_SERIAL = "345F36533334"          # 確認済み

# 3軸とも同じモータ・エンコーダなので共通設定。odrive_single.launch.py と同じ値。
COMMON = {
    "vel_limit": 200.0,        # 150 turn/s を出すため余裕をみて上げる
    "current_limit": 20.0,
    "input_mode": "passthrough",  # 押した瞬間に切り替わる（キレる）
    "vel_ramp_rate": 200.0,       # input_mode="ramp" のときの傾き
    "vel_gain": 0.0,              # 0.0 ならボード保存値を使う（上書きしない）
    "vel_integrator_gain": 0.0,
    "cmd_timeout": 0.5,
    "publish_rate": 20.0,
}


def make(ns, serial, use_axis0, use_axis1):
    params = dict(COMMON)
    params["serial_number"] = serial
    params["use_axis0"] = use_axis0
    params["use_axis1"] = use_axis1
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
        # board A: axis0 と axis1 の両方を使う
        make("odrv_a", BOARD_A_SERIAL, use_axis0=True, use_axis1=True),
        # board B: axis0 のみ使う
        make("odrv_b", BOARD_B_SERIAL, use_axis0=True, use_axis1=False),
    ])