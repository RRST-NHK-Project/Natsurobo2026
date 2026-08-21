
#ros2 launch manual_control_natsu26 manual_launch_natsu_2026.launch.py
#ros2 launch manual_control_natsu26 manual_launch_natsu_2026.launch.py use_ir:=false   # IR受信ESP32が無い時
#ros2 launch manual_control_natsu26 manual_launch_natsu_2026.launch.py use_joy:=false  # joy_nodeを別で立てている時
#ros2 launch manual_control_natsu26 manual_launch_natsu_2026.launch.py use_mux:=false   # GUI併用をやめてPS4直結に戻す時


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_joy = LaunchConfiguration("use_joy")
    use_ir = LaunchConfiguration("use_ir")
    use_mux = LaunchConfiguration("use_mux")

    # use_mux:=true のとき、実機PS4は /joy ではなく /joy_ps4 へ出す。
    # /joy を作るのは joy_mux だけ、という状態を保つため(2重publishはエッジ検出を壊す)。
    # use_mux:=false なら従来通り joy_node が直接 /joy を出す。
    joy_raw_topic = PythonExpression(
        ["'joy_ps4' if '", use_mux, "' == 'true' else 'joy'"])

    return LaunchDescription([

        DeclareLaunchArgument("use_joy", default_value="true",
                              description="PS4コントローラ入力(/joy)を出す joy_node を起動"),
        DeclareLaunchArgument("use_ir", default_value="true",
                              description="IR受信の ir_node を起動(/dev/natsurobo_ir が必要)"),
        DeclareLaunchArgument("use_mux", default_value="true",
                              description="実機PS4(/joy_ps4)とGUI(/joy_gui)を合成する joy_mux を起動"),

        # PS4コントローラ -> /joy_ps4 (use_mux:=false なら /joy)。
        # これが無いと操縦系は一切動かない(ros-jazzy-joy が必要)。
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
            parameters=[{
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }],
            remappings=[("joy", joy_raw_topic)],
            condition=IfCondition(use_joy),
        ),

        # /joy_ps4(実機) + /joy_gui(GUI) -> /joy。
        # 足回りはPS4限定(GUI由来の LS/RS/R2 は joy_mux 側で捨てる)。
        # GUI側は「Joy Topic」を joy_gui にしておくこと(既定値もそうしてある)。
        Node(
            package="manual_control_natsu26",
            executable="joy_mux",
            name="joy_mux",
            output="screen",
            condition=IfCondition(use_mux),
        ),

        Node(
            # このexeは HardWareControl と SwitchInput の2ノードを1プロセスで起動する。
            # name= で上書きすると両方が同名になり /mc_2026 が重複するため指定しない
            # (各ノードはコード側の hardware_control_<id> / switch_input_<id> を使う)。
            package="manual_control_natsu26",
            executable="hardware_control_3",
            output="screen"
        ),
        Node(
            package="natsu_drive_v26",
            executable="omni_drive",
            name="zakiomni",
            output="screen"
        ),
        Node(
            package="upper_control_natsu26",
            executable="unaginobori2026",
            name="t101mm",
            output="screen"
        ),
        Node(
            package='manual_charge',
            executable='charge',
            name='charge_node',
            output='screen',
            # md_number: 装填モーターを繋いだMDの番号(1〜4)。ID4基板は data[1]〜data[4] が
            # MD1〜MD4 にそのまま対応する。現物はMD1が壊れているのでMD2を使っている。
            # 配線を変えたらここだけ直す(ファームの書き換えは不要)。
            parameters=[{'md_number': 2}]
        ),

        Node(
            package="natsu_ir",
            executable="ir_node",
            name="ir_node",
            output="screen",
            condition=IfCondition(use_ir),
        ),

        Node(
            package="natsu_ir",
            executable="ir_led_policy",
            name="ir_led_policy",
            output="screen"
        ),

    ])
