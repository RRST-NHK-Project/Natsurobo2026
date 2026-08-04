from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
      
        Node(
            package="manual_control_natsu26",
            executable="hardware_control_2",
            name="mc_2026",
            output="screen"
        ),
        Node(
            package="natsu_drive_v26",
            executable="omni_drive",
            name="zakiomni",
            output="screen"
        ),
        Node(
            package="natsu_metry26",
            executable="guess_position_26",
            name="natsu2026_odometry",
            output="screen"
        ),  
        Node(
            package="upper_control_natsu26",
            executable="unaginobori2026",
            name="t101mm",
            output="screen"
        ),
        
    ])