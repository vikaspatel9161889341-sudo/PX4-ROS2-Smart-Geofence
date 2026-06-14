import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_failsafe',
            executable='smart_nav_node',
            name='smart_nav_failsafe_node',
            output='screen',
            emulate_tty=True,
            # 🎯 Overriding C++ variables via Launch Parameters!
            parameters=[{
                'power_consumption_constant': 0.009, # Pm (testing default)
                'safety_margin': 0.15,               # S (15%)
                'target_x': 15.0,                    # Badalkar long range kiya!
                'target_y': 15.0,
                'target_z': -5.0                     # Height 5 meters!
            }]
        )
    ])
