"""
sliders.launch.py - MỨC 1: xem URDF trong RViz2 và kéo thanh trượt theo góc servo.

    ros2 launch eezybot_bringup sliders.launch.py

Sơ đồ:
    joint_state_publisher_gui (4 thanh trượt, đơn vị độ)
        -> /servo_states -> slider_node (đổi sang khớp URDF) -> /joint_states
        -> robot_state_publisher -> TF -> RViz2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory('eezybot_bringup')
    description = get_package_share_directory('eezybot_description')
    slider_urdf = os.path.join(bringup, 'config', 'servo_sliders.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Mo RViz2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='servo_sliders',
            arguments=[slider_urdf],          # Đọc file "robot giả" 4 khớp = 4 góc servo
            parameters=[{
                'rate': 30,
                # Vị trí ban đầu của thanh trượt = HOME (90, 60, 40), kẹp mở (40)
                'zeros.q1_de': 90.0,
                'zeros.q2_vai': 60.0,
                'zeros.q3_khuyu': 40.0,
                'zeros.kep': 40.0,
            }],
            remappings=[
                ('joint_states', 'servo_states'),              # Không phát thẳng vào /joint_states
                ('robot_description', 'servo_description'),    # Không để URDF thật ghi đè 4 thanh trượt
            ],
            output='screen',
        ),
        Node(
            package='eezybot_bringup',
            executable='slider_node',
            output='screen',
        ),
    ])
