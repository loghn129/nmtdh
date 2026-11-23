"""
description.launch.py - nạp URDF và (tùy chọn) mở RViz2.

Chỉ chạy robot_state_publisher: nó đọc /joint_states rồi phát TF của các khâu.
Ai phát /joint_states thì do file launch khác quyết định (thanh trượt hoặc node mô phỏng),
nên file này thường được include từ eezybot_bringup, không chạy một mình.

Tham số:
  rviz:=true|false   có mở RViz2 hay không (mặc định true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('eezybot_description')
    xacro_file = PathJoinSubstitution([share, 'urdf', 'eezybot.urdf.xacro'])
    rviz_file = PathJoinSubstitution([share, 'rviz', 'eezybot.rviz'])

    # xacro -> chuỗi URDF. ParameterValue(..., value_type=str) để launch không cố đọc nó như YAML.
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Mo RViz2'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            condition=IfCondition(LaunchConfiguration('rviz')),
            on_exit=Shutdown(),     # Đóng cửa sổ RViz -> dừng luôn các node còn lại
            output='log',
        ),
    ])
