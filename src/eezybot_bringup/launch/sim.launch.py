"""
sim.launch.py - MỨC 2: mô phỏng chu trình gắp - thả + né vật cản trong RViz2 (không cần phần cứng).

    ros2 launch eezybot_bringup sim.launch.py
    ros2 launch eezybot_bringup sim.launch.py speed:=3.0 planner:=co_dinh auto_drop:=false rviz:=false

Tham số:
    speed      tua nhanh chuyển động (1.0 = đúng thời gian của firmware)
    planner    astar | co_dinh
    auto_drop  true: tự thả sản phẩm lần lượt khay 1 -> 2 -> 3; false: chờ lệnh /eezybot/drop
    rviz       có mở RViz2 hay không
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory('eezybot_description')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true', description='Mo RViz2'),
        DeclareLaunchArgument('speed', default_value='1.0', description='Tua nhanh chuyen dong'),
        DeclareLaunchArgument('planner', default_value='astar', description='astar | co_dinh'),
        DeclareLaunchArgument('auto_drop', default_value='true', description='Tu tha san pham khay 1 -> 2 -> 3'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),
        Node(
            package='eezybot_bringup',
            executable='sim_node',
            # ParameterValue ép kiểu: "speed:=3" (số nguyên) vẫn thành số thực như node khai báo
            parameters=[{
                'speed': ParameterValue(LaunchConfiguration('speed'), value_type=float),
                'planner': ParameterValue(LaunchConfiguration('planner'), value_type=str),
                'auto_drop': ParameterValue(LaunchConfiguration('auto_drop'), value_type=bool),
            }],
            output='screen',
        ),
    ])
