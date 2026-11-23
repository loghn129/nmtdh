"""
real.launch.py - MỨC 3: RViz2 chuyển động theo TAY THẬT (ESP32 qua cổng Serial).

    ros2 launch eezybot_bringup real.launch.py                      # cổng /dev/ttyUSB0
    ros2 launch eezybot_bringup real.launch.py port:=/dev/ttyACM0 planner:=co_dinh

Firmware không gửi góc khớp liên tục nên tư thế trong RViz là ƯỚC LƯỢNG (real_arm.py).
Node không có camera: tay thật chỉ né những vật cản khai báo trong RViz (nút Publish Point).
Nhớ TẮT Serial Monitor của Arduino IDE trước: 1 cổng chỉ mở được ở 1 nơi.

Chưa có tay thật thì thử bằng ESP32 ảo:
    ros2 run eezybot_bringup fake_esp32 --link /tmp/eezybot_tty
    ros2 launch eezybot_bringup real.launch.py port:=/tmp/eezybot_tty

Tham số:
    port     cổng Serial của ESP32 (mặc định /dev/ttyUSB0)
    baud     tốc độ (mặc định 115200)
    planner  astar | co_dinh
    rviz     có mở RViz2 hay không
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
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Cong Serial cua ESP32'),
        DeclareLaunchArgument('baud', default_value='115200', description='Toc do Serial'),
        DeclareLaunchArgument('planner', default_value='astar', description='astar | co_dinh'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description, 'launch', 'description.launch.py')),
            launch_arguments={'rviz': LaunchConfiguration('rviz')}.items(),
        ),
        Node(
            package='eezybot_bringup',
            executable='sim_node',
            parameters=[{
                'robot': 'that',
                'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
                'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
                'planner': ParameterValue(LaunchConfiguration('planner'), value_type=str),
            }],
            # RobotLink in từng dòng của ESP32 bằng print(): không đệm để dòng hiện ra ngay
            additional_env={'PYTHONUNBUFFERED': '1'},
            output='screen',
        ),
    ])
