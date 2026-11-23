"""
core/ - BẢN COPY NGUYÊN VẸN các file Python lõi của dự án (newcode/), KHÔNG SỬA NỘI DUNG:
    robot_model.py, protocol.py, robot_link.py, fake_robot.py, avoid_planner.py, astar_planner.py, sim_view.py

Các file đó import lẫn nhau theo kiểu "import robot_model as rm" (tên trần, không có tên package).
Để chúng chạy được trong package ROS mà không phải sửa dòng import nào, file này thêm chính thư mục
core/ vào sys.path. Cách dùng ở các node:

    from eezybot_bringup import core  # noqa: F401  (phải đứng trước các dòng import bên dưới)
    import robot_model as rm
    from fake_robot import FakeRobotLink

Muốn đổi hằng số: sửa ở newcode/robot_model.py và KINEMATICS.ino trước, rồi copy đè vào đây.
test/test_core_copy.py báo lỗi nếu bản ở đây khác bản newcode/.
"""

import os
import sys

CORE_DIR = os.path.dirname(os.path.abspath(__file__))
CORE_FILES = ("robot_model.py", "protocol.py", "robot_link.py", "fake_robot.py",
              "avoid_planner.py", "astar_planner.py", "sim_view.py")

if CORE_DIR not in sys.path:
    sys.path.insert(0, CORE_DIR)
