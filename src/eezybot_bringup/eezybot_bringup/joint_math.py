"""
joint_math.py - ĐỔI GÓC CỦA FIRMWARE SANG GÓC KHỚP CỦA URDF
-----------------------------------------------------------
Firmware và giao thức dùng (q1, q2, q3, kẹp), đơn vị độ:
    q1  = góc đế THẬT trên mặt bàn
    q2  = góc SERVO vai,   q3 = góc SERVO khuỷu
    kẹp = góc servo tay kẹp (40 = mở, 140 = kẹp chặt)

URDF (eezybot_description/urdf/eezybot.urdf.xacro) dùng chuỗi nối tiếp, đơn vị radian và mét:
    joint_base     = rad(q1)
    joint_shoulder = rad(th2)          th2, phi = robot_model.link_angles(q2, q3)
    joint_elbow    = rad(phi - th2)
    joint_wrist    = rad(-phi)         -> cụm kẹp luôn nằm ngang (cơ cấu hình bình hành)
    joint_finger_* = nửa độ mở má kẹp + nửa bề dày má (mét)

KHÔNG có bù rơ ở đây: mô phỏng không có rơ cơ khí, q1 đã là góc thật.
"""

import math

from eezybot_bringup import core  # noqa: F401 - thêm core/ vào sys.path
import robot_model as rm  # noqa: E402

JOINT_NAMES = ["joint_base", "joint_shoulder", "joint_elbow", "joint_wrist",
               "joint_finger_left", "joint_finger_right"]

CUBE_MM = 50.0           # Cạnh khối sản phẩm (ước lượng, giống web/static/app.js)
FINGER_HALF_MM = 3.0     # Nửa bề dày má kẹp trong URDF


def grip_gap_mm(grip_deg, holding=False):
    """Độ hở giữa 2 má kẹp (mm) theo góc servo kẹp. ƯỚC LƯỢNG ĐỂ VẼ (công thức gripGap của app.js):
    servo 35 -> 78 mm, servo 140 -> 10 mm. Đang kẹp sản phẩm thì má không khép quá bề rộng khối."""
    gap = min(max(78.0 - (grip_deg - 35.0) * 68.0 / 105.0, 10.0), 78.0)
    if holding:
        gap = max(gap, CUBE_MM + 6.0)
    return gap


def joint_positions(q1, q2, q3, grip_deg=rm.GRIPPER_OPEN, holding=False):
    """(q1, q2, q3, kẹp) độ -> list 6 giá trị khớp theo đúng thứ tự JOINT_NAMES."""
    th2, phi = rm.link_angles(q2, q3)
    finger = (grip_gap_mm(grip_deg, holding) / 2.0 + FINGER_HALF_MM) / 1000.0
    return [math.radians(q1), math.radians(th2), math.radians(phi - th2), math.radians(-phi),
            finger, finger]
