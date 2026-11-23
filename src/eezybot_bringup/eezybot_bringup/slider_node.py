"""
slider_node.py - MỨC 1: KÉO THANH TRƯỢT GÓC SERVO, XEM TAY CHUYỂN ĐỘNG TRONG RVIZ
------------------------------------------------------------------------------
joint_state_publisher_gui hiện 4 thanh trượt theo đúng con số của firmware (đơn vị ĐỘ):
    q1_de (góc đế thật), q2_vai (servo vai), q3_khuyu (servo khuỷu), kep (servo tay kẹp)
và phát chúng lên topic /servo_states. Node này đổi sang 6 khớp của URDF rồi phát /joint_states.

Vì sao không kéo thẳng khớp URDF: khớp cổ tay phụ thuộc cả vai lẫn khuỷu (cơ cấu hình bình hành),
thẻ <mimic> của URDF không mô tả được. Kéo theo góc servo thì giới hạn cũng đúng như tay thật.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import MarkerArray

from eezybot_bringup import joint_math, scene_markers as sm
import robot_model as rm  # core/ đã được joint_math thêm vào sys.path

SERVO_JOINTS = ("q1_de", "q2_vai", "q3_khuyu", "kep")   # Tên khớp giả trong config/servo_sliders.urdf


class SliderNode(Node):
    def __init__(self):
        super().__init__("eezybot_slider")
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "eezybot/markers", 10)
        self.create_subscription(JointState, "servo_states", self._on_servo, 10)
        self.book = sm.MarkerBook()
        self._warned = False
        self._last_key = None
        self.get_logger().info("Keo thanh truot (don vi DO, goc servo) de xem tay chuyen dong.")

    def _on_servo(self, msg):
        vals = dict(zip(msg.name, msg.position))
        if not all(name in vals for name in SERVO_JOINTS):
            if not self._warned:
                self.get_logger().warn(f"/servo_states thieu khop, can {SERVO_JOINTS}, nhan {list(msg.name)}")
                self._warned = True
            return
        q1, q2, q3, grip = (vals[name] for name in SERVO_JOINTS)

        out = JointState()
        out.header.stamp = self.get_clock().now().to_msg()
        out.name = list(joint_math.JOINT_NAMES)
        out.position = joint_math.joint_positions(q1, q2, q3, grip)
        self.pub_js.publish(out)

        # Hiện góc và vị trí tâm kẹp ngay trong RViz
        x, y, z = rm.fk(q1, q2, q3)
        r, deg, _ = rm.xyz_to_polar(x, y, z)
        ok = rm.ik(x, y, z)[0]
        text = (f"q1={q1:.1f}  q2={q2:.1f}  q3={q3:.1f}  kep={grip:.0f}\n"
                f"Tam kep: x={x:.1f} y={y:.1f} z={z:.1f} mm\n"
                f"r={r:.1f} mm  goc={deg:.1f} do" + ("" if ok else "\n(IK cua firmware KHONG nhan diem nay)"))
        self.pub_markers.publish(self.book.array(sm.workspace_markers() + sm.text_marker(text)))
        key = (round(q1, 1), round(q2, 1), round(q3, 1), round(grip))
        if key != self._last_key:      # Chỉ in khi thanh trượt thật sự đổi (và không quá 2 lần/giây)
            self._last_key = key
            self.get_logger().info(text.replace("\n", " | "), throttle_duration_sec=0.5)


def main():
    rclpy.init()
    node = SliderNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
