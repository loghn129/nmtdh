"""
test_urdf_fk.py - URDF PHẢI CHO CÙNG KẾT QUẢ VỚI robot_model.py (mục 9.3 của BAN_GIAO_ROS2.md)
---------------------------------------------------------------------------------------------
Chạy xacro, tự tính động học thuận dọc chuỗi khớp của URDF (giống việc robot_state_publisher làm),
rồi so vị trí khung "tcp" với robot_model.fk() cho nhiều bộ (q1, q2, q3) ngẫu nhiên: sai số < 0.1 mm.

Chạy:  cd eezy_ws/src/eezybot_bringup && python3 -m pytest test -q
"""

import math
import os
import random
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import xacro

from eezybot_bringup import joint_math
import robot_model as rm  # core/ đã được joint_math thêm vào sys.path

HERE = os.path.dirname(os.path.abspath(__file__))
XACRO_FILE = os.path.normpath(os.path.join(HERE, "..", "..", "eezybot_description", "urdf", "eezybot.urdf.xacro"))
TOL_MM = 0.1


def _rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _rot_axis(axis, angle):
    x, y, z = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return np.array([[c + x * x * v, x * y * v - z * s, x * z * v + y * s],
                     [y * x * v + z * s, c + y * y * v, y * z * v - x * s],
                     [z * x * v - y * s, z * y * v + x * s, c + z * z * v]])


def _tf(rot=np.eye(3), xyz=(0.0, 0.0, 0.0)):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rot, xyz
    return t


class Urdf:
    """Đọc các khớp của URDF và tính vị trí / hướng của một khâu so với base_link."""

    def __init__(self, xml_text):
        self.joints = {}       # tên khớp -> dict
        self.parent_of = {}    # khâu con -> tên khớp nối nó với khâu cha
        for j in ET.fromstring(xml_text).findall("joint"):
            origin, axis, limit = j.find("origin"), j.find("axis"), j.find("limit")
            num = lambda el, key, default: [float(v) for v in (el.get(key, default) if el is not None else default).split()]  # noqa: E731
            info = {
                "type": j.get("type"), "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
                "xyz": num(origin, "xyz", "0 0 0"), "rpy": num(origin, "rpy", "0 0 0"),
                "axis": num(axis, "xyz", "1 0 0"),
                "lower": float(limit.get("lower")) if limit is not None else None,
                "upper": float(limit.get("upper")) if limit is not None else None,
            }
            self.joints[j.get("name")] = info
            self.parent_of[info["child"]] = j.get("name")

    def link_tf(self, link, values):
        """Ma trận 4x4 của khâu trong khung base_link, với values = {tên khớp: giá trị}."""
        chain = []
        while link in self.parent_of:
            name = self.parent_of[link]
            chain.append(name)
            link = self.joints[name]["parent"]
        t = np.eye(4)
        for name in reversed(chain):
            j = self.joints[name]
            t = t @ _tf(_rot_rpy(*j["rpy"]), j["xyz"])
            if j["type"] == "revolute":
                t = t @ _tf(_rot_axis(j["axis"], values[name]))
            elif j["type"] == "prismatic":
                t = t @ _tf(xyz=np.asarray(j["axis"]) * values[name])
        return t


@pytest.fixture(scope="module")
def urdf():
    return Urdf(xacro.process_file(XACRO_FILE).toxml())


def _values(q1, q2, q3, grip=rm.GRIPPER_OPEN, holding=False):
    return dict(zip(joint_math.JOINT_NAMES, joint_math.joint_positions(q1, q2, q3, grip, holding)))


def _random_q(rng):
    return (rng.uniform(*rm.LIM_BASE), rng.uniform(*rm.LIM_SHOULDER), rng.uniform(*rm.LIM_ELBOW))


def test_tcp_khop_robot_model_fk(urdf):
    """500 tư thế ngẫu nhiên trong giới hạn servo: tcp của URDF trùng rm.fk() dưới 0.1 mm."""
    rng = random.Random(2026)
    worst = 0.0
    for _ in range(500):
        q = _random_q(rng)
        tcp_mm = urdf.link_tf("tcp", _values(*q))[:3, 3] * 1000.0
        worst = max(worst, math.dist(tcp_mm, rm.fk(*q)))
    assert worst < TOL_MM, f"sai so lon nhat {worst:.4f} mm"


def test_cac_tu_the_cua_chu_trinh(urdf):
    """Mọi điểm dọc chu trình 3 khay: tcp của URDF trùng rm.fk()."""
    for bin_id in rm.BINS:
        for mv in rm.cycle_moves(bin_id):
            for q in rm.sample_move(mv, 10):
                tcp_mm = urdf.link_tf("tcp", _values(*q))[:3, 3] * 1000.0
                assert math.dist(tcp_mm, rm.fk(*q)) < TOL_MM


def test_home_4_diem(urdf):
    """Tư thế HOME: vai, khuỷu, cổ tay, tâm kẹp trong mặt phẳng tay (r, z) đúng số ở mục 4.5 của file bàn giao."""
    v = _values(*rm.POSE_HOME)
    expect = {"upper_arm": (6.0, 90.0), "forearm": (-34.74, 226.03), "gripper_base": (119.72, 184.29),
              "tcp": (204.50, 179.65)}
    for link, (r, z) in expect.items():
        x, y, zz = urdf.link_tf(link, v)[:3, 3] * 1000.0
        # HOME: q1 = 90 độ -> mặt phẳng tay là mặt phẳng YZ, bán kính r nằm dọc +Y
        assert abs(x) < 0.01, link
        assert abs(y - r) < 0.01 and abs(zz - z) < 0.01, f"{link}: ({y:.2f}, {zz:.2f}) khac ({r}, {z})"


def test_cum_kep_luon_nam_ngang(urdf):
    """Cơ cấu hình bình hành: trục X của cụm kẹp luôn nằm ngang, trục Z luôn thẳng đứng."""
    rng = random.Random(7)
    for _ in range(200):
        rot = urdf.link_tf("gripper_base", _values(*_random_q(rng)))[:3, :3]
        assert abs(rot[2, 0]) < 1e-9 and abs(rot[2, 2] - 1.0) < 1e-9


def test_goc_khop_nam_trong_gioi_han_urdf(urdf):
    """Mọi tư thế trong giới hạn servo (và 2 trạng thái kẹp) đều nằm trong <limit> của URDF."""
    rng = random.Random(11)
    samples = [_values(*_random_q(rng), grip, holding) for _ in range(500)
               for grip, holding in ((rm.LIM_GRIPPER[0], False), (rm.GRIPPER_CLOSE, False), (rm.GRIPPER_CLOSE, True))]
    corners = [_values(q1, q2, q3) for q1 in rm.LIM_BASE for q2 in rm.LIM_SHOULDER for q3 in rm.LIM_ELBOW]
    for v in samples + corners:
        for name, value in v.items():
            j = urdf.joints[name]
            assert j["lower"] - 1e-9 <= value <= j["upper"] + 1e-9, f"{name}={value:.4f} ngoai [{j['lower']:.4f}, {j['upper']:.4f}]"


def test_hang_so_urdf_giong_robot_model(urdf):
    """Vị trí gốc các khớp trong URDF = hằng số hình học của robot_model.py (mm -> m)."""
    j = urdf.joints
    assert j["joint_shoulder"]["xyz"] == pytest.approx([rm.R_OFF / 1000.0, 0.0, rm.Z_BASE / 1000.0])
    assert j["joint_elbow"]["xyz"] == pytest.approx([rm.L1 / 1000.0, 0.0, 0.0])
    assert j["joint_wrist"]["xyz"] == pytest.approx([rm.L2 / 1000.0, 0.0, 0.0])
    assert j["tool_center"]["xyz"] == pytest.approx([rm.TOOL_R / 1000.0, 0.0, rm.TOOL_Z / 1000.0])
    assert j["joint_base"]["lower"] == pytest.approx(math.radians(rm.LIM_BASE[0]))
    assert j["joint_base"]["upper"] == pytest.approx(math.radians(rm.LIM_BASE[1]))


def test_canh_vat_dung_vi_tri(urdf):
    """Khung máng nằm đúng dưới điểm gắp, khung 3 khay đúng trên cung R_PATH ở góc của robot_model.BINS."""
    x, y, _ = rm.polar_to_xyz(*rm.P_PICK)
    assert urdf.link_tf("chute", {})[:2, 3] * 1000.0 == pytest.approx([x, y], abs=0.01)
    for bin_id, info in rm.BINS.items():
        x, y, _ = rm.polar_to_xyz(rm.R_PATH, info["deg"], 0.0)
        assert urdf.link_tf(f"bin{bin_id}", {})[:2, 3] * 1000.0 == pytest.approx([x, y], abs=0.01)
