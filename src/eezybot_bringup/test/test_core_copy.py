"""
test_core_copy.py - CÁC FILE TRONG core/ PHẢI GIỐNG HỆT BẢN GỐC Ở newcode/
------------------------------------------------------------------------
Quy ước của dự án: không sửa code lõi khi làm ROS2. Muốn đổi hằng số thì đổi ở newcode/ (và KINEMATICS.ino)
trước, rồi copy đè vào eezybot_bringup/core/. Test này báo lỗi nếu hai bản lệch nhau.
Không tìm thấy thư mục newcode/ (ví dụ chỉ mang workspace sang máy khác) thì bỏ qua.
"""

import hashlib
import os

import pytest

from eezybot_bringup import core

HERE = os.path.dirname(os.path.abspath(__file__))
# test/ -> eezybot_bringup -> src -> eezy_ws -> thư mục dự án (chứa newcode/)
NEWCODE_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "newcode"))


def _sha256(path):
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


@pytest.mark.parametrize("name", core.CORE_FILES)
def test_file_loi_giong_ban_goc(name):
    src = os.path.join(NEWCODE_DIR, name)
    if not os.path.isfile(src):
        pytest.skip(f"Khong co {src}")
    assert _sha256(os.path.join(core.CORE_DIR, name)) == _sha256(src), f"core/{name} khac newcode/{name}"


def test_import_duoc_cac_file_loi():
    """Sau khi import eezybot_bringup.core, các file lõi import được bằng tên trần như bản gốc."""
    import astar_planner
    import avoid_planner
    import fake_robot
    import protocol
    import robot_model

    for mod in (astar_planner, avoid_planner, fake_robot, protocol, robot_model):
        assert os.path.dirname(os.path.abspath(mod.__file__)) == core.CORE_DIR
