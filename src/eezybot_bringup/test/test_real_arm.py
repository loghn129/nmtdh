"""
test_real_arm.py - MỨC 3 (TAY THẬT) VÀ BẢNG CHỌN CỠ VẬT CẢN
----------------------------------------------------------
Không có ESP32 trong lúc test, nên "tay thật" ở đây là fake_esp32.SerialBridge: firmware giả lập ngồi sau một
cổng serial giả (pty). RealArm mở cổng đó bằng pyserial y như mở /dev/ttyUSB0 và chỉ thấy các dòng chữ,
không thấy góc khớp -> so tư thế ƯỚC LƯỢNG của RealArm với tư thế "thật" bên trong firmware giả lập.

Test chu trình chạy đúng thời gian thật (không tua nhanh được: tư thế ước lượng phát lại theo đồng hồ),
mỗi chu trình khoảng 20 giây.
"""

import math
import os
import time

import pytest

from eezybot_bringup import core, scene_markers as sm
from eezybot_bringup import pose_estimator
from eezybot_bringup.fake_esp32 import SerialBridge
from eezybot_bringup.real_arm import RealArm
import astar_planner as aa
import protocol
import robot_model as rm
import sim_view

HERE = os.path.dirname(os.path.abspath(__file__))
NEWCODE_DIR = os.path.normpath(os.path.join(HERE, "..", "..", "..", "..", "newcode"))


def _class_source(text, name):
    """Nguyên văn 1 lớp ở mức ngoài cùng: từ dòng "class <name>" tới trước dòng không thụt lề kế tiếp."""
    lines = text.splitlines()
    start = next(i for i, line in enumerate(lines) if line.startswith(f"class {name}"))
    end = next((i for i in range(start + 1, len(lines)) if lines[i] and not lines[i][0].isspace()), len(lines))
    return "\n".join(lines[start:end]).rstrip()


def test_pose_estimator_giong_ban_goc():
    """Lớp PoseEstimator và 2 mẫu dòng trong package giống từng chữ với newcode/web_controller.py."""
    src = os.path.join(NEWCODE_DIR, "web_controller.py")
    if not os.path.isfile(src):
        pytest.skip(f"Khong co {src}")
    with open(src, encoding="utf-8") as f:
        original = f.read()
    with open(pose_estimator.__file__, encoding="utf-8") as f:
        ours = f.read()
    assert _class_source(ours, "PoseEstimator") == _class_source(original, "PoseEstimator")
    for name in ("POSE_RE", "FK_RE", "HOME_MS"):
        line = next(ln for ln in ours.splitlines() if ln.startswith(f"{name} = "))
        assert line.split("#")[0].rstrip() in original, name


# ---------------------------------------------------------------------------
# Bảng chọn cỡ vật cản
# ---------------------------------------------------------------------------
def test_bang_chon_co_vat_can():
    slots = sm.palette_slots()
    assert [r for _n, r, _x, _y in slots] == [r for _n, r in sm.OBSTACLE_SIZES]
    for name, r, x, y in slots:
        assert sm.palette_hit(x, y) == r                    # Bấm giữa ô -> đúng cỡ đó
        assert sm.palette_hit(x, y + r - 1.0) == r          # Bấm sát mép ô vẫn trúng
        assert sm.size_name(r) == name
        # Ô nằm ngoài tầm với (kể cả lề an toàn) -> bấm chọn cỡ không bao giờ trùng chỗ cần đặt vật cản
        assert math.hypot(x, y) - r > rm.R_MAX + 40.0
        assert -450.0 < x - r and x + r < 650.0 and y + r < 550.0      # Nằm trọn trên mặt bàn (scene.xacro)
    for (_n1, r1, x1, _y1), (_n2, r2, x2, _y2) in zip(slots, slots[1:]):
        reach1, reach2 = max(r1, 22.0) + 5.0, max(r2, 22.0) + 5.0     # Bán kính vùng bấm của từng ô
        assert abs(x1 - x2) > reach1 + reach2                         # Vùng bấm của 2 ô kề nhau không chồng nhau
    for x, y in ((0.0, 300.0), (172.0, 246.0), (-179.0, 240.0), (0.0, 0.0)):   # Trong vùng làm việc
        assert sm.palette_hit(x, y) is None
    assert sm.size_name(33.0) == "TUY CHON"
    assert sm.clamp_radius(1.0) == sm.OBSTACLE_R_MIN and sm.clamp_radius(999.0) == sm.OBSTACLE_R_MAX
    texts = [m.text for m in sm.palette_markers(45.0) if m.text]
    assert any("LON (r=45 mm)" in t for t in texts)


@pytest.mark.parametrize("r_mm, deg", [(15.0, 82.0), (28.0, 82.0), (45.0, 82.0), (60.0, 98.0)])
def test_planner_ne_duoc_moi_co_vat_can(r_mm, deg):
    """Vật cản đặt trên cung mặc định r = 300: cỡ nào cũng có chỗ planner tìm được đường né cho khay 1,
    và đường đó cách vật cản đủ lề 40 mm. (Cỡ RAT LON chỉ né được ở ít vị trí: nó choán gần hết bề rộng quạt,
    đặt ở giữa cung thì tay đang ở HOME đã nằm trong lề an toàn -> BLOCKED.)"""
    x, y, _ = rm.polar_to_xyz(rm.R_PATH, deg, 0.0)
    ob = sim_view.make_obstacle_mm(x, y, r_mm)
    p = aa.plan(1, [ob], 9999.0, 40.0, 25.0)
    assert p.kind == "PATH", p.reason
    assert not any(res["hit"] for res in p.results)
    gap = min(math.hypot(px - x, py - y) for res in p.results if res["move"].carrying
              for px, py, _z in res["tip_path"]) - r_mm
    assert gap >= 40.0 - 0.5, f"khe ho {gap:.1f} mm"


# ---------------------------------------------------------------------------
# Tay thật (qua cổng serial giả)
# ---------------------------------------------------------------------------
class Rig:
    """ESP32 ảo + RealArm + người trả lời REQ, giống cách sim_node nối chúng với nhau."""

    def __init__(self):
        self.bridge = SerialBridge(speed=1.0)
        self.logs = []
        self.obstacles = []
        self.replies = []
        self.reply_delay_s = 0.0
        self.arm = RealArm(self.bridge.port, log=lambda level, text: self.logs.append((level, text)), retry_s=0.3)
        self.arm.set_req_handler(self._on_req)

    def _on_req(self, bin_id):
        p = aa.plan(bin_id, list(self.obstacles), 9999.0, 40.0, 25.0)
        time.sleep(self.reply_delay_s)
        self.arm.note_plan(bin_id, p)
        self.arm.send(p.reply)
        self.replies.append(p.kind)

    def run(self, seconds, until=None):
        """Chạy vòng lặp 100 Hz như timer của node. Trả về sai lệch lớn nhất giữa ước lượng và "tay thật":
        (góc khớp, độ), (vị trí tâm kẹp, mm), (góc kẹp, độ)."""
        dq = dtcp = dgrip = 0.0
        t_end = time.monotonic() + seconds
        while time.monotonic() < t_end:
            self.arm.tick()
            q_est, g_est = self.arm.pose()
            q_true, g_true = self.bridge.fake.pose()
            dq = max(dq, max(abs(a - b) for a, b in zip(q_est, q_true)))
            dtcp = max(dtcp, math.dist(rm.fk(*q_est), rm.fk(*q_true)))
            dgrip = max(dgrip, abs(g_est - g_true))
            if until is not None and until():
                break
            time.sleep(0.01)
        return dq, dtcp, dgrip

    def close(self):
        self.arm.close()
        self.bridge.close()


@pytest.fixture
def rig():
    aa.warm_up()                       # Tính sẵn lưới: REQ đầu tiên cũng trả lời kịp trong PCWAIT
    r = Rig()
    r.run(6.0, until=lambda: r.arm.connected and r.bridge.fake.pc_timeout_ms == 1500)
    yield r
    r.close()


def _idle(rig, bin_id, n):
    return lambda: rig.arm.delivered[bin_id] == n and rig.arm.status()["state"] == "WAIT_NEW_FRUIT" \
        and not rig.arm.status()["move"]


def test_ket_noi_gui_auto_off_va_pcwait(rig):
    assert rig.arm.connected and rig.arm.err is None
    assert rig.bridge.fake.auto is False                     # "AUTO OFF" đã tới firmware
    assert rig.bridge.fake.pc_timeout_ms == 1500             # "PCWAIT 1500" đã tới firmware
    assert rig.arm.pc_timeout_ms == 1500
    assert rig.arm.pose()[0] == pytest.approx(rm.POSE_HOME)  # "[FK] Goc: ..." -> đồng bộ về HOME
    # Không gửi lệnh bù rơ nào
    assert not any(line.startswith(("[RO]", "[BACKLASH]")) for line in rig.bridge.fake.lines)


def test_run_khay_2_tu_the_uoc_luong_khop_tay_that(rig):
    """RUN 2 không vật cản: CLEAR, tư thế ước lượng bám sát "tay thật" suốt chu trình."""
    assert rig.arm.send("RUN 2")
    dq, dtcp, dgrip = rig.run(40.0, until=_idle(rig, 2, 1))
    assert rig.replies == ["CLEAR"]
    assert rig.arm.delivered == {1: 0, 2: 1, 3: 0} and rig.bridge.fake.delivered == {1: 0, 2: 1, 3: 0}
    assert rig.arm.pose()[0] == pytest.approx(rm.POSE_HOME)
    print(f"\nRUN 2 (CLEAR): lech lon nhat goc khop {dq:.2f} do, tam kep {dtcp:.1f} mm, kep {dgrip:.1f} do")
    assert dq < 6.0 and dtcp < 25.0, f"lech {dq:.2f} do / {dtcp:.1f} mm"


def test_run_co_vat_can_di_duong_ne(rig):
    """Có vật cản trên cung: trả lời PATH, ước lượng phát lại đúng đường né mà firmware chạy."""
    rig.obstacles = [sim_view.make_obstacle_mm(*rm.polar_to_xyz(rm.R_PATH, 82.0, 0.0)[:2], 45.0)]
    assert rig.arm.send("RUN 1")
    dq, dtcp, _ = rig.run(50.0, until=_idle(rig, 1, 1))
    assert rig.replies == ["PATH"]
    assert rig.bridge.fake.waypoints                          # Firmware đã nhận và chạy các điểm né
    assert rig.arm.delivered[1] == 1 and rig.bridge.fake.delivered[1] == 1
    print(f"\nRUN 1 (PATH): lech lon nhat goc khop {dq:.2f} do, tam kep {dtcp:.1f} mm")
    assert dq < 6.0 and dtcp < 25.0, f"lech {dq:.2f} do / {dtcp:.1f} mm"


def test_bi_chan_thi_dung_o_home_roi_chay_lai(rig):
    """Vật cản ngay điểm gắp: BLOCKED, ước lượng đứng yên ở HOME; bỏ vật cản thì chạy và thả được."""
    x, y, _ = rm.polar_to_xyz(*rm.P_PICK)
    rig.obstacles = [sim_view.make_obstacle_mm(x, y, 28.0)]
    assert rig.arm.send("RUN 3")
    rig.run(8.0, until=lambda: len(rig.replies) >= 2)
    assert set(rig.replies) == {protocol.BLOCKED}
    assert rig.arm.status()["state"] in ("BLOCKED", "WAIT_PC")
    assert rig.arm.pose()[0] == pytest.approx(rm.POSE_HOME) and rig.arm.delivered[3] == 0
    rig.obstacles = []
    rig.run(45.0, until=_idle(rig, 3, 1))
    assert rig.replies[-1] == "CLEAR" and rig.arm.delivered[3] == 1


def test_tra_loi_muon_thi_uoc_luong_di_duong_mac_dinh(rig):
    """PC trả lời PATH quá muộn: firmware đã tự đi đường mặc định -> ước lượng cũng phải đi đường mặc định."""
    rig.obstacles = [sim_view.make_obstacle_mm(0.0, 300.0, 28.0)]
    rig.reply_delay_s = 2.0                                   # > PCWAIT 1500 ms
    assert rig.arm.send("RUN 1")
    dq, dtcp, _ = rig.run(45.0, until=_idle(rig, 1, 1))
    assert rig.replies == ["PATH"]
    assert rig.bridge.fake.waypoints == []                    # Firmware không dùng đường né
    assert any("Bo qua" in line for line in rig.bridge.fake.lines)
    assert dq < 6.0 and dtcp < 25.0, f"lech {dq:.2f} do / {dtcp:.1f} mm"


def test_lenh_pose_bang_tay(rig):
    """Lệnh tay POSE: ước lượng nội suy theo dòng "[POSE] ... T=..ms" của firmware."""
    assert rig.arm.send("POSE 70 80 30 100 800")
    dq, _dtcp, dgrip = rig.run(1.5)
    assert rig.arm.pose()[0] == pytest.approx((70.0, 80.0, 30.0)) and rig.arm.pose()[1] == pytest.approx(100.0)
    assert dq < 4.0 and dgrip < 8.0


def test_mat_cong_thi_tu_ket_noi_lai(tmp_path):
    """Cổng chưa có: không ném lỗi, báo lý do; khi cổng xuất hiện thì tự kết nối."""
    logs = []
    path = str(tmp_path / "tty_gia")
    arm = RealArm(path, log=lambda level, text: logs.append((level, text)), retry_s=0.3)
    bridge = None
    try:
        time.sleep(1.0)
        arm.tick()
        assert not arm.connected and arm.err and arm.status()["state"] == "DISCONNECTED"
        assert sum(1 for level, _t in logs if level == "error") == 1      # Không in lặp lại cùng 1 lỗi
        assert arm.send("RUN 1") is False
        bridge = SerialBridge(link_path=path)
        t0 = time.monotonic()
        while not arm.connected and time.monotonic() - t0 < 8.0:
            time.sleep(0.05)
        assert arm.connected and arm.err is None
    finally:
        arm.close()
        if bridge is not None:
            bridge.close()


def test_core_co_du_file_cho_muc_3():
    assert {"robot_link.py", "sim_view.py", "fake_robot.py"} <= set(core.CORE_FILES)


def test_log_tu_real_arm_khong_lam_chet_node():
    """rclpy nhớ mức log theo DÒNG gọi: gọi info / warn / error qua cùng 1 dòng là ném ValueError.
    Lỗi này từng làm chết node đúng lúc rút cáp -> _log_from_arm phải chịu được cả 3 mức xen kẽ."""
    import rclpy.logging
    from eezybot_bringup.sim_node import SimNode

    class Stub:
        def get_logger(self):
            return rclpy.logging.get_logger("test_log_from_arm")

    for level in ("info", "error", "warn", "info", "error"):
        SimNode._log_from_arm(Stub(), level, f"thu muc log {level}")
