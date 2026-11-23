"""
pose_estimator.py - TƯ THẾ ƯỚC LƯỢNG CỦA TAY THẬT (MỨC 3)
---------------------------------------------------------
Firmware không gửi góc khớp liên tục. Muốn RViz chuyển động theo tay thật thì phải ĐOÁN tư thế:
phát lại đúng các đoạn của chu trình theo thời gian (sim_view.Timeline) và nội suy theo các dòng
"[POSE] ... T=..ms" / "[FK] Goc: ..." mà firmware in ra.

Lớp PoseEstimator dưới đây COPY NGUYÊN từ newcode/web_controller.py (không viết mới - mục 6 của file bàn giao).
Không import thẳng web_controller.py vì file đó kéo theo camera, Flask và 06_auto_run.py.
test/test_real_arm.py so từng chữ của lớp này với bản gốc; lệch là báo lỗi.
"""

import re
import time

from eezybot_bringup import core  # noqa: F401 - thêm core/ vào sys.path
import robot_model as rm  # noqa: E402
import sim_view  # noqa: E402

HOME_MS = 1500   # Thời gian firmware về HOME (moveToPose(POSE_HOME, 1500)) - giống web_controller.HOME_MS
# Hai mẫu dòng dưới đây cũng copy từ web_controller.py
POSE_RE = re.compile(r"^\[POSE\] B=(-?[\d.]+) \(servo -?[\d.]+\) S=(-?[\d.]+) E=(-?[\d.]+) G=(-?[\d.]+) T=(\d+)ms")
FK_RE = re.compile(r"^\[FK\] Goc: \(B=(-?[\d.]+), S=(-?[\d.]+), E=(-?[\d.]+)\)")


class PoseEstimator:
    """Tư thế ước lượng của robot THẬT (firmware không gửi góc liên tục)."""

    def __init__(self):
        self.q = tuple(rm.POSE_HOME)
        self.grip = rm.GRIPPER_OPEN
        self.holding = False
        self.move = ""
        self._joint = None      # (q0, g0, q1, g1, t0, ms)
        self._timeline = None   # (Timeline, t0)

    def joint(self, q3, grip, ms):
        self._timeline = None
        self._joint = (self.q, self.grip, tuple(q3), float(grip), time.monotonic(), max(1.0, float(ms)))
        self.move = "POSE"

    def timeline(self, moves):
        self._joint = None
        self._timeline = (sim_view.Timeline(moves), time.monotonic())

    def snap(self, q3, grip=None):
        self._joint = self._timeline = None
        self.q = tuple(q3)
        if grip is not None:
            self.grip = float(grip)

    def freeze(self):
        self._joint = self._timeline = None
        self.move = ""

    def update(self):
        now = time.monotonic()
        if self._joint is not None:
            q0, g0, q1, g1, t0, ms = self._joint
            s = rm.quintic((now - t0) * 1000.0 / ms)
            self.q = tuple(a + (b - a) * s for a, b in zip(q0, q1))
            self.grip = g0 + (g1 - g0) * s
            if s >= 1.0:
                self._joint, self.move = None, ""
        elif self._timeline is not None:
            tl, t0 = self._timeline
            t = (now - t0) * 1000.0
            q, g, carrying, name = tl.at(t)
            self.q, self.grip, self.holding, self.move = tuple(q), g, carrying, name
            if t >= tl.total_ms:
                self._timeline, self.move, self.holding = None, "", False
        return self.q, self.grip
