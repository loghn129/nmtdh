"""
real_arm.py - MỨC 3: TAY THẬT QUA CỔNG SERIAL, CÙNG GIAO DIỆN VỚI FakeRobotLink
------------------------------------------------------------------------------
sim_node chỉ cần: pose(), status(), delivered, fruit_bin, pc_timeout_ms, send(), set_req_handler(), close().
Ở mức 2 các thứ đó do core/fake_robot.FakeRobotLink cung cấp. Lớp RealArm cung cấp đúng các thứ đó cho tay thật:

    core/robot_link.RobotLink(port)   đọc / ghi Serial ở luồng nền, gọi hàm trả lời "REQ k"
    pose_estimator.PoseEstimator      đoán tư thế (firmware không gửi góc khớp liên tục)

Tư thế trong RViz vì vậy là ƯỚC LƯỢNG, đồng bộ theo các dòng firmware in ra:
    "[RUN] Ve HOME..." / "[AUTO] BAT" / "[EXEC] Ve HOME"   -> nội suy về HOME trong 1.5 s
    "[PC] CLEAR ..." / "[PC] PATH ..."                     -> phát lại các đoạn của kế hoạch vừa trả lời
    "[PC] Khong tra loi -> duong mac dinh"                 -> phát lại chu trình mặc định (PC trả lời muộn)
    "[PC] BLOCKED ..." / "[PC LOI] ..."                    -> đứng ở HOME
    "[AUTO] Da ve HOME"                                    -> chốt về HOME, xong 1 chu trình
    "[POSE] ... T=..ms" / "[FK] Goc: ..."                  -> lệnh tay / đồng bộ lại góc
Mốc bắt đầu chu trình lấy theo dòng "[PC] ..." của firmware (lúc nó THẬT SỰ bắt đầu chạy), không theo lúc PC gửi.

Không dùng bù rơ ở đây: node không gửi RO / BACKLASH, tư thế vẽ ra là góc lý tưởng. Firmware của tay thật
tự bù rơ bên trong nó như cũ.

Cổng chưa cắm / rút cáp: không ném lỗi, cứ 3 giây thử mở lại. Mở cổng xong gửi "AUTO OFF", "PCWAIT 1500", "FK"
giống web_controller.Controller._attach; ESP32 tự khởi động lại thì gửi lại.
"""

import threading
import time

from eezybot_bringup import core  # noqa: F401 - thêm core/ vào sys.path
from eezybot_bringup.pose_estimator import FK_RE, HOME_MS, POSE_RE, PoseEstimator
import protocol  # noqa: E402
import robot_model as rm  # noqa: E402
from robot_link import RESET_LINE, RobotLink  # noqa: E402

PC_WAIT_MS = 1500        # Giống 06_auto_run.py
RETRY_S = 3.0            # Mở cổng không được thì bao lâu thử lại


class RealArm:
    def __init__(self, port, baud=115200, log=None, retry_s=RETRY_S):
        """log(level, text): hàm ghi log, level là "info" / "warn" / "error"."""
        self.port, self.baud = port, int(baud)
        self._log = log or (lambda level, text: print(f"[{level}] {text}"))
        self._retry_s = retry_s

        self.delivered = {1: 0, 2: 0, 3: 0}   # Số sản phẩm đã thả vào từng khay (đếm theo ước lượng)
        self.fruit_bin = None                 # Sản phẩm đang nằm ở điểm gắp (biết từ "REQ k")
        self.pc_timeout_ms = protocol.PC_TIMEOUT_MS
        self.connected = False
        self.err = None                       # Lý do chưa kết nối được

        self._lock = threading.RLock()        # Giữ est và các biến trạng thái bên dưới
        self.est = PoseEstimator()
        self.state = "DISCONNECTED"
        self.auto = False
        self.target_bin = None
        self._cycle_bin = None                # Khay của chu trình đang chạy
        self._counted = False                 # Chu trình đang chạy đã được cộng vào delivered chưa
        self._pending = None                  # (khay, Plan) vừa trả lời cho REQ, chờ firmware xác nhận
        self._link = None
        self._mark = 0
        self._resets = 0
        self._req_handler = None
        self._closed = False
        self._last_err = None
        threading.Thread(target=self._connect_loop, daemon=True).start()

    # ------------------------------------------------------------------
    # Giao diện giống FakeRobotLink (phần sim_node dùng)
    # ------------------------------------------------------------------
    def set_req_handler(self, fn):
        self._req_handler = fn
        link = self._link
        if link is not None:
            link.set_req_handler(fn)

    def send(self, cmd):
        link = self._link
        return bool(link is not None and link.send(cmd))

    def pose(self):
        with self._lock:
            return self.est.q, self.est.grip

    def status(self):
        with self._lock:
            return {"state": self.state, "move": self.est.move, "holding": self.est.holding,
                    "target_bin": self.target_bin, "auto": self.auto}

    def close(self):
        self._closed = True
        link, self._link = self._link, None
        self.connected = False
        if link is not None:
            if link.ok:
                link.send(f"PCWAIT {protocol.PC_TIMEOUT_MS}")   # Trả ESP32 về thời gian chờ mặc định
                time.sleep(0.05)
            link.close()

    # ------------------------------------------------------------------
    # Phần riêng của tay thật
    # ------------------------------------------------------------------
    def note_plan(self, bin_id, plan):
        """Gọi NGAY TRƯỚC khi gửi câu trả lời REQ: nhớ kế hoạch để phát lại khi firmware xác nhận."""
        with self._lock:
            self._pending = (bin_id, plan)

    def tick(self):
        """Gọi đều đặn từ timer của node (không chặn): đọc dòng mới của ESP32, cập nhật tư thế ước lượng."""
        link = self._link
        if link is None:
            return
        link.poll()
        lines, self._mark = link.lines_since(self._mark)
        with self._lock:
            for line in lines:
                self._handle_line(line)
            was_holding = self.est.holding
            self.est.update()
            if self.est.holding:
                self.fruit_bin = None            # Đã kẹp lên -> điểm gắp trống
            elif was_holding:
                self._count_delivered()          # Vừa nhả sản phẩm vào khay
        if link.err is not None:                 # Rút cáp / mất cổng
            self._log("error", f"{link.err} -> thu ket noi lai moi {self._retry_s:g} s")
            self._detach(link)
        elif link.resets != self._resets:        # ESP32 khởi động lại: PCWAIT về 500 ms, AUTO tự bật
            self._resets = link.resets
            self._log("warn", f"ESP32 khoi dong lai -> gui lai AUTO OFF / PCWAIT {PC_WAIT_MS}")
            self._hello(link)

    def _connect_loop(self):
        while not self._closed:
            if self._link is None:
                link = RobotLink(self.port, self.baud)    # Chặn khoảng 2.5 s nếu mở được cổng
                if self._closed:
                    link.close()
                    return
                if link.ok:
                    self._attach(link)
                else:
                    self.err = link.err
                    if link.err != self._last_err:        # Không in lặp lại cùng một lỗi mỗi 3 giây
                        self._log("error", f"{link.err} -> thu lai moi {self._retry_s:g} s")
                        self._last_err = link.err
                    link.close()
            time.sleep(self._retry_s if self._link is None else 0.2)

    def _attach(self, link):
        with self._lock:
            self.est = PoseEstimator()
            self.state, self.auto = "WAIT_NEW_FRUIT", False
            self.target_bin = self._cycle_bin = self._pending = None
            self.fruit_bin = None
            self._mark, self._resets = link.n_lines, link.resets
        link.set_req_handler(self._req_handler)
        self._hello(link)
        link.send("FK")           # Firmware in "[FK] Goc: ..." -> lấy đúng tư thế hiện tại
        self._link = link
        self.connected, self.err, self._last_err = True, None, None
        self._log("info", f"Da ket noi tay that {self.port} ({self.baud} baud)")

    def _hello(self, link):
        link.send("AUTO OFF")     # Giống mọi script PC: kết nối xong thì tắt AUTO
        ok = link.send(f"PCWAIT {PC_WAIT_MS}")
        self.pc_timeout_ms = PC_WAIT_MS if ok else protocol.PC_TIMEOUT_MS

    def _detach(self, link):
        link.close()
        with self._lock:
            self.est.freeze()
            self.state, self._cycle_bin, self._pending = "DISCONNECTED", None, None
        self._link = None
        self.connected, self.err = False, link.err

    def _count_delivered(self):
        if self._cycle_bin is not None and not self._counted:
            self.delivered[self._cycle_bin] += 1
            self._counted = True

    def _start_cycle(self, kind):
        """Firmware vừa báo nó bắt đầu chu trình (kind = CLEAR / PATH) -> phát lại đúng các đoạn đó."""
        bin_id, plan = self._pending or (self.target_bin, None)
        self._pending = None
        moves = None
        if plan is not None and plan.kind == kind and plan.moves and bin_id == self.target_bin:
            moves = plan.moves
        elif kind == protocol.CLEAR and self.target_bin in rm.BINS:
            moves = rm.cycle_moves(self.target_bin)          # Đường mặc định
        if moves is None:
            # PATH do nơi khác gửi (gõ tay ở Serial Monitor...) -> không biết điểm né, không đoán bừa
            self._log("warn", "Firmware chay PATH khong phai do node nay gui -> khong uoc luong duoc tu the")
            self.est.freeze()
        else:
            self.est.timeline(moves)
        self.state, self._cycle_bin, self._counted = "CYCLE", self.target_bin, False

    def _handle_line(self, line):
        """Một dòng của ESP32 (đã giữ _lock). Phần đoán tư thế theo web_controller.Controller._handle_line."""
        if line.startswith("[AUTO] BAT"):
            self.auto = True
        elif line.startswith(("[AUTO] TAT", "[AUTO] Da tat")):
            self.auto = False

        bin_id = protocol.parse_req(line)
        if bin_id is not None:
            self.target_bin, self.fruit_bin, self.state = bin_id, bin_id, "WAIT_PC"
            return
        m = POSE_RE.match(line)
        if m:
            b, s, e, g, t = (float(v) for v in m.groups())
            self.est.joint((b, s, e), g, t)
            return
        m = FK_RE.match(line)
        if m:
            if self.est.move == "":      # Đang chuyển động thì không giật về số cũ
                self.est.snap(tuple(float(v) for v in m.groups()))
            return

        if line.startswith("[PC] CLEAR"):
            self._start_cycle(protocol.CLEAR)
        elif line.startswith("[PC] PATH"):
            self._start_cycle("PATH")
        elif line.startswith("[PC] Khong tra loi"):
            self._pending = None         # Trả lời muộn: firmware đã tự đi đường mặc định
            self._start_cycle(protocol.CLEAR)
        elif line.startswith("[PC] Bo qua"):
            self._pending = None         # Câu trả lời tới muộn / thừa: firmware không dùng
        elif line.startswith(("[PC] BLOCKED", "[PC LOI]")):
            self._pending = None
            self.state = "BLOCKED"
        elif line.startswith(("[EXEC] Ve HOME", "[AUTO] BAT", "[RUN] Ve HOME")):
            self.est.joint(rm.POSE_HOME, rm.GRIPPER_OPEN, HOME_MS)
            self.est.holding = False
            if line.startswith("[RUN]"):
                self.state = "CYCLE"
            elif self.state != "DISCONNECTED":
                self.state = "WAIT_NEW_FRUIT"
        elif line.startswith("[AUTO] Da ve HOME"):
            self._count_delivered()
            self._cycle_bin = None
            self.fruit_bin = None
            self.est.snap(rm.POSE_HOME, rm.GRIPPER_OPEN)     # Firmware nói đã ở HOME -> chốt lại, hết trôi
            self.est.holding, self.est.move = False, ""
            self.state = "WAIT_NEW_FRUIT"
        elif line.startswith(("[AUTO LOI]", RESET_LINE, "[MAT KET NOI]")):
            self._cycle_bin = self._pending = None
            self.est.freeze()
            if line.startswith("[AUTO LOI]"):
                self.state = "ERROR"
