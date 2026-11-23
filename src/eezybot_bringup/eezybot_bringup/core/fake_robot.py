"""
fake_robot.py - ESP32 + CÁNH TAY GIẢ LẬP (dùng khi KHÔNG có phần cứng)
---------------------------------------------------------------------
- Cùng giao diện với RobotLink (send, run_sequence, poll, busy, last_line, close, lines, n_lines,
  lines_since, ok, err, resets, set_req_handler) -> script nào dùng RobotLink cũng dùng được FakeRobotLink.
- Mô phỏng firmware KINEMATICS.ino ở mức giao thức + chuyển động:
    * Vòng lặp giống loop(): cập nhật chuyển động -> máy trạng thái AUTO -> đọc 1 lệnh Serial.
    * Chu trình lấy từ robot_model.cycle_moves() (cùng hình học, cùng thời gian từng đoạn),
      nội suy bậc 5 giống RobotArmController::update().
    * Lọc nhiễu màu 3 lần đọc, gửi "REQ <khay>", chờ CLEAR / PATH / BLOCKED tối đa PC_TIMEOUT_MS,
      không trả lời -> đường mặc định. In các dòng giống firmware ("[DONE]", "[IK LOI]", ...).
    * RUN n: về HOME, nghỉ HOME_REST_MS rồi mới gửi REQ; đang chạy chu trình / chờ PC thì bỏ qua.
    * RO / RO DE|VAI|KHUYU <deg> / RO TAM ... / BACKLASH: lưu & in độ rơ như firmware. Tay giả
      KHÔNG có rơ cơ khí, nên bù rơ chỉ làm lệch "góc servo đã ghi" (servo_out), không lệch self.q.
      flash (NVS) và backlash (RAM) tách riêng: RO TAM chỉ đổi RAM; simulate_reset() = ESP32 khởi động lại
      (AUTO ON, HOME, PCWAIT mặc định, lấy lại bù rơ từ flash, in dòng chào + "[RESET] ...").
- put_fruit(khay): giả lập đặt quả có màu ứng với khay đó vào điểm gắp.
- speed: tua nhanh chuyển động (speed=5 -> nhanh gấp 5). Thời gian chờ PC luôn tính theo giờ thật.
LƯU Ý: đây là mô hình của firmware, không phải firmware. Firmware thật kiểm tra bằng biên dịch + Wokwi.
"""

import collections
import math
import threading
import time

import protocol
import robot_model as rm
from robot_link import DONE_TIMEOUT_S, RESET_LINE

COLOR_SCAN_MS = 350
COLOR_SAME_NEEDED = 3
HOME_REST_MS = 400
GRIP_CLOSE_MS = 900
GRIP_OPEN_MS = 600
BACKLASH_KEYS = {"DE": "de", "VAI": "vai", "KHUYU": "khuyu"}
BACKLASH_MAX = 15.0


class FakeRobotLink:
    def __init__(self, port="GIA LAP", baud=115200, speed=1.0, verbose=True):
        self.port = port
        self.err = None
        self.speed = float(speed)
        self.verbose = verbose
        self.lines = collections.deque(maxlen=60)
        self.n_lines = 0
        self.resets = 0
        self.history = []                     # Tên các đoạn đã chạy (để kiểm tra)
        self.delivered = {1: 0, 2: 0, 3: 0}   # Số quả đã thả vào từng khay
        self.pc_timeout_ms = protocol.PC_TIMEOUT_MS
        self.flash = {"de": 2.39, "vai": 0.0, "khuyu": 0.0}  # "Flash" (NVS): RO DE/VAI/KHUYU ghi vào đây
        self.backlash = dict(self.flash)                      # Đang dùng (RAM): RO TAM chỉ đổi ở đây

        self._lock = threading.RLock()
        self._inbox = collections.deque()
        self._queue = collections.deque()
        self._waiting = False
        self._t_sent = 0.0
        self._req_handler = None
        self._t0 = time.monotonic()

        # Trạng thái "phần cứng"
        self.q = rm.POSE_HOME
        self.grip = rm.GRIPPER_OPEN
        self._motion = None      # (Move, t_bắt_đầu_ms, grip_đầu, grip_cuối)
        self._was_busy = False

        # Trạng thái FSM
        self.auto = True
        self.state = "WAIT_NEW_FRUIT"
        self.fruit_bin = None    # Quả đang nằm ở điểm gắp
        self.holding = False
        self.target_bin = None
        self.waypoints = []
        self.current_move = ""
        self._steps = collections.deque()
        self._run_pending = False
        self._same = 0
        self._last_scan = -1e9
        self._wait_until = 0.0
        self._t_req = 0.0
        self._retry_at = 0.0

        self._running = True
        threading.Thread(target=self._loop, daemon=True).start()
        self._emit("ESP32 GIA LAP - FK/IK AUTO SORT san sang")

    # ------------------------------------------------------------------
    # Giao diện giống RobotLink
    # ------------------------------------------------------------------
    @property
    def ok(self):
        return self._running

    def lines_since(self, mark):
        """(các dòng mới kể từ mốc mark = n_lines lúc trước, mốc mới) - giống RobotLink."""
        with self._lock:
            n = self.n_lines
            k = min(n - mark, len(self.lines))
            return (list(self.lines)[-k:] if k > 0 else []), n

    def set_req_handler(self, fn):
        """fn(khay) được gọi (ở luồng riêng) mỗi khi robot gửi 'REQ <khay>'."""
        self._req_handler = fn

    def send(self, cmd):
        if not self._running:
            return False
        if self.verbose:
            print(f"[PC -> ESP32] {cmd}")
        self._inbox.append(cmd.strip())
        return True

    def run_sequence(self, cmds):
        with self._lock:
            self._queue.clear()
            self._queue.extend(cmds)
            self._waiting = False
            self._pump_locked()

    def poll(self):
        with self._lock:
            if self._waiting and time.time() - self._t_sent > DONE_TIMEOUT_S:
                self._add_line("[TIMEOUT] Khong nhan duoc [DONE] -> huy chuoi lenh")
                self._waiting = False
                self._queue.clear()

    def busy(self):
        with self._lock:
            return self._waiting or bool(self._queue)

    def last_line(self):
        return self.lines[-1] if self.lines else ""

    def close(self):
        self._running = False
        with self._lock:
            self._queue.clear()
            self._waiting = False

    # ------------------------------------------------------------------
    # Phần riêng của bản giả lập
    # ------------------------------------------------------------------
    def put_fruit(self, bin_id):
        with self._lock:
            self.fruit_bin = bin_id

    def pose(self):
        with self._lock:
            return self.q, self.grip

    def simulate_reset(self):
        """Giả lập ESP32 khởi động lại (sụt áp / nút reset): về mặc định như firmware vừa boot
        (AUTO ON, HOME, PCWAIT mặc định, bù rơ lấy lại từ flash), in dòng chào -> RobotLink thấy
        '[RESET]' và resets tăng 1, chuỗi lệnh đang chờ bị hủy."""
        with self._lock:
            self._motion = None
            self._steps.clear()
            self.q, self.grip, self.holding = rm.POSE_HOME, rm.GRIPPER_OPEN, False
            self.auto, self.state, self._run_pending, self._same = True, "WAIT_NEW_FRUIT", False, 0
            self.target_bin, self.waypoints, self.current_move = None, [], ""
            self.pc_timeout_ms = protocol.PC_TIMEOUT_MS
            self.backlash = dict(self.flash)
            self._queue.clear()
            self._waiting = False
            self.resets += 1
            self._emit("ESP32 GIA LAP - FK/IK AUTO SORT san sang")
            self._add_line(RESET_LINE)

    def status(self):
        with self._lock:
            return {"state": self.state, "move": self.current_move, "holding": self.holding,
                    "target_bin": self.target_bin, "waypoints": list(self.waypoints), "auto": self.auto}

    # ------------------------------------------------------------------
    # Bên trong
    # ------------------------------------------------------------------
    def _now(self):
        """Đồng hồ mô phỏng (ms), đã nhân tốc độ tua."""
        return (time.monotonic() - self._t0) * 1000.0 * self.speed

    def _pump_locked(self):
        if self._waiting or not self._queue:
            return
        self.send(self._queue.popleft())
        self._waiting = True
        self._t_sent = time.time()

    def _add_line(self, line):
        self.lines.append(line)
        self.n_lines += 1

    def _emit(self, line):
        self._add_line(line)
        if self.verbose:
            print(f"[ESP32] {line}")
        if line.startswith("[DONE]"):
            self._waiting = False
            self._pump_locked()
        elif line.startswith("[LOI]") or line.startswith("[IK LOI]"):
            self._waiting = False
            self._queue.clear()
        bin_id = protocol.parse_req(line)
        if bin_id is not None and self._req_handler is not None:
            threading.Thread(target=self._req_handler, args=(bin_id,), daemon=True).start()

    def _loop(self):
        while self._running:
            with self._lock:
                self._update_motion()
                self._handle_auto()
                if self._inbox:
                    self._handle_cmd(self._inbox.popleft())
                busy = self._motion is not None
                if self._was_busy and not busy and not self.auto:
                    self._emit("[DONE]")
                self._was_busy = busy
            time.sleep(0.01)

    # ---- Chuyển động ----
    def _start_move(self, move, grip_to=None):
        self._motion = (move, self._now(), self.grip, self.grip if grip_to is None else grip_to)
        self.current_move = move.name

    def _move_joint(self, target_q, ms, grip_to=None, name="POSE"):
        self._start_move(rm.Move(name, self.q, tuple(target_q), False, ms=max(300, int(ms))), grip_to)

    def _update_motion(self):
        if self._motion is None:
            return
        move, t_start, g0, g1 = self._motion
        tau = (self._now() - t_start) / float(max(move.ms, 300))
        s = rm.quintic(tau)
        try:
            self.q = rm.move_pose(move, s)
        except ValueError:
            pass  # Giống firmware: điểm trung gian lỗi IK thì giữ nguyên tư thế
        self.grip = g0 + (g1 - g0) * s
        if tau >= 1.0:
            self._motion = None

    # ---- Máy trạng thái AUTO (giống handleAutoSort) ----
    def _send_req(self):
        self._emit(f"REQ {self.target_bin}")
        self.state = "WAIT_PC"
        self._t_req = time.monotonic()

    def _start_cycle(self, waypoints, direct_home=False):
        try:
            moves = rm.cycle_moves(self.target_bin, waypoints or None, direct_home)
        except ValueError as e:
            self._emit(f"[AUTO LOI] IK that bai: {e} -> DUNG. Go HOME de chay lai.")
            self.state = "ERROR"
            return
        self.waypoints = list(waypoints or [])
        self._steps.clear()
        for mv in moves:
            self._steps.append(("move", mv))
            if mv.name == "HA CHAM":
                self._steps.append(("grip", rm.GRIPPER_CLOSE, GRIP_CLOSE_MS))
            elif mv.name == "HA XUONG KHAY":
                self._steps.append(("grip", rm.GRIPPER_OPEN, GRIP_OPEN_MS))
        self.state = "CYCLE"

    def _handle_auto(self):
        if not self.auto or self._motion is not None or self._now() < self._wait_until:
            return
        if self.state == "WAIT_NEW_FRUIT":
            if self._run_pending:  # RUN n: đã về HOME và nghỉ xong -> hỏi PC luôn
                self._run_pending = False
                self._send_req()
                return
            if self._now() - self._last_scan < COLOR_SCAN_MS:
                return
            self._last_scan = self._now()
            if self.fruit_bin is None:
                self._same = 0
                return
            self._same += 1
            if self._same < COLOR_SAME_NEEDED:
                return
            self._same = 0
            self.target_bin = self.fruit_bin
            self._emit(f"[AUTO] PHAT HIEN qua -> khay {self.target_bin}")
            self._send_req()
        elif self.state == "WAIT_PC":
            if (time.monotonic() - self._t_req) * 1000.0 > self.pc_timeout_ms:
                self._emit("[PC] Khong tra loi -> duong mac dinh")
                self._start_cycle(None)
        elif self.state == "BLOCKED":
            if self._now() >= self._retry_at:
                self._send_req()
        elif self.state == "CYCLE":
            if not self._steps:
                self._emit("[AUTO] Da ve HOME. Bat dau do mau qua tiep theo ngay!")
                self.current_move = ""
                self._wait_until = self._now() + HOME_REST_MS
                self._same = 0
                self.state = "WAIT_NEW_FRUIT"
                return
            step = self._steps.popleft()
            if step[0] == "grip":
                _kind, value, ms = step
                if value == rm.GRIPPER_CLOSE:
                    self.holding, self.fruit_bin = True, None
                else:
                    self.holding = False
                    self.delivered[self.target_bin] += 1
                self._move_joint(self.q, ms, grip_to=value, name="KEP" if value == rm.GRIPPER_CLOSE else "THA")
            else:
                mv = step[1]
                self.history.append(mv.name)
                if mv.pa is None:  # Đi theo khớp: firmware nội suy từ tư thế HIỆN TẠI (moveToPose)
                    mv = rm.Move(mv.name, self.q, mv.b, mv.carrying, ms=mv.ms)
                # Đoạn đầu chu trình mở kẹp (rot = POSE_HOME có gripper = OPEN), như startCycle()
                self._start_move(mv, rm.GRIPPER_OPEN if mv.name == "XOAY DE TOI GAP" else None)

    # ---- Lệnh Serial (giống handleSerial) ----
    def _set_manual(self):
        self._run_pending = False
        if self.auto:
            self.auto = False
            self._emit("[AUTO] TAT (dang dieu khien tay). Go 'AUTO ON' de chay lai chu trinh.")

    def _ik_move(self, r, deg, z, grip, ms):
        ok, q, msg = rm.ik_polar(r, deg, z)
        if not ok:
            self._emit(f"[IK LOI] {msg}")
            return False
        grip = grip if math.isfinite(grip) else rm.GRIPPER_OPEN
        self._move_joint(q, ms, grip_to=min(max(grip, rm.LIM_GRIPPER[0]), rm.LIM_GRIPPER[1]), name="POL")
        return True

    def _print_backlash(self):
        b = self.backlash
        self._emit(f"[RO] de={b['de']:.2f} vai={b['vai']:.2f} khuyu={b['khuyu']:.2f} "
                   "(de: do thuc, vai/khuyu: do servo)")

    @staticmethod
    def _parse_backlash(txt):
        v = float(txt)  # ValueError nếu không phải số
        if not math.isfinite(v) or not 0.0 <= v <= BACKLASH_MAX:
            raise ValueError(txt)
        return v

    def _handle_backlash(self, args):
        """'RO' | 'RO DE|VAI|KHUYU <deg>' | 'RO TAM DE|VAI|KHUYU <deg>' (giống handleBacklashCmd)."""
        a = args.upper().split()
        if not a:
            self._print_backlash()
            return
        save = a[0] != "TAM"
        if not save:
            a = a[1:]
        try:
            if len(a) != 2 or a[0] not in BACKLASH_KEYS:
                raise ValueError
            key = BACKLASH_KEYS[a[0]]
            self.backlash[key] = self._parse_backlash(a[1])
            if save:
                self.flash[key] = self.backlash[key]
        except ValueError:
            self._emit("[LOI] Cu phap: RO | RO DE|VAI|KHUYU <0..15> | RO TAM DE|VAI|KHUYU <0..15>")
            return
        self._print_backlash()
        self._emit("[RO] Da luu flash (ap dung tu chuyen dong tiep theo)" if save
                   else "[RO] TAM THOI (chua luu flash, khoi dong lai se mat)")

    def _handle_reply(self, cmd):
        if not self.auto or self.state != "WAIT_PC":
            self._emit(f"[PC] Bo qua '{cmd.split()[0]}' (khong o trang thai cho PC)")
            return
        if cmd == protocol.CLEAR:
            self._emit("[PC] CLEAR -> duong mac dinh")
            self._start_cycle(None)
        elif cmd == protocol.BLOCKED:
            self._block("[PC] BLOCKED -> dung cho")
        else:
            try:
                wps, direct = protocol.parse_path_full(cmd)
                for mv in rm.transfer_moves(self.target_bin, wps):
                    rm.sample_move(mv, 20)  # Giống parsePath(): kiểm tra IK 20 điểm mỗi đoạn
            except ValueError as e:
                self._block(f"[PC LOI] PATH sai ({e}) -> coi nhu BLOCKED")
                return
            self._emit(f"[PC] PATH {len(wps)} diem ne" + (", ve thang HOME" if direct else ""))
            self._start_cycle(wps, direct)

    def _block(self, msg):
        self._emit(msg)
        self.state = "BLOCKED"
        self._retry_at = self._now() + protocol.BLOCKED_RETRY_MS

    def _to_wait_new_fruit(self):
        self._run_pending = False
        self._same = 0
        self._wait_until = 0.0
        self.state = "WAIT_NEW_FRUIT"

    def _handle_cmd(self, cmd):
        if not cmd:
            return
        up = cmd.upper()
        parts = cmd.split()
        try:
            # Câu trả lời của PC so sánh ĐÚNG chữ hoa như firmware ("clear" là lệnh sai)
            if cmd in (protocol.CLEAR, protocol.BLOCKED) or cmd.startswith("PATH "):
                self._handle_reply(cmd)
            elif up == "AUTO ON":
                self.auto = True
                self._to_wait_new_fruit()
                self._move_joint(rm.POSE_HOME, 1500, rm.GRIPPER_OPEN, "HOME")
                self._emit("[AUTO] BAT. Ve HOME va cho qua.")
            elif up == "AUTO OFF":
                self._set_manual()
                self._emit("[AUTO] Da tat.")
            elif up.startswith("RUN "):
                b = int(parts[1])
                idle = not self.auto or self.state in ("WAIT_NEW_FRUIT", "BLOCKED", "ERROR")
                if b not in (1, 2, 3):
                    self._emit("[LOI] Cu phap: RUN <1|2|3>")
                elif not idle or self._run_pending:
                    self._emit("[RUN] Bo qua: dang chay chu trinh / cho PC (doi '[AUTO] Da ve HOME' roi gui lai)")
                else:
                    # Giống firmware: về HOME (state CYCLE không còn bước -> "Da ve HOME", nghỉ) rồi mới REQ
                    self.target_bin, self.auto, self._run_pending = b, True, True
                    self._steps.clear()
                    self._wait_until = 0.0
                    self._move_joint(rm.POSE_HOME, 1500, rm.GRIPPER_OPEN, "HOME")
                    self.state = "CYCLE"
                    self._emit(f"[RUN] Ve HOME roi hoi duong -> Khay {b}")
            elif up.startswith("PCWAIT "):
                self.pc_timeout_ms = max(100, int(parts[1]))
                self._emit(f"[PC] Cho PC toi da {self.pc_timeout_ms}ms")
            elif up in ("HOME", "H"):
                self._move_joint(rm.POSE_HOME, 1500, rm.GRIPPER_OPEN, "HOME")  # POSE_HOME mở kẹp
                self._to_wait_new_fruit()
                self._emit("[EXEC] Ve HOME.")
            elif up == "PICK":
                self._set_manual()
                self._emit("[EXEC] Di chuyen den diem GAP...")
                self._ik_move(*rm.P_PICK, rm.GRIPPER_OPEN, 1800)
            elif up in ("BIN1", "B1", "BIN2", "B2", "BIN3", "B3"):
                self._set_manual()
                b = int(up[-1])
                self._ik_move(rm.R_PATH, rm.BINS[b]["deg"], rm.Z_BIN, rm.GRIPPER_CLOSE, 2000)
            elif up in ("OPEN", "CLOSE"):
                self._set_manual()
                self._move_joint(self.q, 600, rm.GRIPPER_OPEN if up == "OPEN" else rm.GRIPPER_CLOSE, up)
                self._emit("[EXEC] MO tay kep (40 do)." if up == "OPEN" else "[EXEC] DONG kep chat (140 do).")
            elif up == "COLOR":
                c = {1: "Qua chin", 2: "Qua vua", 3: "Qua xanh"}.get(self.fruit_bin, "Khong co qua")
                self._emit(f"[SENSOR] TCS3200 f=(0, 0, 0)Hz rgb=(0, 0, 0) -> GIA LAP: {c}")
            elif up == "FK":
                x, y, z = rm.fk(*self.q)
                r, deg, _ = rm.xyz_to_polar(x, y, z)
                self._emit(f"[FK] Goc: (B={self.q[0]:.1f}, S={self.q[1]:.1f}, E={self.q[2]:.1f}) -> "
                           f"(X={x:.1f}, Y={y:.1f}, Z={z:.1f})mm | Cung: r={r:.1f}mm")
            elif up == "BACKLASH" or up.startswith("BACKLASH "):
                if len(parts) > 1:
                    try:
                        self.backlash["de"] = self.flash["de"] = self._parse_backlash(" ".join(parts[1:]))
                    except ValueError:
                        self._emit("[LOI] Cu phap: BACKLASH <0..15>")
                        return
                self._emit(f"[BACKLASH] Bu ro de = {self.backlash['de']:.2f} do thuc")
            elif up == "RO" or up.startswith("RO "):
                self._handle_backlash(cmd[2:])
            elif up.startswith("POL ") or up.startswith("XYZ "):
                v = [float(t) for t in parts[1:]]
                grip = v[3] if len(v) > 3 else rm.GRIPPER_OPEN
                ms = int(v[4]) if len(v) > 4 else 1800
                self._set_manual()
                pol = (v[0], v[1], v[2]) if up.startswith("POL") else rm.xyz_to_polar(v[0], v[1], v[2])
                if self._ik_move(*pol, grip, ms):
                    self._emit(f"[POL] r={pol[0]:.1f} goc={pol[1]:.1f} z={pol[2]:.1f}" if up.startswith("POL")
                               else f"[XYZ] Toa do ({v[0]:.1f}, {v[1]:.1f}, {v[2]:.1f}) mm")
                else:
                    self._emit("[LOI] Diem ngoai vung lam viec cua IK!")
            elif up.startswith("POSE "):
                v = [float(t) for t in parts[1:]]
                if len(v) < 4:
                    raise ValueError
                if not all(math.isfinite(t) for t in v[:4]):
                    self._emit("[LOI] Goc khong hop le (nan/inf)")
                    return
                self._set_manual()
                q = (min(max(v[0], rm.LIM_BASE[0]), rm.LIM_BASE[1]),
                     min(max(v[1], rm.LIM_SHOULDER[0]), rm.LIM_SHOULDER[1]),
                     min(max(v[2], rm.LIM_ELBOW[0]), rm.LIM_ELBOW[1]))
                grip = min(max(v[3], rm.LIM_GRIPPER[0]), rm.LIM_GRIPPER[1])
                ms = int(v[4]) if len(v) > 4 else 1500
                if q != tuple(v[:3]) or grip != v[3]:
                    self._emit("[CANH BAO] Goc vuot gioi han -> da cat ve gioi han an toan!")
                self._move_joint(q, ms, grip, "POSE")
                self._emit(f"[POSE] B={q[0]:.1f} (servo {rm.base_deg_to_servo(q[0]):.1f}) S={q[1]:.1f} "
                           f"E={q[2]:.1f} G={grip:.1f} T={ms}ms")
            elif up in ("HELP", "?"):
                self._emit("[HELP] Ban gia lap: AUTO ON/OFF, RUN n, PCWAIT ms, HOME, PICK, BINn, OPEN, CLOSE, "
                           "COLOR, FK, POL, XYZ, POSE, RO, BACKLASH, CLEAR, PATH, BLOCKED")
            else:
                self._emit("[?] Lenh khong hop le. Go 'HELP' de xem danh sach lenh!")
        except (ValueError, IndexError):
            self._emit(f"[LOI] Cu phap sai: {cmd}")
