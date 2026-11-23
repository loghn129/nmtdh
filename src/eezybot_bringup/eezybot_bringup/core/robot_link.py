"""
robot_link.py - KẾT NỐI SERIAL TỚI ESP32 (KINEMATICS.ino)
---------------------------------------------------------
- Đọc phản hồi của ESP32 ở luồng nền (không làm đứng cửa sổ OpenCV).
- run_sequence([...]): gửi lần lượt từng lệnh, chờ "[DONE]" rồi mới gửi lệnh sau.
  Nếu ESP32 báo lỗi IK ("[LOI]" / "[IK LOI]") thì hủy các lệnh còn lại.
- set_req_handler(fn): khi ESP32 gửi "REQ <khay>" thì gọi fn(khay) ở luồng riêng
  (fn tự tính đường né rồi send() câu trả lời CLEAR / PATH / BLOCKED - xem protocol.py).
- lines (60 dòng cuối) + n_lines (tổng số dòng đã nhận, chỉ tăng) -> lines_since(mark) lấy đúng
  các dòng MỚI kể từ một mốc, kể cả khi lines đã đầy và xoay vòng.
- Mất kết nối (rút cáp): ok = False, err ghi lý do, hủy chuỗi lệnh, có dòng "[MAT KET NOI] ...".
- ESP32 tự khởi động lại (sụt áp...): thêm dòng "[RESET] ESP32 khoi dong lai", hủy chuỗi lệnh,
  resets tăng 1 -> script tự gửi lại AUTO OFF / PCWAIT nếu cần.
  1 lần khởi động in TỚI 3 dòng dấu hiệu (ROM: "ets ...", "rst:..."; firmware: dòng chào
  "... FK/IK AUTO SORT ...") -> resets có thể tăng 3. Script nên gửi lại MỖI LẦN resets đổi:
  lệnh gửi lúc ROM đang chạy bị mất, chỉ lệnh gửi sau dòng chào mới tới được firmware.
- close(): ok = False, send() trả về False (không ném lỗi), không thêm dòng [MAT KET NOI].
- LƯU Ý: Phải TẮT Serial Monitor của Arduino IDE trước, vì 1 cổng COM chỉ mở được 1 nơi.
"""

import collections
import threading
import time
import unicodedata

import protocol

try:
    import serial
except ImportError:  # Chưa cài pyserial
    serial = None

DONE_TIMEOUT_S = 8.0
RESET_MARKERS = ("FK/IK AUTO SORT", "rst:", "ets ")  # Dòng chào của firmware / ROM khi khởi động
RESET_LINE = "[RESET] ESP32 khoi dong lai"
RESET_JUNK = "\x00\ufffd "  # Byte rác (0x00 / 0xFF -> U+FFFD) hay dính ở đầu dòng ROM lúc reset


def is_reset_line(line):
    return RESET_MARKERS[0] in line or line.lstrip(RESET_JUNK).startswith(RESET_MARKERS[1:])


def ascii_text(s):
    """Chữ ASCII để vẽ lên cửa sổ OpenCV: thông báo lỗi của Windows có thể có dấu tiếng Việt."""
    s = unicodedata.normalize("NFKD", str(s).replace("đ", "d").replace("Đ", "D"))
    return s.encode("ascii", "ignore").decode("ascii")


class RobotLink:
    def __init__(self, port, baud=115200):
        self.port = port
        self.ser = None
        self.err = None
        self.lines = collections.deque(maxlen=60)
        self.n_lines = 0
        self.resets = 0
        self._queue = collections.deque()
        self._waiting = False
        self._t_sent = 0.0
        self._lock = threading.Lock()
        self._running = False
        self._closed = False
        self._req_handler = None

        if serial is None:
            self.err = "Chua cai pyserial: pip install pyserial"
            return
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
        except Exception as e:  # noqa: BLE001 - báo mọi lỗi mở cổng cho người dùng
            self.err = f"Khong mo duoc {port}: {ascii_text(e)}"
            self.ser = None
            return

        # ESP32 tự reset khi mở cổng -> chờ khởi động xong rồi xóa chữ in lúc boot
        time.sleep(2.5)
        try:
            self.ser.reset_input_buffer()
        except Exception as e:  # noqa: BLE001 - rút cáp trong lúc chờ: báo lỗi, không ném ra ngoài
            self.err = f"Mat ket noi: {ascii_text(e)}"
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
            return
        self._running = True
        threading.Thread(target=self._reader, daemon=True).start()

    @property
    def ok(self):
        return self.ser is not None and self.err is None and not self._closed

    def _add_line(self, line):
        """Mọi dòng vào lines đều đi qua đây để n_lines luôn đúng."""
        self.lines.append(line)
        self.n_lines += 1

    def lines_since(self, mark):
        """(các dòng mới kể từ mốc mark = n_lines lúc trước, mốc mới). Mất bớt nếu > 60 dòng."""
        with self._lock:  # Đọc n_lines và lines cùng lúc: luồng đọc không chen dòng mới vào giữa
            n = self.n_lines
            k = min(n - mark, len(self.lines))
            return (list(self.lines)[-k:] if k > 0 else []), n

    def _lost_locked(self, why):
        """Mất kết nối (đã giữ _lock): ghi lý do 1 lần, hủy chuỗi lệnh, dừng luồng đọc."""
        why = ascii_text(why)
        if self.err is None:
            self.err = f"Mat ket noi: {why}"
            self._add_line(f"[MAT KET NOI] {why}")
        self._queue.clear()
        self._waiting = False
        self._running = False

    def _lost(self, why):
        with self._lock:
            self._lost_locked(why)

    def _reader(self):
        while self._running:
            try:
                raw = self.ser.readline()
            except Exception as e:  # noqa: BLE001
                if self._running:  # Lỗi do chính close() đóng cổng thì không phải mất kết nối
                    self._lost(e)
                break
            if not raw:
                continue
            try:
                self._handle_line(raw.decode("utf-8", errors="replace").strip())
            except Exception as e:  # noqa: BLE001 - 1 dòng lỗi không được làm chết luồng đọc
                print(f"[ESP32] (loi xu ly dong: {ascii(e)})")

    def _handle_line(self, line):
        if not line:
            return
        print(f"[ESP32] {ascii(line)[1:-1]}")  # ascii(): console cp1252 không in được ký tự lạ
        reset = is_reset_line(line)
        with self._lock:
            self._add_line(line)
            if reset:
                self.resets += 1
                self._add_line(RESET_LINE)
                self._queue.clear()
                self._waiting = False
            elif line.startswith("[DONE]"):
                self._waiting = False
                self._pump_locked()
            elif line.startswith("[LOI]") or line.startswith("[IK LOI]"):
                self._waiting = False
                self._queue.clear()
        bin_id = protocol.parse_req(line)
        if bin_id is not None and self._req_handler is not None:
            threading.Thread(target=self._req_handler, args=(bin_id,), daemon=True).start()

    def _pump_locked(self):
        if self._waiting or not self._queue:
            return
        if self._write(self._queue.popleft()):
            self._waiting = True
            self._t_sent = time.monotonic()  # monotonic: chỉnh giờ hệ thống không gây [TIMEOUT] giả
        else:
            self._queue.clear()

    def _write(self, cmd):
        """Gửi 1 dòng (gọi khi ĐÃ giữ _lock). Lỗi cổng -> mất kết nối, trả về False."""
        if not self.ok:
            return False
        print(f"[PC -> ESP32] {ascii(cmd)[1:-1]}")
        try:
            self.ser.write((cmd.strip() + "\n").encode("ascii", errors="replace"))
        except Exception as e:  # noqa: BLE001 - SerialException / OSError khi rút cáp
            self._lost_locked(e)  # Hủy luôn chuỗi lệnh -> busy() = False (không treo tới [TIMEOUT])
            return False
        return True

    def set_req_handler(self, fn):
        """fn(khay) được gọi (ở luồng riêng) mỗi khi ESP32 gửi 'REQ <khay>'."""
        self._req_handler = fn

    def send(self, cmd):
        """Gửi ngay 1 lệnh không cần chờ [DONE] (VD: AUTO OFF, FK, HELP). Trả về False nếu không gửi được."""
        with self._lock:
            return self._write(cmd)

    def run_sequence(self, cmds):
        """Hủy chuỗi cũ, chạy chuỗi lệnh chuyển động mới (mỗi lệnh chờ [DONE])."""
        with self._lock:
            self._queue.clear()
            self._queue.extend(cmds)
            self._waiting = False
            self._pump_locked()

    def poll(self):
        """Gọi mỗi khung hình: xử lý trường hợp không nhận được [DONE]."""
        with self._lock:
            if self._waiting and time.monotonic() - self._t_sent > DONE_TIMEOUT_S:
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
        self._closed = True  # Sau close(): ok = False, mọi lệnh gửi bị bỏ qua (không ném lỗi cổng đã đóng)
        with self._lock:
            self._queue.clear()
            self._waiting = False
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:  # noqa: BLE001
                pass
