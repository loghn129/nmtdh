"""
fake_esp32.py - "ESP32 ẢO" SAU MỘT CỔNG SERIAL GIẢ (để thử MỨC 3 khi chưa cắm tay thật)
-------------------------------------------------------------------------------------
Tạo một cổng serial giả (pty) và cho core/fake_robot.FakeRobotLink ngồi sau nó: mọi dòng PC ghi vào cổng
được đưa cho firmware giả lập, mọi dòng firmware giả lập in ra được ghi ngược lại cổng. Chương trình nào mở
cổng này bằng pyserial (RobotLink, real.launch.py, cả Serial Monitor) đều thấy nó như một ESP32 thật.

    ros2 run eezybot_bringup fake_esp32                      # in ra tên cổng, ví dụ /dev/pts/7
    ros2 run eezybot_bringup fake_esp32 --link /tmp/eezybot_tty
    ros2 launch eezybot_bringup real.launch.py port:=/tmp/eezybot_tty

Đây là mô hình của firmware (fake_robot.py), KHÔNG phải firmware và không có rơ bánh răng.
speed phải để 1.0 nếu muốn tư thế ước lượng ở phía PC khớp với "tay" này.
"""

import argparse
import os
import pty
import select
import signal
import threading
import time
import tty

from eezybot_bringup import core  # noqa: F401 - thêm core/ vào sys.path
from fake_robot import FakeRobotLink  # noqa: E402
from robot_link import RESET_LINE  # noqa: E402


class SerialBridge:
    """FakeRobotLink nối với đầu "master" của một pty; đầu "slave" (self.port) là cổng cho PC mở."""

    def __init__(self, speed=1.0, link_path=None):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)                 # Không echo, không đổi ký tự xuống dòng
        os.set_blocking(self.master, False)    # Không ai đọc thì bỏ dòng, không treo
        self.port = os.ttyname(self.slave)
        self.link_path = link_path
        if link_path:
            if os.path.islink(link_path):
                os.unlink(link_path)
            os.symlink(self.port, link_path)
            self.port = link_path
        self.fake = FakeRobotLink(speed=speed, verbose=False)
        self._mark = 0
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        buf = b""
        while self._running:
            ready, _, _ = select.select([self.master], [], [], 0.005)
            if ready:
                try:
                    buf += os.read(self.master, 4096)
                except OSError:
                    pass
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    cmd = raw.decode("ascii", errors="replace").strip()
                    if cmd:
                        self.fake.send(cmd)
            lines, self._mark = self.fake.lines_since(self._mark)
            for line in lines:
                if line == RESET_LINE:         # Dòng do phía PC tự thêm, ESP32 thật không gửi
                    continue
                try:
                    os.write(self.master, (line + "\r\n").encode("ascii", errors="replace"))
                except OSError:                # Bộ đệm đầy vì chưa ai mở cổng -> bỏ dòng này
                    pass

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.fake.close()
        if self.link_path and os.path.islink(self.link_path):
            os.unlink(self.link_path)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def main():
    ap = argparse.ArgumentParser(description="ESP32 + canh tay gia lap sau mot cong serial gia (pty)")
    ap.add_argument("--link", default="", help="tao duong dan co dinh tro toi cong, vi du /tmp/eezybot_tty")
    ap.add_argument("--speed", type=float, default=1.0, help="tua nhanh chuyen dong (de 1.0 khi thu muc 3)")
    args, _ros_args = ap.parse_known_args()

    def on_term(_sig, _frame):     # SIGTERM cũng dọn dẹp (xoá đường dẫn --link) như Ctrl+C
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, on_term)
    bridge = SerialBridge(speed=args.speed, link_path=args.link or None)
    print(f"ESP32 gia lap dang cho o cong: {bridge.port}", flush=True)
    print(f"  ros2 launch eezybot_bringup real.launch.py port:={bridge.port}", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
