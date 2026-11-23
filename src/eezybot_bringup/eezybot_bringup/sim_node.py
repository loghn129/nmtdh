"""
sim_node.py - MỨC 2 VÀ 3: CHU TRÌNH GẮP - THẢ + NÉ VẬT CẢN TRONG RVIZ, TAY GIẢ LẬP HOẶC TAY THẬT
----------------------------------------------------------------------------------------------
Node này là "cầu nối" giữa robot và RViz:

    robot.pose()       ->  /joint_states (30 Hz)  ->  robot_state_publisher  ->  TF  ->  RViz
    "REQ k" của robot  ->  planner (A* hoặc cố định) ->  trả lời CLEAR / PATH ... / BLOCKED

Tham số robot chọn nguồn chuyển động:
    robot=gia_lap (MỨC 2)  core/fake_robot.FakeRobotLink: ESP32 + cánh tay giả lập, chạy ở luồng nền riêng,
                           cùng hình học và cùng thời gian từng đoạn với firmware. Không cần phần cứng.
    robot=that    (MỨC 3)  real_arm.RealArm: tay thật qua cổng Serial (tham số port, baud). Firmware không gửi
                           góc khớp, nên tư thế trong RViz là ƯỚC LƯỢNG (xem real_arm.py).
                           !!! Node KHÔNG có camera: tay thật chỉ né những vật cản bạn khai báo trong RViz.

KHÔNG CÓ RƠ BÁNH RĂNG trong hình vẽ: góc đưa lên RViz là góc lý tưởng, node không gửi lệnh RO / BACKLASH
và không gọi hàm bù rơ nào.

Điều khiển:
    Nút "Publish Point" trong RViz  bấm 1 ô ở bảng "CO VAT CAN" -> chọn cỡ; bấm lên mặt bàn -> thêm vật cản
                                    cỡ đang chọn; bấm vào vật cản -> xoá nó
    /eezybot/obstacle_size   std_msgs/String      chọn cỡ: nho | vua | lon | rat_lon, hoặc bán kính bằng mm ("35")
    /eezybot/add_obstacle    geometry_msgs/Point  x, y (mét); z = bán kính (mét, 0 = cỡ đang chọn)
    /eezybot/clear_obstacles std_srvs/Empty       xoá hết vật cản
    /eezybot/drop            std_msgs/Int32       khay 1..3 (0 = ngẫu nhiên). Giả lập: thả 1 sản phẩm xuống máng.
                                                  Tay thật: gửi "RUN k" (chạy 1 chu trình, không cần cảm biến màu)
    /eezybot/auto_drop       std_srvs/SetBool     Giả lập: bật / tắt tự thả lần lượt khay 1 -> 2 -> 3.
                                                  Tay thật: gửi "AUTO ON" / "AUTO OFF"
    /eezybot/cmd             std_msgs/String      gửi nguyên 1 dòng lệnh cho firmware (HOME, POSE ..., FK, ...)
Theo dõi:
    /eezybot/status          std_msgs/String (JSON)   trạng thái, số đã thả, câu trả lời REQ gần nhất
    /eezybot/markers         MarkerArray              sản phẩm, vật cản, bảng chọn cỡ, đường đã lập, vệt tâm kẹp

Tham số (mặc định lấy theo newcode/calib_config.json):
    robot=gia_lap|that  port=/dev/ttyUSB0  baud=115200
    speed=1.0  planner=astar|co_dinh  safe_margin_mm=40  safe_z_margin_mm=25  obs_height_mm=9999
    obstacle_radius_mm=28  auto_drop=true
"""

import collections
import json
import math
import random
import threading
import time

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32, String
from std_srvs.srv import Empty, SetBool
from visualization_msgs.msg import MarkerArray

from eezybot_bringup import joint_math, scene_markers as sm
from eezybot_bringup.real_arm import PC_WAIT_MS, RealArm
# core/ đã được joint_math thêm vào sys.path -> import các file lõi bằng tên trần như bản gốc
import astar_planner as aa
import avoid_planner as ap
import protocol
import robot_model as rm
from fake_robot import FakeRobotLink

PLANNERS = {"co_dinh": ap, "astar": aa}   # Cùng hàm plan() / warm_up(), cùng kiểu Plan (giống 06_auto_run.py)
RATE_HZ = 30.0
SLIDE_MS = 1400          # Sản phẩm trượt từ đầu máng tới điểm gắp (giống web_controller.py)
TRAIL_POINTS = 600
REAL_RATE_HZ = 100.0     # Tay thật: đọc dòng của ESP32 dày hơn để mốc thời gian ước lượng ít trễ
STATE_TEXT = {"WAIT_NEW_FRUIT": "CHO SAN PHAM", "WAIT_PC": "CHO PC TRA LOI", "BLOCKED": "BI CHAN - DUNG O HOME",
              "CYCLE": "DANG CHAY CHU TRINH", "ERROR": "LOI", "DISCONNECTED": "CHUA KET NOI TAY THAT"}
SIZE_ALIASES = {name.lower().replace(" ", "_"): r for name, r in sm.OBSTACLE_SIZES}   # "rat_lon" -> 60.0


class SimNode(Node):
    def __init__(self):
        super().__init__("eezybot_sim")

        # ---- Tham số ----
        self.speed = float(self.declare_parameter("speed", 1.0).value)
        planner_name = self.declare_parameter("planner", "astar").value
        if planner_name not in PLANNERS:
            self.get_logger().warn(f"planner='{planner_name}' khong hop le -> dung 'astar'")
            planner_name = "astar"
        self.planner = PLANNERS[planner_name]
        self.margin = float(self.declare_parameter("safe_margin_mm", 40.0).value)
        self.z_margin = float(self.declare_parameter("safe_z_margin_mm", 25.0).value)
        self.obs_height = float(self.declare_parameter("obs_height_mm", 9999.0).value)
        self.obs_radius = sm.clamp_radius(self.declare_parameter("obstacle_radius_mm", 28.0).value)
        self.auto_drop = bool(self.declare_parameter("auto_drop", True).value)
        robot = self.declare_parameter("robot", "gia_lap").value
        if robot not in ("gia_lap", "that"):
            self.get_logger().warn(f"robot='{robot}' khong hop le -> dung 'gia_lap'")
            robot = "gia_lap"
        self.real = robot == "that"
        port = self.declare_parameter("port", "/dev/ttyUSB0").value
        baud = int(self.declare_parameter("baud", 115200).value)

        # ---- Trạng thái dùng chung giữa luồng ROS và luồng của FakeRobotLink ----
        self._lock = threading.Lock()        # Giữ obstacles, plan, last_req
        self._plan_lock = threading.Lock()   # Mỗi lúc chỉ 1 lần tính đường
        self.obstacles = []                  # list dict {x_mm, y_mm, r_mm, ...} như sim_view.make_obstacle_mm
        self.plan = None                     # Plan gần nhất (để vẽ)
        self.last_req = None                 # dict tóm tắt câu trả lời REQ gần nhất
        self.ready = False                   # Planner đã tính sẵn lưới xong chưa
        self._last_blocked = None            # (khay, lý do) của câu BLOCKED vừa in ra log

        self.chute = None                    # (khay, t0): sản phẩm đang trượt trên máng
        self.next_bin = 1                    # Khay của lần tự thả kế tiếp
        self.trail = collections.deque(maxlen=TRAIL_POINTS)
        self.book = sm.MarkerBook()
        self._ticks = 0

        # ---- ROS ----
        self.pub_js = self.create_publisher(JointState, "joint_states", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "eezybot/markers", 10)
        self.pub_status = self.create_publisher(String, "eezybot/status", 10)
        self.create_subscription(Int32, "eezybot/drop", self._on_drop, 10)
        self.create_subscription(Point, "eezybot/add_obstacle", self._on_add_obstacle, 10)
        self.create_subscription(String, "eezybot/obstacle_size", self._on_obstacle_size, 10)
        self.create_subscription(String, "eezybot/cmd", self._on_cmd, 10)
        self.create_subscription(PointStamped, "clicked_point", self._on_click, 10)
        self.create_service(Empty, "eezybot/clear_obstacles", self._on_clear)
        self.create_service(SetBool, "eezybot/auto_drop", self._on_auto_drop)

        # ---- Robot: giả lập hoặc tay thật (cả hai tự chạy ở luồng nền riêng) ----
        if self.real:
            self.link = RealArm(port, baud, log=self._log_from_arm)
            self.link.set_req_handler(self._on_req)
            self.create_timer(1.0 / REAL_RATE_HZ, self.link.tick)
        else:
            self.link = FakeRobotLink(speed=self.speed, verbose=False)
            self.link.set_req_handler(self._on_req)
            self.link.send(f"PCWAIT {PC_WAIT_MS}")
        threading.Thread(target=self._warm_up, daemon=True).start()

        # Timer chỉ ĐỌC trạng thái rồi phát topic - không gọi hàm chặn nào trong callback
        self.create_timer(1.0 / RATE_HZ, self._tick)
        if self.real:
            self.get_logger().info(
                f"TAY THAT qua {port} ({baud} baud): planner={planner_name}, le an toan={self.margin:g} mm. "
                "Tu the trong RViz la UOC LUONG. Dang tinh san luoi cho planner...")
            self.get_logger().warn(
                "Node KHONG co camera: tay that chi ne nhung vat can ban khai bao trong RViz. "
                "Khong khai bao vat can nao thi REQ duoc tra loi CLEAR.")
        else:
            self.get_logger().info(
                f"Mo phong EEZYbotARM: planner={planner_name}, speed={self.speed:g}, le an toan={self.margin:g} mm, "
                f"tu tha={'BAT' if self.auto_drop else 'TAT'}. Dang tinh san luoi cho planner...")

    def _log_from_arm(self, level, text):
        # Mỗi mức log phải gọi ở MỘT DÒNG RIÊNG: rclpy nhớ mức log theo dòng gọi, đổi mức trên cùng 1 dòng
        # là ném ValueError("Logger severity cannot be changed between calls") và làm chết node.
        log = self.get_logger()
        if level == "error":
            log.error(text)
        elif level == "warn":
            log.warn(text)
        else:
            log.info(text)

    # ------------------------------------------------------------------
    # Planner
    # ------------------------------------------------------------------
    def _warm_up(self):
        """Tính sẵn lưới A* và các đoạn cố định (vài giây) để lần REQ đầu tiên cũng trả lời kịp."""
        t0 = time.perf_counter()
        try:
            with self._plan_lock:
                ap.warm_up()
                aa.warm_up()
        except Exception as e:  # noqa: BLE001 - lỗi ở đây không được làm chết node
            self.get_logger().error(f"warm_up loi: {type(e).__name__}: {e}")
        self.ready = True
        self.get_logger().info(f"Planner san sang sau {time.perf_counter() - t0:.1f} s.")

    def _on_req(self, bin_id):
        """Robot hỏi đường "REQ <khay>". Hàm này chạy ở LUỒNG CỦA link (không phải luồng ROS).
        Không bao giờ ném lỗi: planner lỗi -> BLOCKED (không trả lời = robot tự đi đường mặc định)."""
        t0 = time.perf_counter()
        with self._lock:
            obstacles = list(self.obstacles)
        try:
            with self._plan_lock:
                p = self.planner.plan(bin_id, obstacles, self.obs_height, self.margin, self.z_margin)
        except Exception as e:  # noqa: BLE001
            p = ap.Plan(protocol.BLOCKED, protocol.BLOCKED, [], f"LOI PLANNER ({type(e).__name__}: {e}) -> BLOCKED",
                        [], [], 0, (time.perf_counter() - t0) * 1000.0, "PC")
        if self.real:
            self.link.note_plan(bin_id, p)   # Tư thế ước lượng sẽ phát lại đúng kế hoạch này
        self.link.send(p.reply)
        ms = (time.perf_counter() - t0) * 1000.0
        late = ms > self.link.pc_timeout_ms
        with self._lock:
            self.plan = p
            self.last_req = {"bin": bin_id, "kind": p.kind, "algo": p.algo, "ms": round(ms, 1), "reason": p.reason,
                             "reply": p.reply, "n_obstacles": len(obstacles), "late": late}
        text = f"REQ {bin_id} -> {p.reply}  [{p.algo}, {ms:.0f} ms, {len(obstacles)} vat can] {p.reason}"
        if late:
            self.get_logger().warn(text + " | TRA LOI MUON: robot da tu di duong mac dinh")
        elif p.kind == protocol.BLOCKED and (bin_id, p.reason) == self._last_blocked:
            pass   # Bị chặn thì robot hỏi lại mỗi 1.5 s: chỉ in lần đầu, không in lặp lại
        else:
            self.get_logger().info(text)
        self._last_blocked = (bin_id, p.reason) if p.kind == protocol.BLOCKED else None

    # ------------------------------------------------------------------
    # Thả sản phẩm
    # ------------------------------------------------------------------
    def _start_drop(self, bin_id):
        """Thả 1 sản phẩm ở đầu máng; SLIDE_MS sau nó tới điểm gắp và cảm biến màu thấy nó."""
        if self.chute is not None or self.link.fruit_bin is not None:
            return False
        self.chute = (bin_id, time.monotonic())
        return True

    def _on_drop(self, msg):
        bin_id = msg.data if msg.data != 0 else random.randint(1, 3)
        if bin_id not in rm.BINS:
            self.get_logger().warn(f"/eezybot/drop: khay phai la 1, 2, 3 (0 = ngau nhien), nhan {msg.data}")
        elif self.real:
            # Tay thật: không có sản phẩm ảo để thả -> chạy 1 chu trình sang khay đó bằng lệnh RUN của firmware
            self._send_cmd(f"RUN {bin_id}")
        elif self._start_drop(bin_id):
            self.get_logger().info(f"Tha san pham cho khay {bin_id}")
        else:
            self.get_logger().warn("Mang dang co san pham -> bo qua lenh tha")

    def _on_auto_drop(self, request, response):
        if self.real:    # Tay thật: bật / tắt chu trình phân loại theo cảm biến màu của firmware
            response.success = self._send_cmd("AUTO ON" if request.data else "AUTO OFF")
            response.message = "AUTO " + ("ON" if request.data else "OFF") + ("" if response.success else ": KHONG GUI DUOC")
            return response
        self.auto_drop = bool(request.data)
        response.success = True
        response.message = "Tu tha: " + ("BAT" if self.auto_drop else "TAT")
        self.get_logger().info(response.message)
        return response

    def _send_cmd(self, cmd):
        ok = self.link.send(cmd)
        if ok:
            self.get_logger().info(f"PC -> robot: {cmd}")
        else:
            self.get_logger().warn(f"Khong gui duoc '{cmd}' (chua ket noi robot)")
        return ok

    def _on_cmd(self, msg):
        cmd = msg.data.strip()
        if cmd:
            self._send_cmd(cmd)

    def _update_chute(self, holding):
        if self.real:
            return       # Tay thật: sản phẩm do cảm biến màu của firmware phát hiện, không thả ảo
        if self.chute is not None:
            bin_id, t0 = self.chute
            if (time.monotonic() - t0) * 1000.0 >= SLIDE_MS / max(self.speed, 0.01):
                self.chute = None
                self.link.put_fruit(bin_id)      # Sản phẩm đã nằm ở điểm gắp
        elif self.auto_drop and self.ready and not holding and self.link.fruit_bin is None:
            # Tay vừa nhả sản phẩm trước (hoặc đang rảnh) -> thả sản phẩm kế tiếp
            if self._start_drop(self.next_bin):
                self.next_bin = self.next_bin % 3 + 1

    # ------------------------------------------------------------------
    # Vật cản
    # ------------------------------------------------------------------
    def _add_obstacle(self, x_mm, y_mm, r_mm):
        ob = {"x_mm": x_mm, "y_mm": y_mm, "r_mm": r_mm, "dist_mm": math.hypot(x_mm, y_mm),
              "deg": math.degrees(math.atan2(y_mm, x_mm)), "source": "GIA LAP", "big": False}
        with self._lock:
            self.obstacles.append(ob)
            n = len(self.obstacles)
        self.get_logger().info(f"Them vat can: r={ob['dist_mm']:.0f} mm, goc={ob['deg']:.1f} do, "
                               f"ban kinh {r_mm:.0f} mm (tong {n}). Ap dung tu chu trinh ke tiep.")

    def _on_add_obstacle(self, msg):
        r_mm = sm.clamp_radius(msg.z * 1000.0) if msg.z > 0.0 else self.obs_radius
        self._add_obstacle(msg.x * 1000.0, msg.y * 1000.0, r_mm)

    def _set_obstacle_size(self, r_mm):
        self.obs_radius = sm.clamp_radius(r_mm)
        self.get_logger().info(f"Co vat can dang chon: {sm.size_name(self.obs_radius)} "
                               f"(ban kinh {self.obs_radius:.0f} mm). Ap dung cho vat can dat sau nay.")

    def _on_obstacle_size(self, msg):
        """"nho" / "vua" / "lon" / "rat_lon" hoặc bán kính bằng mm ("35")."""
        key = msg.data.strip().lower().replace(" ", "_")
        try:
            r_mm = SIZE_ALIASES[key] if key in SIZE_ALIASES else float(key)
            if not math.isfinite(r_mm) or r_mm <= 0.0:
                raise ValueError
        except ValueError:
            self.get_logger().warn(f"/eezybot/obstacle_size: can {' | '.join(SIZE_ALIASES)} hoac ban kinh (mm), "
                                   f"nhan '{msg.data}'")
            return
        self._set_obstacle_size(r_mm)

    def _on_click(self, msg):
        """Nút "Publish Point" của RViz: bấm 1 ô ở bảng chọn cỡ -> đổi cỡ; bấm vào vật cản -> xoá nó;
        bấm chỗ trống -> thêm vật cản cỡ đang chọn."""
        if msg.header.frame_id not in ("", sm.BASE_FRAME):
            self.get_logger().warn(f"Diem bam o khung '{msg.header.frame_id}', can '{sm.BASE_FRAME}' -> bo qua")
            return
        x_mm, y_mm = msg.point.x * 1000.0, msg.point.y * 1000.0
        picked = sm.palette_hit(x_mm, y_mm)
        if picked is not None:
            self._set_obstacle_size(picked)
            return
        with self._lock:
            hit = [ob for ob in self.obstacles if math.hypot(ob["x_mm"] - x_mm, ob["y_mm"] - y_mm) <= ob["r_mm"] + 5.0]
            if hit:
                self.obstacles.remove(hit[0])
        if hit:
            self.get_logger().info("Xoa 1 vat can.")
        else:
            self._add_obstacle(x_mm, y_mm, self.obs_radius)

    def _on_clear(self, _request, response):
        with self._lock:
            self.obstacles = []
        self.get_logger().info("Xoa het vat can.")
        return response

    # ------------------------------------------------------------------
    # Vòng lặp 30 Hz
    # ------------------------------------------------------------------
    def _tick(self):
        (q1, q2, q3), grip = self.link.pose()
        st = self.link.status()
        holding = st["holding"]
        stamp = self.get_clock().now()

        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.name = list(joint_math.JOINT_NAMES)
        js.position = joint_math.joint_positions(q1, q2, q3, grip, holding)
        self.pub_js.publish(js)

        tip = rm.fk(q1, q2, q3)
        if not self.trail or math.dist(tip, self.trail[-1]) > 3.0:
            self.trail.append(tip)
        self._update_chute(holding)

        self._ticks += 1
        if self._ticks % 3 == 0:      # 10 Hz
            self._publish_markers(st)
        if self._ticks % 6 == 0:      # 5 Hz
            self._publish_status(st, (q1, q2, q3), grip, tip, stamp.nanoseconds)

    def _publish_markers(self, st):
        with self._lock:
            obstacles, plan, req = list(self.obstacles), self.plan, self.last_req
        markers = sm.workspace_markers()

        fruit = self.link.fruit_bin
        if self.chute is not None:
            bin_id, t0 = self.chute
            k = (time.monotonic() - t0) * 1000.0 / (SLIDE_MS / max(self.speed, 0.01))
            markers.append(sm.cube_on_chute(bin_id, k))
        elif fruit is not None and not st["holding"]:
            markers.append(sm.cube_on_chute(fruit, 1.0))
        if st["holding"] and st["target_bin"] in rm.BINS:
            markers.append(sm.cube_carried(st["target_bin"]))
        for bin_id in rm.BINS:
            n = self.link.delivered[bin_id]
            markers += sm.cubes_in_bin(bin_id, (n - 1) % 4 + 1 if n else 0)   # Đầy 4 khối thì coi như đổi khay mới

        for i, ob in enumerate(obstacles):
            markers += sm.obstacle_markers(i, ob, self.obs_height, self.margin)
        markers += sm.palette_markers(self.obs_radius)
        markers += sm.plan_markers(plan)
        markers += sm.trail_marker(list(self.trail))

        d = self.link.delivered
        lines = [f"{STATE_TEXT.get(st['state'], st['state'])}" + (f" | {st['move']}" if st["move"] else ""),
                 f"Da tha: Khay1={d[1]}  Khay2={d[2]}  Khay3={d[3]}   Vat can: {len(obstacles)}"]
        if self.real:
            lines.insert(0, f"TAY THAT {self.link.port} - tu the UOC LUONG" if self.link.connected
                         else f"TAY THAT {self.link.port}: {(self.link.err or 'dang mo cong...')[:60]}")
        if req is not None:
            lines.append(f"REQ {req['bin']} -> {req['kind']} ({req['algo']}, {req['ms']:.0f} ms)")
            lines.append(req["reason"][:70])
        elif not self.ready:
            lines.append("Dang chuan bi planner...")
        markers += sm.text_marker("\n".join(lines))
        self.pub_markers.publish(self.book.array(markers))

    def _publish_status(self, st, q, grip, tip, stamp_ns):
        with self._lock:
            req, n_obs = self.last_req, len(self.obstacles)
            obs = [[round(ob["x_mm"], 1), round(ob["y_mm"], 1), round(ob["r_mm"], 1)] for ob in self.obstacles]
        msg = String()
        msg.data = json.dumps({
            "state": st["state"], "move": st["move"], "holding": st["holding"], "target_bin": st["target_bin"],
            "auto": st["auto"], "ready": self.ready, "auto_drop": self.auto_drop,
            "robot": "that" if self.real else "gia_lap", "connected": self.link.connected if self.real else True,
            "obstacle_radius_mm": self.obs_radius,
            "delivered": {str(k): v for k, v in self.link.delivered.items()},
            "fruit_at_pick": self.link.fruit_bin, "chute_bin": self.chute[0] if self.chute else None,
            "n_obstacles": n_obs, "obstacles": obs, "req": req,
            "q_deg": [round(v, 4) for v in q], "grip_deg": round(grip, 2),
            "tcp_mm": [round(v, 4) for v in tip],
            "stamp_ns": stamp_ns,      # = header.stamp của /joint_states cùng nhịp (để đối chiếu với TF)
        })
        self.pub_status.publish(msg)

    def close(self):
        self.link.close()


def main():
    rclpy.init()
    node = SimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
