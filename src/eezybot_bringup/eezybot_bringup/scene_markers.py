"""
scene_markers.py - CÁC HÌNH VẼ ĐỘNG TRONG RVIZ (visualization_msgs/Marker)
-------------------------------------------------------------------------
Phần đứng yên (bàn, máng, khay) đã nằm trong URDF (eezybot_description/urdf/scene.xacro).
File này vẽ những thứ thay đổi theo thời gian: khối sản phẩm, vật cản, bảng chọn cỡ vật cản, đường đã lập,
vệt tâm kẹp, chữ.

Đơn vị: dự án dùng mm, RViz dùng mét -> mọi hàm nhận mm và tự đổi (MM = 0.001).
Kích thước khối sản phẩm và máng là ƯỚC LƯỢNG ĐỂ VẼ, phải giống scene.xacro.
"""

import math

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from eezybot_bringup import core  # noqa: F401 - thêm core/ vào sys.path
import robot_model as rm  # noqa: E402

MM = 0.001
BASE_FRAME = "base_link"

# Màu 3 khay (đỏ / vàng / xanh) - giống scene.xacro
BIN_RGB = {1: (0.898, 0.282, 0.302), 2: (0.961, 0.647, 0.141), 3: (0.275, 0.655, 0.345)}

# Khối sản phẩm và máng (mm) - giống scene.xacro
CUBE = 50.0
HALF = CUBE / 2.0
CH_TOP = rm.Z_PICK - HALF          # Mặt máng ở điểm gắp: tâm khối nằm đúng Z_PICK
SLOPE = math.radians(22.0)
S_FLAT = 45.0                      # Hết đoạn phẳng, bắt đầu dốc
S_START = 380.0                    # Đầu trên của máng


def _marker(ns, mid, mtype, rgba, scale, frame=BASE_FRAME, xyz_mm=(0.0, 0.0, 0.0), quat=(0.0, 0.0, 0.0, 1.0)):
    """Tạo 1 Marker. scale tính bằng mét, xyz_mm tính bằng mm."""
    m = Marker()
    m.header.frame_id = frame     # stamp = 0: RViz dùng phép đổi khung mới nhất
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (v * MM for v in xyz_mm)
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = quat
    m.scale.x, m.scale.y, m.scale.z = scale
    m.color.r, m.color.g, m.color.b, m.color.a = rgba
    return m


def _pt(x_mm, y_mm, z_mm):
    return Point(x=x_mm * MM, y=y_mm * MM, z=z_mm * MM)


def _cube(ns, mid, bin_id, frame, xyz_mm, quat=(0.0, 0.0, 0.0, 1.0)):
    return _marker(ns, mid, Marker.CUBE, BIN_RGB[bin_id] + (1.0,), (CUBE * MM,) * 3, frame, xyz_mm, quat)


# ---------------------------------------------------------------------------
# Khối sản phẩm
# ---------------------------------------------------------------------------
def cube_on_chute(bin_id, k):
    """Khối đang ở trên máng. k = 0 (vừa thả ở đầu máng) ... 1 (đã tới chặn cuối = điểm gắp).
    Vẽ trong khung "chute": +X chạy ngược dốc, gốc = hình chiếu điểm gắp lên mặt bàn."""
    k = min(max(k, 0.0), 1.0)
    e = k * k * (3.0 - 2.0 * k)                       # Trượt êm lúc đầu và lúc cuối
    s = (S_START - 40.0) * (1.0 - e)                  # Vị trí dọc máng (mm)
    tilt = min(max((s - S_FLAT) / 25.0, 0.0), 1.0) * SLOPE
    z = CH_TOP + max(0.0, s - S_FLAT) * math.tan(SLOPE) + HALF / math.cos(tilt)
    quat = (0.0, math.sin(-tilt / 2.0), 0.0, math.cos(-tilt / 2.0))   # Nghiêng theo máng (quay quanh Y)
    return _cube("cube_chute", 0, bin_id, "chute", (s, 0.0, z), quat)


def cube_carried(bin_id):
    """Khối đang bị kẹp: gắn cứng vào khung "tcp" (tâm khối = tâm kẹp, giống twin 3D trên web)."""
    m = _cube("cube_carry", 0, bin_id, "tcp", (0.0, 0.0, 0.0))
    m.frame_locked = True          # Đi theo khung tcp ở mọi khung hình, không chờ marker mới
    return m


def cubes_in_bin(bin_id, n):
    """n khối (tối đa 4) đã thả trong khay: 2 khối theo bán kính, chồng 2 tầng."""
    out = []
    for i in range(min(n, 4)):
        x = 30.0 if i % 2 else -30.0
        z = 4.0 + HALF + (i // 2) * CUBE
        out.append(_cube(f"cube_bin{bin_id}", i, bin_id, f"bin{bin_id}", (x, 0.0, z)))
    return out


# ---------------------------------------------------------------------------
# Vật cản (trụ đứng) + vành lề an toàn
# ---------------------------------------------------------------------------
def obstacle_markers(i, ob, obs_height_mm, margin_mm):
    h = 420.0 if obs_height_mm > 1000.0 else obs_height_mm   # "Cao vô hạn": vẽ trụ 420 mm
    d = 2.0 * ob["r_mm"] * MM
    body = _marker("obstacle", i, Marker.CYLINDER, C_OBSTACLE + (0.75,), (d, d, h * MM),
                   xyz_mm=(ob["x_mm"], ob["y_mm"], h / 2.0))
    dm = 2.0 * (ob["r_mm"] + margin_mm) * MM
    ring = _marker("obstacle_margin", i, Marker.CYLINDER, (0.961, 0.620, 0.043, 0.30), (dm, dm, 0.002),
                   xyz_mm=(ob["x_mm"], ob["y_mm"], 1.5))
    return [body, ring]


# ---------------------------------------------------------------------------
# Bảng chọn cỡ vật cản: 4 ô tròn vẽ trên mặt bàn, bấm bằng nút "Publish Point" của RViz
# ---------------------------------------------------------------------------
# (tên, bán kính mm). 28 mm là cỡ vật cản ảo mặc định của 06_auto_run.py; camera thật nhận tối đa 62 mm.
OBSTACLE_SIZES = (("NHO", 15.0), ("VUA", 28.0), ("LON", 45.0), ("RAT LON", 60.0))
OBSTACLE_R_MIN, OBSTACLE_R_MAX = 5.0, 150.0     # Giới hạn khi nhập bán kính bằng số
PALETTE_Y = 470.0        # Hàng ô nằm sát mép bàn phía trước, ngoài tầm với của tay (R_MAX = 380)
PALETTE_X_END = -50.0    # Mép phải của ô nhỏ nhất; các ô lớn dần về phía -X
PALETTE_GAP = 30.0       # Khe giữa 2 ô
C_OBSTACLE = (0.486, 0.227, 0.929)
C_SELECT = (0.961, 0.420, 0.043)


def clamp_radius(r_mm):
    return min(max(float(r_mm), OBSTACLE_R_MIN), OBSTACLE_R_MAX)


def size_name(r_mm):
    """Tên cỡ ứng với bán kính (ví dụ "VUA"), hoặc "TUY CHON" nếu không trùng cỡ nào."""
    for name, r in OBSTACLE_SIZES:
        if abs(r - r_mm) < 0.05:
            return name
    return "TUY CHON"


def palette_slots():
    """[(tên, bán kính, x, y)] của từng ô, tính bằng mm trong khung base_link."""
    out, edge = [], PALETTE_X_END
    for name, r in OBSTACLE_SIZES:
        out.append((name, r, edge - r, PALETTE_Y))
        edge -= 2.0 * r + PALETTE_GAP
    return out


def palette_hit(x_mm, y_mm):
    """Điểm bấm rơi vào ô nào thì trả về bán kính của cỡ đó; không trúng ô nào -> None.
    Ô nhỏ cũng nhận vùng bấm rộng tối thiểu 22 mm cho dễ trúng."""
    for _name, r, x, y in palette_slots():
        if math.hypot(x_mm - x, y_mm - y) <= max(r, 22.0) + 5.0:
            return r
    return None


def palette_markers(selected_r_mm):
    """Vẽ bảng chọn cỡ: ô đang chọn đậm màu, có viền cam; các ô khác mờ."""
    out = []
    for i, (name, r, x, y) in enumerate(palette_slots()):
        chosen = abs(r - selected_r_mm) < 0.05
        d = 2.0 * r * MM
        out.append(_marker("size_palette", i, Marker.CYLINDER, C_OBSTACLE + ((0.95 if chosen else 0.30),),
                           (d, d, 0.004), xyz_mm=(x, y, 2.0)))
        ring = _marker("size_palette_ring", i, Marker.LINE_STRIP,
                       (C_SELECT if chosen else C_OBSTACLE) + (1.0,), (0.005 if chosen else 0.0015, 0.0, 0.0))
        ring.points = [_pt(x + (r + 3.0) * math.cos(a), y + (r + 3.0) * math.sin(a), 5.0)
                       for a in (2.0 * math.pi * k / 40.0 for k in range(41))]
        out.append(ring)
        label = _marker("size_palette_text", i, Marker.TEXT_VIEW_FACING,
                        (C_SELECT if chosen else (0.25, 0.28, 0.35)) + (1.0,), (0.0, 0.0, 0.020 if chosen else 0.016),
                        xyz_mm=(x, y, 38.0))
        label.text = f"{name}\nr={r:.0f}"
        out.append(label)
    slots = palette_slots()
    mid_x = (slots[0][2] + slots[-1][2]) / 2.0
    title = _marker("size_palette_text", 100, Marker.TEXT_VIEW_FACING, (0.11, 0.14, 0.20, 1.0), (0.0, 0.0, 0.017),
                    xyz_mm=(mid_x, PALETTE_Y, 92.0))
    title.text = (f"CO VAT CAN - dang chon: {size_name(selected_r_mm)} (r={selected_r_mm:.0f} mm)\n"
                  "Publish Point: bam 1 o de doi co, bam mat ban de dat vat can")
    out.append(title)
    return out


# ---------------------------------------------------------------------------
# Đường đã lập (Plan của planner), vệt tâm kẹp, chữ trạng thái
# ---------------------------------------------------------------------------
def plan_markers(plan):
    """Đường tâm kẹp của từng đoạn trong chu trình: đỏ = chạm vật cản, xanh lá = đang mang sản phẩm,
    xanh ngọc = tay không. Điểm né = chấm tím."""
    out = []
    if plan is None:
        return out
    for i, res in enumerate(plan.results):
        path = res["tip_path"]
        if len(path) < 2:
            continue
        if res["hit"]:
            rgba, width = (0.863, 0.149, 0.149, 1.0), 0.004
        elif res["move"].carrying:
            rgba, width = (0.086, 0.639, 0.290, 0.9), 0.003
        else:
            rgba, width = (0.055, 0.647, 0.643, 0.9), 0.002
        m = _marker("plan", i, Marker.LINE_STRIP, rgba, (width, 0.0, 0.0))
        m.points = [_pt(*p) for p in path]
        out.append(m)
    if plan.waypoints:
        m = _marker("plan_waypoints", 0, Marker.SPHERE_LIST, (0.753, 0.149, 0.827, 1.0), (0.014, 0.014, 0.014))
        m.points = [_pt(*rm.polar_to_xyz(r, deg, z)) for r, deg, z in plan.waypoints]
        out.append(m)
    return out


def trail_marker(points_mm):
    """Vệt quỹ đạo tâm kẹp (các điểm (x, y, z) mm gần đây)."""
    if len(points_mm) < 2:
        return []
    m = _marker("trail", 0, Marker.LINE_STRIP, (0.145, 0.388, 0.922, 0.6), (0.002, 0.0, 0.0))
    m.points = [_pt(*p) for p in points_mm]
    return [m]


def text_marker(text, xyz_mm=(100.0, 150.0, 470.0), size_m=0.022, mid=0):
    """Chữ luôn quay về phía camera. Viết KHÔNG DẤU: phông mặc định của RViz không chắc có đủ dấu tiếng Việt."""
    m = _marker("text", mid, Marker.TEXT_VIEW_FACING, (0.11, 0.14, 0.20, 1.0), (0.0, 0.0, size_m), xyz_mm=xyz_mm)
    m.text = text
    return [m]


def workspace_markers():
    """Viền vùng làm việc (quạt 55.1°..126.7°, bán kính R_MIN..R_MAX) và cung R_PATH nét đứt trên mặt bàn."""
    lo, hi = rm.LIM_BASE
    degs = [lo + (hi - lo) * i / 48.0 for i in range(49)]
    outer = [_pt(*rm.polar_to_xyz(rm.R_MAX, d, 1.0)) for d in degs]
    inner = [_pt(*rm.polar_to_xyz(rm.R_MIN, d, 1.0)) for d in reversed(degs)]
    fan = _marker("workspace", 0, Marker.LINE_STRIP, (0.145, 0.388, 0.922, 0.55), (0.002, 0.0, 0.0))
    fan.points = outer + inner + [outer[0]]
    arc = _marker("workspace", 1, Marker.LINE_LIST, (0.145, 0.388, 0.922, 0.8), (0.002, 0.0, 0.0))
    arc_pts = [_pt(*rm.polar_to_xyz(rm.R_PATH, d, 1.0)) for d in degs]
    for a, b in zip(arc_pts[0::2], arc_pts[1::2]):   # LINE_LIST: từng cặp điểm là 1 nét -> thành nét đứt
        arc.points += [a, b]
    return [fan, arc]


# ---------------------------------------------------------------------------
# Gom marker thành MarkerArray, tự xoá những marker không còn dùng
# ---------------------------------------------------------------------------
class MarkerBook:
    """RViz giữ marker cũ cho tới khi có lệnh DELETE. Lớp này nhớ các marker đã gửi lần trước;
    cái nào lần này không còn (ví dụ vật cản vừa bị xoá) thì gửi kèm lệnh DELETE cho nó."""

    def __init__(self):
        self._prev = {}   # (ns, id) -> frame_id

    def array(self, markers):
        arr = MarkerArray()
        cur = {}
        for m in markers:
            cur[(m.ns, m.id)] = m.header.frame_id
            arr.markers.append(m)
        for (ns, mid), frame in self._prev.items():
            if (ns, mid) not in cur:
                d = Marker()
                d.header.frame_id = frame
                d.ns, d.id, d.action = ns, mid, Marker.DELETE
                arr.markers.append(d)
        self._prev = cur
        return arr
