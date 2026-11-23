"""
robot_model.py - MÔ HÌNH ĐỘNG HỌC CÁNH TAY EEZYbotARM MK2 (BẢN PYTHON CỦA KINEMATICS.ino)
--------------------------------------------------------------------------------------
!!! MỌI HẰNG SỐ Ở ĐÂY PHẢI GIỐNG HỆT KINEMATICS/KINEMATICS.ino !!!
Nếu sửa một bên thì phải sửa bên kia, rồi chạy:  python robot_model.py  để tự kiểm tra.

HỆ TỌA ĐỘ ROBOT (dùng chung cho firmware, camera và thuật toán né):
  - Gốc (0, 0) = tâm trục xoay đế, trục Z hướng lên, đơn vị mm.
  - Góc đế q1 (độ) = góc THỰC trên mặt bàn: x = r*cos(q1), y = r*sin(q1).
  - Đế có bánh răng giảm tốc: servo 0° -> 180° chỉ quay được ~72° thực, KHÔNG đối xứng quanh 90°.
      servo   0° = Điểm Gắp (55.1°) | servo 90° = 90° (HOME) | servo 180° = Khay 1 (126.7°)
      Mô hình 3 mốc đo bằng 07 [GOC DE]: nội suy thẳng từng nửa 0->90 và 90->180.
  - Mọi đường đi ngang đều là CUNG TRÒN quanh đế: (bán kính r, góc q1, độ cao z).
    Khi hai đầu đoạn đi có cùng (r, z) thì nội suy khớp chỉ quay đế -> đúng một cung tròn.

KHỚP & GIỚI HẠN CƠ KHÍ AN TOÀN:
  - Đế      : servo [0°, 180°]  = q1 thực [55.1°, 126.7°]
  - Vai     : [20°, 160°]   (giới hạn tính theo GÓC SERVO q2, q3)
  - Khuỷu   : [ 0°,  75°]
  - Tay kẹp : [35°, 140°] (mở 40°, kẹp chặt 140° - servo lắp ngược)

QUY ĐỔI GÓC SERVO -> GÓC THẬT CỦA KHÂU (so với phương ngang, đo bằng fit_geometry.py):
  - Tay trên (L1): th2 = SHOULDER_GAIN * q2 + SHOULDER_OFFSET_DEG
  - Cẳng tay (L2): phi = ELBOW_COUPLING * th2 + ELBOW_GAIN * q3 + ELBOW_OFFSET_DEG
      ELBOW_COUPLING = 1 -> góc khuỷu tương đối so với tay trên; = 0 -> cơ cấu hình bình hành
      giữ góc cẳng tay tuyệt đối (không đổi khi chỉ quay vai).
  - Giá trị mặc định (1, 0, 1, -1, 0) cho đúng mô hình cũ: phi = q2 - q3.
"""

import math
from dataclasses import dataclass

# =========================================================================
# 1. THÔNG SỐ HÌNH HỌC (mm) - GIỐNG KINEMATICS.ino
# =========================================================================
# Số đo thật 28-29/10/2026 (do_chieu_dai.csv, do_goc.csv -> python fit_geometry.py),
# khớp 5 tư thế đo với sai số trung bình ~11mm.
L1 = 142.0      # Chốt vai -> chốt khuỷu
L2 = 160.0      # Chốt khuỷu -> chốt cổ tay (cuối cẳng tay)
Z_BASE = 90.0   # Độ cao chốt vai so với mặt bàn
R_OFF = 6.0     # Khoảng cách ngang từ tâm trục đế tới chốt vai (+ = chốt vai lệch về phía tay kẹp)
# Cụm kẹp MK2 được hình bình hành giữ luôn NẰM NGANG -> tâm kẹp lệch một đoạn CỐ ĐỊNH so với cổ tay
TOOL_R = 84.775  # Ngang: chốt cổ tay -> tâm kẹp (+ = ra phía trước)
TOOL_Z = -4.640  # Dọc: chốt cổ tay -> tâm kẹp (+ = cao hơn cổ tay)

# Quy đổi góc servo -> góc thật của khâu (xem docstring đầu file)
# Vai: servo tăng -> tay trên ngả về phía trước. Khuỷu: góc cẳng tay tuyệt đối (hình bình hành).
SHOULDER_GAIN = -0.850
SHOULDER_OFFSET_DEG = 157.671
ELBOW_COUPLING = 0.024
ELBOW_GAIN = 0.901
ELBOW_OFFSET_DEG = -53.721

R_MIN = 70.0
R_MAX = 380.0   # Tâm kẹp vươn xa nhất ~383mm
D_MIN = 100.0   # Chốt vai -> cổ tay ngắn nhất (tư thế gập nhất đã đo: ~121mm)

# =========================================================================
# 2. ĐẾ CÓ BÁNH RĂNG GIẢM TỐC: 3 MỐC servo 0° / 90° / 180° -> góc thật
# =========================================================================
# Đo 01/11/2026 bằng 07 [GOC DE] (đã bù rơ). Servo 90° = 90° theo định nghĩa (HOME).
# Góc thật ở 2 đầu là góc tới được khi đi RA phía đầu đó (đi xuống servo 0°, đi lên servo 180°).
# Nếu servo lắp ngược chiều thì chỉ cần đổi chỗ 2 số (servo 0° -> góc lớn).
BASE_DEG_AT_SERVO0 = 55.1    # Điểm gắp
BASE_DEG_AT_SERVO180 = 126.7  # Khay 1
BASE_DEG_MIN = min(BASE_DEG_AT_SERVO0, BASE_DEG_AT_SERVO180)
BASE_DEG_MAX = max(BASE_DEG_AT_SERVO0, BASE_DEG_AT_SERVO180)
BASE_SPAN_DEG = BASE_DEG_MAX - BASE_DEG_MIN  # Chỉ để hiển thị (71.6°)


def base_servo_to_deg(servo_deg):
    """Góc servo đế -> góc thật (tâm bánh răng), nội suy thẳng từng nửa qua 3 mốc."""
    if servo_deg <= 90.0:
        return 90.0 + (BASE_DEG_AT_SERVO0 - 90.0) * (90.0 - servo_deg) / 90.0
    return 90.0 + (BASE_DEG_AT_SERVO180 - 90.0) * (servo_deg - 90.0) / 90.0


def _base_knots_to_servo(q1_deg, k0, k180):
    """Ngược của đường gấp khúc (0, k0) - (90, 90) - (180, k180)."""
    if (q1_deg - 90.0) * (k0 - 90.0) >= 0.0:
        return 90.0 - 90.0 * (q1_deg - 90.0) / (k0 - 90.0)
    return 90.0 + 90.0 * (q1_deg - 90.0) / (k180 - 90.0)


def base_deg_to_servo(q1_deg):
    """Góc thật -> góc servo đế (không bù rơ, không kẹp 0..180)."""
    return _base_knots_to_servo(q1_deg, BASE_DEG_AT_SERVO0, BASE_DEG_AT_SERVO180)


def base_deg_to_servo_dir(q1_deg, direction, backlash_deg):
    """Giống baseDegToServoDir() trong firmware: góc servo THẬT SỰ được ghi khi đế đang đi theo
    chiều direction (+1 góc thật tăng, -1 giảm, 0 chưa biết) với bù rơ backlash_deg (độ thật).
    Tâm bánh răng lệch ra ngoài b/2 ở 2 đầu -> đi RA phía đầu nào thì tới đúng servo 0° / 180°."""
    h = 0.5 * backlash_deg
    k0 = BASE_DEG_AT_SERVO0 + (-h if BASE_DEG_AT_SERVO0 < 90.0 else h)
    k180 = BASE_DEG_AT_SERVO180 + (-h if BASE_DEG_AT_SERVO180 < 90.0 else h)
    s = _base_knots_to_servo(q1_deg + direction * h, k0, k180)
    return min(max(s, 0.0), 180.0)


LIM_BASE = (BASE_DEG_MIN, BASE_DEG_MAX)
LIM_SHOULDER = (20.0, 160.0)
LIM_ELBOW = (0.0, 75.0)
LIM_GRIPPER = (35.0, 140.0)

GRIPPER_OPEN = 40.0
GRIPPER_CLOSE = 140.0

# =========================================================================
# 3. CUNG ĐƯỜNG ĐI & CÁC VỊ TRÍ (GIỐNG KINEMATICS.ino)
# =========================================================================
R_PATH = 300.0      # Bán kính cung đường gắp/thả (mm) - quả và khay đặt cách tâm đế 30cm
Z_PICK = 120.0      # Độ cao tâm kẹp khi kẹp quả (12cm)
Z_APPROACH = 160.0  # Từ đây hạ chậm xuống Z_PICK
Z_TRAVEL = 190.0    # Độ cao mang quả đi theo cung (đáy quả ~150mm)
Z_BIN = 150.0       # Độ cao thả vào khay (15cm)

DEG_PICK = base_servo_to_deg(0.0)  # servo 0°
BINS = {
    1: {"name": "KHAY 1 - CHIN (DO)", "deg": base_servo_to_deg(180.0), "color": (0, 0, 255)},     # servo 180°
    2: {"name": "KHAY 2 - VUA (VANG)", "deg": base_servo_to_deg(150.0), "color": (0, 215, 255)},  # servo 150°
    3: {"name": "KHAY 3 - XANH", "deg": base_servo_to_deg(120.0), "color": (0, 255, 0)},          # servo 120°
}

POSE_HOME = (90.0, 60.0, 40.0)  # (q1, q2, q3) độ

# Thời gian đoạn đi qua điểm né (legMs() trong firmware): lớn nhất của (độ dài đoạn trong (r, cung, z) x
# LEG_MS_PER_MM, góc đế phải quay x LEG_MS_PER_DEG, LEG_MS_MIN) -> điểm né gần đế không làm đế quay vụt
LEG_MS_PER_MM = 10.0
LEG_MS_PER_DEG = 30.0
LEG_MS_MIN = 700

# Thông số dùng khi kiểm tra va chạm
PAYLOAD_DROP_MM = 40.0  # Quả treo thấp hơn tâm kẹp bao nhiêu mm khi đang kẹp


# =========================================================================
# 4. ĐỘNG HỌC THUẬN / NGHỊCH
# =========================================================================
def link_angles(q2, q3):
    """Góc servo (q2, q3) -> góc thật (th2, phi) của tay trên và cẳng tay so với phương ngang (độ)."""
    th2 = SHOULDER_GAIN * q2 + SHOULDER_OFFSET_DEG
    phi = ELBOW_COUPLING * th2 + ELBOW_GAIN * q3 + ELBOW_OFFSET_DEG
    return th2, phi


def servo_angles(th2, phi):
    """Ngược lại của link_angles: góc thật (th2, phi) -> góc servo (q2, q3)."""
    q2 = (th2 - SHOULDER_OFFSET_DEG) / SHOULDER_GAIN
    q3 = (phi - ELBOW_COUPLING * th2 - ELBOW_OFFSET_DEG) / ELBOW_GAIN
    return q2, q3


def arm_rz(q2, q3):
    """Vị trí (r, z) của chốt vai, chốt khuỷu, chốt cổ tay và tâm kẹp trong mặt phẳng đứng của tay."""
    th2, phi = (math.radians(a) for a in link_angles(q2, q3))
    shoulder = (R_OFF, Z_BASE)
    elbow = (R_OFF + L1 * math.cos(th2), Z_BASE + L1 * math.sin(th2))
    wrist = (elbow[0] + L2 * math.cos(phi), elbow[1] + L2 * math.sin(phi))
    tip = (wrist[0] + TOOL_R, wrist[1] + TOOL_Z)
    return shoulder, elbow, wrist, tip


def fk(q1, q2, q3):
    """(q1, q2, q3) độ -> (x, y, z) mm của tâm kẹp."""
    r, z = arm_rz(q2, q3)[-1]
    a1 = math.radians(q1)
    return r * math.cos(a1), r * math.sin(a1), z


def ik(x, y, z):
    """Giống hệt InverseKinematics::compute() trong firmware.
    Trả về (ok, (q1, q2, q3), thong_bao_loi).
    Giải bài toán 2 khâu cho (th2, phi), thử nhánh khuỷu-trên trước rồi khuỷu-dưới,
    đổi sang góc servo và kiểm tra giới hạn servo."""
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return False, None, "Toa do khong hop le"  # NaN/inf lọt qua mọi phép so sánh bên dưới
    if y < -0.05:
        return False, None, "Y phai >= 0"

    q1 = math.degrees(math.atan2(y, x))
    if q1 < 0.0:
        q1 += 360.0
    if q1 > 180.0:
        q1 = 360.0 - q1
    if q1 < LIM_BASE[0] - 0.25 or q1 > LIM_BASE[1] + 0.25:
        return False, None, f"Goc de {q1:.1f} ngoai [{LIM_BASE[0]:.1f}, {LIM_BASE[1]:.1f}]"
    q1 = min(max(q1, LIM_BASE[0]), LIM_BASE[1])

    r = math.hypot(x, y)
    r_rel = r - R_OFF - TOOL_R  # Vị trí cổ tay so với chốt vai
    z_rel = z - Z_BASE - TOOL_Z
    d = math.hypot(r_rel, z_rel)
    if d > (L1 + L2) or d < D_MIN or r < R_MIN or r > R_MAX:
        return False, None, f"Ngoai tam voi D={d:.1f}mm r={r:.1f}mm"

    cos_g = (d * d - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    gamma = math.acos(min(max(cos_g, -1.0), 1.0))  # Góc gập giữa 2 khâu
    alpha = math.atan2(z_rel, r_rel)
    beta = math.atan2(L2 * math.sin(gamma), L1 + L2 * math.cos(gamma))

    err = ""
    for sign in (1.0, -1.0):  # 1 = khuỷu ở trên, -1 = khuỷu ở dưới
        th2 = math.degrees(alpha + sign * beta)
        phi = th2 - sign * math.degrees(gamma)
        q2, q3 = servo_angles(th2, phi)
        if q2 < LIM_SHOULDER[0] or q2 > LIM_SHOULDER[1]:
            err = err or f"Goc vai {q2:.1f} ngoai [{LIM_SHOULDER[0]:.0f}, {LIM_SHOULDER[1]:.0f}]"
        elif q3 < LIM_ELBOW[0] or q3 > LIM_ELBOW[1]:
            err = err or f"Goc khuyu {q3:.1f} ngoai [{LIM_ELBOW[0]:.0f}, {LIM_ELBOW[1]:.0f}]"
        else:
            return True, (q1, q2, q3), ""
    return False, None, err


def polar_to_xyz(r, deg, z):
    rad = math.radians(deg)
    return r * math.cos(rad), r * math.sin(rad), z


def xyz_to_polar(x, y, z):
    return math.hypot(x, y), math.degrees(math.atan2(y, x)), z


def ik_polar(r, deg, z):
    if not (math.isfinite(r) and math.isfinite(deg) and math.isfinite(z)):
        return False, None, "Toa do khong hop le"  # math.cos(inf) sẽ ném ValueError
    return ik(*polar_to_xyz(r, deg, z))


def reach_band(z, deg=90.0, step=0.5):
    """Khoảng bán kính [r_min, r_max] mà IK chấp nhận ở độ cao z. None nếu không có."""
    ok_r = []
    r = 0.0
    while r <= R_MAX + 1e-6:
        if ik_polar(r, deg, z)[0]:
            ok_r.append(r)
        r += step
    if not ok_r:
        return None
    return ok_r[0], ok_r[-1]


# =========================================================================
# 5. CHU TRÌNH GẮP - THẢ (GIỐNG MÁY TRẠNG THÁI TRONG FIRMWARE)
# =========================================================================
@dataclass
class Move:
    name: str
    a: tuple            # (q1, q2, q3) đầu đoạn
    b: tuple            # (q1, q2, q3) cuối đoạn
    carrying: bool      # Đang kẹp quả?
    pa: tuple = None    # (r, góc, z) đầu đoạn - chỉ có khi đi thẳng (moveArcLinear)
    pb: tuple = None    # (r, góc, z) cuối đoạn
    ms: int = 1000      # Thời gian đoạn đi (giống firmware)


def _ik_or_raise(r, deg, z, label):
    ok, q, msg = ik_polar(r, deg, z)
    if not ok:
        raise ValueError(f"{label} (r={r:.0f}, goc={deg:.1f}, z={z:.0f}): {msg}")
    return q


def _linear(name, pa, pb, carrying, ms):
    """Đoạn đi thẳng trong (r, góc, z) giống moveArcLinear() trong firmware."""
    return Move(name, _ik_or_raise(*pa, name), _ik_or_raise(*pb, name), carrying, pa, pb, ms)


def leg_ms(pa, pb):
    """Thời gian đi thẳng pa -> pb (giống legMs() trong firmware)."""
    r_mean = (pa[0] + pb[0]) / 2.0
    d_deg = abs(pb[1] - pa[1])
    arc = r_mean * math.radians(d_deg)
    dist = math.sqrt((pb[0] - pa[0]) ** 2 + arc ** 2 + (pb[2] - pa[2]) ** 2)
    return max(LEG_MS_MIN, int(dist * LEG_MS_PER_MM), int(d_deg * LEG_MS_PER_DEG))


# Các điểm cố định của chu trình
P_LIFT = (R_PATH, DEG_PICK, Z_TRAVEL)       # Ngay trên điểm gắp, ở độ cao mang quả
P_APPROACH = (R_PATH, DEG_PICK, Z_APPROACH)
P_PICK = (R_PATH, DEG_PICK, Z_PICK)
Q_HOME_ROT = (DEG_PICK, POSE_HOME[1], POSE_HOME[2])  # Tư thế HOME nhưng đế quay về góc gắp


def above_bin(bin_id):
    return (R_PATH, BINS[bin_id]["deg"], Z_TRAVEL)


def approach_moves():
    """HOME -> quay đế -> vươn tới trên điểm gắp -> hạ thẳng đứng -> kẹp -> nâng thẳng đứng.
    Ở HOME tâm kẹp ở r~204mm, z~180mm (thấp hơn Z_TRAVEL) -> đoạn quay đế là cung thấp nhất cố định;
    đoạn vươn ra nâng tâm kẹp z~180 -> ~193 -> Z_TRAVEL."""
    return [
        Move("XOAY DE TOI GAP", POSE_HOME, Q_HOME_ROT, False, ms=1500),
        Move("VUON TOI TREN GAP", Q_HOME_ROT, _ik_or_raise(*P_LIFT, "Tren diem gap"), False, ms=1000),
        _linear("HA NHANH", P_LIFT, P_APPROACH, False, 900),
        _linear("HA CHAM", P_APPROACH, P_PICK, False, 1200),
        _linear("NANG LEN", P_PICK, P_LIFT, True, 1200),
    ]


def transfer_moves(bin_id, waypoints=None):
    """Mang quả từ trên điểm gắp sang trên khay.
    - Không có điểm né: 1 cung tròn R_PATH (2000ms).
    - Có điểm né: P_LIFT -> wp1 -> ... -> wpn -> trên khay, mỗi đoạn đi thẳng trong (r, góc, z)."""
    if not waypoints:
        return [_linear("CUNG -> KHAY", P_LIFT, above_bin(bin_id), True, 2000)]
    pts = [P_LIFT] + [tuple(w) for w in waypoints] + [above_bin(bin_id)]
    moves = []
    for i in range(len(pts) - 1):
        name = f"NE {i + 1}" if i < len(waypoints) else "NE -> KHAY"
        moves.append(_linear(name, pts[i], pts[i + 1], True, leg_ms(pts[i], pts[i + 1])))
    return moves


def bin_moves(bin_id):
    p_above, p_bin = above_bin(bin_id), (R_PATH, BINS[bin_id]["deg"], Z_BIN)
    return [
        _linear("HA XUONG KHAY", p_above, p_bin, True, 800),
        _linear("NHAC KHOI KHAY", p_bin, p_above, False, 700),
    ]


def return_moves(bin_id, waypoints=None, direct_home=False):
    """Về HOME.
    - Không có điểm né, hoặc direct_home (PATH có "H"): nội suy khớp thẳng từ trên khay về HOME (1800ms).
    - Có điểm né: đi NGƯỢC lại đúng các điểm né -> trên điểm gắp -> thu tay lên -> quay đế về HOME.
      Mọi đoạn đều trùng hình học với đoạn đã kiểm tra lúc đi, nên không cần kiểm tra va chạm lại."""
    if not waypoints or direct_home:
        return [Move("KHAY->HOME", _ik_or_raise(*above_bin(bin_id), "Tren khay"), POSE_HOME, False, ms=1800)]
    pts = [above_bin(bin_id)] + [tuple(w) for w in reversed(waypoints)] + [P_LIFT]
    moves = []
    for i in range(len(pts) - 1):
        name = f"VE NE {len(waypoints) - i}" if i < len(waypoints) else "VE TREN GAP"
        moves.append(_linear(name, pts[i], pts[i + 1], False, leg_ms(pts[i], pts[i + 1])))
    moves.append(Move("THU TAY LEN", _ik_or_raise(*P_LIFT, "Tren diem gap"), Q_HOME_ROT, False, ms=1000))
    moves.append(Move("XOAY VE HOME", Q_HOME_ROT, POSE_HOME, False, ms=1500))
    return moves


def cycle_moves(bin_id, waypoints=None, direct_home=False):
    """Các đoạn chuyển động của 1 chu trình (giống FSM trong firmware):
    quay đế ở trên cao -> vươn tới trên điểm gắp -> hạ thẳng đứng -> kẹp -> nâng thẳng đứng
    -> cung tròn (hoặc qua các điểm né) sang khay -> hạ -> thả -> nâng -> về HOME
    (direct_home: về thẳng HOME dù có điểm né, xem return_moves)."""
    return (approach_moves() + transfer_moves(bin_id, waypoints) + bin_moves(bin_id)
            + return_moves(bin_id, waypoints, direct_home))


def quintic(tau):
    """Hệ số nội suy bậc 5 (giống RobotArmController::quintic)."""
    if tau <= 0.0:
        return 0.0
    if tau >= 1.0:
        return 1.0
    return tau * tau * tau * (10.0 + tau * (-15.0 + 6.0 * tau))


def move_pose(move, s):
    """(q1, q2, q3) tại hệ số s ∈ [0, 1] của đoạn đi."""
    if move.pa is not None:
        p = tuple(a + (b - a) * s for a, b in zip(move.pa, move.pb))
        return _ik_or_raise(*p, move.name)
    return tuple(a + (b - a) * s for a, b in zip(move.a, move.b))


def sample_move(move, n=30):
    """Trả về danh sách (q1, q2, q3) dọc đoạn đi.
    - Đi thẳng (pa, pb): nội suy (r, góc, z) rồi tính IK từng điểm - giống moveArcLinear().
    - Đi theo khớp: firmware nội suy 3 khớp cùng hệ số s -> nội suy tuyến tính theo s.
    Điểm nào IK lỗi -> ValueError."""
    return [move_pose(move, i / float(n)) for i in range(n + 1)]


def arm_points(q, n_per_link=5):
    """Các điểm (x, y, z) dọc thân tay: vai -> khuỷu -> cổ tay -> tâm kẹp (điểm cuối = tâm kẹp).
    Chỉ dùng để VẼ (n_per_link=1: khuỷu, cổ tay, tâm kẹp). Kiểm tra va chạm đo trên cả ĐOẠN THẲNG
    (arm_joints + segment_clearance bên dưới), không dùng các điểm này."""
    a1 = math.radians(q[0])
    c1, s1 = math.cos(a1), math.sin(a1)
    joints = arm_rz(q[1], q[2])  # vai, khuỷu, cổ tay, tâm kẹp
    pts = []
    for (ar, az), (br, bz) in zip(joints[:-1], joints[1:]):
        for i in range(1, n_per_link + 1):
            t = i / float(n_per_link)
            r, z = ar + t * (br - ar), az + t * (bz - az)
            pts.append((r * c1, r * s1, z))
    return pts


def arm_joints(q):
    """4 khớp (x, y, z) của thân tay: chốt vai, chốt khuỷu, chốt cổ tay, tâm kẹp (điểm cuối = tâm kẹp).
    Thân tay = 3 ĐOẠN THẲNG nối liên tiếp các điểm này (cụm kẹp nằm ngang: cổ tay -> tâm kẹp)."""
    a1 = math.radians(q[0])
    c1, s1 = math.cos(a1), math.sin(a1)
    return [(r * c1, r * s1, z) for r, z in arm_rz(q[1], q[2])]


# ---- LUẬT VA CHẠM DÙNG CHUNG: check_collision (05, 06, sim_view, so_sanh), avoid_planner, astar_planner ----
# Vật cản = trụ đứng (tâm x_mm, y_mm, bán kính r_mm, cao obs_h). Thân tay = 3 đoạn thẳng 3D, cộng quả treo
# khi đang kẹp = đoạn đứng từ tâm kẹp xuống PAYLOAD_DROP_MM. Mỗi đoạn chỉ xét PHẦN nằm thấp hơn
# z_top = obs_h + z_margin (cắt đoạn thẳng tại mặt phẳng đó; phần cao hơn đi phía trên vật), rồi đo khoảng
# cách NGANG (2D) từ tâm trụ tới phần đó:      chạm  <=>  khoảng cách - r_mm < xy_margin
# Đo giải tích trên cả đoạn thẳng -> không lọt khe giữa các điểm như cách cũ (5 điểm / khâu, cách nhau ~30mm).
import numpy as np  # noqa: E402 - chỉ phần kiểm tra va chạm cần numpy

N_ARM_SEGS = 3  # vai->khuỷu, khuỷu->cổ tay, cổ tay->tâm kẹp; đoạn thứ 4 (nếu có) = quả treo


def pose_segments(joints, carrying=True):
    """joints (..., 4, 3) (từ arm_joints) -> các đoạn thẳng (..., S, 2, 3):
    S = 3 khâu thân tay, + 1 đoạn quả treo nếu carrying."""
    j = np.asarray(joints, dtype=float)
    segs = np.stack([j[..., :-1, :], j[..., 1:, :]], axis=-2)
    if not carrying:
        return segs
    tip = j[..., -1:, :]
    drop = tip.copy()
    drop[..., 2] -= PAYLOAD_DROP_MM
    return np.concatenate([segs, np.stack([tip, drop], axis=-2)], axis=-3)


def obstacle_arrays(obstacles):
    """list vật cản (dict x_mm, y_mm, r_mm) -> (tâm (N, 2), bán kính (N,)) numpy.
    Planner đổi 1 lần mỗi lần hỏi đường rồi truyền tuple này cho mọi hàm bên dưới (kể cả check_collision)."""
    if isinstance(obstacles, tuple) and len(obstacles) == 2 and isinstance(obstacles[0], np.ndarray):
        return obstacles  # Đã đổi sẵn (tuple vật cản dict / tuple rỗng thì vẫn đổi bên dưới)
    xy = np.array([(ob["x_mm"], ob["y_mm"]) for ob in obstacles], dtype=float).reshape(-1, 2)
    return xy, np.array([ob["r_mm"] for ob in obstacles], dtype=float)


def _seg_prep(segs, z_top):
    """Phần không phụ thuộc vật cản của mỗi đoạn a -> b: (ax, ay, dx, dy, 1/|d|², t0, t1, có_phần_thấp).
    Phần thấp (z <= z_top) của đoạn = tham số t trong [t0, t1]."""
    segs = np.asarray(segs, dtype=float)
    a, b = segs[..., 0, :], segs[..., 1, :]
    za, zb = a[..., 2], b[..., 2]
    low_a, low_b = za <= z_top, zb <= z_top
    dz = zb - za
    tc = np.minimum(np.maximum((z_top - za) / np.where(dz != 0.0, dz, 1.0), 0.0), 1.0)  # Chỗ cắt mặt z = z_top
    ax, ay = a[..., 0], a[..., 1]
    dx, dy = b[..., 0] - ax, b[..., 1] - ay
    dd = dx * dx + dy * dy
    inv = np.where(dd > 1e-12, 1.0 / np.where(dd > 1e-12, dd, 1.0), 0.0)  # Đoạn đứng (quả treo): chỉ 1 điểm (x, y)
    return ax, ay, dx, dy, inv, np.where(low_a, 0.0, tc), np.where(low_b, 1.0, tc), low_a | low_b


def _seg_t(p, cx, cy):
    """Tham số t của điểm trên phần thấp gần tâm (cx, cy) nhất (phép tính từng phần tử, broadcast được)."""
    ax, ay, dx, dy, inv, t0, t1 = p[:7]
    return np.minimum(np.maximum(((cx - ax) * dx + (cy - ay) * dy) * inv, t0), t1)


def _seg_clear(p, cx, cy, r):
    """Khoảng cách ngang từ tâm (cx, cy) tới phần thấp của đoạn, trừ bán kính r (từng phần tử)."""
    ax, ay, dx, dy = p[:4]
    t = _seg_t(p, cx, cy)
    ex = cx - ax - t * dx
    ey = cy - ay - t * dy
    return np.sqrt(ex * ex + ey * ey) - r


def segment_clearance(segs, obstacles, z_top):
    """segs (..., 2, 3) -> clear (..., N) với N vật cản: khoảng cách ngang từ tâm trụ tới phần đoạn thẳng
    có z <= z_top, trừ bán kính trụ (mm); +inf nếu cả đoạn cao hơn z_top."""
    c, rad = obstacle_arrays(obstacles)
    p = [v[..., None] for v in _seg_prep(segs, z_top)]
    clear = _seg_clear(p, c[:, 0], c[:, 1], rad)
    return np.where(p[7], clear, np.inf)


def segment_closest_xy(seg, z_top, cx, cy):
    """Điểm (x, y) trên phần thấp của 1 đoạn seg (2, 3) gần tâm (cx, cy) nhất (để vẽ dấu X chỗ chạm)."""
    p = _seg_prep(seg, z_top)
    t = float(_seg_t(p, cx, cy))
    return float(p[0] + t * p[2]), float(p[1] + t * p[3])


def segments_hit(segs, obstacles, obs_h, xy_margin, z_margin):
    """segs (..., 2, 3) -> (...) bool: đoạn thẳng nào chạm ít nhất 1 vật cản đã phình lề.
    Cùng phép tính với segment_clearance (cùng kết luận), chỉ bỏ qua sớm các cặp (đoạn, vật) ở xa nhau."""
    c, rad = obstacle_arrays(obstacles)
    segs = np.asarray(segs, dtype=float)
    shape = segs.shape[:-2]
    flat = segs.reshape(-1, 2, 3)
    p = _seg_prep(flat, obs_h + z_margin)
    lo_x, hi_x = np.minimum(flat[:, 0, 0], flat[:, 1, 0]), np.maximum(flat[:, 0, 0], flat[:, 1, 0])
    lo_y, hi_y = np.minimum(flat[:, 0, 1], flat[:, 1, 1]), np.maximum(flat[:, 0, 1], flat[:, 1, 1])
    hit = np.zeros(len(flat), dtype=bool)
    todo = p[7].copy()  # Đoạn có phần thấp, chưa chạm vật nào
    for (cx, cy), r in zip(c, rad):
        reach = r + xy_margin + 1e-6  # Tâm trụ ngoài hộp bao của đoạn + reach -> chắc chắn không chạm
        idx = np.flatnonzero(todo & (lo_x < cx + reach) & (hi_x > cx - reach) & (lo_y < cy + reach) & (hi_y > cy - reach))
        if len(idx):
            near = _seg_clear([v[idx] for v in p[:7]], cx, cy, r) < xy_margin
            hit[idx[near]] = True
            todo[idx[near]] = False
    return hit.reshape(shape)


class SampledMove(list):
    """Các mẫu dọc 1 đoạn đi: list [(q, [vai, khuỷu, cổ tay, tâm kẹp]), ...] như cũ, kèm segs =
    mảng (n, 4, 2, 3) các đoạn thân tay + quả treo tính sẵn 1 lần -> mỗi lần kiểm tra chỉ còn đo khoảng cách."""

    def __init__(self, samples=()):
        super().__init__(samples)
        joints = [pts for _q, pts in self]
        self.segs = pose_segments(joints, True) if joints else np.zeros((0, N_ARM_SEGS + 1, 2, 3))


def move_segments(mv, samples):
    """Các đoạn thẳng (n, S, 2, 3) cần kiểm tra của 1 đoạn đi: thân tay, + quả treo nếu đang kẹp."""
    segs = getattr(samples, "segs", None)
    if segs is None:
        samples = SampledMove(samples)
        segs = samples.segs
    return segs if mv.carrying else segs[:, :N_ARM_SEGS]


def sample_moves(moves, n=30):
    """Lấy mẫu sẵn các đoạn đi: list (move, SampledMove[(q, [vai, khuỷu, cổ tay, tâm kẹp]) ...])."""
    return [(mv, SampledMove((q, arm_joints(q)) for q in sample_move(mv, n))) for mv in moves]


def sample_cycle(bin_id, n=30, waypoints=None, direct_home=False):
    """Lấy mẫu sẵn toàn bộ chu trình (dùng lại mỗi khung hình)."""
    return sample_moves(cycle_moves(bin_id, waypoints, direct_home), n)


def check_collision(sampled_cycle, obstacles, obs_height_mm, xy_margin_mm=22.0, z_margin_mm=15.0):
    """Vật cản = trụ đứng (tâm x_mm, y_mm, bán kính r_mm, cao obs_height_mm) - luật ở segment_clearance().
    Va chạm khi một đoạn thân tay (hoặc quả đang kẹp) có phần thấp hơn đỉnh trụ + z_margin_mm
    lọt vào trụ đã phình thêm lề xy_margin_mm. obstacles: list dict, hoặc tuple của obstacle_arrays().
    Trả về list theo từng đoạn: dict(move, hit, min_clear_mm, hit_xy, tip_path)."""
    obs = obstacle_arrays(obstacles)
    z_top = obs_height_mm + z_margin_mm
    results = []
    for mv, samples in sampled_cycle:
        hit, hit_xy, min_clear = False, None, 9999.0
        tip_path = [pts[-1] for _q, pts in samples]
        if len(obs[1]) and len(samples):
            segs = move_segments(mv, samples)
            clear = segment_clearance(segs, obs, z_top)  # (mẫu, đoạn, vật)
            low = np.isfinite(clear)
            if low.any():
                min_clear = min(min_clear, float(clear[low].min()))
            bad = np.argwhere(clear < xy_margin_mm)
            if len(bad):  # Chỗ chạm đầu tiên theo thời gian
                i, k, j = bad[0]
                hit, hit_xy = True, segment_closest_xy(segs[i, k], z_top, obs[0][j, 0], obs[0][j, 1])
        results.append({"move": mv, "hit": hit, "min_clear_mm": min_clear, "hit_xy": hit_xy, "tip_path": tip_path})
    return results


# =========================================================================
# 6. TỰ KIỂM TRA:  python robot_model.py
# =========================================================================
def _self_test():
    print("=== CAC VI TRI DINH SAN (r, goc, z) -> (q1, q2, q3) | servo de ===")
    named = [("Tren diem gap", R_PATH, DEG_PICK, Z_APPROACH), ("Diem gap", R_PATH, DEG_PICK, Z_PICK),
             ("Nhac len", R_PATH, DEG_PICK, Z_TRAVEL)]
    for b_id, b in BINS.items():
        named.append((f"Tren {b['name']}", R_PATH, b["deg"], Z_TRAVEL))
        named.append((b["name"], R_PATH, b["deg"], Z_BIN))
    all_ok = True
    for label, r, deg, z in named:
        ok, q, msg = ik_polar(r, deg, z)
        if not ok:
            all_ok = False
            print(f"  [LOI] {label:26s} r={r:.0f} goc={deg:6.2f} z={z:.0f} -> {msg}")
            continue
        x, y, zz = fk(*q)
        err = math.dist((x, y, zz), polar_to_xyz(r, deg, z))
        print(f"  {label:26s} r={r:.0f} goc={deg:6.2f} z={z:3.0f} -> q=({q[0]:6.2f}, {q[1]:6.2f}, {q[2]:6.2f})"
              f" servo_de={base_deg_to_servo(q[0]):6.1f} | sai so FK={err:.4f}mm")

    hx, hy, hz = fk(*POSE_HOME)
    hr, hdeg, _ = xyz_to_polar(hx, hy, hz)
    print(f"  HOME {POSE_HOME} -> tam kep r={hr:.1f}mm goc={hdeg:.1f} z={hz:.1f}mm")

    print("\n=== DAI BAN KINH VUON TOI (IK chap nhan) THEO DO CAO ===")
    for z in (Z_PICK, Z_APPROACH, Z_BIN, Z_TRAVEL, 205.0, 245.0):
        band = reach_band(z)
        txt = "KHONG VUON TOI" if band is None else f"r = {band[0]:.1f} -> {band[1]:.1f} mm (rong {band[1] - band[0]:.0f}mm)"
        print(f"  z = {z:5.1f} mm : {txt}")

    print("\n=== CHU TRINH 3 KHAY (kiem tra IK doc toan bo duong di) ===")
    for b_id in BINS:
        try:
            moves = cycle_moves(b_id)
            low = min(fk(*q)[2] for mv in moves for q in sample_move(mv))
            print(f"  Khay {b_id}: OK ({len(moves)} doan) | tam kep thap nhat z={low:.1f}mm")
        except ValueError as e:
            all_ok = False
            print(f"  Khay {b_id}: LOI {e}")
    print("\nKET QUA:", "TAT CA HOP LE" if all_ok else "CO LOI - xem tren")


if __name__ == "__main__":
    _self_test()
