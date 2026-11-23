"""
protocol.py - GIAO THỨC SERIAL GIỮA ESP32 (KINEMATICS.ino) VÀ MÁY TÍNH (CHẶNG 3)
------------------------------------------------------------------------------
!!! PC_TIMEOUT_MS VÀ MAX_WAYPOINTS PHẢI GIỐNG HỆT KINEMATICS.ino !!!
Mô tả đầy đủ: GIAO_THUC.md

ESP32 -> PC (khi tay đang ở HOME và đã đọc được màu ổn định 3 lần liên tiếp):
    REQ <khay>                      khay = 1 / 2 / 3

PC -> ESP32 (trả lời trong PC_TIMEOUT_MS):
    CLEAR                           đi đường mặc định (cung R_PATH ở Z_TRAVEL)
    PATH <n> r1 g1 z1 ... rn gn zn [H]
                                    đi qua n điểm né (r mm, góc THỰC độ, z mm) từ trên điểm gắp tới trên khay;
                                    lúc về: không có H -> đi ngược lại đúng các điểm đó về trên điểm gắp rồi HOME
                                            có H     -> về THẲNG HOME (PC đã kiểm tra đường đó không chạm)
    BLOCKED                         không có đường an toàn -> đứng ở HOME, 1.5s sau hỏi lại
    (không trả lời)                 ESP32 tự đi đường mặc định -> hệ vẫn chạy khi không có PC
"""

import math

PC_TIMEOUT_MS = 500   # ESP32 chờ PC trả lời tối đa bao lâu
MAX_WAYPOINTS = 6     # Số điểm né tối đa trong 1 lệnh PATH
BLOCKED_RETRY_MS = 1500

CLEAR = "CLEAR"
BLOCKED = "BLOCKED"
DIRECT_HOME = "H"     # Chữ cuối câu PATH: về thẳng HOME


def parse_req(line):
    """'REQ 2' -> 2. Dòng khác (hoặc khay sai) -> None."""
    parts = line.strip().split()
    if len(parts) != 2 or parts[0] != "REQ":
        return None
    try:
        bin_id = int(parts[1])
    except ValueError:
        return None
    return bin_id if bin_id in (1, 2, 3) else None


def format_path(waypoints, direct_home=False):
    """[(r, góc, z), ...] -> 'PATH n r1 g1 z1 ...' (1 chữ số thập phân), thêm ' H' nếu direct_home."""
    if not 1 <= len(waypoints) <= MAX_WAYPOINTS:
        raise ValueError(f"So diem ne phai tu 1 den {MAX_WAYPOINTS}, dang co {len(waypoints)}")
    if not all(math.isfinite(v) for wp in waypoints for v in wp):
        raise ValueError("PATH co so khong hop le (nan/inf)")
    nums = " ".join(f"{r:.1f} {deg:.1f} {z:.1f}" for r, deg, z in waypoints)
    return f"PATH {len(waypoints)} {nums}" + (f" {DIRECT_HOME}" if direct_home else "")


def parse_path(line):
    """'PATH n ... [H]' -> [(r, góc, z), ...]. Sai cú pháp -> ValueError (giống parsePath() trong firmware)."""
    return parse_path_full(line)[0]


def parse_path_full(line):
    """'PATH n ... [H]' -> ([(r, góc, z), ...], về thẳng HOME?). Sai cú pháp -> ValueError."""
    parts = line.strip().split()
    if not parts or parts[0] != "PATH":
        raise ValueError("Khong phai lenh PATH")
    direct = len(parts) > 2 and parts[-1] == DIRECT_HOME
    if direct:
        parts = parts[:-1]
    try:
        if any(c in tok for tok in parts[1:] for c in "_xX"):  # Python nhận '1_90' / '0x1', firmware thì không
            raise ValueError("so co ky tu la")
        n = int(parts[1])
        vals = [float(v) for v in parts[2:]]
    except (IndexError, ValueError) as e:
        raise ValueError(f"PATH sai cu phap: {e}") from e
    if not 1 <= n <= MAX_WAYPOINTS:
        raise ValueError(f"PATH: n={n} ngoai [1, {MAX_WAYPOINTS}]")
    if len(vals) != 3 * n:
        raise ValueError(f"PATH: can {3 * n} so, nhan duoc {len(vals)}")
    if not all(math.isfinite(v) for v in vals):
        raise ValueError("PATH co so khong hop le (nan/inf)")  # float('nan') không báo lỗi
    return [tuple(vals[i:i + 3]) for i in range(0, 3 * n, 3)], direct
