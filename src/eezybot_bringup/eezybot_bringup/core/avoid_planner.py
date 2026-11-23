"""
avoid_planner.py - THUẬT TOÁN NÉ VẬT CẢN (CHẶNG 4)
--------------------------------------------------
Đầu vào : khay đích + danh sách vật cản (trụ đứng: tâm x_mm, y_mm, bán kính r_mm) + chiều cao giả định.
Đầu ra  : Plan với kind = "CLEAR" / "PATH" / "BLOCKED" và câu trả lời gửi cho ESP32 (protocol.py).

CÁCH LÀM (thử lần lượt, dừng ở phương án đầu tiên an toàn):
  0. Các đoạn KHÔNG né được (HOME -> hạ xuống gắp -> nâng; hạ xuống khay -> nhấc) mà chạm vật
     -> BLOCKED ngay (vật nằm đè lên điểm gắp / khay, đi đường nào cũng chạm).
  1. Đường mặc định (cung R_PATH ở Z_TRAVEL, về HOME bằng nội suy khớp) an toàn -> CLEAR.
  2. Đi qua 2 điểm né ở bán kính r, độ cao z:
         trên gắp -> (r, góc gắp, z) -> (r, góc khay, z) -> trên khay
     Các cặp (r, z) được thử theo CHI PHÍ tăng dần: |r - R_PATH| + Z_COST * (z - Z_TRAVEL)
       - Đổi r = đi vòng ngoài / vòng trong (trong dải với tới ở độ cao z, lấy từ robot_model.reach_band)
       - Tăng z = nâng cao vượt qua vật
       - Cả hai cùng lúc
     Lúc về, robot đi ngược lại đúng các điểm này -> không cần kiểm tra thêm.
     Riêng đường VỀ THẲNG (trên khay -> HOME, nội suy khớp như CLEAR) được kiểm tra thêm: không chạm
     -> gửi PATH ... H, ESP32 về thẳng HOME thay vì quay lại qua điểm gắp (nhanh hơn, ít đổi hướng).
     Riêng (R_PATH, Z_TRAVEL) = đi đúng cung mặc định, chỉ đổi đường VỀ (về HOME nội suy khớp bị chạm)
     -> gửi 1 điểm né "trên khay" (như A*), không có 2 đoạn dài 0mm (mỗi đoạn mất 500ms).
     Điểm né được làm tròn 0.1 như câu PATH gửi đi (protocol.format_path) TRƯỚC khi kiểm tra
     -> kiểm tra đúng đường ESP32 sẽ chạy.
     Vòng ngoài không đi xa hơn detour_r_max(): camera phải còn thấy vật cản ở đó (calib_config.json:
     detour_r_max_mm, mặc định scan_r_max_mm - safe_margin_mm; cùng giới hạn với lưới A*).
  3. Không còn phương án -> BLOCKED.
Toàn bộ hình học lấy từ robot_model.py, nên khi đo lại L1/L2/Z_BASE thì planner tự đúng theo
(khởi động lại 06 sau khi sửa robot_model.py: các phương án được tính sẵn 1 lần lúc warm_up).
Va chạm dùng CHUNG luật với robot_model.check_collision (đo trên cả đoạn thẳng thân tay, xem segment_clearance).
Mọi đường đi được lấy mẫu sẵn (warm_up) thành mảng numpy; mỗi lần hỏi chỉ đo khoảng cách, thử từng nhóm
CHUNK phương án theo thứ tự chi phí và dừng ở nhóm đầu tiên có đường an toàn -> vài ms, kể cả vài chục vật cản.
Quá DEADLINE_MS (0.6 x PC_TIMEOUT_MS) mà chưa xong -> BLOCKED cho an toàn (trả lời muộn thì ESP32 tự đi
đường mặc định, có thể đâm vào vật).
"""

import functools
import math
import time
from dataclasses import dataclass

import numpy as np

import protocol
import robot_model as rm

Z_LEVELS = tuple(rm.Z_TRAVEL + dz for dz in (0.0, 20.0, 40.0, 60.0, 80.0))  # Các độ cao mang quả được thử
R_STEP_MM = 10.0
Z_COST = 1.5      # Nâng 1mm "đắt" hơn đi vòng 1mm (nâng cao làm quả lắc, đi lâu hơn)
N_SAMPLES = 30    # Số điểm lấy mẫu mỗi đoạn khi kiểm tra va chạm
DEADLINE_MS = 0.6 * protocol.PC_TIMEOUT_MS  # Tính quá lâu -> BLOCKED (ESP32 đứng chờ, 1.5s sau hỏi lại)
CHUNK = 8         # Số phương án điểm né đo cùng lúc bằng numpy trước khi xem đã có đường chưa


@dataclass
class Plan:
    kind: str          # "CLEAR" / "PATH" / "BLOCKED"
    reply: str         # Chuỗi gửi cho ESP32
    waypoints: list    # [(r, góc, z), ...] (rỗng nếu CLEAR / BLOCKED)
    reason: str        # Giải thích ngắn (hiển thị + log)
    moves: list        # Các đoạn của chu trình sẽ chạy (BLOCKED: chu trình mặc định)
    results: list      # check_collision() của các đoạn trên (để vẽ)
    tried: int         # Số phương án đã thử (A*: số nút đã mở rộng)
    ms: float          # Thời gian tính (ms)
    algo: str = "CO DINH"   # Thuật toán đã tính: "CO DINH" (file này) / "A*" (astar_planner.py)
    direct_home: bool = False  # PATH có "H": sau khi thả quả về THẲNG HOME (đường đó đã kiểm tra không chạm)


def detour_r_max_from_cfg(cfg, visible_r_max=None):
    """Bán kính đi vòng NGOÀI xa nhất (mm) mà camera còn thấy vật cản ở đó:
      - cfg["detour_r_max_mm"] nếu có, mặc định scan_r_max_mm - safe_margin_mm (vật sát mép vùng quét
        vẫn được thấy đủ cả lề), không có cấu hình -> R_MAX (không giới hạn thêm);
      - không quá visible_r_max (bán kính mà cả cung quét còn nằm trong khung hình, nếu biết);
      - không dưới R_PATH (cung mặc định luôn phải nằm trong lưới / danh sách phương án).
    Dùng CHUNG cho avoid_planner.candidates() và lưới astar_planner.Grid -> 2 thuật toán cùng giới hạn."""
    if cfg.get("detour_r_max_mm") is not None:
        cap = float(cfg["detour_r_max_mm"])
    elif cfg.get("scan_r_max_mm") is not None:
        cap = float(cfg["scan_r_max_mm"]) - float(cfg.get("safe_margin_mm", 40.0))
    else:
        cap = rm.R_MAX
    if visible_r_max is not None:
        cap = min(cap, float(visible_r_max))
    return min(max(cap, rm.R_PATH), rm.R_MAX)


@functools.lru_cache(maxsize=1)
def detour_r_max():
    """detour_r_max_from_cfg() với calib_config.json (chỉ đọc, 1 lần). Đổi cấu hình -> khởi động lại 06 / sim_view.
    Lấy thêm vision_utils.visible_r_max(cfg) nếu vision_utils có hàm đó (bán kính camera còn thấy trọn cung)."""
    try:
        import vision_utils as vu
        cfg = vu.load_config()
    except Exception as e:  # noqa: BLE001 - không có cấu hình / OpenCV thì planner vẫn chạy (không giới hạn thêm)
        print(f"[PLANNER] Khong doc duoc calib_config.json ({e!r}) -> di vong toi da R_MAX = {rm.R_MAX:.0f}mm")
        return float(rm.R_MAX)
    visible = None
    fn = getattr(vu, "visible_r_max", None)
    if fn is not None:
        try:
            visible = fn(cfg)
        except Exception:  # noqa: BLE001
            visible = None
    cap = detour_r_max_from_cfg(cfg, visible)
    seen = f", camera thay tron cung toi r = {visible:.0f}mm" if visible is not None else ""
    print(f"[PLANNER] Di vong ngoai toi da r = {cap:.0f}mm (vung quet scan_r_max_mm = {cfg.get('scan_r_max_mm', '?')}"
          f" - le {cfg.get('safe_margin_mm', 40.0)}mm{seen}; doi bang detour_r_max_mm trong calib_config.json)")
    return cap


@functools.lru_cache(maxsize=1)
def candidates():
    """Danh sách (r, z) đã sắp theo chi phí tăng dần (tính 1 lần). Vòng ngoài không quá detour_r_max()."""
    out = []
    r_cap = detour_r_max()
    for z in Z_LEVELS:
        band = rm.reach_band(z)
        if band is None:
            continue
        r_lo = math.ceil(band[0] / R_STEP_MM) * R_STEP_MM
        r_hi = math.floor(min(band[1], r_cap) / R_STEP_MM) * R_STEP_MM
        rs = {rm.R_PATH} if band[0] <= rm.R_PATH <= band[1] else set()
        r = r_lo
        while r <= r_hi + 1e-6:
            rs.add(r)
            r += R_STEP_MM
        for r in rs:
            out.append((abs(r - rm.R_PATH) + Z_COST * (z - rm.Z_TRAVEL), r, z))
    out.sort()
    return [(r, z) for _cost, r, z in out]


def detour_waypoints(bin_id, r, z):
    """Điểm né của phương án (r, z), đã làm tròn đúng như câu PATH gửi đi (format_path -> parse_path)."""
    if r == rm.R_PATH and z == rm.Z_TRAVEL:
        wps = [rm.above_bin(bin_id)]  # Đúng cung mặc định, chỉ đổi đường về -> 1 điểm né (như A*)
    else:
        wps = [(r, rm.DEG_PICK, z), (r, rm.BINS[bin_id]["deg"], z)]
    return protocol.parse_path(protocol.format_path(wps))


# ---- Lấy mẫu sẵn (IK là phần tốn thời gian nhất, và không phụ thuộc vật cản) ----
# Nhờ vậy mỗi lần ESP32 hỏi chỉ còn phải đo khoảng cách tới vật cản -> trả lời nhanh trong PC_TIMEOUT_MS.
@functools.lru_cache(maxsize=None)
def _sampled(key, bin_id, r=None, z=None, direct_home=False):
    if key == "fixed":
        moves = rm.approach_moves() + rm.bin_moves(bin_id)
    elif key == "default":
        moves = rm.transfer_moves(bin_id) + rm.return_moves(bin_id)
    elif key == "home":  # Về thẳng: trên khay -> HOME (nội suy khớp, giống CLEAR)
        moves = rm.return_moves(bin_id)
    elif key == "cycle":
        moves = rm.cycle_moves(bin_id, detour_waypoints(bin_id, r, z) if r is not None else None, direct_home)
    else:  # "detour": đoạn mang quả qua 2 điểm né
        try:
            moves = rm.transfer_moves(bin_id, detour_waypoints(bin_id, r, z))
            return moves, rm.sample_moves(moves, N_SAMPLES)
        except ValueError:
            return None  # Có điểm trên đường đi mà IK không tới được
    return moves, rm.sample_moves(moves, N_SAMPLES)


@functools.lru_cache(maxsize=None)
def _detour_pack(bin_id):
    """Mọi phương án điểm né của 1 khay (thứ tự chi phí) gộp thành 1 mảng numpy để đo cùng lúc:
    ([(r, z), ...], segs (T, S, 2, 3) = đoạn thân tay + quả của mọi mẫu nối tiếp nhau,
     starts (len + 1,) = chỉ số mẫu đầu tiên của từng phương án)."""
    rz, parts, starts, n = [], [], [], 0
    for r, z in candidates():
        detour = _sampled("detour", bin_id, r, z)
        if detour is None:
            continue
        segs = np.concatenate([rm.move_segments(mv, s) for mv, s in detour[1]])  # Đoạn mang quả: + quả treo
        rz.append((r, z))
        parts.append(segs)
        starts.append(n)
        n += len(segs)
    big = np.concatenate(parts) if parts else np.zeros((0, rm.N_ARM_SEGS + 1, 2, 3))
    return rz, big, np.array(starts + [n])


def warm_up():
    """Tính sẵn mọi phương án (khoảng 1-2s). Gọi lúc khởi động để lần REQ đầu cũng nhanh."""
    for b in rm.BINS:
        _sampled("fixed", b)
        _sampled("default", b)
        _sampled("cycle", b)
        for r, z in candidates():
            if _sampled("detour", b, r, z) is not None:
                _sampled("cycle", b, r, z)
                _sampled("cycle", b, r, z, True)
        _detour_pack(b)
        _moves_pack("fixed", b)
        _moves_pack("default", b)
        _moves_pack("home", b)


def _hits(sampled, args):
    """Tên các đoạn chạm vật. args = (vật cản, cao vật, lề ngang, lề đứng) - cùng luật rm.check_collision."""
    return [mv.name for mv, s in sampled if rm.segments_hit(rm.move_segments(mv, s), *args).any()]


@functools.lru_cache(maxsize=None)
def _moves_pack(key, bin_id):
    """Các đoạn "fixed" / "default" gộp thành 1 mảng để đo 1 lần:
    (tên đoạn, segs (K, 2, 3) mọi đoạn thẳng của mọi mẫu, chỉ số đoạn thẳng đầu tiên của từng đoạn đi)."""
    names, parts, starts, n = [], [], [], 0
    for mv, s in _sampled(key, bin_id)[1]:
        segs = rm.move_segments(mv, s).reshape(-1, 2, 3)  # Đang kẹp: + quả treo
        names.append(mv.name)
        parts.append(segs)
        starts.append(n)
        n += len(segs)
    return names, np.concatenate(parts), np.array(starts)


def _hits_key(key, bin_id, args):
    """Như _hits(_sampled(key, bin_id)[1], args) nhưng đo cả chuỗi đoạn trong 1 lần gọi numpy."""
    names, segs, starts = _moves_pack(key, bin_id)
    per = np.logical_or.reduceat(rm.segments_hit(segs, *args), starts)
    return [names[i] for i in np.flatnonzero(per)]


def direct_home_ok(bin_id, args):
    """Đường về thẳng (trên khay -> HOME, nội suy khớp) không chạm vật -> PATH được gửi kèm "H"."""
    return not _hits_key("home", bin_id, args)


def _first_free(bin_id, args, t0, deadline_ms):
    """Thử các phương án điểm né theo từng nhóm CHUNK. Trả về (chỉ số phương án đầu tiên không chạm
    hoặc None, số phương án đã thử, hết giờ?)."""
    rz, big, starts = _detour_pack(bin_id)
    for i in range(0, len(rz), CHUNK):
        if (time.perf_counter() - t0) * 1000.0 > deadline_ms:
            return None, i, True
        j = min(i + CHUNK, len(rz))
        lo = starts[i]
        hit = rm.segments_hit(big[lo:starts[j]], *args).any(axis=-1)          # (mẫu,)
        free = np.flatnonzero(~np.logical_or.reduceat(hit, starts[i:j] - lo))  # (phương án,)
        if len(free):
            return i + int(free[0]), i + int(free[0]) + 1, False
    return None, len(rz), False


def plan(bin_id, obstacles, obs_height_mm, xy_margin_mm=22.0, z_margin_mm=15.0, deadline_ms=DEADLINE_MS):
    t0 = time.perf_counter()
    obs = rm.obstacle_arrays(obstacles)  # Đổi sang numpy 1 lần
    args = (obs, obs_height_mm, xy_margin_mm, z_margin_mm)

    def finish(kind, rz, reason, tried, direct=False):
        moves, sampled = _sampled("cycle", bin_id, *rz, direct) if rz else _sampled("cycle", bin_id)
        waypoints = detour_waypoints(bin_id, *rz) if rz else []
        reply = protocol.format_path(waypoints, direct) if kind == "PATH" else kind
        return Plan(kind, reply, waypoints, reason, moves, rm.check_collision(sampled, *args), tried,
                    (time.perf_counter() - t0) * 1000.0, direct_home=direct)

    if not len(obs[1]):
        return finish("CLEAR", None, "Khong co vat can", 0)

    # 0. Đoạn không né được
    fixed_hits = _hits_key("fixed", bin_id, args)
    if fixed_hits:
        return finish("BLOCKED", None, f"Vat can o doan khong ne duoc: {', '.join(fixed_hits)}", 0)

    # 1. Đường mặc định
    if not _hits_key("default", bin_id, args):
        return finish("CLEAR", None, "Duong mac dinh an toan", 1)

    # 2. Điểm né (thứ tự chi phí tăng dần, dừng ở phương án đầu tiên an toàn)
    k, n_tried, late = _first_free(bin_id, args, t0, deadline_ms)
    tried = 1 + n_tried
    if k is not None:
        r, z = _detour_pack(bin_id)[0][k]
        how = []
        if r != rm.R_PATH:
            how.append(f"{'vong ngoai' if r > rm.R_PATH else 'vong trong'} r={r:.0f}")
        if z != rm.Z_TRAVEL:
            how.append(f"nang z={z:.0f}")
        direct = direct_home_ok(bin_id, args)
        if direct:
            how.append("ve thang HOME")
        return finish("PATH", (r, z), "Ne: " + (", ".join(how) if how else "ve theo cung"), tried, direct)
    if late:
        return finish("BLOCKED", None, f"Tinh qua {deadline_ms:.0f}ms (thu {tried} phuong an) -> BLOCKED cho an toan",
                      tried)

    # 3. Hết cách
    return finish("BLOCKED", None, f"Thu {tried} phuong an, khong co duong an toan", tried)
