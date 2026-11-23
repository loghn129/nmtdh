"""
sim_view.py - MÔ PHỎNG CÁNH TAY + THUẬT TOÁN NÉ (KHÔNG CẦN ROBOT, KHÔNG CẦN CAMERA)
------------------------------------------------------------------------------------
Hai hình chiếu, cùng hệ tọa độ robot (mm) của robot_model.py:
  - NHÌN TỪ TRÊN (x, y): quạt góc đế BASE_DEG_MIN-BASE_DEG_MAX (55.1°-126.7°) tới R_MAX, cung R_PATH, điểm gắp,
    3 khay, vật cản (trụ) + vòng lề an toàn, đường tâm kẹp của cả chu trình (ĐỎ = chạm, XANH = đang mang quả,
    XANH NGỌC = tay không), điểm né.
  - NHÌN NGANG (r, z): vùng với tới của IK, 2 khâu của tay, vật cản chiếu lên (r, z), đường tâm kẹp.

CÁCH DÙNG (chỉ dùng chuột, không cần phím):
  python sim_view.py
  - Click TRÁI vào hình nhìn từ trên: thêm vật cản (bán kính 30mm).
  - Giữ + kéo chuột PHẢI: di chuyển vật cản gần nhất (nhả chuột ở đâu cũng thôi kéo).
  - [KHAY 1/2/3] chọn khay, [CAO] đổi chiều cao giả định của vật, [CHAY] xem tay chạy theo kế hoạch.
  - [CO DINH] / [A*] đổi thuật toán né: avoid_planner.py <-> astar_planner.py (cùng vật cản, so sánh trực tiếp).
  Lề an toàn (safe_margin_mm, safe_z_margin_mm), chiều cao vật ban đầu (obs_height_mm) và thuật toán ban đầu
  (planner) đọc từ calib_config.json GIỐNG 06_auto_run.py (chỉ đọc, không ghi) -> mô phỏng dự đoán đúng robot.
  python sim_view.py --anh ket_qua.png  -> chỉ vẽ 1 ảnh mẫu rồi thoát (dùng cho báo cáo / kiểm tra).
"""

import math
import sys
import time

import cv2
import numpy as np

import astar_planner as aa
import avoid_planner as ap
import robot_model as rm
from fake_robot import GRIP_CLOSE_MS, GRIP_OPEN_MS  # Thời gian kẹp / thả giống firmware (setGripper)

VIEW_W = 560
TOP_H = 460
SIDE_H = 300
TOOLBAR_H = 84
TOP_TEXT_H = 70                    # 3 dòng chữ trạng thái ở trên cùng hình nhìn từ trên (y = 18 / 40 / 60)
TOP_O = (VIEW_W // 2, TOP_H - 28)  # Pixel của tâm đế
S_TOP = (TOP_O[1] - TOP_TEXT_H) / (rm.R_MAX + 15.0)  # px/mm: cả quạt tới R_MAX nằm gọn dưới dòng chữ (~0.92)
S_SIDE = 0.8                       # px/mm hình nhìn ngang
SIDE_O = (50, SIDE_H - 30)         # Pixel của điểm (r=0, z=0) trong khung nhìn ngang
HEIGHT_OPTIONS = [60.0, 100.0, 150.0, 250.0, 9999.0]
OBS_RADIUS_MM = 30.0

C_HIT, C_CARRY, C_EMPTY = (0, 0, 255), (0, 220, 0), (200, 200, 0)
LABELS = ["KHAY 1", "KHAY 2", "KHAY 3", "CAO", "NE", "CHAY", "XOA VAT", "THOAT"]


def height_label(h):
    return "2D" if h >= 9999.0 else f"{h:.0f}mm"


def top_uv(x, y):
    return int(round(TOP_O[0] + x * S_TOP)), int(round(TOP_O[1] - y * S_TOP))


def top_xy(u, v):
    return (u - TOP_O[0]) / S_TOP, (TOP_O[1] - v) / S_TOP


def side_uv(r, z):
    return int(round(SIDE_O[0] + r * S_SIDE)), int(round(SIDE_O[1] - z * S_SIDE))


def make_obstacle_mm(x, y, r_mm=OBS_RADIUS_MM):
    return {"x_mm": x, "y_mm": y, "r_mm": r_mm, "dist_mm": math.hypot(x, y),
            "deg": math.degrees(math.atan2(y, x)), "source": "GIA LAP", "big": False}


_REACH_POLY = None
_TOP_BAND = None


def reach_polygon():
    """Biên vùng với tới (r, z) của IK - tính 1 lần."""
    global _REACH_POLY
    if _REACH_POLY is None:
        lo, hi = [], []
        for z in range(0, 320, 5):
            band = rm.reach_band(float(z), step=1.0)
            if band is not None:
                lo.append(side_uv(band[0], z))
                hi.append(side_uv(band[1], z))
        _REACH_POLY = np.array(lo + hi[::-1], dtype=np.int32) if lo else None
    return _REACH_POLY


def top_band_polygon():
    """Dải với tới ở Z_TRAVEL trên hình nhìn từ trên (pixel) - tính 1 lần (reach_band = 761 lần IK)."""
    global _TOP_BAND
    if _TOP_BAND is None:
        band = rm.reach_band(rm.Z_TRAVEL)
        pts = [top_uv(band[1] * math.cos(math.radians(d)), band[1] * math.sin(math.radians(d)))
               for d in np.linspace(rm.BASE_DEG_MIN, rm.BASE_DEG_MAX, 40)]
        pts += [top_uv(band[0] * math.cos(math.radians(d)), band[0] * math.sin(math.radians(d)))
                for d in np.linspace(rm.BASE_DEG_MAX, rm.BASE_DEG_MIN, 40)]
        _TOP_BAND = np.array(pts, dtype=np.int32)
    return _TOP_BAND


# =========================================================================
# DÒNG THỜI GIAN CHU TRÌNH (để xem tay chạy) - giống thứ tự trong firmware / fake_robot
# =========================================================================
class Timeline:
    def __init__(self, moves):
        self.steps = []  # (t_bắt_đầu, ms, move hoặc None, grip_đầu, grip_cuối, đang_mang)
        t, grip = 0.0, rm.GRIPPER_OPEN
        for mv in moves:
            self.steps.append((t, mv.ms, mv, grip, grip, mv.carrying))
            t += mv.ms
            if mv.name == "HA CHAM":
                self.steps.append((t, GRIP_CLOSE_MS, None, grip, rm.GRIPPER_CLOSE, False))
                t, grip = t + GRIP_CLOSE_MS, rm.GRIPPER_CLOSE
            elif mv.name == "HA XUONG KHAY":
                self.steps.append((t, GRIP_OPEN_MS, None, grip, rm.GRIPPER_OPEN, True))
                t, grip = t + GRIP_OPEN_MS, rm.GRIPPER_OPEN
        self.total_ms = t
        self._last_q = rm.POSE_HOME

    def at(self, t_ms):
        """-> (q, grip, đang_mang_quả, tên_đoạn)."""
        for t0, ms, mv, g0, g1, carrying in self.steps:
            if t_ms < t0 + ms:
                s = rm.quintic((t_ms - t0) / float(ms))
                if mv is not None:
                    self._last_q = rm.move_pose(mv, s)
                return self._last_q, g0 + (g1 - g0) * s, carrying, (mv.name if mv else "KEP / THA")
        return rm.POSE_HOME, rm.GRIPPER_OPEN, False, "XONG"


# =========================================================================
# VẼ
# =========================================================================
def _poly(img, pts, color, thickness):
    if len(pts) >= 2:
        cv2.polylines(img, [np.array(pts, dtype=np.int32).reshape((-1, 1, 2))], False, color, thickness, cv2.LINE_AA)


def _seg_color(res):
    return C_HIT if res["hit"] else (C_CARRY if res["move"].carrying else C_EMPTY)


def _draw_order(results):
    """Vẽ đoạn tay không trước, đoạn mang quả sau, đoạn chạm cuối cùng (để không bị che)."""
    return sorted(results, key=lambda res: (res["hit"], res["move"].carrying))


def draw_top(img, q, plan, obstacles, margin, holding):
    # Quạt, cung R_MAX, dải với tới ở Z_TRAVEL (tính sẵn 1 lần), cung R_PATH
    tint = img.copy()
    cv2.fillPoly(tint, [top_band_polygon()], (60, 90, 140))
    cv2.addWeighted(tint, 0.35, img, 0.65, 0, img)
    for deg in (rm.BASE_DEG_MIN, rm.BASE_DEG_MAX):
        cv2.line(img, top_uv(0, 0), top_uv(*rm.polar_to_xyz(rm.R_MAX, deg, 0)[:2]), (120, 120, 120), 1, cv2.LINE_AA)
    for rad, col in ((rm.R_MAX, (110, 110, 110)), (rm.R_PATH, (200, 200, 0))):
        _poly(img, [top_uv(*rm.polar_to_xyz(rad, d, 0)[:2]) for d in np.linspace(rm.BASE_DEG_MIN, rm.BASE_DEG_MAX, 50)],
              col, 1)

    # Điểm gắp + khay
    pu = top_uv(*rm.polar_to_xyz(rm.R_PATH, rm.DEG_PICK, 0)[:2])
    cv2.circle(img, pu, 9, (255, 255, 0), -1)
    cv2.putText(img, "GAP", (pu[0] + 10, pu[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    for b_id, b in rm.BINS.items():
        u = top_uv(*rm.polar_to_xyz(rm.R_PATH, b["deg"], 0)[:2])
        cv2.circle(img, u, 9, b["color"], -1)
        cv2.putText(img, f"K{b_id}", (u[0] - 28, u[1] + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, b["color"], 1)

    # Vật cản + vòng lề
    for ob in obstacles:
        c = top_uv(ob["x_mm"], ob["y_mm"])
        cv2.circle(img, c, int(ob["r_mm"] * S_TOP), (0, 140, 255), -1, cv2.LINE_AA)
        cv2.circle(img, c, int((ob["r_mm"] + margin) * S_TOP), (0, 140, 255), 1, cv2.LINE_AA)

    # Đường tâm kẹp theo kế hoạch
    if plan is not None:
        for res in _draw_order(plan.results):
            _poly(img, [top_uv(x, y) for x, y, _z in res["tip_path"]], _seg_color(res), 3 if res["hit"] else 2)
            if res["hit_xy"] is not None:
                cv2.drawMarker(img, top_uv(*res["hit_xy"]), C_HIT, cv2.MARKER_TILTED_CROSS, 16, 3)
        for r, deg, _z in plan.waypoints:
            cv2.drawMarker(img, top_uv(*rm.polar_to_xyz(r, deg, 0)[:2]), (255, 0, 255), cv2.MARKER_DIAMOND, 14, 2)

    # Cánh tay
    if q is not None:
        pts = [top_uv(0, 0)] + [top_uv(x, y) for x, y, _z in rm.arm_points(q, 1)]
        _poly(img, pts, (255, 160, 60), 5)
        cv2.circle(img, pts[-1], 6, (255, 255, 255), -1)
        if holding:
            cv2.circle(img, pts[-1], 11, (0, 0, 220), 2)
    cv2.circle(img, top_uv(0, 0), 8, (0, 0, 255), -1)
    cv2.putText(img, "NHIN TU TREN (x, y)", (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)


def draw_side(img, q, plan, obstacles, obs_h):
    poly = reach_polygon()
    if poly is not None:
        tint = img.copy()
        cv2.fillPoly(tint, [poly], (60, 90, 140))
        cv2.addWeighted(tint, 0.35, img, 0.65, 0, img)
    cv2.line(img, side_uv(0, 0), side_uv(340, 0), (150, 150, 150), 1)
    for z in (rm.Z_PICK, rm.Z_BIN, rm.Z_TRAVEL):
        cv2.line(img, side_uv(rm.R_PATH - 8, z), side_uv(rm.R_PATH + 8, z), (200, 200, 0), 1)

    # Vật cản chiếu lên mặt phẳng (r, z)
    h_draw = min(obs_h, 310.0)
    for ob in obstacles:
        cv2.rectangle(img, side_uv(ob["dist_mm"] - ob["r_mm"], 0), side_uv(ob["dist_mm"] + ob["r_mm"], h_draw),
                      (0, 140, 255), -1 if obs_h < 9999.0 else 2)

    if plan is not None:
        for res in _draw_order(plan.results):
            pts = [side_uv(math.hypot(x, y), z) for x, y, z in res["tip_path"]]
            _poly(img, pts, _seg_color(res), 2)

    # Bệ đế + 2 khâu
    cv2.rectangle(img, side_uv(-25, 0), side_uv(25, rm.Z_BASE), (90, 90, 90), -1)
    if q is not None:
        joints = rm.arm_rz(q[1], q[2])  # vai, khuỷu, cổ tay, tâm kẹp
        tip = joints[-1]
        _poly(img, [side_uv(*p) for p in joints], (255, 160, 60), 5)
        cv2.circle(img, side_uv(*tip), 6, (255, 255, 255), -1)
    cv2.putText(img, f"NHIN NGANG (r, z) - vat cao {height_label(obs_h)}, chieu theo r (bo qua goc)", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)


def render(q, plan, obstacles, obs_h, margin, holding=False):
    """Ảnh (TOP_H + SIDE_H) x VIEW_W: trên = nhìn từ trên, dưới = nhìn ngang (vẽ riêng nên không đè nhau)."""
    top = np.full((TOP_H, VIEW_W, 3), 30, dtype=np.uint8)
    side = np.full((SIDE_H, VIEW_W, 3), 30, dtype=np.uint8)
    draw_top(top, q, plan, obstacles, margin, holding)
    draw_side(side, q, plan, obstacles, obs_h)
    img = np.vstack([top, side])
    cv2.line(img, (0, TOP_H), (VIEW_W, TOP_H), (90, 90, 90), 1)
    if plan is not None:
        col = {"CLEAR": (0, 255, 0), "PATH": (255, 0, 255), "BLOCKED": (0, 0, 255)}[plan.kind]
        cv2.putText(img, f"{plan.kind}: {plan.reason}"[:75], (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.48, col, 1)
        tried = f"mo rong {plan.tried} nut" if plan.algo == "A*" else f"thu {plan.tried} phuong an"
        cv2.putText(img, f"[{plan.algo}] {tried}, {plan.ms:.0f}ms", (8, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (200, 200, 200), 1)
    return img


# =========================================================================
# CỬA SỔ TƯƠNG TÁC
# =========================================================================
def load_settings():
    """(lề ngang, lề đứng, chiều cao vật, thuật toán) giống 06_auto_run.py, đọc calib_config.json - KHÔNG ghi."""
    try:
        from vision_utils import load_config
        cfg = load_config()
    except Exception as e:  # noqa: BLE001 - mô phỏng vẫn chạy được khi thiếu cấu hình
        print(f"[CANH BAO] Khong doc duoc calib_config.json ({e!r}) -> dung gia tri mac dinh")
        cfg = {}
    return (float(cfg.get("safe_margin_mm", 40.0)), float(cfg.get("safe_z_margin_mm", 25.0)),
            float(cfg.get("obs_height_mm", 150.0)), "A*" if cfg.get("planner") == "astar" else "CO DINH")


def handle_mouse(st, event, u, v, flags, buttons):
    """Chuột của cửa sổ mô phỏng ((u, v) = tọa độ CỬA SỔ, thanh nút cao TOOLBAR_H ở trên).
    st: trạng thái (bin, h, obs, drag, dirty, plan, anim, quit, algo)."""
    if event == cv2.EVENT_RBUTTONUP:
        st["drag"] = None  # Nhả chuột phải ở BẤT KỲ đâu (cả ngoài hình nhìn từ trên) -> thôi kéo
        return
    if event == cv2.EVENT_LBUTTONDOWN and v <= TOOLBAR_H:
        for lbl in LABELS:
            if inside_rect(u, v, buttons[lbl]):
                if lbl == "THOAT":
                    st["quit"] = True
                elif lbl.startswith("KHAY"):
                    st["bin"] = int(lbl[-1])
                elif lbl == "CAO":
                    nxt = [h for h in HEIGHT_OPTIONS if h > st["h"]]
                    st["h"] = nxt[0] if nxt else HEIGHT_OPTIONS[0]
                elif lbl == "NE":
                    st["algo"] = "A*" if st["algo"] == "CO DINH" else "CO DINH"
                elif lbl == "XOA VAT":
                    st["obs"], st["drag"] = [], None
                elif lbl == "CHAY" and st["plan"] is not None and st["plan"].kind != "BLOCKED":
                    st["anim"] = (Timeline(st["plan"].moves), time.time())
                st["dirty"] = True
        return
    vv = v - TOOLBAR_H
    if vv < 0 or vv >= TOP_H:
        return
    x, y = top_xy(u, vv)
    if event == cv2.EVENT_LBUTTONDOWN:
        st["obs"].append(make_obstacle_mm(x, y))
        st["dirty"] = True
    elif event == cv2.EVENT_RBUTTONDOWN and st["obs"]:
        st["drag"] = min(range(len(st["obs"])),
                         key=lambda i: math.hypot(st["obs"][i]["x_mm"] - x, st["obs"][i]["y_mm"] - y))
    elif event == cv2.EVENT_MOUSEMOVE and st["drag"] is not None:
        if not flags & cv2.EVENT_FLAG_RBUTTON or st["drag"] >= len(st["obs"]):
            st["drag"] = None  # Đã nhả nút chuột phải ở ngoài cửa sổ / vật đã bị xóa
            return
        st["obs"][st["drag"]] = make_obstacle_mm(x, y)
        st["dirty"] = True


def inside_rect(u, v, rect):
    """Giống vision_utils.inside (để handle_mouse không phải nạp vision_utils)."""
    return rect[0] <= u <= rect[2] and rect[1] <= v <= rect[3]


def main():
    from vision_utils import draw_button, make_buttons

    margin, z_margin, h0, algo0 = load_settings()
    print(f"[CAU HINH] Le ngang {margin:.0f}mm, le dung {z_margin:.0f}mm, cao vat {height_label(h0)}, ne {algo0}")
    print("[PLANNER] Dang tinh san cac phuong an ne (ca 2 thuat toan)...")
    ap.warm_up()
    aa.warm_up()
    planners = {"CO DINH": ap, "A*": aa}

    if "--anh" in sys.argv:
        out = sys.argv[sys.argv.index("--anh") + 1]
        obs = [make_obstacle_mm(*rm.polar_to_xyz(rm.R_PATH, 90.0, 0)[:2])]
        p = planners[algo0].plan(1, obs, h0, margin, z_margin)
        cv2.imwrite(out, render(rm.POSE_HOME, p, obs, h0, margin))
        print(f"[DA LUU] {out}: {p.kind} {p.reply}")
        return

    colors = [(0, 0, 180), (0, 140, 180), (0, 140, 0), (120, 60, 120), (150, 0, 150), (150, 90, 0), (110, 80, 0),
              (0, 0, 200)]
    buttons = make_buttons(LABELS, VIEW_W)
    st = {"bin": 1, "h": h0, "obs": [], "drag": None, "dirty": True, "plan": None, "anim": None, "quit": False,
          "algo": algo0}

    def on_mouse(event, u, v, flags, param):
        handle_mouse(st, event, u, v, flags, buttons)

    win = "Mo phong - Thuat toan ne vat can"
    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    while not st["quit"]:
        if st["dirty"]:
            st["plan"] = planners[st["algo"]].plan(st["bin"], st["obs"], st["h"], margin, z_margin)
            st["dirty"] = False
            print(f"[KE HOACH] Khay {st['bin']}: {st['plan'].reply} | {st['plan'].reason} | {st['plan'].ms:.0f}ms")

        q, holding, step = rm.POSE_HOME, False, ""
        if st["anim"] is not None:
            tl, t0 = st["anim"]
            t_ms = (time.time() - t0) * 1000.0
            q, _grip, holding, step = tl.at(t_ms)
            if t_ms > tl.total_ms:
                st["anim"] = None

        img = render(q, st["plan"], st["obs"], st["h"], margin, holding)
        frame = np.zeros((TOOLBAR_H + img.shape[0], VIEW_W, 3), dtype=np.uint8)
        frame[TOOLBAR_H:] = img
        cv2.putText(frame, f"Gui ESP32: {st['plan'].reply}"[:70], (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (255, 255, 255), 1)
        cv2.putText(frame, f"Trai: them vat | Phai+keo: di chuyen | Le {margin:.0f}/{z_margin:.0f}mm"
                    + (f" | Dang chay: {step}" if step else ""), (8, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                    (180, 180, 180), 1)
        for lbl, c in zip(LABELS, colors):
            show = f"CAO {height_label(st['h'])[:-2] if st['h'] < 9999.0 else '2D'}" if lbl == "CAO" else (
                f"> {lbl}" if lbl == f"KHAY {st['bin']}" else lbl)
            if lbl == "NE":
                show = st["algo"]
            draw_button(frame, buttons[lbl], show, c, 0.4)
        cv2.imshow(win, frame)
        key = cv2.waitKey(20) & 0xFF
        if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1 or key in (ord("q"), 27):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
