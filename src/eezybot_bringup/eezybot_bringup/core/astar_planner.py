"""
astar_planner.py - THUẬT TOÁN NÉ VẬT CẢN A* (PHƯƠNG ÁN 2, SO SÁNH VỚI avoid_planner.py)
-------------------------------------------------------------------------------------
Cùng đầu vào / đầu ra với avoid_planner.plan() (Plan, câu trả lời CLEAR / PATH / BLOCKED cho ESP32)
-> 06_auto_run.py và sim_view.py đổi qua lại bằng nút [NE], firmware KHÔNG phải sửa gì.

KHÔNG GIAN TÌM KIẾM: lưới 2 chiều (góc đế × bán kính r) ở một độ cao z cố định
  - Cột : góc đế BASE_DEG_MIN -> BASE_DEG_MAX (55.1° -> 126.7°), bước ~2° (thêm đúng góc gắp và góc 3 khay)
  - Hàng: bán kính r = R_PATH ± k*R_STEP_MM, trong dải IK với tới được ở độ cao z, vòng ngoài không quá
    avoid_planner.detour_r_max() (camera còn thấy vật ở đó: detour_r_max_mm / scan_r_max_mm - safe_margin_mm)
  - Mỗi nút nối 8 nút xung quanh. Cạnh = đi thẳng trong (r, góc, z), giống moveArcLinear() của firmware
  - Thêm 2 nút ảo: S = trên điểm gắp (nối tới MỌI nút của cột góc gắp),
                   G = trên khay      (nối từ MỌI nút của cột góc khay)
  - Chi phí cạnh g = độ dài đường tâm kẹp (mm) = sqrt(dr² + (r_tb * dgóc)² + dz²)   (giống legMs())
  - Heuristic   h = sqrt(dr² + (r_nhỏ_nhất * dgóc)² + dz²) từ nút tới G: không bao giờ lớn hơn chi phí thật
    -> A* (luôn mở rộng nút có f = g + h nhỏ nhất) chắc chắn tìm ra đường NGẮN NHẤT trên lưới.

CÁCH LÀM:
  0, 1. Giống avoid_planner: đoạn không né được chạm vật -> BLOCKED; đường mặc định an toàn -> CLEAR.
  2. Với từng độ cao z trong Z_LEVELS (thấp trước, cao sau):
       a. Đánh dấu nút / cạnh chạm vật cản (đoạn thẳng thân tay + quả treo, luật chung robot_model.segments_hit
          = robot_model.check_collision)
       b. A* từ S tới G
       c. Rút gọn đường (kéo dây): bỏ điểm giữa nếu đi thẳng vẫn an toàn -> ít điểm né (<= 6, giới hạn giao thức).
          Mỗi đoạn được kiểm tra với ĐÚNG 31 mẫu như robot_model.sample_moves -> kiểm tra lại cả chu trình sẽ qua.
          Cạnh lưới (chỉ lấy mẫu ở 2 đầu + trung điểm) mà kiểm tra dày bị chạm -> đánh dấu cạnh đó, chạy lại A*.
       d. Làm tròn điểm né 0.1 như câu PATH gửi đi (protocol.format_path), rồi kiểm tra lại CẢ CHU TRÌNH
          bằng robot_model.check_collision -> PATH (kiểm tra đúng đường ESP32 sẽ chạy).
          Đường về thẳng (khay -> HOME) không chạm -> PATH ... H (về thẳng, giống avoid_planner).
  3. Mọi độ cao đều không có đường -> BLOCKED. Tính quá DEADLINE_MS (0.6 x PC_TIMEOUT_MS) -> BLOCKED cho an toàn.
Mọi đường của avoid_planner (S -> (r, góc gắp) -> cung r -> (r, góc khay) -> G) đều nằm trong lưới này,
nên A* tìm được ở mọi chỗ avoid_planner tìm được, với đường NGẮN hơn (cắt chéo thay vì đi vòng hết cung).
Về nguyên tắc A* còn đi zig-zag được (vòng ngoài chỗ này, vòng trong chỗ khác), nhưng với tay này (dải r
140-372mm rất rộng + được nâng cao) thử 1900 kịch bản chưa gặp chỗ nào CHỈ A* mới qua được
(python so_sanh_planner.py để xem số liệu).
Phần tốn thời gian (IK + đoạn thân tay của mọi nút / cạnh) tính sẵn 1 lần (warm_up); mỗi lần hỏi chỉ đo
khoảng cách tới vật cản bằng numpy rồi chạy A* trên vài trăm nút.
"""

import functools
import heapq
import math
import time

import numpy as np

import avoid_planner as ap
import protocol
import robot_model as rm

DEG_STEP = 2.0      # Bước góc đế của lưới (độ)
R_STEP_MM = 10.0    # Bước bán kính của lưới (mm)
Z_LEVELS = ap.Z_LEVELS
LAZY_TRIES = 20     # Số lần chạy lại A* tối đa sau khi loại cạnh lưới bị chạm lúc kiểm tra dày
NEIGHBORS = ((1, 0), (0, 1), (1, 1), (1, -1))  # Cạnh vô hướng: mỗi cặp nút chỉ lưu 1 lần
START, GOAL = -1, -2  # Nút ảo: trên điểm gắp / trên khay


def leg_len(pa, pb):
    """Độ dài đường tâm kẹp khi đi thẳng pa -> pb trong (r, góc, z) - cùng cách đo với rm.leg_ms()."""
    arc = (pa[0] + pb[0]) / 2.0 * math.radians(abs(pb[1] - pa[1]))
    return math.sqrt((pb[0] - pa[0]) ** 2 + arc ** 2 + (pb[2] - pa[2]) ** 2)


def path_len(bin_id, waypoints):
    """Độ dài đường mang quả: trên điểm gắp -> các điểm né -> trên khay (mm)."""
    pts = [rm.P_LIFT] + [tuple(w) for w in waypoints] + [rm.above_bin(bin_id)]
    return sum(leg_len(a, b) for a, b in zip(pts[:-1], pts[1:]))


def _pose_points(q):
    """Các đoạn thẳng thân tay (vai -> tâm kẹp) + quả treo dưới tâm kẹp (đường né luôn đang mang quả):
    mảng POSE_SHAPE = (S, 2, 3) - đúng các đoạn rm.check_collision kiểm tra."""
    return rm.pose_segments(rm.arm_joints(q), carrying=True)


POSE_SHAPE = _pose_points(rm.POSE_HOME).shape  # Lấy từ hàm chung, không viết cứng số đoạn / số điểm


def _hit_mask(segs, obstacles, obs_h, xy_margin, z_margin):
    """segs (M, ..., 2, 3) -> (M,) True nếu có đoạn chạm trụ vật cản đã phình lề (luật chung rm.segments_hit)."""
    if segs.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    hit = rm.segments_hit(segs, obstacles, obs_h, xy_margin, z_margin)
    return hit.reshape(segs.shape[0], -1).any(axis=1)


def sample_segment(pa, pb):
    """Đoạn thân tay + quả dọc đường thẳng pa -> pb, ĐÚNG các mẫu s = k/N_SAMPLES như rm.sample_moves.
    Trả về mảng (N_SAMPLES + 1,) + POSE_SHAPE, hoặc None nếu có điểm IK không tới được."""
    n = ap.N_SAMPLES
    joints = []
    for k in range(n + 1):
        s = k / float(n)
        good, q, _ = rm.ik_polar(*(a + (b - a) * s for a, b in zip(pa, pb)))
        if not good:
            return None
        joints.append(rm.arm_joints(q))
    return rm.pose_segments(joints, carrying=True)


def segment_free(pa, pb, args):
    pts = sample_segment(pa, pb)
    return pts is not None and not _hit_mask(pts, *args).any()


class Links:
    """Các đoạn nối nút ảo (trên điểm gắp / trên khay) với một cột của lưới, đã lấy mẫu sẵn."""

    def __init__(self, ids, pts, costs):
        self.ids, self.costs = ids, costs
        seg = POSE_SHAPE[1:]  # (2, 3): 1 đoạn thẳng = 2 đầu mút
        self.pts = np.array(pts).reshape((len(ids), -1) + seg) if ids else np.zeros((0, 1) + seg)

    def free(self, args):
        """{nút: chi phí} các đoạn nối KHÔNG chạm vật."""
        hit = _hit_mask(self.pts, *args)
        return {n: c for n, c, h in zip(self.ids, self.costs, hit) if not h}


class Grid:
    """Lưới (góc × bán kính) ở độ cao z. Đoạn thân tay + quả của mọi nút, trung điểm mọi cạnh
    và các đoạn nối với nút ảo được tính sẵn (không phụ thuộc vật cản)."""

    def __init__(self, z):
        self.z = z
        self.ok = False
        band = rm.reach_band(z)
        if band is None:
            return
        k_lo = math.ceil((band[0] - rm.R_PATH) / R_STEP_MM)
        k_hi = math.floor((min(band[1], ap.detour_r_max()) - rm.R_PATH) / R_STEP_MM)  # Cùng giới hạn vòng ngoài
        self.rs = [rm.R_PATH + k * R_STEP_MM for k in range(k_lo, k_hi + 1)]
        n = int(round((rm.BASE_DEG_MAX - rm.BASE_DEG_MIN) / DEG_STEP))
        degs = {round(float(d), 6) for d in np.linspace(rm.BASE_DEG_MIN, rm.BASE_DEG_MAX, n + 1)}
        degs |= {round(rm.DEG_PICK, 6)} | {round(b["deg"], 6) for b in rm.BINS.values()}
        self.degs = sorted(degs)
        nd, nr = len(self.degs), len(self.rs)
        self.nr = nr

        # Nút: id = i_góc * nr + j_bán_kính
        self.node_ok = np.zeros(nd * nr, dtype=bool)
        self.node_pts = np.zeros((nd * nr,) + POSE_SHAPE)
        for i, d in enumerate(self.degs):
            for j, r in enumerate(self.rs):
                good, q, _ = rm.ik_polar(r, d, z)
                if good:
                    self.node_ok[i * nr + j] = True
                    self.node_pts[i * nr + j] = _pose_points(q)

        # Cạnh: 2 đầu đã là nút; lưu thêm trung điểm để không lọt vật nhỏ nằm giữa 2 nút
        self.adj = [[] for _ in range(nd * nr)]
        self.edge_id = {}
        mids = []
        for i in range(nd):
            for j in range(nr):
                a = i * nr + j
                if not self.node_ok[a]:
                    continue
                for di, dj in NEIGHBORS:
                    i2, j2 = i + di, j + dj
                    if i2 >= nd or not 0 <= j2 < nr or not self.node_ok[i2 * nr + j2]:
                        continue
                    b = i2 * nr + j2
                    pa, pb = self.polar(a), self.polar(b)
                    good, q, _ = rm.ik_polar(*((u + v) / 2.0 for u, v in zip(pa, pb)))
                    if not good:
                        continue
                    e, cost = len(mids), leg_len(pa, pb)
                    mids.append(_pose_points(q))
                    self.adj[a].append((b, e, cost))
                    self.adj[b].append((a, e, cost))
                    self.edge_id[(min(a, b), max(a, b))] = e
        self.mid_pts = np.array(mids) if mids else np.zeros((0,) + POSE_SHAPE)

        # Nút ảo: trên điểm gắp -> cột góc gắp; cột góc khay -> trên khay
        self.start_links = self._links(rm.DEG_PICK, lambda p: (rm.P_LIFT, p))
        self.goal_links = {b: self._links(bd["deg"], lambda p, b=b: (p, rm.above_bin(b))) for b, bd in rm.BINS.items()}
        self.ok = True

    def _links(self, deg, ends):
        i = self.degs.index(round(deg, 6))
        ids, pts, costs = [], [], []
        for j in range(self.nr):
            n = i * self.nr + j
            if not self.node_ok[n]:
                continue
            seg = sample_segment(*ends(self.polar(n)))
            if seg is not None:
                ids.append(n)
                pts.append(seg)
                costs.append(leg_len(*ends(self.polar(n))))
        return Links(ids, pts, costs)

    def polar(self, n):
        """id nút -> (r, góc, z)."""
        return self.rs[n % self.nr], self.degs[n // self.nr], self.z

    def blocked(self, obstacles, obs_h, xy_margin, z_margin):
        """(nút bị chặn, cạnh bị chặn) - mảng bool, tính bằng numpy cho mọi nút / cạnh cùng lúc."""
        node_hit = _hit_mask(self.node_pts, obstacles, obs_h, xy_margin, z_margin) | ~self.node_ok
        return node_hit, _hit_mask(self.mid_pts, obstacles, obs_h, xy_margin, z_margin)


@functools.lru_cache(maxsize=None)
def grid_at(z):
    return Grid(z)


def astar(start, goal, neighbors, h):
    """A* tổng quát. neighbors(n) -> [(nút kề, chi phí)], h(n) -> ước lượng chi phí còn lại.
    Trả về (danh sách nút từ start tới goal, số nút đã mở rộng); không có đường -> (None, ...)."""
    g = {start: 0.0}
    parent = {start: None}
    open_heap = [(h(start), 0.0, start)]  # (f = g + h, g, nút)
    closed = set()
    while open_heap:
        _f, g_n, n = heapq.heappop(open_heap)
        if n in closed:
            continue  # Bản cũ (đã có đường tốt hơn tới nút này)
        closed.add(n)
        if n == goal:
            path = []
            while n is not None:
                path.append(n)
                n = parent[n]
            return path[::-1], len(closed)
        for m, cost in neighbors(n):
            if m in closed:
                continue
            g_m = g_n + cost
            if g_m < g.get(m, math.inf):
                g[m] = g_m
                parent[m] = n
                heapq.heappush(open_heap, (g_m + h(m), g_m, m))
    return None, len(closed)


def shortcut(pts, args):
    """Kéo dây: từ mỗi điểm, nối thẳng tới điểm XA NHẤT phía trước mà đoạn thẳng vẫn an toàn.
    Trả về (đường đã rút gọn, None), hoặc (None, i) nếu ngay đoạn pts[i] -> pts[i+1] đã chạm (cạnh lưới bị lọt)."""
    out = [pts[0]]
    i = 0
    while i < len(pts) - 1:
        if not segment_free(pts[i], pts[i + 1], args):
            return None, i
        j = i + 1
        while j + 1 < len(pts) and segment_free(pts[i], pts[j + 1], args):
            j += 1
        out.append(pts[j])
        i = j
    return out, None


def _search_z(bin_id, z, args, late=lambda: False):
    """Tìm đường ở độ cao z. Trả về (điểm né hoặc None, số nút đã mở rộng, lý do thất bại).
    late() -> True khi đã hết thời gian tính (dừng, coi như không có đường)."""
    grid = grid_at(z)
    if not grid.ok:
        return None, 0, "khong voi toi"
    node_hit, edge_hit = grid.blocked(*args)
    starts = grid.start_links.free(args)
    goals = grid.goal_links[bin_id].free(args)
    lift, above = rm.P_LIFT, rm.above_bin(bin_id)
    r_lo = grid.rs[0]

    def polar(n):
        return lift if n == START else (above if n == GOAL else grid.polar(n))

    def h(n):
        r, deg, zz = polar(n)
        return math.sqrt((r - above[0]) ** 2 + (r_lo * math.radians(abs(deg - above[1]))) ** 2 + (zz - above[2]) ** 2)

    def neighbors(n):
        if n == START:
            return [(m, c) for m, c in starts.items() if not node_hit[m]]
        out = [(m, c) for m, e, c in grid.adj[n] if not node_hit[m] and not edge_hit[e]]
        if n in goals:
            out.append((GOAL, goals[n]))
        return out

    expanded = 0
    for _ in range(LAZY_TRIES):
        if late():
            return None, expanded, "het thoi gian"
        nodes, n_exp = astar(START, GOAL, neighbors, h)
        expanded += n_exp
        if nodes is None:
            return None, expanded, "A* khong co duong"
        # Bỏ điểm trùng (ở z = Z_TRAVEL nút lưới (R_PATH, góc gắp) trùng S) - giữ id nút lưới
        pairs = []
        for n in nodes:
            p = polar(n)
            if pairs and all(abs(u - v) < 1e-6 for u, v in zip(p, pairs[-1][0])):
                if n >= 0:
                    pairs[-1] = (p, n)
                continue
            pairs.append((p, n))
        simple, bad = shortcut([p for p, _n in pairs], args)
        if bad is None:
            break
        a, b = pairs[bad][1], pairs[bad + 1][1]  # Đoạn lưới kiểm tra thưa thì qua, kiểm tra dày thì chạm -> loại, tìm lại
        if a == START:
            starts.pop(b, None)
        elif b == GOAL:
            goals.pop(a, None)
        else:
            edge_hit[grid.edge_id[(min(a, b), max(a, b))]] = True
    else:
        return None, expanded, f"loai {LAZY_TRIES} canh van cham"

    waypoints = simple[1:-1]
    if not waypoints:
        # Đi thẳng được (đúng cung mặc định) nhưng đường về mặc định (nội suy khớp) chạm vật
        # -> gửi 1 điểm né trùng "trên khay" để ESP32 đi về NGƯỢC lại đúng cung đó
        waypoints = [simple[-1]]
    if len(waypoints) > protocol.MAX_WAYPOINTS:
        return None, expanded, f"{len(waypoints)} diem ne > {protocol.MAX_WAYPOINTS}"
    return waypoints, expanded, ""


def warm_up():
    """Tính sẵn lưới mọi độ cao + các đoạn cố định (vài giây). Gọi lúc khởi động để lần REQ đầu cũng nhanh."""
    for z in Z_LEVELS:
        grid_at(z)
    for b in rm.BINS:
        ap._moves_pack("fixed", b)
        ap._moves_pack("default", b)
        ap._moves_pack("home", b)
        ap._sampled("cycle", b)


def plan(bin_id, obstacles, obs_height_mm, xy_margin_mm=22.0, z_margin_mm=15.0, deadline_ms=ap.DEADLINE_MS):
    t0 = time.perf_counter()
    args = (rm.obstacle_arrays(obstacles), obs_height_mm, xy_margin_mm, z_margin_mm)  # Đổi sang numpy 1 lần

    def late():
        return (time.perf_counter() - t0) * 1000.0 > deadline_ms

    def finish(kind, waypoints, reason, tried, cycle=None, results=None, direct=False):
        moves, sampled = cycle or ap._sampled("cycle", bin_id)
        reply = protocol.format_path(waypoints, direct) if kind == "PATH" else kind
        if results is None:
            results = rm.check_collision(sampled, *args)
        return ap.Plan(kind, reply, list(waypoints or []), reason, moves, results, tried,
                       (time.perf_counter() - t0) * 1000.0, "A*", direct)

    if not len(args[0][1]):
        return finish("CLEAR", None, "Khong co vat can", 0)

    # 0, 1. Giống avoid_planner (dùng chung phần lấy mẫu sẵn) -> so sánh 2 thuật toán công bằng
    fixed_hits = ap._hits_key("fixed", bin_id, args)
    if fixed_hits:
        return finish("BLOCKED", None, f"Vat can o doan khong ne duoc: {', '.join(fixed_hits)}", 0)
    if not ap._hits_key("default", bin_id, args):
        return finish("CLEAR", None, "Duong mac dinh an toan", 0)

    # 2. A* ở từng độ cao, thấp trước
    expanded, fails = 0, []
    for z in Z_LEVELS:
        if late():
            fails.append("het thoi gian")
            break
        waypoints, n, why = _search_z(bin_id, z, args, late)
        expanded += n
        if waypoints is None:
            fails.append(f"z{z:.0f}: {why}")
            continue
        # Làm tròn như câu PATH gửi đi -> kiểm tra ĐÚNG đường ESP32 sẽ chạy
        waypoints = protocol.parse_path(protocol.format_path(waypoints))
        # Kiểm tra độc lập cả chu trình (cùng hàm với 05_check_collision.py). Đường về thẳng
        # (khay -> HOME) không chạm -> về thẳng; ngược lại về theo đúng các điểm né.
        direct = ap.direct_home_ok(bin_id, args)
        try:
            moves = rm.cycle_moves(bin_id, waypoints, direct)
        except ValueError as e:
            fails.append(f"z{z:.0f}: {e}")
            continue
        sampled = rm.sample_moves(moves, ap.N_SAMPLES)
        results = rm.check_collision(sampled, *args)
        hit = [res["move"].name for res in results if res["hit"]]
        if hit:
            fails.append(f"z{z:.0f}: kiem tra lai bi cham ({', '.join(hit)})")
            continue
        how = f"A* z={z:.0f}, {len(waypoints)} diem ne, dai {path_len(bin_id, waypoints):.0f}mm"
        if direct:
            how += ", ve thang HOME"
        return finish("PATH", waypoints, how, expanded, (moves, sampled), results, direct)

    # 3. Hết cách (hoặc hết thời gian tính)
    if late():
        return finish("BLOCKED", None, f"Tinh qua {deadline_ms:.0f}ms -> BLOCKED cho an toan (" + "; ".join(fails) + ")",
                      expanded)
    return finish("BLOCKED", None, "A* khong co duong (" + "; ".join(fails) + ")", expanded)
