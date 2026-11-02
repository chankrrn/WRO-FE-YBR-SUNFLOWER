"""
Pure maths on one lidar scan: no hardware, so it can be tested offline on a
saved scan (run.py --load) or a synthetic one.

From a single 360-slot scan (LidarManager.get_scan(): mm, index = bearing,
0 ahead, +90 right, nan = nothing) this works out:

    heading      how far the nose is turned off the corridor, from the
                 field's walls all being square to each other
    outer wall   which side it is on and how far away (perpendicular)
    island       how far away its face is and where it starts/ends
    front wall   perpendicular distance ahead (used to check odometry)
    bay          the two magenta blades sticking out of the outer wall,
                 and the middle of the gap between them

FRAMES. "Robot frame" is x forward, y right, from the lidar. "Aligned frame"
is the same origin rotated by the heading, so the corridor runs along X and
the walls are lines of constant Y. heading_deg is + when the nose is turned
toward the right (clockwise seen from above), the same sense as a + (right)
steering command turns it.
"""
import math
from dataclasses import dataclass, field

import numpy as np


@dataclass
class ScanResult:
    ok: bool = False
    message: str = ""
    heading_deg: float = float("nan")
    heading_support: int = 0          # wall segments that agreed on it
    outer_side: int = 0               # +1 right, -1 left, 0 unknown
    outer_wall_mm: float = float("nan")
    island_mm: float = float("nan")
    island_x_mm: tuple = (float("nan"), float("nan"))   # its face, along X
    front_wall_mm: float = float("nan")
    obstacle_ahead_mm: float = float("nan")   # nearest thing in the robot's path
    obstacle_behind_mm: float = float("nan")  # ...and behind it, from the lidar
    rear_wall_mm: float = float("nan")
    blades_x_mm: list = field(default_factory=list)
    bays: list = field(default_factory=list)  # every (middle X, gap) that fits
    bay_center_x_mm: float = float("nan")
    bay_gap_mm: float = float("nan")
    points_xy: np.ndarray = None      # aligned frame, for drawing

    @property
    def side_name(self):
        return {1: "right", -1: "left"}.get(self.outer_side, "unknown")

    @property
    def has_bay(self):
        return not math.isnan(self.bay_center_x_mm)

    def summary(self):
        lines = [f"  heading      {self.heading_deg:+6.1f} deg off the corridor "
                 f"(+ = nose right, {self.heading_support} wall segments agree)",
                 f"  outer wall   {self.side_name}, {self.outer_wall_mm:6.0f} mm from the lidar"]
        if not math.isnan(self.island_mm):
            island_side = "left" if self.outer_side == 1 else "right"
            lines.append(f"  island       {island_side}, "
                         f"{self.island_mm:6.0f} mm, face runs X {self.island_x_mm[0]:+.0f} "
                         f"to {self.island_x_mm[1]:+.0f} mm")
        else:
            lines.append("  island       not seen")
        lines.append(f"  front wall   {self.front_wall_mm:6.0f} mm" if not math.isnan(self.front_wall_mm)
                     else "  front wall   not seen")
        if not math.isnan(self.obstacle_ahead_mm) and (
                math.isnan(self.front_wall_mm) or self.obstacle_ahead_mm < self.front_wall_mm - 50):
            lines.append(f"  in the path  something {self.obstacle_ahead_mm:.0f} mm ahead "
                         f"(nearer than the wall - a block?)")
        blades = ", ".join(f"{x:+.0f}" for x in self.blades_x_mm) or "none"
        lines.append(f"  rear wall    {self.rear_wall_mm:6.0f} mm" if not math.isnan(self.rear_wall_mm)
                     else "  rear wall    not seen")
        lines.append(f"  blades at X  {blades} mm")
        if self.has_bay:
            where = "ahead" if self.bay_center_x_mm > 0 else "behind"
            lines.append(f"  BAY          middle {abs(self.bay_center_x_mm):.0f} mm {where} of the lidar, "
                         f"gap {self.bay_gap_mm:.0f} mm")
        else:
            lines.append("  BAY          not found")
        return "\n".join(lines)


def _wrap45(deg):
    return (deg + 45.0) % 90.0 - 45.0


def scan_to_points(scan, min_mm, max_mm):
    """(bearing_deg, x, y) arrays for every valid slot, robot frame."""
    bearings = np.flatnonzero(~np.isnan(scan))
    d = scan[bearings]
    keep = (d >= min_mm) & (d <= max_mm)
    bearings, d = bearings[keep], d[keep]
    rad = np.radians(bearings)
    return bearings, d * np.cos(rad), d * np.sin(rad)


def estimate_heading(bearings, x, y, cfg):
    """
    Every field wall is parallel or perpendicular to the corridor, so the
    direction of short wall segments, folded mod 90, piles up at one angle.
    Returns (heading_deg, support).
    """
    if len(bearings) < 10:
        return float("nan"), 0
    step = int(cfg["segment_step"])
    order = np.argsort(bearings)
    b, x, y = bearings[order], x[order], y[order]
    j = (np.arange(len(b)) + step) % len(b)
    # Only pair points that really are `step` degrees apart (no gap in the
    # scan between them) and close enough to be the same surface.
    gap_deg = (b[j] - b) % 360
    dx, dy = x[j] - x, y[j] - y
    length = np.hypot(dx, dy)
    rng = np.hypot(x, y)
    gate = rng * math.radians(step) * cfg["segment_gate_factor"] + 30.0
    good = (gap_deg == step) & (length > 5.0) & (length < gate)
    if np.count_nonzero(good) < 5:
        return float("nan"), 0
    alpha = np.degrees(np.arctan2(dy[good], dx[good])) % 90.0

    hist = np.bincount(alpha.astype(int) % 90, minlength=90).astype(float)
    smooth = sum(np.roll(hist, k) for k in (-2, -1, 0, 1, 2))
    peak = float(np.argmax(smooth)) + 0.5
    near = np.abs(_wrap45(alpha - peak)) <= cfg["heading_window_deg"]
    if np.count_nonzero(near) < 3:
        return float("nan"), 0
    # circular mean on the 90-degree circle
    ang4 = np.radians(alpha[near] * 4.0)
    mean = math.degrees(math.atan2(np.sin(ang4).mean(), np.cos(ang4).mean())) / 4.0
    # A wall along the corridor appears in the robot frame at -heading.
    return _wrap45(-mean), int(np.count_nonzero(near))


def line_peaks(values, min_points, bin_mm=20.0, max_mm=5000.0):
    """
    Distances (values all positive) where a solid line of points sits, as
    [(distance_mm, points)] nearest first, each refined to the median of the
    points around it. Weak peaks (under a quarter of the strongest) are
    dropped, so a blade tip or a corner cannot pass for a wall.
    """
    values = values[(values > 0) & (values < max_mm)]
    if len(values) < min_points:
        return []
    hist = np.bincount((values / bin_mm).astype(int), minlength=int(max_mm / bin_mm) + 2)
    smooth = hist + np.roll(hist, 1) + np.roll(hist, -1)
    peaks = []
    i = 0
    while i < len(smooth):
        if smooth[i] < min_points:
            i += 1
            continue
        end = i
        while end + 1 < len(smooth) and smooth[end + 1] >= min_points:
            end += 1
        centre = (i + int(np.argmax(smooth[i:end + 1]))) * bin_mm + bin_mm / 2
        close = values[np.abs(values - centre) <= 2 * bin_mm]
        peaks.append((float(np.median(close)), len(close)))
        i = end + 1
    if not peaks:
        return []
    strongest = max(n for _, n in peaks)
    return [(d, n) for d, n in peaks if n >= max(min_points, strongest / 4)]


def farthest_line(values, min_points):
    peaks = line_peaks(values, min_points)
    return peaks[-1][0] if peaks else float("nan")


def along_corridor(bearings, X, Y):
    """
    True for points on a surface running ALONG the corridor (or with no
    neighbour to tell), False for one running across it, judged from the
    neighbouring points in the aligned frame. Close up, a wall across the
    corridor is dense enough to pass for a line at every sideways distance,
    so the side walls are only looked for in the True points.
    """
    n = len(bearings)
    along = np.ones(n, bool)
    if n < 3:
        return along
    rng = np.hypot(X, Y)
    dx_sum, dy_sum, known = np.zeros(n), np.zeros(n), np.zeros(n, bool)
    for step in (1, -1):
        j = (np.arange(n) + step) % n
        gap = (bearings[j] - bearings) * step % 360
        dx, dy = np.abs(X[j] - X), np.abs(Y[j] - Y)
        ok = (gap >= 1) & (gap <= 3) & (np.hypot(dx, dy) < rng * math.radians(3) * 3 + 30)
        dx_sum += np.where(ok, dx, 0.0)
        dy_sum += np.where(ok, dy, 0.0)
        known |= ok
    along[known] = dx_sum[known] >= dy_sum[known]
    return along


def nearest_line(values, min_points):
    peaks = line_peaks(values, min_points)
    return peaks[0][0] if peaks else float("nan")


def refine_heading(X, Y, lines, heading):
    """
    Total-least-squares fit through the points on the found wall lines
    (each line's own offset removed), as a correction to the heading.
    """
    dx, dy = [], []
    for y_line in lines:
        on = (np.abs(Y - y_line) <= 30.0) & (np.abs(X) < 1500.0)
        if np.count_nonzero(on) >= 5:
            dx.append(X[on] - X[on].mean())
            dy.append(Y[on] - Y[on].mean())
    if not dx:
        return heading
    dx, dy = np.concatenate(dx), np.concatenate(dy)
    angle = 0.5 * math.degrees(math.atan2(2 * np.sum(dx * dy),
                                          np.sum(dx * dx) - np.sum(dy * dy)))
    # a wall that looks tilted by +angle means the heading is that much too big
    return heading - angle


def analyze(scan, cfg, outer_side=0, find_bay=True):
    """
    I/O:
        scan:       360-slot array from LidarManager.get_scan()
        cfg:        the [scan] table of config.toml as a dict
        outer_side: +1 / -1 to force the outer wall's side, 0 to work it out
        find_bay:   False skips the blade search (the drive loop's fast path)
        return:     ScanResult
    """
    result = ScanResult()
    bearings, x, y = scan_to_points(np.asarray(scan, dtype=float),
                                    cfg["min_range_mm"], cfg["max_range_mm"])
    if len(bearings) < cfg["min_points"]:
        result.message = f"only {len(bearings)} lidar points - is it spinning?"
        return result

    heading, support = estimate_heading(bearings, x, y, cfg)
    result.heading_deg, result.heading_support = heading, support
    if math.isnan(heading):
        result.message = "no straight walls in the scan"
        return result

    def aligned(heading):
        h = math.radians(heading)
        return x * math.cos(h) - y * math.sin(h), x * math.sin(h) + y * math.cos(h)

    def find_walls(X, Y):
        """
        Per side: (wall, nearest line, points behind the wall).

        Only points on surfaces running along the corridor count. The lines
        within a blade's length of the nearest one form a group (blade tips
        and the wall behind them, or the wall and something seen over it);
        the wall is the STRONGEST line in that group.
        """
        in_window = (np.abs(X) < cfg["side_window_mm"]) & along_corridor(bearings, X, Y)
        found = {}
        for side in (1, -1):
            sy = side * Y
            peaks = line_peaks(sy[in_window & (sy > cfg["min_range_mm"])], min_wall)
            if peaks:
                near = peaks[0][0]
                group = [peaks[0]]
                for d, n in peaks[1:]:
                    if d - group[-1][0] > cfg["blade_length_mm"] + 80.0:
                        break
                    group.append((d, n))
                wall = max(group, key=lambda p: p[1])[0]
                found[side] = (wall, near,
                               int(np.count_nonzero(sy > wall + cfg["beyond_margin_mm"])))
        return found

    def classify(walls, outer_side):
        """
        Blades only stand on the outer wall, so if a bay fits on exactly one
        side, that is the outer wall. Otherwise: the outer wall has nothing
        behind it; past the island there is the rest of the field.
        Returns (side, None) or (0, why not).
        """
        if outer_side != 0:
            return outer_side, None
        if not walls:
            return 0, "no wall found on either side"
        with_bay = [side for side, (wall, _, _) in walls.items()
                    if blades_and_bays(side, wall, X, Y, cfg)[1]]
        if len(with_bay) == 1:
            return with_bay[0], None
        if len(walls) == 1:
            (only, (_, _, behind)), = walls.items()
            return (only if behind < cfg["beyond_max_points"] else -only), None
        if walls[1][2] == walls[-1][2]:
            return 0, "cannot tell the outer wall from the island - set outer_side"
        return min(walls, key=lambda s: walls[s][2]), None

    min_wall = int(cfg["min_wall_points"])
    for _ in range(3):
        X, Y = aligned(heading)
        walls = find_walls(X, Y)
        side, why = classify(walls, outer_side)
        if side == 0:
            break
        # Refine on the two lines that really run the whole way along: the
        # outer wall (farthest on its side) and the island's face (nearest).
        lines = []
        if side in walls:
            lines.append(side * walls[side][0])
        if -side in walls:
            lines.append(-side * walls[-side][1])
        heading = _wrap45(refine_heading(X, Y, lines, heading))
    X, Y = aligned(heading)
    walls = find_walls(X, Y)
    outer_side, why = classify(walls, outer_side)
    result.heading_deg = heading
    result.points_xy = np.column_stack([X, Y])
    if outer_side == 0:
        result.message = why
        return result
    result.outer_side = outer_side
    if outer_side not in walls:
        result.message = f"no wall on the {result.side_name} side"
        return result
    result.outer_wall_mm = walls[outer_side][0]

    if -outer_side in walls:
        island = walls[-outer_side][1]
        result.island_mm = island
        on_face = np.abs(-outer_side * Y - island) <= 40.0
        if np.count_nonzero(on_face) >= 2:
            result.island_x_mm = (float(X[on_face].min()), float(X[on_face].max()))

    # The end walls are searched across the whole corridor, from the island
    # face to just short of the blades, and the FARTHEST line wins: a block
    # in the corridor only hides a slice of the wall, and nothing is ever
    # beyond the wall itself.
    u = outer_side * Y                      # + toward the outer wall
    inner = result.island_mm - 40.0 if not math.isnan(result.island_mm) else cfg["side_window_mm"]
    across = (u > -inner) & (u < result.outer_wall_mm - cfg["blade_length_mm"] - 60.0)
    min_end = max(3, min_wall // 2)
    result.front_wall_mm = farthest_line(X[across & (X > cfg["min_range_mm"])], min_end)
    result.rear_wall_mm = farthest_line(-X[across & (X < -cfg["min_range_mm"])], min_end)
    # Anything at all in the robot's own path, wall or block.
    # What is in the robot's own path, along the way IT points (robot frame
    # x, y) - not along the corridor, which is a different direction once
    # the robot is turned (into the bay, say).
    in_lane = np.abs(y) < cfg["front_half_width_mm"]
    # The bay's own blades stick out from the outer wall into that lane when
    # the robot runs close to the wall. They are not in the way as long as
    # the robot's body clears their tips, so then they are left out.
    clear_of_tips = (result.outer_wall_mm - cfg["robot_half_width_mm"]
                     > cfg["blade_length_mm"] + cfg["blade_clearance_mm"])
    if clear_of_tips:
        depth = result.outer_wall_mm - outer_side * Y
        in_lane &= depth > cfg["blade_length_mm"] + 40.0
    path = (x > cfg["min_range_mm"]) & in_lane
    if np.any(path):
        result.obstacle_ahead_mm = float(x[path].min())
    # Behind, only what is past the TAIL: anything nearer than that is
    # beside the body (a blade, the wall), not in the way backward.
    path = (x < -cfg["lidar_to_tail_mm"]) & in_lane
    if np.any(path):
        result.obstacle_behind_mm = float(-x[path].max())

    if find_bay:
        _find_bay(result, X, Y, cfg)
    result.ok = True
    result.message = "ok" if result.has_bay else "walls ok, bay not found"
    return result


def blades_and_bays(side, wall_mm, X, Y, cfg, ahead=None, back=None):
    """
    The blades stand out from the outer wall by up to blade_length_mm. Their
    points sit between the wall and that far in, bunched at one X each.

    I/O:
        side, wall_mm: which side the outer wall is on, and how far
        ahead, back:   search limits along X (default bay_search_mm)
        return:        (blade centres X, [(bay middle X, gap)])
    """
    reach = cfg["bay_search_mm"]
    ahead = reach if ahead is None else ahead
    back = reach if back is None else back
    depth = wall_mm - side * Y   # how far in from the wall
    band = ((depth > cfg["blade_min_depth_mm"])
            & (depth < cfg["blade_length_mm"] + 60.0)
            & (X < ahead) & (X > -back))
    order = np.argsort(X[band])
    xs, ds = X[band][order], depth[band][order]
    half = cfg["blade_thickness_mm"] / 2
    blades = []
    if len(xs):
        cuts = np.flatnonzero(np.diff(xs) > cfg["blade_gap_mm"]) + 1
        for g, gd in zip(np.split(xs, cuts), np.split(ds, cuts)):
            if (g[-1] - g[0]) > cfg["blade_max_width_mm"]:
                # Wider than a blade: keep its densest part if that holds a
                # good share of the points (a blade with a stray trail of
                # returns beside it), else it is not a blade.
                w = cfg["blade_max_width_mm"]
                counts = [np.count_nonzero((g >= x0) & (g <= x0 + w)) for x0 in g]
                best = int(np.argmax(counts))
                if counts[best] < 0.4 * len(g):
                    continue
                keep = (g >= g[best]) & (g <= g[best] + w)
                g, gd = g[keep], gd[keep]
            # A blade stands OUT from the wall: some of its points reach well
            # out (its face, or just its tip when it is right beside the
            # lidar). A few stray returns near the wall (something lying in
            # the bay) do not.
            if len(g) >= cfg["blade_min_points"] and np.max(gd) >= cfg["blade_min_reach_mm"]:
                face = float(np.median(g))
                # the lidar sees the face turned toward it: the centre is
                # half a thickness further away. Right beside the lidar it
                # sees the tip instead, which is already the centre.
                blades.append(face + math.copysign(half, face) if abs(face) > 50.0 else face)

    want, tol = cfg["bay_gap_mm"], cfg["bay_gap_tolerance_mm"]
    bays = []
    for i, a in enumerate(blades):      # every pair, not just neighbours
        for b in blades[i + 1:]:
            gap = b - a - 2 * half      # inside face to inside face
            if abs(gap - want) <= tol:
                bays.append(((a + b) / 2, gap))
    return blades, bays


def _find_bay(result, X, Y, cfg):
    reach = cfg["bay_search_mm"]
    # where the outer wall meets the front/back wall looks like a blade too
    ahead = min(reach, result.front_wall_mm - 60.0) if not math.isnan(result.front_wall_mm) else reach
    back = min(reach, result.rear_wall_mm - 60.0) if not math.isnan(result.rear_wall_mm) else reach
    result.blades_x_mm, result.bays = blades_and_bays(
        result.outer_side, result.outer_wall_mm, X, Y, cfg, ahead, back)
    if result.bays:
        result.bay_center_x_mm, result.bay_gap_mm = min(result.bays, key=lambda b: abs(b[0]))


def draw(result, path, cfg, size_px=800, range_mm=2000.0):
    """PNG of the aligned frame: points, walls, blades, bay. For tuning over SSH."""
    import cv2

    img = np.full((size_px, size_px, 3), 255, np.uint8)
    scale = size_px / (2 * range_mm)

    def px(X, Y):   # X up the image, Y to the right
        return int(size_px / 2 + Y * scale), int(size_px / 2 - X * scale)

    for r in range(500, int(range_mm * 1.5), 500):
        cv2.circle(img, px(0, 0), int(r * scale), (230, 230, 230), 1)
    if result.points_xy is not None:
        for X, Y in result.points_xy:
            cv2.circle(img, px(X, Y), 2, (90, 90, 90), -1)
    s = result.outer_side
    if not math.isnan(result.outer_wall_mm):
        cv2.line(img, px(-range_mm, s * result.outer_wall_mm),
                 px(range_mm, s * result.outer_wall_mm), (200, 120, 0), 2)
    if not math.isnan(result.island_mm):
        a, b = result.island_x_mm
        cv2.line(img, px(a, -s * result.island_mm), px(b, -s * result.island_mm), (0, 150, 0), 2)
    for bx in result.blades_x_mm:
        cv2.line(img, px(bx, s * result.outer_wall_mm),
                 px(bx, s * (result.outer_wall_mm - cfg["blade_length_mm"])), (200, 0, 200), 3)
    if result.has_bay:
        cv2.drawMarker(img, px(result.bay_center_x_mm, s * (result.outer_wall_mm - 100)),
                       (200, 0, 200), cv2.MARKER_CROSS, 24, 2)
    # the robot, nose drawn at its real heading
    h = math.radians(result.heading_deg) if not math.isnan(result.heading_deg) else 0.0
    cv2.circle(img, px(0, 0), 6, (0, 0, 220), -1)
    cv2.line(img, px(0, 0), px(250 * math.cos(h), 250 * math.sin(h)), (0, 0, 220), 2)
    cv2.putText(img, f"heading {result.heading_deg:+.1f}  outer {result.side_name} "
                f"{result.outer_wall_mm:.0f}mm", (10, 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 0, 0), 1)
    cv2.imwrite(str(path), img)
