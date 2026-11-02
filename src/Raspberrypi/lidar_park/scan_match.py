"""
How far the robot really moved between two scans, from the scans alone.

The field does not move, so if the robot moves, every wall in the new scan
shows up shifted and turned by the opposite amount. Lining the new scan up
with an earlier one (ICP - iterative closest point: pair every new point
with the nearest old one, find the shift and turn that best lines the pairs
up, repeat) gives that shift and turn. Unlike the wall-based odometry it
does not care which way the robot is pointing, so it keeps working through
the tight turns of a tuck into the bay.

Frames are LidarManager's robot frame: x forward, y right, in mm; turns are
+ clockwise seen from above (nose to the right), like heading_deg.
"""
import math

import numpy as np


def scan_points(scan, min_mm, max_mm, max_points=150):
    """(N, 2) array of x, y for the valid readings, thinned to about max_points."""
    scan = np.asarray(scan, dtype=float)
    bearings = np.flatnonzero(~np.isnan(scan))
    d = scan[bearings]
    keep = (d >= min_mm) & (d <= max_mm)
    bearings, d = bearings[keep], d[keep]
    step = max(1, len(d) // max_points)
    bearings, d = bearings[::step], d[::step]
    rad = np.radians(bearings)
    return np.column_stack([d * np.cos(rad), d * np.sin(rad)])


def _transform(points, pose):
    tx, ty, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack([c * points[:, 0] - s * points[:, 1] + tx,
                            s * points[:, 0] + c * points[:, 1] + ty])


def match(ref, cur, guess=(0.0, 0.0, 0.0), iterations=12, gate_mm=(200.0, 50.0)):
    """
    Lines `cur` up with `ref`.

    I/O:
        ref, cur: (N, 2) point arrays (scan_points)
        guess:    starting pose (x, y, turn in radians) - pass the last
                  tick's answer: it only has to find the small change since,
                  which is what keeps it right through a big turn
        gate_mm:  (first, last) - points further apart than this are not
                  paired; it tightens from the first to the last
        return:   (pose, rms_mm, inlier_fraction). pose is where the lidar
                  is NOW, in the reference scan's frame: x mm forward, y mm
                  right, turn radians clockwise.
    """
    pose = np.array(guess, dtype=float)
    if len(ref) < 20 or len(cur) < 20:
        return tuple(pose), float("inf"), 0.0
    rms, frac = float("inf"), 0.0
    for i in range(iterations):
        # pairs further apart than this are not the same wall: start loose,
        # tighten as it converges
        f = min(1.0, i / max(1, iterations // 2))
        gate = gate_mm[0] + (gate_mm[1] - gate_mm[0]) * f
        moved = _transform(cur, pose)
        d2 = ((moved[:, None, :] - ref[None, :, :]) ** 2).sum(axis=2)
        nearest = d2.argmin(axis=1)
        dist = np.sqrt(d2[np.arange(len(moved)), nearest])
        ok = dist < gate
        frac = float(ok.mean())
        if ok.sum() < 15:
            return tuple(pose), float("inf"), frac
        p, q = moved[ok], ref[nearest[ok]]
        rms = float(np.sqrt((dist[ok] ** 2).mean()))
        # best rigid shift + turn taking p onto q (2D Kabsch)
        pc, qc = p.mean(axis=0), q.mean(axis=0)
        h = ((p - pc).T @ (q - qc))
        dth = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c, s = math.cos(dth), math.sin(dth)
        r = np.array([[c, -s], [s, c]])
        dt = qc - r @ pc
        # compose: new pose = (r, dt) after the old one
        old_t = pose[:2]
        pose[:2] = r @ old_t + dt
        pose[2] += dth
        if abs(dth) < 1e-4 and np.hypot(*dt) < 0.2:
            break
    return tuple(pose), rms, frac


def axle_travel(pose, lidar_ahead_mm):
    """
    How far the REAR AXLE (the driven wheels) has rolled to get the lidar to
    `pose`, assuming one steady steering angle - an arc, so the straight
    chord is stretched by the turn: arc = chord * (turn/2) / sin(turn/2).
    """
    tx, ty, th = pose
    # the axle sits lidar_ahead_mm behind the lidar, along the robot's axis
    ax = tx - lidar_ahead_mm * math.cos(th) + lidar_ahead_mm
    ay = ty - lidar_ahead_mm * math.sin(th)
    chord = math.hypot(ax, ay)
    half = abs(th) / 2
    return chord * (half / math.sin(half)) if half > 1e-3 else chord
