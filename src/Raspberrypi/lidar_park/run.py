"""
Lidar-only parking, standalone - no camera, no nav filter, no TaskContext.
It only borrows the low-level drivers (LidarManager, BoardManager,
MotorManager), so nothing here can change how the main rounds behave.

    STEP 1  scan: heading off the corridor, outer wall, island, the bay
    STEP 2  drive forward until the wall ahead is drive.front_target_mm
            away, steering to end up drive.wall_target_mm from the outer
            wall and pointing straight
    STEP 3  drive back (or on) until the first blade of the bay is
            to_bay.blade_stop_x_mm from the lidar (0 = beside it), same steering
            (to_bay.enabled = false skips it)
    STEP 4  optional extra moves listed in config.toml ([[finish.moves]]),
            steering + toward the bay, - away from it

Every tunable is in lidar_park/config.toml.

    uv run python -m lidar_park.run                  scan, then drive
    uv run python -m lidar_park.run --scan-only      lidar only, motors untouched
    uv run python -m lidar_park.run --no-drive       steering live, motor off
    uv run python -m lidar_park.run --button         wait for the start button

While it runs, pressing the button stops it at once (park.stop_button).

After the main round, as a separate run (the round releases the hardware
when it ends, so this can take it straight over):

    uv run python main.py final && uv run python -m lidar_park.run

Tuning without the robot moving:

    uv run python -m lidar_park.run --scan-only --save-scan s.npy --image s.png
    uv run python -m lidar_park.run --load-scan s.npy --image s.png

Other code that already has the lidar and motor running can call
run_parking(lidar, motor) directly.
"""
import argparse
import math
import sys
import time
import tomllib
import warnings
from pathlib import Path

import numpy as np

from lidar_park.scan_analysis import analyze, draw
from lidar_park.scan_match import axle_travel, match, scan_points
from lidar_park.stop_button import StopRequested, StopWatcher, check_stop

CONFIG_PATH = Path(__file__).resolve().parent / "config.toml"
SIDES = {"auto": 0, "right": 1, "left": -1}


def clamp(value, limit):
    return max(-limit, min(limit, value))


def load_config(path=None):
    with open(path or CONFIG_PATH, "rb") as f:
        cfg = tomllib.load(f)
    # the scan analysis needs it too, to look for obstacles only past the tail
    cfg["scan"]["lidar_to_tail_mm"] = cfg["to_bay"]["lidar_to_tail_mm"]
    return cfg


def take_scan(lidar, samples=5, interval_s=0.1):
    """Per-degree median of a few snapshots, so one bad sweep cannot decide it."""
    snaps = []
    for _ in range(samples):
        check_stop()
        snaps.append(lidar.get_scan())
        time.sleep(interval_s)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)   # all-nan slots stay nan
        return np.nanmedian(np.array(snaps), axis=0)


# ============================================================================
# STEP 1 - where are we
# ============================================================================

def locate(lidar, cfg, outer_side=0):
    time.sleep(cfg["scan"]["settle_s"])
    scan = take_scan(lidar)
    return analyze(scan, cfg["scan"], outer_side=outer_side), scan


# ============================================================================
# Where the robot is along the corridor, and where the blades are
# ============================================================================

class Steady:
    """
    A distance that cannot jump. The lidar sees over the 100mm field walls
    and past blocks, so for a tick or two a "wall" can read as something
    else entirely. A reading more than scan.jump_mm from the current value is
    held off until scan.jump_confirm readings in a row agree on it - then it
    really did change.

    I/O:
        update(): return (value, jumped) - jumped is True when this reading
                  moved the value by more than jump_mm (a confirmed jump, or
                  the first reading), so it is not a distance travelled.
    """

    def __init__(self, cfg):
        self.value = float("nan")
        self.max_step = cfg["scan"]["jump_mm"]
        self.confirm = cfg["scan"]["jump_confirm"]
        self._candidates = []

    def update(self, reading):
        if math.isnan(reading):
            return self.value, True
        if math.isnan(self.value):
            self.value = reading
            return self.value, True
        if abs(reading - self.value) <= self.max_step:
            self.value = reading
            self._candidates = []
            return self.value, False
        if self._candidates and abs(reading - self._candidates[-1]) <= self.max_step:
            self._candidates.append(reading)
        else:
            self._candidates = [reading]
        if len(self._candidates) >= self.confirm:
            self.value = reading
            self._candidates = []
        return self.value, True


class Odometry:
    """
    The robot's position along the corridor (mm, + = the way it faces), from
    the lidar alone - the encoder on this robot cannot be trusted.

    Every tick, the walls at both ends of the corridor say how far it moved:
    the wall ahead comes closer, the wall behind goes further. Whichever of
    them is seen (jump-filtered) is used, averaged if both are. With neither,
    it carries on at the speed the lidar last measured, so it never stops
    knowing where it is.
    """

    def __init__(self, cfg):
        self.pos = 0.0
        self.front = Steady(cfg)
        self.rear = Steady(cfg)
        self.mm_s_per_speed = cfg["drive"]["mm_s_per_speed"]
        self.source = "start"
        self._t = None

    def update(self, r, speed_cmd):
        """r: this tick's ScanResult (or None); speed_cmd: the signed speed driven since the last tick."""
        now = time.monotonic()
        dt = 0.0 if self._t is None else now - self._t
        self._t = now
        f_prev, b_prev = self.front.value, self.rear.value
        f, f_jump = self.front.update(r.front_wall_mm if r is not None else float("nan"))
        b, b_jump = self.rear.update(r.rear_wall_mm if r is not None else float("nan"))
        moves = []
        if not f_jump and not math.isnan(f_prev):
            moves.append(f_prev - f)
        if not b_jump and not math.isnan(b_prev):
            moves.append(b - b_prev)
        if moves:
            step = sum(moves) / len(moves)
            self.source = "lidar"
            if dt > 0.02 and abs(speed_cmd) > 5:
                # learn the real speed per command unit while the walls are in view
                measured = abs(step / dt) / abs(speed_cmd)
                self.mm_s_per_speed += 0.1 * (measured - self.mm_s_per_speed)
        else:
            step = speed_cmd * self.mm_s_per_speed * dt
            self.source = "speed" if speed_cmd else "still"
        self.pos += step
        return self.pos


class BladeMap:
    """
    Every blade ever seen, at its position along the corridor (odometry
    position + where it was seen relative to the lidar). A sighting within
    to_bay.merge_mm of a known blade moves it to_bay.map_follow of the way to
    the new sighting - the newest sightings count most, so the map keeps up
    with any slow drift in the odometry. A blade only counts once it has
    been seen to_bay.min_sightings times, so a stray return cannot become a
    blade. The blades are on the outer wall only, so this IS the bay.
    """

    def __init__(self, cfg):
        self.merge = cfg["to_bay"]["merge_mm"]
        self.follow = cfg["to_bay"]["map_follow"]
        self.min_sightings = cfg["to_bay"]["min_sightings"]
        self.blades = []         # [{"pos": mm, "seen": n}]

    def add(self, r, pos):
        if r is None:
            return
        for x in r.blades_x_mm:
            where = pos + x
            near = min(self.blades, key=lambda b: abs(b["pos"] - where), default=None)
            if near is not None and abs(near["pos"] - where) <= self.merge:
                near["pos"] += self.follow * (where - near["pos"])
                near["seen"] += 1
            else:
                self.blades.append({"pos": where, "seen": 1})

    def confirmed(self):
        return [b for b in self.blades if b["seen"] >= self.min_sightings]

    def describe(self, pos):
        good = sorted(self.confirmed(), key=lambda b: b["pos"])
        return ", ".join(f"{b['pos'] - pos:+.0f} (x{b['seen']})" for b in good) or "none"


class Tracker:
    """Everything the run knows: odometry, the blade map, the outer side."""

    def __init__(self, cfg, outer_side):
        self.cfg = cfg
        self.odo = Odometry(cfg)
        self.map = BladeMap(cfg)
        self.side = outer_side          # 0 until known
        self.last = None                # this tick's usable ScanResult, if any

    def observe(self, r, speed_cmd):
        """Feed one ScanResult (may be None). Locks the outer side the first time it is known."""
        usable = r is not None and r.ok
        self.last = r if usable else None
        pos = self.odo.update(r if usable else None, speed_cmd)
        if usable:
            if self.side == 0:
                self.side = r.outer_side
                print(f"  outer wall is on the {r.side_name} - locked")
            self.map.add(r, pos)
        return pos

    def scan(self, lidar):
        """A stationary median scan, folded in like any other tick."""
        result, _ = locate(lidar, self.cfg, self.side)
        self.observe(result, 0)
        return result


# ============================================================================
# STEP 2 / 3 - drive onto the wall line, straight, until a stop point
# ============================================================================

class FrontStop:
    """STEP 2: stop when the wall ahead reads drive.front_target_mm."""

    def __init__(self, cfg, tracker):
        self.target = cfg["drive"]["front_target_mm"]
        self.tracker = tracker
        self.max_travel_mm = cfg["drive"]["max_forward_mm"]

    def remaining(self):
        front = self.tracker.odo.front.value
        return float("inf") if math.isnan(front) else front - self.target

    def status(self):
        front = self.tracker.odo.front.value
        return f"front {front:5.0f} (want {self.target:.0f})" if not math.isnan(front) \
            else "front  not seen yet"


class BladeStop:
    """
    STEP 3: stop when the FIRST blade of the bay the robot comes to is
    to_bay.blade_stop_x_mm from the lidar (0 = right beside it).

    The blade is picked from the map - the nearest confirmed blade in the
    direction of travel - and locked. Its position in the map keeps being
    refined, and the odometry says where the robot is, so it is known
    whether it is in sight this tick or not. With no blade in the map yet,
    the robot keeps going and takes the first one confirmed.
    """

    def __init__(self, cfg, tracker, direction):
        self.target = cfg["to_bay"]["blade_stop_x_mm"]
        self.tracker = tracker
        self.direction = direction
        self.blade = None
        self.start_pos = tracker.odo.pos
        self.max_travel_mm = cfg["to_bay"]["search_mm"]
        self.overshoot = cfg["to_bay"]["overshoot_mm"]
        self._pick()

    def _pick(self):
        pos = self.tracker.odo.pos
        ahead = [b for b in self.tracker.map.confirmed()
                 if self.direction * (b["pos"] - pos - self.target) > 0]
        if ahead:
            self.blade = min(ahead, key=lambda b: self.direction * (b["pos"] - pos - self.target))
            travelled = abs(pos - self.start_pos)
            self.max_travel_mm = travelled + abs(self.blade["pos"] - pos - self.target) + self.overshoot

    @property
    def blade_x(self):
        """
        Where the locked blade is now, relative to the lidar: this tick's
        own sighting of it when there is one (exact), else the map's
        position for it through the odometry.
        """
        if self.blade is None:
            return float("nan")
        mapped = self.blade["pos"] - self.tracker.odo.pos
        last = self.tracker.last
        if last is not None:
            merge = self.tracker.map.merge
            live = [x for x in last.blades_x_mm if abs(x - mapped) <= merge]
            if live:
                return min(live, key=lambda x: abs(x - mapped))
        return mapped

    def remaining(self):
        if self.blade is None:
            self._pick()
            if self.blade is None:
                return float("inf")         # still looking: keep going
        return self.direction * (self.blade_x - self.target)

    def status(self):
        if self.blade is None:
            return "blade  searching"
        return f"blade {self.blade_x:+5.0f} (want {self.target:+.0f}, seen x{self.blade['seen']})"


def drive_leg(lidar, motor, cfg, tracker, direction, stop, timeout_s=None, log=None):
    """
    Drives (direction +1 forward, -1 backward) until stop.remaining() says
    it is there, steering onto drive.wall_target_mm from the outer wall and
    finishing straight. It never gives up because something is not seen:
    with no outer wall it just holds the heading straight, with no heading
    it holds the wheels straight. Only these end a leg early: something in
    the robot's path, the travel limit, or the timeout.

    Reversing with front-wheel steering flips two things: the steering's
    effect on heading (so steer is multiplied by direction), and which way a
    heading moves the robot sideways (so the wanted heading is too).

    I/O:
        return: reason it stopped ("reached" when it got there)
    """
    d, scfg = cfg["drive"], cfg["scan"]
    forward = direction > 0
    leg = d if isinstance(stop, FrontStop) else cfg["to_bay"]
    timeout_s = leg["timeout_s"] if timeout_s is None else timeout_s
    period = 1.0 / d["loop_hz"]
    heading_kp = leg.get("heading_kp", d["heading_kp"])
    speed = abs(leg["speed"]) * direction
    coast = leg["coast_mm"]
    outer = Steady(cfg)                    # the outer wall, jump-filtered
    if forward:
        path_stop = d["obstacle_stop_mm"]
    else:
        # behind, the robot's own tail is between the lidar and the obstacle
        path_stop = cfg["to_bay"]["lidar_to_tail_mm"] + d["obstacle_stop_mm"] \
            if d["obstacle_stop_mm"] > 0 else 0

    start_pos = tracker.odo.pos
    start = time.monotonic()
    last_print = 0.0
    steer = 0.0
    driven = 0                             # the speed actually commanded last tick
    reason = "reached"
    try:
        while True:
            check_stop()
            now = time.monotonic()
            r = analyze(lidar.get_scan(), scfg, outer_side=tracker.side, find_bay=True)
            pos = tracker.observe(r, driven)
            travelled = abs(pos - start_pos)
            remaining = stop.remaining()
            if log is not None:
                side = {1: "right", -1: "left"}.get(tracker.side, "unknown")
                log.write(f"{now - start:.2f},{direction},{side},{pos:.0f},{tracker.odo.source},"
                          f"{tracker.odo.mm_s_per_speed:.2f},{remaining:.0f},{stop.status()},"
                          f"{r.ok},{r.outer_wall_mm:.0f},{r.heading_deg:.1f},"
                          f"{r.front_wall_mm:.0f},{r.rear_wall_mm:.0f},"
                          f"{' '.join(f'{x:.0f}' for x in r.blades_x_mm)},"
                          f"{tracker.map.describe(pos)},{steer:.1f}\n")
            if remaining <= coast:
                break
            if travelled >= stop.max_travel_mm:
                reason = f"drove {travelled:.0f}mm (limit {stop.max_travel_mm:.0f}mm) - see the log"
                break
            if now - start > timeout_s:
                reason = f"timeout after {timeout_s:.0f}s"
                break
            in_path = r.obstacle_ahead_mm if forward else r.obstacle_behind_mm
            if in_path < path_stop:
                reason = f"something in the path {in_path:.0f}mm {'ahead' if forward else 'behind'}"
                break

            wall, _ = outer.update(r.outer_wall_mm if r.ok else float("nan"))
            if not math.isnan(r.heading_deg):
                if tracker.side != 0 and not math.isnan(wall):
                    error = wall - d["wall_target_mm"]
                    room = remaining - coast - d["straighten_mm"]
                    if room <= 0:
                        wanted = 0.0     # last stretch: only straighten
                    else:
                        # steep enough to close the gap before the straighten
                        # stretch, and never gentler than wall_kp asks for
                        closing = math.degrees(math.atan2(abs(error), min(room, 1e6)))
                        wanted = math.copysign(max(abs(d["wall_kp"] * error), closing), error)
                        wanted = direction * tracker.side * clamp(wanted, d["max_approach_deg"])
                else:
                    wanted = 0.0         # no wall to follow: just keep straight
                steer = clamp(direction * heading_kp * (wanted - r.heading_deg), d["max_steer_deg"])
                if d["steer_invert"]:
                    steer = -steer
            # else: no heading this tick - hold the last steering

            check_stop()                 # never drive after the stop button
            motor.drive(steer, speed)
            driven = speed

            if now - last_print >= 0.25:
                last_print = now
                print(f"  {stop.status()}  wall {wall:5.0f} (want {d['wall_target_mm']:.0f})  "
                      f"heading {r.heading_deg:+5.1f}  steer {steer:+5.1f}  "
                      f"pos {pos:+5.0f} ({tracker.odo.source})", flush=True)
            time.sleep(max(0.0, period - (time.monotonic() - now)))
    finally:
        motor.stop()
        motor.steer_center()
    return reason


def _report_wall(result, d):
    if result.ok:
        print(f"  wall error {result.outer_wall_mm - d['wall_target_mm']:+.0f}mm, "
              f"heading {result.heading_deg:+.1f} deg")


# ============================================================================
# The whole thing, on hardware that is already running
# ============================================================================

LOG_HEADER = ("t_s,direction,outer_side,pos_mm,pos_source,mm_s_per_speed,remaining_mm,"
              "stop,scan_ok,outer_wall_mm,heading_deg,front_mm,rear_mm,blades_x,"
              "blade_map,steer\n")


def run_parking(lidar, motor, cfg=None, outer_side=None, image=None, timeout_s=None,
                log_path=None, button=None):
    """
    I/O:
        lidar, motor: started LidarManager / MotorManager
        outer_side:   "auto" / "left" / "right"; None takes park.outer_side
        button:       a started StartButton - pressing it stops the parking
                      at once (park.stop_button); None = no button stop
        return:       True if every step finished (False if stopped)
    """
    cfg = cfg or load_config()
    log = None
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        log = open(log_path, "w")
        log.write(LOG_HEADER)
    watcher = None
    if button is not None and cfg["park"].get("stop_button", True):
        watcher = StopWatcher(button, motor)
        watcher.start()
    try:
        return _run_parking(lidar, motor, cfg, outer_side, image, timeout_s, log)
    except StopRequested:
        print("\nSTOPPED by button.")
        return False
    finally:
        try:
            motor.stop()
            motor.steer_center()
        except Exception:
            pass
        if watcher is not None:
            watcher.stop()
        if log is not None:
            log.close()
            print(f"  log: {log_path}")


def _run_parking(lidar, motor, cfg, outer_side, image, timeout_s, log):
    d, tb = cfg["drive"], cfg["to_bay"]
    tracker = Tracker(cfg, SIDES[outer_side or cfg["park"]["outer_side"]])

    print("\nSTEP 1 - scanning")
    first = tracker.scan(lidar)
    print(first.summary())
    if image:
        draw(first, image, cfg["scan"])
        print(f"  picture: {image}")
    if not first.ok:
        print(f"  ({first.message} - driving anyway, it will work it out on the move)")
    if not first.has_bay and cfg["park"]["require_bay"]:
        print("STOP: no bay found and park.require_bay is on")
        return False

    print(f"\nSTEP 2 - forward until the wall ahead is {d['front_target_mm']:.0f}mm, "
          f"to {d['wall_target_mm']:.0f}mm from the outer wall")
    reason = drive_leg(lidar, motor, cfg, tracker, +1, FrontStop(cfg, tracker), timeout_s, log)
    print(f"  stopped: {reason}")
    step2_ok = reason == "reached"

    after = tracker.scan(lidar)
    print("\nAFTER STEP 2")
    print(after.summary())
    _report_wall(after, d)
    if step2_ok and not math.isnan(after.front_wall_mm):
        print(f"  front wall {after.front_wall_mm:.0f}mm (wanted {d['front_target_mm']:.0f}, "
              f"{after.front_wall_mm - d['front_target_mm']:+.0f}) - add that to drive.coast_mm")
    print(f"  blades on the map: {tracker.map.describe(tracker.odo.pos)}")
    if not tb["enabled"]:
        return step2_ok

    # The first blade to come to: behind if there is one, else ahead, else
    # back up and look (the bay was most likely passed on the way here).
    pos = tracker.odo.pos
    known = [b["pos"] - pos for b in tracker.map.confirmed()]
    direction = +1 if known and all(x > tb["blade_stop_x_mm"] for x in known) else -1
    stop = BladeStop(cfg, tracker, direction)
    where = f"now {stop.blade_x:+.0f}mm" if stop.blade else "none on the map yet - looking"
    print(f"\nSTEP 3 - {'backward' if direction < 0 else 'forward'} to the bay's first blade "
          f"({where}, stopping at {tb['blade_stop_x_mm']:+.0f}mm)")
    reason = drive_leg(lidar, motor, cfg, tracker, direction, stop, timeout_s, log)
    print(f"  stopped: {reason}")

    final = tracker.scan(lidar)
    print("\nAFTER STEP 3")
    print(final.summary())
    _report_wall(final, d)
    if stop.blade is not None:
        miss = stop.blade_x - tb["blade_stop_x_mm"]
        print(f"  first blade {stop.blade_x:+.0f}mm by the map (wanted {tb['blade_stop_x_mm']:+.0f}, "
              f"{miss:+.0f}) - {'add' if direction < 0 else 'subtract'} that "
              f"{'to' if direction < 0 else 'from'} to_bay.coast_mm")
    step3_ok = reason == "reached"

    moves_ok = run_moves(lidar, motor, cfg, tracker, log)
    return step2_ok and step3_ok and moves_ok


# ============================================================================
# STEP 4 - extra moves from the config, in order
# ============================================================================

def run_moves(lidar, motor, cfg, tracker, log=None):
    """
    Drives every [[finish.moves]] entry in order: one steering angle held
    for a distance, forward or backward. steer_deg is relative to the BAY:
    + steers toward the outer wall (the bay's side), - away from it, so the
    same list works whichever side the bay is on.

    Distance is how far the REAR AXLE (the driven wheels) really rolls,
    measured by lining every scan up with one taken still at the start of
    the move (scan_match.py) - it sees the true shift and turn whatever the
    motor, battery or floor did. Only on a tick where that match is poor
    does it fall back to time x the speed learned in steps 2 and 3. When
    each move ends, a still scan measures what actually happened.

    I/O:
        return: True if every move covered its distance
    """
    fin = cfg.get("finish", {})
    moves = fin.get("moves", [])
    if not moves:
        return True
    side = tracker.side or 1
    if tracker.side == 0:
        print("  (outer side never found - taking + steer as right)")
    d, scfg = cfg["drive"], cfg["scan"]
    period = 1.0 / d["loop_hz"]
    lidar_ahead = scfg["lidar_ahead_mm"]
    max_rms, min_inliers = fin.get("match_max_rms_mm", 40.0), fin.get("match_min_inliers", 0.5)

    def points(scan):
        return scan_points(scan, scfg["min_range_mm"], scfg["max_range_mm"])

    def good(rms, frac):
        return rms <= max_rms and frac >= min_inliers

    all_ok = True
    print(f"\nSTEP 4 - {len(moves)} extra move(s)")
    for i, move in enumerate(moves, start=1):
        direction = -1 if move.get("reverse", False) else +1
        steer = clamp(move.get("steer_deg", 0.0) * side, d["max_steer_deg"])
        if d["steer_invert"]:
            steer = -steer
        speed = abs(move.get("speed", fin.get("speed", 45))) * direction
        distance = abs(move["distance_mm"])
        timeout_s = move.get("timeout_s", fin.get("timeout_s", 5.0))
        # step 4 is close-quarters on purpose: its own, smaller margin
        margin = move.get("obstacle_stop_mm", fin.get("obstacle_stop_mm", 50.0))
        if direction > 0:
            path_stop = margin
        else:
            path_stop = cfg["to_bay"]["lidar_to_tail_mm"] + margin if margin > 0 else 0
        asked = move.get("steer_deg", 0.0)
        way = "straight" if asked == 0 else ("toward the bay" if asked > 0 else "away from the bay")
        print(f"  move {i}: {'back' if direction < 0 else 'forward'} {distance:.0f}mm, "
              f"steer {asked:+.0f} ({way}) -> command {steer:+.0f}")
        coast = move.get("coast_mm", fin.get("coast_mm", 0.0))
        check_stop()
        motor.steer(steer)
        time.sleep(0.15)                     # let the steering get there first
        ref = points(take_scan(lidar, samples=3, interval_s=0.05))
        pose = (0.0, 0.0, 0.0)
        moved = 0.0                          # rear axle travel, best estimate
        good_moved, good_t = 0.0, None       # the last well-matched answer
        source = "match"
        start = time.monotonic()
        driven = 0
        reason = "done"
        try:
            while True:
                check_stop()
                now = time.monotonic()
                raw = lidar.get_scan()
                r = analyze(raw, scfg, outer_side=tracker.side, find_bay=True)
                pos = tracker.observe(r, driven)
                new_pose, rms, frac = match(ref, points(raw), guess=pose)
                if good(rms, frac):
                    pose = new_pose
                    good_moved, good_t = axle_travel(pose, lidar_ahead), now
                    moved, source = good_moved, "match"
                else:
                    # poor match this tick: carry on from the last good one
                    # at the learned speed
                    since = now - (good_t if good_t is not None else start)
                    moved = good_moved + abs(driven) * tracker.odo.mm_s_per_speed * since
                    source = "speed"
                if log is not None:
                    log.write(f"{now - start:.2f},{direction},move{i},{pos:.0f},{source},"
                              f"{tracker.odo.mm_s_per_speed:.2f},{distance - moved:.0f},move {i},"
                              f"{r.ok},{r.outer_wall_mm:.0f},{r.heading_deg:.1f},"
                              f"{r.front_wall_mm:.0f},{r.rear_wall_mm:.0f},"
                              f"{' '.join(f'{x:.0f}' for x in r.blades_x_mm)},"
                              f"{tracker.map.describe(pos)},{steer:.1f}\n")
                if moved >= distance - coast:
                    break
                if now - start > timeout_s:
                    reason = f"timeout after {timeout_s:.0f}s ({moved:.0f}mm)"
                    break
                in_path = r.obstacle_ahead_mm if direction > 0 else r.obstacle_behind_mm
                if in_path < path_stop:
                    reason = f"something in the path {in_path:.0f}mm away ({moved:.0f}mm)"
                    break
                check_stop()
                motor.drive(steer, speed)
                driven = speed
                time.sleep(max(0.0, period - (time.monotonic() - now)))
        finally:
            motor.stop()
        # what really happened, from a still scan
        time.sleep(scfg["settle_s"])
        end_pose, rms, frac = match(ref, points(take_scan(lidar, samples=3, interval_s=0.05)),
                                    guess=pose)
        if good(rms, frac):
            rolled = axle_travel(end_pose, lidar_ahead)
            turned = math.degrees(end_pose[2])
            print(f"    {reason}: rolled {rolled:.0f}mm (wanted {distance:.0f}, {rolled - distance:+.0f}), "
                  f"turned {turned:+.1f} deg")
        else:
            print(f"    {reason}: about {moved:.0f}mm (the end scan did not match well enough to measure)")
        all_ok = all_ok and reason == "done"
        if move.get("pause_s"):
            time.sleep(move["pause_s"])
    motor.steer_center()
    final = tracker.scan(lidar)
    print("\nAFTER STEP 4")
    print(final.summary())
    return all_ok


# ============================================================================
# Command line
# ============================================================================

def main(argv=None):
    p = argparse.ArgumentParser(description="Lidar-only parking, standalone")
    p.add_argument("--config", default=None, help="default: lidar_park/config.toml")
    p.add_argument("--side", choices=SIDES, default=None,
                   help="outer wall side (default: park.outer_side)")
    p.add_argument("--front", type=float, default=None, help="override drive.front_target_mm")
    p.add_argument("--blade-x", type=float, default=None, help="override to_bay.blade_stop_x_mm")
    p.add_argument("--wall", type=float, default=None, help="override drive.wall_target_mm")
    p.add_argument("--speed", type=int, default=None, help="override drive.speed")
    p.add_argument("--timeout", type=float, default=None, help="override drive.timeout_s")
    p.add_argument("--scan-only", action="store_true",
                   help="step 1 only; the motor board is never opened")
    p.add_argument("--no-drive", action="store_true",
                   help="steering follows the loop but the motor never turns")
    p.add_argument("--button", action="store_true", help="wait for the start button")
    p.add_argument("--save-scan", default=None, help="save the first scan (.npy)")
    p.add_argument("--load-scan", default=None,
                   help="analyse a saved scan and exit - no hardware at all")
    p.add_argument("--image", default=None, help="write a PNG of what the scan found")
    p.add_argument("--log", default=str(Path(__file__).resolve().parent / "logs" / "last_run.csv"),
                   help="per-tick CSV of every drive (default: lidar_park/logs/last_run.csv)")
    p.add_argument("--lidar-port", default=None)
    p.add_argument("--debug", action="store_true")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    for key, value in (("front_target_mm", args.front), ("wall_target_mm", args.wall),
                       ("speed", args.speed)):
        if value is not None:
            cfg["drive"][key] = value
    if args.blade_x is not None:
        cfg["to_bay"]["blade_stop_x_mm"] = args.blade_x
    side = args.side or cfg["park"]["outer_side"]

    if args.load_scan:
        result = analyze(np.load(args.load_scan), cfg["scan"], outer_side=SIDES[side])
        print(result.summary())
        print(f"  ({result.message})")
        if args.image:
            draw(result, args.image, cfg["scan"])
            print(f"  picture: {args.image}")
        return result.ok

    from classes.lidar_manager import LidarManager

    lidar = LidarManager(port=args.lidar_port, debug=args.debug)
    board = motor = button = None
    try:
        print("Starting lidar...")
        lidar.start()
        if not lidar.wait_until_ready(timeout=5.0):
            print("WARNING: lidar returned little data - is it spinning?")

        if args.scan_only:
            result, scan = locate(lidar, cfg, SIDES[side])
            print(result.summary())
            print(f"  ({result.message})")
            if args.save_scan:
                np.save(args.save_scan, scan)
                print(f"  scan saved: {args.save_scan}")
            if args.image:
                draw(result, args.image, cfg["scan"])
                print(f"  picture: {args.image}")
            return result.ok

        from classes.board_manager import BoardManager
        from classes.motor_manager import MotorManager

        print("Starting motor board...")
        board = BoardManager(debug=args.debug)
        board.start()
        motor = MotorManager(board, debug=args.debug, drive_enabled=not args.no_drive)
        motor.start()

        # The button: --button waits for it to start, and either way it is
        # the stop button while the parking runs.
        from classes.start_button import StartButton

        try:
            button = StartButton(debug=args.debug)
            button.start()
        except Exception as e:
            print(f"WARNING: no button ({e}) - it cannot stop the parking this run")
            button = None
        if args.button and button is not None:
            print("Waiting for the start button...")
            button.wait()

        if args.save_scan:
            np.save(args.save_scan, take_scan(lidar))
            print(f"scan saved: {args.save_scan}")

        timeout = args.timeout
        if timeout is None and args.no_drive:
            timeout = 60.0   # pushing by hand is slow
        return run_parking(lidar, motor, cfg, side, image=args.image, timeout_s=timeout,
                           log_path=args.log, button=button)
    except KeyboardInterrupt:
        print("\nInterrupted")
        return False
    finally:
        if motor is not None:
            motor.stop_and_close()
        if board is not None:
            board.stop()
        if button is not None:
            button.stop()
        lidar.stop()


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
