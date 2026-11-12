"""
The surprise rule: which traffic sign is not the height it should be.

One sign in the obstacle round (never the first one the robot meets) is
swapped for the opposite colour and made 5cm or 15cm tall instead of 10cm. It
has to be passed on the side of its ORIGINAL colour - the opposite of the
colour the camera sees. Passing it the wrong way ends the attempt, and so does
flipping a sign that was normal, so a false flip costs exactly as much as a
missed one. Everything here leans towards "it is a normal sign".

ObjectSolver ranges every box off its height in the frame, as if it were
10cm tall. That one number cannot tell a 15cm sign 90cm away from a 10cm one
60cm away - they are the same size in the frame. Two other things can:

    THE LIDAR. A sign of a given pixel height is at one of three ranges, one
    per height it could be. The lidar is asked whether there is anything at
    each of them, along the bearing the sign would have from the lidar - not
    from the lens, which is 150mm further forward (classes/robot_geometry.py)
    and sees everything at a different angle up close.

    THE SHAPE. Every sign is 5cm square on the floor, so its height over its
    width is about 1:1 for 5cm, 2:1 for 10cm and 3:1 for 15cm. A square seen
    corner-on is up to 1.41x wider than face-on, and the signs stand square
    to the walls, so the width is corrected by the angle the robot sees the
    sign from. Only trusted near the middle of the frame and up close: far
    away the mask's own blur pulls every ratio towards 1.

    TALL  = the lidar has something at the 15cm range and nothing at the 10cm
            one, AND the shape is tall. (The lidar half alone is enough to
            PLACE the sign at the 15cm range - it is a measured range to
            something that is really there - but never to call it modified.)
    SHORT = the shape is squat AND the lidar agrees - nothing at the 5cm range
            either, if its scan plane passes over a 5cm sign
            (surprise.lidar_sees_short = false); a return at the 5cm range
            and none at the 10cm one, if it does not.
    Anything else is a 10cm sign.

This file judges ONE FRAME. A frame's verdict is a vote; BlockMap counts the
votes per tracked sign and latches it as modified once they are clear enough
(see BlockMap._decide_size). FinalTask then applies the round's own rules: the
first sign is never flipped, and at most one is.

Runs on whichever thread runs the camera (NavigationManager.observe_blocks).
It holds no state between frames except counters: the scan comes from
LidarManager under its own lock, as a copy, and the map is only raycast.
"""
import csv
import math
import time
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from classes.block_map import FACE_TO_CENTER_MM
from classes.object_solver import (BOX_HEIGHT_CM, MAX_DISTANCE_CM,
                                   SHORT_SIGN_HEIGHT_CM, TALL_SIGN_HEIGHT_CM,
                                   rerange)
from classes.robot_geometry import to_field

# The three heights, in millimetres - the unit everything after the solver
# uses.
SHORT_MM = int(round(SHORT_SIGN_HEIGHT_CM * 10.0))
STANDARD_MM = int(round(BOX_HEIGHT_CM * 10.0))
TALL_MM = int(round(TALL_SIGN_HEIGHT_CM * 10.0))
HEIGHTS_MM = (SHORT_MM, STANDARD_MM, TALL_MM)

# DIAGNOSTIC ONLY - nothing is decided from it. How far either side of where
# a 10cm sign would be the lidar is searched for the nearest thing it can
# see, for the log and the bench script. If the camera and the lidar
# disagree about where a sign is, this is how far apart they can be and
# still show it: wide enough to cover a wrong lens offset up close.
LOOK_AROUND_DEG = 20


@dataclass
class SizerSettings:
    """
    Every threshold SignSizer uses, read from tasks/final/config.toml
    [surprise] - see that file for what each one is and how to measure it.
    """
    lidar_sees_short: bool
    lidar_window_deg: int
    lidar_tolerance_mm: float
    lidar_tolerance_fraction: float
    lidar_min_readings: int
    lidar_max_age_s: float
    wall_margin_mm: float
    shape_min_range_mm: float
    shape_max_range_mm: float
    shape_max_off_axis_deg: float
    blob_pad_px: float
    short_max_ratio: float
    standard_min_ratio: float
    standard_max_ratio: float
    tall_min_ratio: float
    log_path: str = ""
    # "confirm": both cues must say the odd height. "veto": the shape decides
    # and the lidar can only overrule it - see SignSizer.judge.
    lidar_role: str = "confirm"

    @classmethod
    def from_settings(cls, setting):
        """
        I/O:
            setting: key -> value, e.g. PathDrivingTask.setting
        """
        def get(name):
            return setting(f"surprise.{name}")

        return cls(
            lidar_sees_short=bool(get("lidar_sees_short")),
            lidar_window_deg=int(get("lidar_window_deg")),
            lidar_tolerance_mm=float(get("lidar_tolerance_mm")),
            lidar_tolerance_fraction=float(get("lidar_tolerance_fraction")),
            lidar_min_readings=int(get("lidar_min_readings")),
            lidar_max_age_s=float(get("lidar_max_age_s")),
            wall_margin_mm=float(get("wall_margin_mm")),
            shape_min_range_mm=float(get("shape_min_range_mm")),
            shape_max_range_mm=float(get("shape_max_range_mm")),
            shape_max_off_axis_deg=float(get("shape_max_off_axis_deg")),
            blob_pad_px=float(get("blob_pad_px")),
            short_max_ratio=float(get("short_max_ratio")),
            standard_min_ratio=float(get("standard_min_ratio")),
            standard_max_ratio=float(get("standard_max_ratio")),
            tall_min_ratio=float(get("tall_min_ratio")),
            log_path=str(get("log_path") or ""),
            lidar_role=str(get("lidar_role") or "confirm"))


@dataclass
class SizeVerdict:
    """One frame's opinion of one sign - see SignSizer.judge."""
    height_mm: int          # what this frame judges the sign to be
    place_mm: int = STANDARD_MM   # the height to RANGE it with - see judge
    vote: int = None        # the height this frame votes for, None = no opinion
    lidar_cue: int = None   # the height the lidar alone points to
    shape_cue: int = None   # the height the shape alone points to
    ratio: float = None     # height / corrected width, when it was measured
    lidar_ranges: dict = None   # {height_mm: (bearing_deg, range_mm)} expected
    lidar_seen: dict = None     # {height_mm: readings_mm} around each bearing
    # (bearing_deg, range_mm) of the nearest thing the lidar sees within
    # LOOK_AROUND_DEG of where a 10cm sign would be - see nearest_around.
    lidar_nearest: tuple = None


class SignSizer:
    """
    Works out, per frame, how tall each red/green sign is, and re-ranges it
    for that height before BlockMap sees it.

        sizer = SignSizer(lidar, nav.map, nav.lidar_offset_mm,
                          nav.blocks.camera_offset_mm, SizerSettings(...))
        nav.sign_sizer = sizer       # NavigationManager.observe_blocks runs it
    """

    def __init__(self, lidar, field_map, lidar_offset_mm, camera_offset_mm,
                 settings, debug=False):
        """
        I/O:
            lidar: LidarManager, or anything with get_scan(max_age_s); None
                   means no lidar cue, which means no sign is ever flipped
            lidar_offset_mm, camera_offset_mm: (forward, right) of each sensor
                   from the rear axle - nav.lidar_offset_mm and
                   nav.blocks.camera_offset_mm, so they follow the config
        """
        self.lidar = lidar
        self.map = field_map
        self.lidar_offset_mm = tuple(float(v) for v in lidar_offset_mm)
        self.camera_offset_mm = tuple(float(v) for v in camera_offset_mm)
        self.settings = settings
        self.debug = debug
        # How fast the robot is going, in mm/s, set by the control loop every
        # tick. One float, so it is safe to write from there and read here.
        # Used to take the robot's own motion out of lidar readings that are
        # older than the frame - see _scan.
        self.speed_mm_s = 0.0
        # When a cue on its own last said "this sign might not be 10cm", by
        # time.monotonic(). Written here, read by the control loop to slow
        # down EARLY - before any block for it exists, which for a 5cm sign
        # is not until both cues agree. One float, safe across threads.
        self.last_suspect_at = 0.0

        # One CSV row per judged detection - see _log. Opened on first use,
        # written only from the camera's thread.
        self._log_file = None
        self._log = None

        self.frames = 0
        self.dropped = 0
        self.votes = {height: 0 for height in HEIGHTS_MM}

    # ========================================================================
    # THE FRAME
    # ========================================================================

    def size(self, pose, detections):
        """
        Judges every detection in one frame and re-ranges it for the height
        it was judged to be.

        Detections the plain solver would have dropped (squat, tiny or far -
        see ObjectSolver.set_surprise) are dropped here again unless they are
        judged SHORT, so a speck the surprise rule let through cannot reach
        the map as a 10cm sign at twice its range.

        I/O:
            pose: the Pose the frame was taken from (BlockMap checks it is
                  reliable; an unreliable one here just means no cues)
            detections: list of DetectedObject from ObjectSolver.detect()
            return: list of DetectedObject, re-ranged, carrying candidates_cm
                    and size_vote for BlockMap's vote
        """
        if not detections:
            return detections
        self.frames += 1
        scan = self._scan()
        sized = []
        for detection in detections:
            verdict = self.judge(pose, detection, scan)
            if SHORT_MM in (verdict.lidar_cue, verdict.shape_cue) or (
                    TALL_MM in (verdict.lidar_cue, verdict.shape_cue)):
                self.last_suspect_at = time.monotonic()
            dropped = verdict.height_mm != SHORT_MM and (
                not detection.plain or self._squat_and_unplaced(verdict))
            self._write_log(pose, detection, verdict, dropped)
            if dropped:
                self.dropped += 1
                continue
            chosen = detection
            if verdict.place_mm != STANDARD_MM:
                chosen = rerange(detection, verdict.place_mm / 10.0)
                # The solver's own range limit, applied to the range the sign
                # is now believed to be at.
                if chosen.distance_cm > MAX_DISTANCE_CM:
                    self.dropped += 1
                    continue
            if verdict.vote is not None:
                self.votes[verdict.vote] += 1
            sized.append(replace(
                chosen,
                candidates_cm={height: self._face_mm(detection, height) / 10.0
                               for height in HEIGHTS_MM},
                size_vote=verdict.vote))
        return sized

    def _squat_and_unplaced(self, verdict):
        """
        A blob that looks square, not judged short, with no lidar return where
        a 10cm sign that size would stand.

        It cannot be mapped. If it is a 5cm sign the 10cm range puts it twice
        as far away as it is - a second pillar, half a metre behind the real
        one, to be dodged on the side of the colour it shows. If it is a 10cm
        sign this was a bad frame of it, and the next one will do. Either
        way the frame is worth less than nothing, so it is dropped.
        """
        return (verdict.ratio is not None
                and verdict.ratio <= self.settings.short_max_ratio
                and verdict.lidar_cue != STANDARD_MM)

    def judge(self, pose, detection, scan=None):
        """
        The two cues for one detection, and what they add up to.

        I/O:
            scan: (distances_mm, ages_s) from _scan(), or None for no lidar
            return: SizeVerdict
        """
        lidar_cue, expected, seen = self._lidar_cue(pose, detection, scan)
        shape_cue, ratio = self._shape_cue(pose, detection)
        nearest = (None if scan is None else
                   self.nearest_around(scan, expected[STANDARD_MM][0]))

        # lidar_role "confirm": BOTH cues, or it is a 10cm sign. Either one
        # alone is wrong too often: the lidar misses a beam or catches a sign
        # behind, and a ratio is a handful of pixels over another handful.
        #
        # lidar_role "veto": the SHAPE decides, and the lidar can only say no.
        # A lidar with no opinion (beams missed, something else in the way)
        # no longer holds the decision back - only one that points at a
        # DIFFERENT height does, which on a normal sign is the usual case: a
        # return right where a 10cm sign that size would stand. More frames
        # get a verdict, so it is decided further out; the price is that a
        # ratio that is wrong for a few frames in a row is now enough.
        if self.settings.lidar_role == "veto":
            agrees = lidar_cue in (None, shape_cue)
        else:
            agrees = lidar_cue == shape_cue
        if shape_cue in (TALL_MM, SHORT_MM) and agrees:
            height = shape_cue
        else:
            height = STANDARD_MM

        if height != STANDARD_MM:
            vote = height
        elif shape_cue == STANDARD_MM or (
                lidar_cue == STANDARD_MM and self.settings.lidar_role != "veto"):
            # A positive sign of 10cm - not just the absence of anything else.
            # Under "veto" only the SHAPE votes: the lidar can block a frame
            # but not vote. Its 10cm cue is a return at the 10cm range within
            # lidar_window_deg, and with the window wide enough to cover a
            # camera bearing error that is as often another sign, or the wall,
            # or (up close, where the 10cm and 15cm ranges are a few cm apart)
            # the tall sign itself - counted as votes, those kept a 15cm sign
            # under latch_fraction for a whole round in the simulator.
            vote = STANDARD_MM
        else:
            vote = None
        # WHERE it is is a separate question from WHAT it is. A lidar return
        # at the 15cm range with a clear beam through the 10cm one is a
        # measured range to something that is really there, so the sign is
        # placed there even before the shape has had its say. Placing it is
        # not flipping it: that still takes both cues, and the votes.
        place = height
        if height == STANDARD_MM and lidar_cue == TALL_MM:
            place = TALL_MM
        return SizeVerdict(height_mm=height, place_mm=place, vote=vote,
                           lidar_cue=lidar_cue, shape_cue=shape_cue, ratio=ratio,
                           lidar_ranges=expected, lidar_seen=seen,
                           lidar_nearest=nearest)

    # ========================================================================
    # THE LIDAR
    # ========================================================================

    def _scan(self):
        """
        (distances, ages) of the latest scan, or None without a lidar.

        The scan is a ROLLING snapshot - the oldest slot is a whole rotation
        older than the newest - and the robot is moving, so a slot read 100ms
        ago at racing speed was taken over 100mm further back. Where the
        lidar can say how old each slot is (LidarManager.get_scan_with_age),
        each reading is brought forward by speed x age along its own bearing;
        where it cannot, the readings are taken as they are.
        """
        if self.lidar is None:
            return None
        max_age = self.settings.lidar_max_age_s
        try:
            with_age = getattr(self.lidar, "get_scan_with_age", None)
            if with_age is not None:
                distances, ages = with_age(max_age_s=max_age)
            else:
                distances = self.lidar.get_scan(max_age_s=max_age)
                ages = np.zeros(360)
        except Exception:       # noqa: BLE001 - no scan is no cue, not a crash
            return None
        distances = np.asarray(distances, dtype=float)
        ages = np.nan_to_num(np.asarray(ages, dtype=float))
        if self.speed_mm_s:
            travel = self.speed_mm_s * ages * np.cos(np.radians(np.arange(360.0)))
            distances = distances - travel
        return distances

    def _lidar_cue(self, pose, detection, scan):
        """
        Which of the three ranges the lidar finds something at.

        I/O:
            return: (height_mm or None, {height: (bearing, range)},
                     {height: readings})
        """
        if scan is None:
            return None, None, None
        settings = self.settings
        expected = {height: self._from_lidar(detection, height) for height in HEIGHTS_MM}
        seen = {height: self._readings(scan, expected[height][0]) for height in HEIGHTS_MM}

        def tolerance(range_mm):
            return (settings.lidar_tolerance_mm
                    + settings.lidar_tolerance_fraction * range_mm)

        def hit(height):
            """Something within tolerance of where a sign this tall would be."""
            range_mm = expected[height][1]
            readings = seen[height]
            return bool(np.any(np.abs(readings - range_mm) <= tolerance(range_mm)))

        def clear_through(height):
            """The beam went PAST where a sign this tall would be."""
            range_mm = expected[height][1]
            readings = seen[height]
            return (len(readings) >= settings.lidar_min_readings
                    and bool(np.all(readings > range_mm + tolerance(range_mm))))

        def wall_mm(height):
            """How far the wall or the centre block is along that bearing."""
            if pose is None:
                return None
            lidar_x, lidar_y = to_field(pose.x, pose.y, pose.heading, self.lidar_offset_mm)
            return float(self.map.raycast(
                float(lidar_x), float(lidar_y),
                math.radians(pose.heading + expected[height][0])))

        def short_of_wall(height):
            """A hit there is not just the wall or the centre block."""
            wall = wall_mm(height)
            return wall is not None and expected[height][1] < wall - settings.wall_margin_mm

        def nothing_before_wall(height):
            """Every beam along that bearing went all the way to the wall."""
            wall = wall_mm(height)
            readings = seen[height]
            return (wall is not None
                    and len(readings) >= settings.lidar_min_readings
                    and bool(np.all(readings >= wall - settings.wall_margin_mm)))

        # 10cm first: where the evidence overlaps, it is a normal sign.
        if hit(STANDARD_MM):
            cue = STANDARD_MM
        elif hit(TALL_MM) and short_of_wall(TALL_MM) and clear_through(STANDARD_MM):
            cue = TALL_MM
        elif settings.lidar_sees_short:
            cue = (SHORT_MM if hit(SHORT_MM) and short_of_wall(SHORT_MM)
                   and not hit(TALL_MM) else None)
        else:
            # The scan plane passes over a 5cm sign, so a short one is the
            # beam going straight over where it stands and on to the wall,
            # with nothing in between - a sign the lidar CAN see standing
            # further back along the same line is no evidence either way.
            cue = (SHORT_MM if short_of_wall(SHORT_MM) and clear_through(SHORT_MM)
                   and nothing_before_wall(SHORT_MM) else None)
        return cue, expected, seen

    def _face_mm(self, detection, height_mm):
        """Lens to the sign's near face, for a sign `height_mm` tall."""
        return detection.distance_cm * 10.0 * height_mm / (detection.assumed_height_cm * 10.0)

    def _from_lidar(self, detection, height_mm):
        """
        Where a sign `height_mm` tall would be, as the LIDAR sees it.

        The camera's range and bearing are from the lens; the lidar is
        behind it on the same mast, so the same point is at a different
        bearing and range from there. At 30cm the difference is several
        degrees - more than a 5cm sign is wide.

        I/O:
            return: (bearing_deg, range_mm), lidar frame (0 ahead, + right),
                    range to the sign's near face
        """
        centre = self._face_mm(detection, height_mm) + FACE_TO_CENTER_MM
        bearing = math.radians(detection.bearing_deg)
        forward = (self.camera_offset_mm[0] - self.lidar_offset_mm[0]
                   + centre * math.cos(bearing))
        right = (self.camera_offset_mm[1] - self.lidar_offset_mm[1]
                 + centre * math.sin(bearing))
        return (math.degrees(math.atan2(right, forward)),
                math.hypot(forward, right) - FACE_TO_CENTER_MM)

    @staticmethod
    def nearest_around(scan, bearing_deg, span_deg=LOOK_AROUND_DEG):
        """
        (bearing_deg, range_mm) of the closest reading within span_deg of a
        bearing, or None. Bearing is -180..180, + right.
        """
        centre = int(round(bearing_deg))
        offsets = np.arange(-span_deg, span_deg + 1)
        readings = scan[(centre + offsets) % 360]
        if np.all(np.isnan(readings)):
            return None
        index = int(np.nanargmin(readings))
        return float(centre + offsets[index]), float(readings[index])

    def _readings(self, scan, bearing_deg):
        """Every fresh reading within lidar_window_deg of a bearing."""
        window = self.settings.lidar_window_deg
        centre = int(round(bearing_deg))
        indices = np.arange(centre - window, centre + window + 1) % 360
        readings = scan[indices]
        return readings[~np.isnan(readings)]

    # ========================================================================
    # THE SHAPE
    # ========================================================================

    def _shape_cue(self, pose, detection):
        """
        Which height the blob's height:width ratio points to.

        I/O:
            return: (height_mm or None, ratio or None)
        """
        settings = self.settings
        if pose is None or detection.clipped or detection.width_px <= 0.0:
            return None, None
        if abs(detection.off_axis_deg) > settings.shape_max_off_axis_deg:
            return None, None
        height = detection.height_px - settings.blob_pad_px
        width = detection.width_px - settings.blob_pad_px
        if height <= 0.0 or width <= 0.0:
            return None, None

        # The signs stand square to the walls, so the angle off the sign's
        # face is the sightline's field bearing folded into one quadrant. A
        # square of side s seen at angle a off its face is s(|cos a| + |sin a|)
        # wide.
        sightline = math.radians((pose.heading + detection.bearing_deg) % 90.0)
        width /= abs(math.cos(sightline)) + abs(math.sin(sightline))
        ratio = height / width

        if ratio <= settings.short_max_ratio:
            cue = SHORT_MM
        elif ratio >= settings.tall_min_ratio:
            cue = TALL_MM
        elif settings.standard_min_ratio <= ratio <= settings.standard_max_ratio:
            cue = STANDARD_MM
        else:
            return None, ratio      # between two bands: no opinion

        # Only close: the range the cue ITSELF implies, so a far tall sign
        # (which reads close as a 10cm one) does not count as close.
        range_mm = self._face_mm(detection, cue)
        if not settings.shape_min_range_mm <= range_mm <= settings.shape_max_range_mm:
            return None, ratio
        return cue, ratio

    # ========================================================================
    # REPORTING
    # ========================================================================

    LOG_COLUMNS = ("t_s", "color", "bearing_deg", "off_axis_deg", "height_px",
                   "width_px", "plain", "clipped", "ratio", "lidar_cue",
                   "shape_cue", "verdict", "vote", "dropped", "speed_mm_s",
                   "robot_x", "robot_y", "heading",
                   "lidar_50_expect", "lidar_50_nearest",
                   "lidar_100_expect", "lidar_100_nearest",
                   "lidar_150_expect", "lidar_150_nearest",
                   "lidar_closest_bearing", "lidar_closest_range")

    def _write_log(self, pose, detection, verdict, dropped):
        """
        One row per detection judged, to surprise.log_path. This is the file
        to read when a sign was decided late or wrongly: every cue, for every
        frame, with what the lidar expected and what it actually saw.
        """
        path = self.settings.log_path
        if not path:
            return
        try:
            if self._log is None:
                Path(path).parent.mkdir(parents=True, exist_ok=True)
                self._log_file = open(path, "w", newline="", buffering=1)
                self._log = csv.writer(self._log_file)
                self._log.writerow(self.LOG_COLUMNS)
                self._log_started = time.monotonic()
            lidar = []
            for height in HEIGHTS_MM:
                if not verdict.lidar_ranges:
                    lidar += ["", ""]
                    continue
                expect = verdict.lidar_ranges[height][1]
                seen = verdict.lidar_seen[height]
                nearest = (float(seen[np.argmin(np.abs(seen - expect))])
                           if len(seen) else "")
                lidar += [round(expect), nearest if nearest == "" else round(nearest)]
            self._log.writerow([
                round(time.monotonic() - self._log_started, 3),
                detection.color.name, round(detection.bearing_deg, 1),
                round(detection.off_axis_deg, 1),
                round(detection.height_px, 1), round(detection.width_px, 1),
                int(detection.plain), int(detection.clipped),
                "" if verdict.ratio is None else round(verdict.ratio, 2),
                verdict.lidar_cue or "", verdict.shape_cue or "", verdict.height_mm,
                verdict.vote or "", int(dropped), round(self.speed_mm_s),
                "" if pose is None else round(pose.x),
                "" if pose is None else round(pose.y),
                "" if pose is None else round(pose.heading, 1)] + lidar
                + (["", ""] if verdict.lidar_nearest is None else
                   [round(verdict.lidar_nearest[0]), round(verdict.lidar_nearest[1])]))
        except OSError as e:
            print(f"WARNING: cannot write {path} ({e!r}) - surprise log off")
            self.settings.log_path = ""

    def close(self):
        if self._log_file is not None:
            self._log_file.close()
            self._log_file = self._log = None

    def status_line(self):
        return (f"sizer {self.frames}f votes 5cm:{self.votes[SHORT_MM]} "
                f"10cm:{self.votes[STANDARD_MM]} 15cm:{self.votes[TALL_MM]} "
                f"dropped:{self.dropped}")
