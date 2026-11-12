"""
The red and green pillars, pinned to the field instead of to the camera.

ObjectSolver answers "there is a red box 80cm ahead and 12 degrees right of the
camera". That is the right answer for dodging something in the next second, and
the wrong answer for everything else: the moment the pillar leaves the 40-degree
camera cone it stops existing, so the robot cannot plan a line through pillars
it has already driven past, and it re-discovers the same pillar four times a lap
as if it were four different ones.

BlockMap converts each detection into field coordinates using the pose from
NavigationManager, then merges detections that land in the same place into one
tracked block. What comes out is a map of the course that persists after the
camera looks away.

    nav.observe_blocks(object_solver.detect(camera.hsv_image))
    for block in nav.blocks.confirmed():
        print(block.color, block.x, block.y)

Blocks are 50mm square and 100mm tall. Orientation is not tracked - a square
looks the same from every side, and the camera cannot measure the yaw of one
anyway.

Everything here is in millimeters and the field frame from field_map.py, even
though ObjectSolver talks centimeters - the conversion happens once, on the way
in, so nothing downstream has to remember which unit it is holding.
"""
import itertools
import math
import threading
import time
from dataclasses import dataclass, field

import cv2

from classes.field_map import FieldMap
from classes.object_solver import WORKING_HORIZONTAL_FOV_DEG
from classes.robot_geometry import (CAMERA_BEHIND_LIDAR_MM,  # noqa: F401 - re-exported
                                    camera_offset_behind_lidar, to_field)
from utils.angle_utils import angle_difference, normalize_angle
from utils.enums import Color
from utils.image_drawing_utils import ImageDrawingUtils

# ============================================================================
# Block geometry (WRO Future Engineers traffic signs)
# ============================================================================
BLOCK_SIZE_MM = 50.0             # 50mm x 50mm footprint
BLOCK_HEIGHT_MM = 100.0          # 100mm tall - what ObjectSolver ranges off

# ObjectSolver measures the FRONT FACE of the block, since that is the surface
# whose height it sees. The block's center is half a block deeper.
FACE_TO_CENTER_MM = BLOCK_SIZE_MM / 2.0

# ============================================================================
# Mapping
# ============================================================================
# Two detections closer together than this are the same block. Comfortably
# bigger than the block itself, because the range estimate is what is uncertain,
# and comfortably smaller than the gap the rules leave between two pillars.
ASSOCIATION_RADIUS_MM = 180.0

# Range beyond which a detection is watched but not mapped. Distance comes from
# angular height, so its error grows with the square of range - at 2m a 5cm box
# is a handful of pixels tall and one pixel of contour noise moves it 20cm.
MAX_MAPPING_RANGE_MM = 1500.0

# A pose this uncertain would smear the block across the field, so detections
# taken during it are dropped rather than mapped into the wrong place.
MIN_POSE_CONFIDENCE = 0.35

# --- Confirmation / pruning model -------------------------------------
# A block goes through three states: TENTATIVE (< HITS_TO_CONFIRM sightings,
# not steered on), CONFIRMED (steered on), and DROPPED. Two independent
# things can drop a block, and both matter:
#
#   MAX_MISSES   ticks up only when the block SHOULD currently be in frame
#                (see _should_be_visible) and is not - a false positive from
#                one bad frame disappears instead of sitting on the map
#                forever.
#   MAX_STALE_S  a real-time backstop that does not care about visibility at
#                all. MAX_MISSES alone has a gap: a block mapped to a wrong
#                position that rarely ends up back in frame (behind you most
#                of the lap, hidden by the centre block) can dodge that check
#                indefinitely and steer the line off a phantom for most of a
#                lap. This forces every block to be RE-confirmed at least
#                this often or be dropped, no exceptions.
HITS_TO_CONFIRM = 2              # sightings before a block is believed - lower
                                  # than it looks: at CAMERA_EVERY_N_TICKS=5,
                                  # every missed/occluded frame along the way
                                  # burns into the approach_mm ramp before the
                                  # robot is even allowed to start dodging
MAX_MISSES = 12                   # unseen-but-should-be-visible frames before dropping
MAX_STALE_S = 6.0                # unconfirmed-regardless-of-visibility timeout
POSITION_SMOOTHING_FLOOR = 0.15  # a settled block can still be nudged this hard

# Blocks live on the mat, so anything solved into a wall is a bad detection.
BLOCK_EDGE_MARGIN_MM = 20.0

# --- The surprise rule: how tall each block is ---------------------------
# Off unless BlockMap.size_voting is set, and then only for detections that
# SignSizer has sized (classes/sign_sizer.py). Every block starts believed to
# be STANDARD_HEIGHT_MM and is only ever believed otherwise once its votes
# latch - see BlockMap._decide_size.
STANDARD_HEIGHT_MM = int(BLOCK_HEIGHT_MM)


@dataclass
class SizeVoting:
    """
    When a block's votes settle its height. From tasks/final/config.toml
    [surprise] - see there for what each one means.
    """
    latch_votes: int         # votes for 5cm (or 15cm) before it can latch
    latch_fraction: float    # ...and the share of all votes they must be
    settle_votes: int        # 10cm votes before a block is settled as 10cm

# ============================================================================
# Camera
# ============================================================================
# Taken from ObjectSolver rather than repeated here: it is a measured property
# of the lens + sensor mode, it has already been re-measured once, and a stale
# copy of it would silently break the "should I have seen that block?" test.
# The CROPPED frame's angle, not the full sensor's: a block outside the zoom
# window is never seen, and must not be dropped for it.
DEFAULT_CAMERA_FOV_DEG = WORKING_HORIZONTAL_FOV_DEG

# Where the lens is relative to the rear axle (the point the pose describes)
# lives in classes/robot_geometry.py with the rest of the robot's dimensions;
# the names are re-exported here for the callers that always found them here.
# Blocks at the very edge of the frame are half-cut and range badly, so the
# "should I have seen it?" test uses a slightly narrower cone than the real one.
VISIBILITY_FOV_MARGIN_DEG = 4.0

# ============================================================================
# Drawing
# ============================================================================
COLOR_BGR = {
    Color.GREEN: (70, 120, 60),
    Color.RED: (140, 30, 30),
}
UNCONFIRMED_BGR = (110, 110, 110)


# A serial number per tracked block, for anything that has to remember
# something ABOUT a block between ticks (see FinalTask's ramp anchors).
# Neither position nor colour identifies a pillar: both are shared by other
# pillars, the position drifts as sightings refine it, and a dropped block
# re-detected later is a genuinely new track that must not inherit the old
# one's state. Never reused, so a stale key cannot land on a live block.
_BLOCK_UIDS = itertools.count(1)


@dataclass
class Block:
    """One tracked pillar, in field millimeters."""
    color: Color
    x: float
    y: float
    hits: int = 1
    misses: int = 0
    last_seen: float = field(default_factory=time.monotonic)
    uid: int = field(default_factory=lambda: next(_BLOCK_UIDS))

    # ---- The surprise rule - see BlockMap._observe_sized -----------------
    # The height its x, y are ranged for: the one most frames placed it with
    # (range_votes), or the latched one once it is modified.
    height_mm: int = STANDARD_HEIGHT_MM
    # Frames that RANGED it with each height, {height_mm: count}. Not the
    # same as votes: the lidar alone can say where a tall sign is, long
    # before both cues agree on what it is.
    range_votes: dict = field(default_factory=dict)
    # LATCHED: the votes said 5cm or 15cm, clearly enough. Only undone if the
    # 10cm votes later win just as clearly - see BlockMap._decide_size.
    modified: bool = False
    # It WAS latched and the 10cm votes took it back. FinalTask clears the
    # round's memory of it on seeing this.
    unlatched: bool = False
    # Frames that voted for each height, {height_mm: count}.
    votes: dict = field(default_factory=dict)
    # A position per height the block could be, {height_mm: [x, y, samples]}.
    # One sign ranged as 10cm and as 15cm lands in two places a third of the
    # range apart, so a single running average would be dragged between them
    # - and one far enough apart would split the sign into two blocks. Each
    # height keeps its own, and x, y is whichever one height_mm says.
    estimates: dict = field(default_factory=dict)
    # Set by BlockMap._decide_size from the rules it was given: settled is
    # "enough votes say 10cm", undecided is "neither settled nor latched".
    # Both stay False while the surprise rule is off.
    settled: bool = False
    undecided: bool = False

    @property
    def is_confirmed(self):
        return self.hits >= HITS_TO_CONFIRM

    @property
    def evidence(self):
        """
        How much the votes favour the height this block is believed to be,
        over every other height: its votes minus all the others'. What
        FinalTask compares when two blocks both latch, to keep only one.
        """
        own = self.votes.get(self.height_mm, 0)
        return own - (sum(self.votes.values()) - own)

    @property
    def position(self):
        return self.x, self.y

    def __str__(self):
        state = "confirmed" if self.is_confirmed else f"{self.hits}/{HITS_TO_CONFIRM}"
        if self.modified:
            state += f" {self.height_mm / 10:.0f}cm MODIFIED"
        elif self.undecided:
            state += " size?"
        return f"{self.color.value:<5} ({self.x / 10:+6.1f}, {self.y / 10:+6.1f})cm [{state}]"


class BlockMap:
    """
    Tracks red/green pillars in field coordinates, fed by ObjectSolver
    detections plus the pose they were taken from.

    Association is nearest-neighbour by color inside ASSOCIATION_RADIUS_MM,
    which is all the structure this problem needs: the pillars do not move, the
    rules space them well apart, and a block that gets mis-associated will be
    voted back into place by the next few sightings anyway.

    Blocks that should be in shot but are not get a miss counted against them,
    so a false positive from a bad frame disappears instead of sitting on the
    map forever. "Should be in shot" accounts for the center block getting in
    the way, or a pillar behind it would be pruned every time the robot drove
    down the far side of the field.
    """

    def __init__(self, field_map=None, camera_offset_mm=None,
                 camera_yaw_deg=0.0, camera_fov_deg=DEFAULT_CAMERA_FOV_DEG,
                 max_range_mm=MAX_MAPPING_RANGE_MM, debug=False):
        """
        I/O:
            camera_offset_mm: (forward, right) of the lens from the REAR
                              AXLE - the point the pose describes - in the
                              robot's own frame. Defaults to
                              CAMERA_BEHIND_LIDAR_MM behind the lidar's own
                              default position; NavigationManager overrides
                              this with the offset derived from wherever its
                              lidar actually is.
            camera_yaw_deg: how far the camera is turned off straight ahead
            camera_fov_deg: horizontal field of view, for the visibility test
        """
        self.map = field_map or FieldMap()
        if camera_offset_mm is None:
            camera_offset_mm = camera_offset_behind_lidar()
        self.camera_offset_mm = tuple(float(v) for v in camera_offset_mm)
        self.camera_yaw_deg = float(camera_yaw_deg)
        self.camera_fov_deg = float(camera_fov_deg)
        self.max_range_mm = float(max_range_mm)
        self.debug = debug
        # The surprise rule's vote rules (SizeVoting), or None for off. Only
        # read for detections SignSizer has sized, so with no sizer this
        # changes nothing.
        self.size_voting = None

        self._blocks = []
        self.observations = 0
        self.rejected = 0

        # The map is WRITTEN by whoever runs the camera and READ by the
        # control loop, and those stop being the same thread once detection
        # moves off the control loop. observe() appends to _blocks mid-frame
        # and rebinds it at the end, and mutates a Block's hits/x/y in place -
        # so an unguarded reader can see a half-applied frame, or a block
        # whose is_confirmed flips underneath it. Reentrant because the public
        # readers call each other (summary -> confirmed).
        self._lock = threading.RLock()

    # ========================================================================
    # READING
    # ========================================================================

    def all(self):
        """Every tracked block, confirmed or not."""
        with self._lock:
            return list(self._blocks)

    def confirmed(self):
        """Blocks seen enough times to steer on."""
        with self._lock:
            return [block for block in self._blocks if block.is_confirmed]

    def nearest(self, x, y, color=None, confirmed_only=True):
        """Closest tracked block to a point, or None."""
        with self._lock:
            candidates = self.confirmed() if confirmed_only else list(self._blocks)
            if color is not None:
                candidates = [block for block in candidates if block.color == color]
            if not candidates:
                return None
            return min(candidates, key=lambda b: math.hypot(b.x - x, b.y - y))

    def clear(self):
        with self._lock:
            self._blocks = []

    def __len__(self):
        with self._lock:
            return len(self._blocks)

    # ========================================================================
    # MAPPING
    # ========================================================================

    def observe(self, pose, detections):
        """
        Folds one frame of ObjectSolver detections into the map.

        I/O:
            pose: the Pose the frame was taken from. Ignored entirely when it
                  is not reliable - a detection placed with a bad pose is
                  worse than no detection, because it lands somewhere real and
                  looks just as trustworthy as a good one.
            detections: list of DetectedObject from ObjectSolver.detect()
            return: list of the Blocks that were hit or created this frame
        """
        if pose is None or pose.confidence < MIN_POSE_CONFIDENCE:
            self.rejected += len(detections or ())
            return []

        camera_x, camera_y = self.camera_position(pose)
        # One frame lands as one atomic change: a reader sees the map either
        # before or after it, never with half the detections merged and the
        # miss count not yet applied.
        with self._lock:
            seen = []
            for detection in detections or ():
                if detection.candidates_cm and self.size_voting is not None:
                    block = self._observe_sized(pose, camera_x, camera_y, detection)
                    if block is None:
                        self.rejected += 1
                    else:
                        seen.append(block)
                    continue
                point = self._to_field(pose, camera_x, camera_y, detection)
                if point is None:
                    self.rejected += 1
                    continue
                seen.append(self._merge(detection.color, *point))

            self.observations += len(seen)
            self._count_misses(pose, camera_x, camera_y, seen)
            return seen

    def camera_position(self, pose):
        """Where the lens is in field coordinates, given the robot's pose."""
        x, y = to_field(pose.x, pose.y, pose.heading, self.camera_offset_mm)
        return float(x), float(y)

    def _to_field(self, pose, camera_x, camera_y, detection):
        """
        One detection's field position, or None if it should not be mapped.

        ObjectSolver's bearing is relative to the camera's forward axis, so the
        field bearing is just the robot's heading plus the camera's mounting
        yaw plus that - all three are clockwise-positive, which is the whole
        reason the convention is shared.
        """
        return self._place(pose, camera_x, camera_y, detection.distance_cm,
                           detection.bearing_deg)

    def _place(self, pose, camera_x, camera_y, distance_cm, bearing_deg):
        """_to_field for a range and bearing rather than a detection."""
        distance = distance_cm * 10.0 + FACE_TO_CENTER_MM
        if distance > self.max_range_mm:
            return None

        bearing = normalize_angle(pose.heading + self.camera_yaw_deg + bearing_deg)
        radians = math.radians(bearing)
        x = camera_x + distance * math.sin(radians)
        y = camera_y + distance * math.cos(radians)

        # A pillar solved into a wall or inside the center block is a bad
        # detection (usually a colored reflection), not a pillar.
        if not self.map.contains(x, y, BLOCK_EDGE_MARGIN_MM):
            return None
        return x, y

    def _merge(self, color, x, y):
        """Updates the nearest matching block, or starts tracking a new one."""
        match = None
        best = ASSOCIATION_RADIUS_MM ** 2
        for block in self._blocks:
            if block.color != color:
                continue
            distance = (block.x - x) ** 2 + (block.y - y) ** 2
            if distance < best:
                best, match = distance, block

        if match is None:
            match = Block(color=color, x=x, y=y)
            self._blocks.append(match)
            if self.debug:
                print(f"[blocks] new {match}")
            return match

        # Running average that never fully freezes: early sightings move the
        # block a lot, later ones only trim it, but a block that is genuinely
        # in the wrong place can still be walked back.
        match.hits += 1
        alpha = max(POSITION_SMOOTHING_FLOOR, 1.0 / match.hits)
        match.x += alpha * (x - match.x)
        match.y += alpha * (y - match.y)
        match.misses = 0
        match.last_seen = time.monotonic()
        return match

    # ========================================================================
    # THE SURPRISE RULE - how tall each block is
    # ========================================================================

    def _observe_sized(self, pose, camera_x, camera_y, detection):
        """
        _merge for a detection SignSizer has sized: placed at every height it
        could be, matched against each block AT THAT BLOCK'S OWN HEIGHT, and
        its vote counted.

        Matching on the frame's own verdict instead would split a tall sign
        in two. Far off, before the shape can be judged, its frames say 10cm
        and place it at the 10cm range; close up they say 15cm and place it
        half as far again - over ASSOCIATION_RADIUS_MM from the first. So
        each block is matched at the height IT is believed to be, and keeps
        a position for each of the others to move to if the votes latch.

        I/O:
            return: the Block hit or created, or None if the frame's own
                    height places it off the map or out of range
        """
        points = {}
        for height_mm, distance_cm in detection.candidates_cm.items():
            point = self._place(pose, camera_x, camera_y, distance_cm,
                                detection.bearing_deg)
            if point is not None:
                points[int(height_mm)] = point
        chosen = int(round(detection.assumed_height_cm * 10.0))
        if chosen not in points:
            return None

        match = None
        best = ASSOCIATION_RADIUS_MM ** 2
        for block in self._blocks:
            if block.color != detection.color:
                continue
            point = points.get(block.height_mm)
            if point is None:
                continue
            distance = (block.x - point[0]) ** 2 + (block.y - point[1]) ** 2
            if distance < best:
                best, match = distance, block

        if match is None:
            x, y = points[chosen]
            match = Block(color=detection.color, x=x, y=y, height_mm=chosen)
            match.range_votes = {chosen: 1}
            match.estimates = {height: [px, py, 1] for height, (px, py) in points.items()}
            self._blocks.append(match)
            if self.debug:
                print(f"[blocks] new {match}")
        else:
            match.hits += 1
            for height, (px, py) in points.items():
                estimate = match.estimates.get(height)
                if estimate is None:
                    match.estimates[height] = [px, py, 1]
                    continue
                # The same running average _merge keeps, per height.
                estimate[2] += 1
                alpha = max(POSITION_SMOOTHING_FLOOR, 1.0 / estimate[2])
                estimate[0] += alpha * (px - estimate[0])
                estimate[1] += alpha * (py - estimate[1])
            match.range_votes[chosen] = match.range_votes.get(chosen, 0) + 1
            if not match.modified:
                # Placed where MOST frames placed it; a tie stays put.
                placed = max(match.range_votes, key=lambda height: (
                    match.range_votes[height], height == match.height_mm))
                if placed in match.estimates:
                    match.height_mm = placed
            match.x, match.y = match.estimates[match.height_mm][:2]
            match.misses = 0
            match.last_seen = time.monotonic()

        if detection.size_vote is not None:
            match.votes[detection.size_vote] = match.votes.get(detection.size_vote, 0) + 1
        self._decide_size(match)
        return match

    def _decide_size(self, block):
        """
        Latches a block as modified once its votes say so clearly, and keeps
        its settled/undecided flags current.

            LATCH   votes for 5cm (or 15cm) reach latch_votes AND are at least
                    latch_fraction of every vote it has had. Once latched it
                    stays latched, and its position moves to the one ranged
                    for that height.
            SETTLED the same test passed by 10cm votes, with settle_votes.
            UNDECIDED  neither yet - the round slows down for these.
            UNLATCH a latched block whose 10cm votes then pass the same test
                    (settle_votes, and latch_fraction of ALL its votes,
                    the odd ones included). A few bad frames in a row can
                    latch a normal sign; dozens of good ones afterwards
                    should be able to say so. Measured on the mat: a normal
                    green latched as 5cm on 4 votes and finished the lap at
                    4 against 44.

        Where an unlatched block STANDS is not decided here - that follows
        range_votes, in _observe_sized.
        """
        rules = self.size_voting
        total = sum(block.votes.values())

        def clear(height, needed):
            count = block.votes.get(height, 0)
            return count >= needed and count >= rules.latch_fraction * total

        if block.modified and clear(STANDARD_HEIGHT_MM, rules.settle_votes):
            block.modified = False
            block.unlatched = True
            self._move_to_height(block, STANDARD_HEIGHT_MM)
            print(f"[blocks] {block.color.name} sign at ({block.x / 10:+.0f}, "
                  f"{block.y / 10:+.0f})cm is a normal 10cm sign after all - "
                  f"votes {self._votes_text(block)}")
        elif not block.modified:
            for height in sorted(block.votes, key=lambda h: -block.votes[h]):
                if height == STANDARD_HEIGHT_MM or not clear(height, rules.latch_votes):
                    continue
                block.modified = True
                self._move_to_height(block, height)
                print(f"[blocks] {block.color.name} sign at ({block.x / 10:+.0f}, "
                      f"{block.y / 10:+.0f})cm is {height / 10:.0f}cm tall - "
                      f"votes {self._votes_text(block)}")
                break
        block.settled = (not block.modified
                         and clear(STANDARD_HEIGHT_MM, rules.settle_votes))
        block.undecided = not block.modified and not block.settled

    @staticmethod
    def _move_to_height(block, height_mm):
        block.height_mm = height_mm
        estimate = block.estimates.get(height_mm)
        if estimate is not None:
            block.x, block.y = estimate[0], estimate[1]

    @staticmethod
    def _votes_text(block):
        return " ".join(f"{height / 10:.0f}cm:{count}"
                        for height, count in sorted(block.votes.items()))

    def _count_misses(self, pose, camera_x, camera_y, seen):
        """
        Penalizes blocks that were in shot and in range but did not turn up,
        and drops the ones that keep failing to.
        """
        now = time.monotonic()
        survivors = []
        for block in self._blocks:
            if block not in seen:
                if now - block.last_seen > MAX_STALE_S:
                    if self.debug:
                        print(f"[blocks] dropping {block} - stale "
                              f"({now - block.last_seen:.1f}s unconfirmed)")
                    continue
                if self._should_be_visible(pose, camera_x, camera_y, block):
                    block.misses += 1
                    if block.misses >= MAX_MISSES:
                        if self.debug:
                            print(f"[blocks] dropping {block} - not there")
                        continue
            survivors.append(block)
        self._blocks = survivors

    def _should_be_visible(self, pose, camera_x, camera_y, block):
        """True if this block ought to have shown up in the frame."""
        offset_x = block.x - camera_x
        offset_y = block.y - camera_y
        distance = math.hypot(offset_x, offset_y)
        if distance > self.max_range_mm or distance < 1.0:
            return False

        bearing = math.degrees(math.atan2(offset_x, offset_y))
        aim = pose.heading + self.camera_yaw_deg
        if abs(angle_difference(bearing, aim)) > (self.camera_fov_deg / 2.0
                                                  - VISIBILITY_FOV_MARGIN_DEG):
            return False

        # The center block hides everything behind it, and half the lap is
        # spent looking straight at it.
        sightline = float(self.map.raycast(camera_x, camera_y, math.radians(bearing)))
        return sightline >= distance - BLOCK_SIZE_MM

    # ========================================================================
    # DRAWING
    # ========================================================================

    def draw(self, canvas, to_px, scale):
        """
        Draws every tracked block onto a top-down canvas.

        I/O:
            to_px: field mm -> pixel callable, from the caller's renderer
            scale: pixels per mm, so the squares come out life-size
        """
        half = BLOCK_SIZE_MM / 2.0
        for block in self.all():
            color = COLOR_BGR.get(block.color, (200, 200, 200))
            corners = to_px(block.x - half, block.y + half), to_px(block.x + half, block.y - half)
            if block.is_confirmed:
                cv2.rectangle(canvas, corners[0], corners[1], color, -1)
                cv2.rectangle(canvas, corners[0], corners[1], (240, 240, 240), 1)
            else:
                # Hollow and grey until it has been seen enough times, so a
                # one-frame false positive is visibly different from a pillar.
                cv2.rectangle(canvas, corners[0], corners[1], UNCONFIRMED_BGR, 1)
                ImageDrawingUtils.draw_label(canvas, str(block.hits),
                                             (corners[1][0] + 3, corners[1][1]),
                                             UNCONFIRMED_BGR, scale=0.35)
        return canvas

    def draw_camera_cone(self, canvas, to_px, pose, color=(70, 70, 90)):
        """The camera's field of view, so it is obvious what the map can see."""
        camera_x, camera_y = self.camera_position(pose)
        origin = to_px(camera_x, camera_y)
        for side in (-1.0, 1.0):
            radians = math.radians(pose.heading + self.camera_yaw_deg
                                   + side * self.camera_fov_deg / 2.0)
            cv2.line(canvas, origin,
                     to_px(camera_x + self.max_range_mm * math.sin(radians),
                           camera_y + self.max_range_mm * math.cos(radians)), color, 1)
        return canvas

    def summary(self):
        with self._lock:
            counts = {color: 0 for color in COLOR_BGR}
            confirmed = self.confirmed()
            for block in confirmed:
                counts[block.color] = counts.get(block.color, 0) + 1
            pending = len(self._blocks) - len(confirmed)
        return (f"{counts.get(Color.GREEN, 0)} green  {counts.get(Color.RED, 0)} red  "
                f"({pending} pending, {self.rejected} rejected)")

    def debug_text(self):
        lines = [f"blocks    {self.summary()}"]
        lines.extend(f"  {block}" for block in sorted(self.all(),
                                                      key=lambda b: (b.color.value, b.x)))
        return "\n".join(lines)
