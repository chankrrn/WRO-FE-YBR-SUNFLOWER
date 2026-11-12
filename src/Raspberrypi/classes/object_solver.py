import math
import threading
from dataclasses import dataclass, replace

import cv2
import numpy as np

from utils.enums import Color
from utils.image_color_utils import ImageColorUtils
from utils.image_drawing_utils import ImageDrawingUtils
from utils.image_transform_utils import ImageTransformUtils

# ============================================================================
# Camera / lens configuration
# ============================================================================
# IMPORTANT: this is NOT the lens' rated FOV (now a 222 deg lens, was 160).
# Picamera2's chosen sensor
# mode + the (CAMERA_PIC_WIDTH x CAMERA_PIC_HEIGHT) main-stream resize crop
# the sensor before any of this code sees a frame, so the *delivered* image
# covers a smaller angle than the lens spec sheet. Measure this directly
# against the actual frame (point the camera at two references a known angle
# apart, e.g. floor tiles or a printed protractor, and count pixels between
# them) rather than trusting the lens datasheet number.
#
# 105 was fitted on the OLD 160 deg lens (red pillar straight ahead at 50 /
# 100 / 150cm read 68 / 36 / 23px tall). On the 222 deg lens a pillar at 50 /
# 70cm read 25 / 19px, implying 292 / 275 - averaged to 283. It can exceed the
# lens' 222 because this is deg-per-px x frame width, and the fisheye's image
# circle is narrower than the frame. Re-measure with:
#   python test_object_solver.py --fov-at 50
# with a pillar 50cm straight ahead, and put the number it prints here.
CAPTURED_HORIZONTAL_FOV_DEG = 230.0

# The camera is not mounted dead straight. Added to every bearing so 0 is the
# ROBOT's straight ahead, not the lens's: + if the camera points left of the
# robot's axis (things straight ahead then read right of the middle), - if
# it points right. Measured 2026-10-26 on the OV5647 / 222 deg camera with a
# pillar 50cm straight ahead, averaged over 30 frames (--place 50 0
# --headless): +6.6. A first single-frame reading had said 8.6. Re-measure
# after anything moves the camera or its lens is refocused:
#   python test_object_solver.py --place 50 0 --headless
CAMERA_YAW_DEG = 6.6

# Full, UNCROPPED main-stream width that CAPTURED_HORIZONTAL_FOV_DEG was
# measured against (see camera_manager.py's configure_camera/transform_image).
CAPTURED_FRAME_WIDTH_PX = ImageTransformUtils.CAMERA_PIC_WIDTH

# Degrees per pixel, assumed the SAME on both axes (square pixels + an
# equidistant/fisheye-style lens map angle linearly to pixel distance from the
# optical center regardless of the sensor's rectangular aspect ratio). Using
# one figure for both axes is what keeps the 5cm height measurement correct on
# a frame that isn't square.
DEG_PER_PX = CAPTURED_HORIZONTAL_FOV_DEG / CAPTURED_FRAME_WIDTH_PX

# What the CROPPED (zoomed) frame actually covers - the angle anything that
# asks "should the camera see this?" has to use, not the full-frame figure.
WORKING_HORIZONTAL_FOV_DEG = ImageTransformUtils.PIC_WIDTH * DEG_PER_PX

# Real-world HEIGHT of the boxes. The WRO traffic signs are 5cm x 5cm on the
# floor and 10cm tall - this is the 10, not the 5. Distance scales linearly
# with it, so the 5.0 that used to be here reported every pillar at half its
# true range.
BOX_HEIGHT_CM = 10.0
MIN_CONTOUR_AREA_PX = 100        # ignore specks smaller than this (noise)
MAX_DISTANCE_CM = 150.0         # ignore solves further than this (noise / grazing angle)
MIN_ANGULAR_HEIGHT_DEG = 0.05   # avoid a division by ~0 on a 1px-tall contour

# THE SURPRISE RULE. One sign per obstacle round may be 5cm or 15cm tall
# instead of BOX_HEIGHT_CM. Nothing in this file decides which one: with
# set_surprise() switched on it only stops throwing the odd one away, and
# still ranges every box as if it were BOX_HEIGHT_CM. classes/sign_sizer.py
# works out the height and re-ranges it (see rerange).
SHORT_SIGN_HEIGHT_CM = 5.0
TALL_SIGN_HEIGHT_CM = 15.0
# A 5cm sign ranged as a 10cm one reads twice as far as it really is, so a
# short sign right at MAX_DISTANCE_CM reads at this. The cut-off is applied
# again, to the re-ranged distance, once the height is known.
SURPRISE_MAX_DISTANCE_CM = MAX_DISTANCE_CM * BOX_HEIGHT_CM / SHORT_SIGN_HEIGHT_CM

TARGET_COLORS = (Color.GREEN, Color.RED)

# The parking bay's walls. Found separately from the pillars, and deliberately:
# they must never reach the block map (a bay wall folded in as a pillar is
# something the lap would swerve around), and they do not pass the tests the
# pillars are found by - see detect_parking.
PARKING_COLOR = Color.PINK
# Lower than the pillars' threshold: a bay wall is a low bar seen from a
# couple of metres away, which is a much smaller blob than a 10cm pillar at
# the same range, and it only has to be caught once while it is still ahead.
MIN_PARKING_AREA_PX = 500

# ============================================================================
# Top-down debug visualizer configuration
# ============================================================================
TOP_DOWN_CANVAS_PX = 500
TOP_DOWN_CM_PER_PX = 1.0        # canvas covers +/-250cm around the robot
TOP_DOWN_RING_STEP_CM = 50

COLOR_BGR = {
    Color.GREEN: (0, 255, 0),
    Color.RED: (0, 0, 255),
}
PARKING_BGR = (255, 0, 255)      # the bay walls, in the debug overlay


@dataclass
class ParkingMark:
    """
    A pink bay wall as the camera sees it: which way, and how big.

    NO DISTANCE, on purpose. Every range in DetectedObject is solved from the
    apparent height of a 10cm pillar; a bay wall is a different object of a
    different size, so that number would be confidently wrong. The bearing
    comes from the pixel centre and is independent of what size the thing is,
    so that is what this carries - enough to say "the bay is over there", which
    is all the camera is asked for.
    """
    bearing_deg: float      # + right of the camera axis, - left
    area_px: float
    width_px: float
    pixel_center: tuple
    box_points: np.ndarray


@dataclass
class DetectedObject:
    color: Color
    distance_cm: float   # straight-line distance from the camera to the box
                         # (forward_cm / cos(bearing) - see _solve_position)
    bearing_deg: float   # angle off the camera's forward axis, +right / -left
    forward_cm: float     # distance projected onto the robot's forward axis
    lateral_cm: float     # distance projected onto the robot's right axis
    pixel_center: tuple
    box_points: np.ndarray  # 4 corners of the detected box, for drawing
    height_px: float = 0.0  # the rect's long side - what distance_cm is solved from

    # ---- The surprise rule - only ever set while set_surprise() is on -------
    # The rect's other side. With the surprise rule on, height_px is the
    # side nearer VERTICAL and this the one nearer horizontal, rather than
    # long and short: a 5cm cube seen corner-on is wider than it is tall.
    width_px: float = 0.0
    # The height distance_cm was solved for. rerange() changes it.
    assumed_height_cm: float = BOX_HEIGHT_CM
    # Bearing off the LENS axis, without CAMERA_YAW_DEG - how far from the
    # middle of the frame, which is where the shape is worth trusting.
    off_axis_deg: float = 0.0
    # The blob touches an edge of the frame, so part of it is cut off and its
    # shape says nothing about the sign's.
    clipped: bool = False
    # Would the solver have reported this with the surprise rule off? False
    # for the squat, the tiny and the far ones it lets through for SignSizer
    # to look at - which SignSizer drops again unless it finds them to be a
    # short sign.
    plain: bool = True
    # Filled in by SignSizer: the face distance for each height it could be,
    # {height_mm: cm}, the height this frame votes for (None = no opinion)
    # and whether either cue alone said the sign is not standard.
    candidates_cm: dict = None
    size_vote: int = None
    size_suspect: bool = False


def range_for_angular_height(angular_height_deg, height_cm=BOX_HEIGHT_CM):
    """
    Straight-line distance to a sign of a known height from the angle it
    spans. Linear in height_cm, which is what lets rerange() rescale a
    distance rather than solve it again.
    """
    return (height_cm / 2.0) / math.tan(math.radians(angular_height_deg / 2.0))


def rerange(detection, height_cm):
    """
    A copy of `detection` with every range solved for a sign `height_cm` tall
    instead of the one it was solved for. The bearing is unchanged: it comes
    from the pixel column, which does not care how tall the sign is.
    """
    scale = height_cm / detection.assumed_height_cm
    return replace(detection,
                   distance_cm=round(detection.distance_cm * scale, 1),
                   forward_cm=round(detection.forward_cm * scale, 1),
                   lateral_cm=round(detection.lateral_cm * scale, 1),
                   assumed_height_cm=height_cm)


@dataclass
class SurpriseDetection:
    """
    What the solver lets through while the surprise rule is on - see
    ObjectSolver.set_surprise. Both numbers come from tasks/final/config.toml
    [surprise].
    """
    min_area_px: float       # smallest blob still worth a look
    max_squat_ratio: float   # widest blob, as width / height, still a sign


class ObjectSolver:
    """
    Detects GREEN and RED boxes in a frame and solves their position relative
    to the camera, using the boxes' known real-world height (BOX_HEIGHT_CM)
    and the lens' angular field of view.

    Coordinate convention (robot/camera frame, centimeters):
        forward_cm : + straight ahead of the camera
        lateral_cm : + to the camera's right, - to the left

    This assumes a level camera. It does not perform fisheye/lens
    undistortion - pixel offsets are mapped to angles linearly across the
    sensor (equidistant projection) using a single deg-per-pixel figure
    (DEG_PER_PX) shared by both axes, which is a reasonable approximation for
    wide-angle lenses without a full calibration, but will be least accurate
    right at the edges of the frame.

    Distance comes from the box's ANGULAR SIZE, which is independent of where
    the box sits in the frame - so camera_manager.transform_image() cropping
    away the top rows does not affect it. Bearing is measured from a single
    pixel column, though, so it does depend on the frame's horizontal origin;
    horizontal_crop_offset_px corrects that for the horizontal crop (it
    defaults to ImageTransformUtils.PIC_X_START, the zoom crop's left edge).
    """

    def __init__(self, debug=False, image_width=None, image_height=None,
                 horizontal_crop_offset_px=None):
        self.debug = debug
        self.image_width = image_width or ImageTransformUtils.PIC_WIDTH
        self.image_height = image_height or ImageTransformUtils.PIC_HEIGHT
        self.horizontal_crop_offset_px = (ImageTransformUtils.PIC_X_START
                                          if horizontal_crop_offset_px is None
                                          else horizontal_crop_offset_px)
        self._last_parking = []
        # None = the surprise rule is off and every blob is judged exactly as
        # it always was. See set_surprise.
        self.surprise = None

        # Debug canvases drawn by detect() and waiting to be put on screen.
        # detect() runs on VisionManager's thread (classes/vision_manager.py)
        # and HighGUI is main-thread only: an imshow() from a worker thread
        # does not raise on OpenCV 5's Qt backend, it prints
        #     QMetaObject::invokeMethod: No such method GuiReceiver::showImage
        # once per frame and drops the picture. So detect() only renders, and
        # show_debug() - called from the thread that owns the windows - is
        # what displays.
        self._debug_frames = {}
        self._debug_lock = threading.Lock()
        self._open_windows = set()

    def set_debug(self, enabled):
        self.debug = enabled

    def set_surprise(self, settings):
        """
        Switches the surprise rule's detection on (a SurpriseDetection) or off
        (None).

        On, three things the plain solver throws away are let through: blobs
        wider than they are tall (a 5cm cube is square face-on and wider than
        tall corner-on), blobs smaller than MIN_CONTOUR_AREA_PX (a 5cm cube is
        under it past ~0.8m), and blobs that range past MAX_DISTANCE_CM as a
        10cm sign (a 5cm one reads twice as far). They are marked plain=False,
        and nothing downstream sees them unless SignSizer decides they are a
        short sign.

        One attribute, read once per frame by detect() on the vision thread,
        so it is safe to set from the control thread.
        """
        self.surprise = settings

    def detect(self, hsv_image, display_image=None):
        """
        Finds GREEN/RED boxes in hsv_image and solves their position.

        I/O:
            hsv_image: HSV frame to search for boxes in
            display_image: optional BGR frame to draw debug overlays on
                            (only used when self.debug is True)
            return: list of DetectedObject, one per box found
        """
        objects = []
        for color in TARGET_COLORS:
            mask = ImageColorUtils.calculate_color_mask(hsv_image, color)
            objects.extend(self._find_objects_in_mask(mask, color))

        if self.debug:
            self._render_debug(objects, display_image)

        return objects

    def detect_parking(self, hsv_image):
        """
        Finds the pink parking-bay walls, biggest first.

        Not part of detect(). A bay wall is a long low bar - wider than it is
        tall - which is exactly the shape _find_objects_in_mask throws away as
        noise, since the pillars it looks for stand upright. And the result
        must stay out of nav.observe_blocks: the lap dodges what it finds
        there, and it should not dodge the thing it is trying to park in.

        I/O:
            hsv_image: HSV frame to search
            return: list of ParkingMark, largest area first
        """
        mask = ImageColorUtils.calculate_color_mask(hsv_image, PARKING_COLOR)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        marks = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < MIN_PARKING_AREA_PX:
                continue
            rect = cv2.minAreaRect(contour)
            (center_x, center_y), (width, height), _ = rect
            marks.append(ParkingMark(
                bearing_deg=round(self._horizontal_pixel_to_angle(center_x), 1),
                area_px=float(area),
                width_px=float(max(width, height)),
                pixel_center=(int(center_x), int(center_y)),
                box_points=np.intp(cv2.boxPoints(rect))))
        marks.sort(key=lambda mark: -mark.area_px)
        self._last_parking = marks      # for the debug overlay
        return marks

    def _find_objects_in_mask(self, mask, color):
        surprise = self.surprise
        if surprise is not None:
            return self._find_signs_in_mask(mask, color, surprise)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        objects = []
        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_CONTOUR_AREA_PX:
                continue
            _, _, box_w, box_h = cv2.boundingRect(cnt)
            if box_w > box_h:
                # Blocks stand upright, so a wider-than-tall bounding box is noise.
                continue
            obj = self._solve_position(cv2.minAreaRect(cnt), color)
            if obj is not None:
                objects.append(obj)
        return objects

    def _find_signs_in_mask(self, mask, color, surprise):
        """
        _find_objects_in_mask with the surprise rule on: the same blobs, plus
        the ones a 5cm sign makes, each marked with whether the plain solver
        would have kept it. See set_surprise.
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        rows, cols = mask.shape[:2]
        objects = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < surprise.min_area_px:
                continue
            box_x, box_y, box_w, box_h = cv2.boundingRect(cnt)
            if box_w > box_h * surprise.max_squat_ratio:
                # Even a 5cm cube corner-on is only ~1.4x wider than tall.
                continue
            obj = self._solve_position(cv2.minAreaRect(cnt), color, upright=True,
                                       max_distance_cm=SURPRISE_MAX_DISTANCE_CM)
            if obj is None:
                continue
            obj.plain = (area >= MIN_CONTOUR_AREA_PX and box_w <= box_h
                         and obj.distance_cm <= MAX_DISTANCE_CM)
            obj.clipped = (box_x <= 0 or box_y <= 0
                           or box_x + box_w >= cols or box_y + box_h >= rows)
            objects.append(obj)
        return objects

    @staticmethod
    def _upright_sides(box_points):
        """
        (height, width) of a rotated rect: the side nearer vertical, then the
        other one. Not (long, short) - a 5cm cube seen corner-on is wider
        than it is tall, and its long side is its width.
        """
        first = box_points[1] - box_points[0]
        second = box_points[2] - box_points[1]
        first_len = float(np.hypot(*first))
        second_len = float(np.hypot(*second))
        first_upright = abs(first[1]) * second_len >= abs(second[1]) * first_len
        return (first_len, second_len) if first_upright else (second_len, first_len)

    def _solve_position(self, rect, color, upright=False,
                        max_distance_cm=MAX_DISTANCE_CM):
        box_points = cv2.boxPoints(rect)
        center_x, center_y = rect[0]

        # The boxes are twice as tall as they are wide, so the LONGEST side of
        # the rotated rect is the height. Measuring the rect's own side (rather
        # than the vertical extent of its axis-aligned bounding box) keeps the
        # reading correct when the box appears tilted in frame - a tilted
        # upright box has a taller bounding box than it does an actual height,
        # which would otherwise read as closer than it really is.
        # With the surprise rule on that stops being true of every sign (a 5cm
        # one is square), so the side nearer vertical is the height instead.
        if upright:
            height_px, width_px = self._upright_sides(box_points)
        else:
            height_px, width_px = max(rect[1]), min(rect[1])

        # A fisheye maps ANGLE FROM THE LENS AXIS linearly to pixels, but only
        # along the radius. Across it - which, for a pillar off to the side,
        # is its height - a degree spans theta/sin(theta) times as many
        # pixels, so a pillar near the edge of the frame looks too tall and
        # reads too close. Measured: one 80cm away 65 deg off the lens axis
        # was 26px tall and read 61cm; scaled back it reads 77cm.
        off_axis_rad = math.radians(self._horizontal_pixel_to_angle(center_x)
                                    - CAMERA_YAW_DEG)
        if abs(off_axis_rad) > 1e-6:
            height_px *= math.sin(off_axis_rad) / off_axis_rad
        angular_height_deg = height_px * DEG_PER_PX
        if angular_height_deg < MIN_ANGULAR_HEIGHT_DEG:
            return None

        # On the 222 deg fisheye the pillar's height in the frame gives its
        # STRAIGHT-LINE distance from the lens, whatever the bearing. Measured:
        # a pillar 100cm away at 41 deg off-axis was 16px tall, which solves
        # to 99.6cm. (The old 160 deg lens behaved the other way - height gave
        # distance AHEAD - and reading this lens that way put every pillar off
        # to the side ~1/cos(bearing) too far: that one at 133cm.)
        distance_cm = range_for_angular_height(angular_height_deg, BOX_HEIGHT_CM)
        if not (0 < distance_cm <= max_distance_cm):
            return None
        bearing_deg = self._horizontal_pixel_to_angle(center_x)
        forward_cm = distance_cm * math.cos(math.radians(bearing_deg))
        lateral_cm = distance_cm * math.sin(math.radians(bearing_deg))

        return DetectedObject(
            color=color,
            distance_cm=round(distance_cm, 1),
            bearing_deg=round(bearing_deg, 1),
            forward_cm=round(forward_cm, 1),
            lateral_cm=round(lateral_cm, 1),
            pixel_center=(int(center_x), int(center_y)),
            box_points=np.intp(box_points),
            height_px=round(height_px, 1),
            width_px=round(width_px, 1),
            off_axis_deg=round(math.degrees(off_axis_rad), 1),
        )

    def _horizontal_pixel_to_angle(self, x):
        full_frame_x = x + self.horizontal_crop_offset_px
        return (full_frame_x - CAPTURED_FRAME_WIDTH_PX / 2.0) * DEG_PER_PX + CAMERA_YAW_DEG

    # ========================================================================
    # DEBUG VISUALIZATION
    # ========================================================================

    def _render_debug(self, objects, display_image):
        """Draws both debug views and parks them for the next show_debug()."""
        frames = {"Object Solver - Top Down": self._render_top_down(objects)}
        if display_image is not None:
            frames["Object Solver - Camera"] = self._render_camera_overlay(
                objects, display_image)
        with self._debug_lock:
            self._debug_frames = frames

    def show_debug(self):
        """
        Puts the last detect()'s debug views on screen.

        MAIN THREAD ONLY, and it needs a cv2.waitKey() after it to paint -
        same contract as NavigationManager.show_debug(). The round's debug
        loop provides both (tasks/path_task.py).

        I/O:
            return: True if there was a new frame to show
        """
        with self._debug_lock:
            frames, self._debug_frames = self._debug_frames, {}
        for window_name, canvas in frames.items():
            cv2.imshow(window_name, canvas)
            self._open_windows.add(window_name)
        return bool(frames)

    def close_debug(self):
        """Closes whatever show_debug() opened. Main thread, same as show."""
        for window_name in self._open_windows:
            cv2.destroyWindow(window_name)
        self._open_windows.clear()

    def _render_camera_overlay(self, objects, display_image):
        overlay = display_image.copy()
        for obj in objects:
            bgr = COLOR_BGR[obj.color]
            cv2.drawContours(overlay, [obj.box_points], 0, bgr, 2)
            label = f"{obj.distance_cm}cm {obj.bearing_deg}deg"
            ImageDrawingUtils.draw_label(overlay, label,
                                         (obj.pixel_center[0] - 45, obj.pixel_center[1] - 10), bgr)
        # The bay walls, when the park has asked for them. Drawn from the last
        # detect_parking rather than found again here: the point of showing
        # them is to see what the PARK is actually being given, not what a
        # second pass on a different frame would have found.
        for mark in self._last_parking:
            cv2.drawContours(overlay, [mark.box_points], 0, PARKING_BGR, 2)
            ImageDrawingUtils.draw_label(
                overlay, f"bay {mark.bearing_deg:+.0f}deg {mark.area_px:.0f}px",
                (mark.pixel_center[0] - 45, mark.pixel_center[1] - 10), PARKING_BGR)
        return overlay

    def _render_top_down(self, objects):
        size = TOP_DOWN_CANVAS_PX
        scale = TOP_DOWN_CM_PER_PX
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        origin = (size // 2, size - 30)  # robot near the bottom, forward = up

        self._draw_fov_cone(canvas, origin)
        self._draw_range_rings(canvas, origin, scale)

        cv2.circle(canvas, origin, 6, (255, 255, 255), -1)
        ImageDrawingUtils.draw_label(canvas, "ROBOT", (origin[0] - 30, origin[1] + 22), (255, 255, 255))

        for obj in objects:
            px = int(origin[0] + obj.lateral_cm / scale)
            py = int(origin[1] - obj.forward_cm / scale)
            if 0 <= px < size and 0 <= py < size:
                bgr = COLOR_BGR[obj.color]
                cv2.circle(canvas, (px, py), 6, bgr, -1)
                ImageDrawingUtils.draw_label(canvas, f"{obj.distance_cm}cm", (px + 10, py + 5), bgr)

        return canvas

    def _draw_fov_cone(self, canvas, origin):
        size = canvas.shape[0]
        working_horizontal_fov_deg = self.image_width * DEG_PER_PX
        half_fov_rad = math.radians(working_horizontal_fov_deg / 2.0)
        for sign in (-1, 1):
            end = (
                int(origin[0] + sign * math.sin(half_fov_rad) * size),
                int(origin[1] - math.cos(half_fov_rad) * size),
            )
            cv2.line(canvas, origin, end, (60, 60, 60), 1)

    @staticmethod
    def _draw_range_rings(canvas, origin, scale):
        size = canvas.shape[0]
        max_cm = int(size * scale)
        for r_cm in range(TOP_DOWN_RING_STEP_CM, max_cm, TOP_DOWN_RING_STEP_CM):
            r_px = int(r_cm / scale)
            cv2.circle(canvas, origin, r_px, (60, 60, 60), 1)
            # label on the up-left diagonal, not straight up: the vertical center
            # axis is where objects at ~0 bearing plot, and the two collide there
            offset = int(r_px * 0.7071)
            x, y = origin[0] - offset + 4, origin[1] - offset - 4
            if x < 4 or y < 12:   # ring's label would run off the canvas edge
                continue
            ImageDrawingUtils.draw_label(canvas, f"{r_cm}cm", (x, y), (150, 150, 150), scale=0.45)
