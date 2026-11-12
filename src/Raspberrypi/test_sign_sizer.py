"""
Bench test for the surprise rule's size check (classes/sign_sizer.py).

Stand ONE red or green sign in front of the robot at a known distance, robot
stationary, and this prints what the camera and the lidar each make of it and
what the size check concludes - the same code the round runs, with the same
[surprise] numbers from tasks/final/config.toml.

    python test_sign_sizer.py --place 40 --height 10
    python test_sign_sizer.py --place 70 --height 15
    python test_sign_sizer.py --place 100 --height 5
                                    a sign of that height (cm) with its near
                                    face that many cm in front of the LENS.
                                    Do each height at 40, 70 and 100.
    python test_sign_sizer.py --place 70 --height 5 --facing 45
                                    ... seen corner-on (degrees the sightline
                                    is off square to the sign's face)
    python test_sign_sizer.py --place 70 --height 15 --side -20
                                    ... 20cm to the LEFT of the lens' straight
                                    ahead (+ = right)
    python test_sign_sizer.py --place 70 --height 15 --wall 150
                                    there is a wall 150cm behind the lens'
                                    line of sight (default: nothing behind)
    python test_sign_sizer.py --place 70 --height 15 --lidar-sees-short
                                    judge as if surprise.lidar_sees_short
                                    were true, whatever the config says
    python test_sign_sizer.py --calibrate --height 10
                                    FIX THE CAMERA'S BEARING against the lidar:
                                    move ONE sign slowly around in front of the
                                    robot (left, middle, right, 40-100cm) for
                                    --seconds; prints the CAMERA_YAW_DEG and
                                    CAMERA_BEARING_SCALE that fit
    python test_sign_sizer.py --synthetic --place 70 --height 15
                                    no hardware: a made-up blob and scan, to
                                    check the script and the arithmetic

What to read off it, and where each number goes in [surprise]:
    ratio              -> short_max_ratio / standard_*_ratio / tall_min_ratio,
                          and blob_pad_px (a 10cm sign should read 2.0 at
                          every distance; if it sags toward 1 with distance,
                          the mask is padding the blob)
    camera vs lidar    -> lidar_tolerance_mm / lidar_tolerance_fraction
    "lidar at the 5cm range" on a 5cm sign
                       -> lidar_sees_short
    blob area          -> min_area_px
"""
import argparse
import math
import time
from collections import Counter, namedtuple
from pathlib import Path

import numpy as np

from classes.block_map import FACE_TO_CENTER_MM
from classes.object_solver import (BOX_HEIGHT_CM, DEG_PER_PX, DetectedObject,
                                   ObjectSolver, SurpriseDetection)
from classes.robot_geometry import camera_offset_behind_lidar
from classes.sign_sizer import (HEIGHTS_MM, LOOK_AROUND_DEG, STANDARD_MM,
                                SignSizer, SizerSettings)
from tasks.path_task import DEFAULTS
from utils.enums import Color
from utils.task_config import TaskConfig

DEFAULT_CONFIG = Path(__file__).parent / "tasks" / "final" / "config.toml"

# A pose is only needed for two things here: the corner-on correction, which
# wants the angle between the sightline and the sign's face, and the "is that
# return just the wall" test. On the bench both come from the command line.
BenchPose = namedtuple("BenchPose", "x y heading")


class BenchMap:
    """Stands in for FieldMap: one wall, a fixed distance down every ray."""

    def __init__(self, wall_mm):
        self.wall_mm = float(wall_mm)

    def raycast(self, x, y, angles_rad, max_range_mm=None, include_obstacles=True):
        return np.full(np.shape(angles_rad), self.wall_mm)


class SyntheticLidar:
    """A scan with a sign in it (or not), for --synthetic."""

    def __init__(self, sign_mm, wall_mm, visible):
        self.sign_mm, self.wall_mm, self.visible = sign_mm, wall_mm, visible

    def get_scan_with_age(self, max_age_s=None):
        scan = np.full(360, min(self.wall_mm, 18000.0))
        if self.visible:
            scan[[359, 0, 1]] = self.sign_mm
        return scan, np.zeros(360)


def load_settings(path, lidar_sees_short=None):
    """The [surprise] numbers the round would use, as (setting, SizerSettings)."""
    config = TaskConfig.load(path)
    if lidar_sees_short:
        config.set("surprise.lidar_sees_short", True)

    def setting(key):
        return config.get(key, DEFAULTS.get(key))

    return setting, SizerSettings.from_settings(setting)


def synthetic_detection(place_cm, height_cm, facing_deg):
    """The blob a sign of that height at that distance should make."""
    face_mm = place_cm * 10.0
    height_px = math.degrees(2.0 * math.atan(height_cm * 5.0 / face_mm)) / DEG_PER_PX
    across = 50.0 * (abs(math.cos(math.radians(facing_deg)))
                     + abs(math.sin(math.radians(facing_deg))))
    width_px = (math.degrees(2.0 * math.atan(across / 2.0 / (face_mm + FACE_TO_CENTER_MM)))
                / DEG_PER_PX)
    as_standard_cm = place_cm * BOX_HEIGHT_CM / height_cm
    return DetectedObject(color=Color.RED, distance_cm=as_standard_cm, bearing_deg=0.0,
                          forward_cm=as_standard_cm, lateral_cm=0.0, pixel_center=(0, 0),
                          box_points=np.zeros((4, 2)), height_px=height_px,
                          width_px=width_px, plain=height_cm >= BOX_HEIGHT_CM)


def name(height_mm):
    return "-" if height_mm is None else f"{height_mm / 10:.0f}cm"


def report(verdicts, detections, sizer, args):
    """Prints the cues and the verdict, over every frame the sign was seen in."""
    def med(values):
        return float(np.median(values))

    settings = sizer.settings
    first = detections[0]
    height_px = med([d.height_px for d in detections])
    width_px = med([d.width_px for d in detections])
    print(f"\nSign seen in {len(detections)}/{args.frames} frames "
          f"({first.color.value}, bearing {med([d.bearing_deg for d in detections]):+.1f}deg, "
          f"{med([d.off_axis_deg for d in detections]):+.1f}deg off the lens axis)")
    clipped = sum(1 for d in detections if d.clipped)
    if clipped:
        print(f"  CUT OFF by the frame edge in {clipped} frames - the shape is "
              f"ignored in those")

    print("\nCAMERA")
    print(f"  blob             {height_px:.1f}px tall x {width_px:.1f}px wide "
          f"(area about {height_px * width_px:.0f}px; surprise.min_area_px = "
          f"{sizer_min_area(args):.0f})")
    ratios = [v.ratio for v in verdicts if v.ratio is not None]
    if ratios:
        print(f"  ratio            {med(ratios):.2f}  (spread {min(ratios):.2f}-"
              f"{max(ratios):.2f}; width corrected for --facing {args.facing:g}deg, "
              f"blob_pad_px {settings.blob_pad_px:g})")
        print(f"                   bands: <= {settings.short_max_ratio:g} is 5cm, "
              f"{settings.standard_min_ratio:g}-{settings.standard_max_ratio:g} is 10cm, "
              f">= {settings.tall_min_ratio:g} is 15cm")
    else:
        print(f"  ratio            not measured - sign more than "
              f"{settings.shape_max_off_axis_deg:g}deg off the lens axis, or cut off")
    print("  range if it is   " + "   ".join(
        f"{name(h)}: {med([d.candidates_cm[h] for d in detections]):.0f}cm"
        for h in HEIGHTS_MM))
    if args.place:
        best = min(HEIGHTS_MM, key=lambda h: abs(
            med([d.candidates_cm[h] for d in detections]) - args.place))
        print(f"                   true distance {args.place:g}cm matches the "
              f"{name(best)} range")
    print(f"  shape says       {tally([v.shape_cue for v in verdicts])}")

    print("\nLIDAR")
    with_lidar = [v for v in verdicts if v.lidar_ranges]
    if not with_lidar:
        print("  no scan (run without --no-lidar, on the robot)")
    else:
        where_is_it(with_lidar, sizer, args)
        for height in HEIGHTS_MM:
            bearing = med([v.lidar_ranges[height][0] for v in with_lidar])
            expected = med([v.lidar_ranges[height][1] for v in with_lidar])
            seen = np.concatenate([v.lidar_seen[height] for v in with_lidar])
            tolerance = (settings.lidar_tolerance_mm
                         + settings.lidar_tolerance_fraction * expected)
            if len(seen) == 0:
                found = "no readings at all"
            else:
                nearest = float(seen[np.argmin(np.abs(seen - expected))])
                hit = abs(nearest - expected) <= tolerance
                found = (f"nearest reading {nearest / 10:.0f}cm "
                         f"({(nearest - expected) / 10:+.0f}cm, tolerance "
                         f"{tolerance / 10:.0f}cm) -> {'SOMETHING THERE' if hit else 'nothing there'}")
            print(f"  at the {name(height):>4} range  expect {expected / 10:.0f}cm at "
                  f"{bearing:+.0f}deg from the lidar: {found}")
        print(f"  lidar says       {tally([v.lidar_cue for v in verdicts])}")
        print(f"  (judged with surprise.lidar_sees_short = "
              f"{str(settings.lidar_sees_short).lower()})")

    print("\nVERDICT")
    counts = Counter(v.height_mm for v in verdicts)
    print(f"  per frame        {tally([v.height_mm for v in verdicts])}")
    votes = Counter(v.vote for v in verdicts if v.vote is not None)
    print(f"  votes            {' '.join(f'{name(h)}:{votes[h]}' for h in HEIGHTS_MM)}"
          f"   (no opinion: {sum(1 for v in verdicts if v.vote is None)})")
    winner = counts.most_common(1)[0][0]
    if args.place and args.place * 10.0 > settings.shape_max_range_mm:
        print(f"  NOTE: {args.place:g}cm is past surprise.shape_max_range_mm "
              f"({settings.shape_max_range_mm / 10:.0f}cm), where the shape is "
              f"not counted - so 10cm is the verdict to expect here whatever "
              f"the sign is. What this distance is for is the ratio and the "
              f"lidar lines above.")
    if args.height:
        wanted = int(round(args.height * 10))
        right = counts[wanted]
        wrong_odd = sum(count for height, count in counts.items()
                        if height != wanted and height != STANDARD_MM)
        print(f"  expected {name(wanted)}: right in {right}/{len(verdicts)} frames"
              + (f", WRONG ODD HEIGHT in {wrong_odd}" if wrong_odd else ""))
        if wanted == STANDARD_MM and right != len(verdicts):
            print("  >>> A NORMAL SIGN READ AS MODIFIED. In a round this is the "
                  "start of a false flip. Do not enable the rule until this "
                  "reads 10cm in every frame.")
        elif wanted != STANDARD_MM and winner != wanted:
            print("  >>> the modified sign reads as normal here - it would be "
                  "passed by its colour (a missed flip).")
    return winner


def where_is_it(verdicts, sizer, args):
    """
    The camera, the lidar and the tape measure on where the sign is, side by
    side, from the lidar's point of view. Whichever two agree, the third is
    the one with a wrong number behind it - see the hints it prints.
    """
    def med(values):
        return float(np.median(values))

    height = int(round(args.height * 10)) if args.height else STANDARD_MM
    camera_b = med([v.lidar_ranges[height][0] for v in verdicts])
    camera_r = med([v.lidar_ranges[height][1] for v in verdicts])
    seen = [v.lidar_nearest for v in verdicts if v.lidar_nearest]
    lens_ahead = sizer.camera_offset_mm[0] - sizer.lidar_offset_mm[0]
    print(f"  WHERE IS THE SIGN, seen from the lidar (bearing + = right):")
    print(f"    camera says ({name(height)} sign)  {camera_b:+6.1f}deg  {camera_r / 10:6.1f}cm")
    if seen:
        lidar_b = med([s[0] for s in seen])
        lidar_r = med([s[1] for s in seen])
        print(f"    lidar's nearest return      {lidar_b:+6.1f}deg  {lidar_r / 10:6.1f}cm"
              f"   (closest thing within {LOOK_AROUND_DEG}deg of the camera's bearing)")
    else:
        lidar_b = lidar_r = None
        print(f"    lidar's nearest return      nothing within {LOOK_AROUND_DEG}deg")
    if args.place:
        forward = lens_ahead + args.place * 10.0
        right = args.side * 10.0
        true_b = math.degrees(math.atan2(right, forward))
        true_r = math.hypot(forward, right)
        print(f"    where you put it            {true_b:+6.1f}deg  {true_r / 10:6.1f}cm"
              f"   (if the lens really is {lens_ahead:.0f}mm ahead of the lidar)")
        if lidar_b is not None:
            off_b, off_r = lidar_b - true_b, (lidar_r - true_r) / 10.0
            if abs(off_b) > 3 or abs(off_r) > 4:
                print(f"    >>> the LIDAR disagrees with the tape by {off_b:+.0f}deg / "
                      f"{off_r:+.0f}cm. Check CAMERA_BEHIND_LIDAR_MM and "
                      f"robot.lidar_ahead_mm (a range error that grows when the "
                      f"sign is off to the side), or the lidar's angle_offset_deg "
                      f"(a bearing error that is the same everywhere). Is anything "
                      f"else nearer than the sign in that direction?")
            else:
                print(f"    lidar and tape agree ({off_b:+.0f}deg, {off_r:+.0f}cm)")
        off_b, off_r = camera_b - true_b, (camera_r - true_r) / 10.0
        if abs(off_b) > 3 or abs(off_r) > 4:
            print(f"    >>> the CAMERA disagrees with the tape by {off_b:+.0f}deg / "
                  f"{off_r:+.0f}cm. A bearing error is CAMERA_YAW_DEG (same "
                  f"everywhere) or CAPTURED_HORIZONTAL_FOV_DEG (grows toward the "
                  f"edge); a range error is CAPTURED_HORIZONTAL_FOV_DEG - re-measure "
                  f"with test_object_solver.py --place.")
        else:
            print(f"    camera and tape agree ({off_b:+.0f}deg, {off_r:+.0f}cm)")
    print(f"    the lidar is searched {sizer.settings.lidar_window_deg}deg either "
          f"side of the camera's bearing (surprise.lidar_window_deg)")
    print()


def tally(values):
    counts = Counter(values)
    return "  ".join(f"{name(value)} x{count}" for value, count in counts.most_common())


def sizer_min_area(args):
    return float(load_settings(args.config)[0]("surprise.min_area_px"))


# --calibrate: how far either side of where the camera puts the sign the
# lidar is searched for it, and how close in range the lidar return has to be
# to the camera's for the two to be the same sign. Wide in bearing, because a
# wrong bearing is what is being measured; tight in range, because the ranges
# already agree and that is what tells the sign from the wall behind it.
CALIBRATE_SEARCH_DEG = 25
CALIBRATE_RANGE_TOLERANCE = 0.15       # of the range
CALIBRATE_MIN_SAMPLES = 30
CALIBRATE_MIN_SPAN_DEG = 40.0


def run_calibrate(args):
    """
    Fits the camera's bearing to the lidar's.

    The lidar measures bearing directly, to a degree, and its ranges agree
    with the camera's - so for one sign at a time, the lidar return at the
    camera's range near the camera's bearing IS that sign, and the angle it
    is at, seen from the lens, is what the camera should have said. Every
    frame gives one (pixel angle, true angle) pair; a straight line through
    them is CAMERA_BEARING_SCALE and CAMERA_YAW_DEG.
    """
    from camera_manager import CameraManager
    from classes.lidar_manager import LidarManager
    from classes.object_solver import CAMERA_BEARING_SCALE, CAMERA_YAW_DEG

    setting, settings = load_settings(args.config)
    lidar_offset = (float(setting("robot.lidar_ahead_mm")), 0.0)
    camera_offset = camera_offset_behind_lidar(lidar_offset)
    height = int(round((args.height or 10.0) * 10))

    camera = CameraManager()
    camera.start_camera()
    time.sleep(1.0)
    lidar = LidarManager(port=args.lidar_port)
    lidar.start()
    if not lidar.wait_until_ready(timeout=5.0):
        print("WARNING: the lidar returned little data - is it spinning?")
    solver = ObjectSolver(debug=False)
    solver.set_surprise(SurpriseDetection(
        min_area_px=float(setting("surprise.min_area_px")),
        max_squat_ratio=float(setting("surprise.max_squat_ratio"))))
    sizer = SignSizer(lidar, BenchMap(1.0e6), lidar_offset, camera_offset, settings)

    print(f"Calibrating with ONE {height / 10:.0f}cm sign. Move it slowly around in "
          f"front of the robot - far left to far right, 40 to 100cm - for "
          f"{args.seconds:.0f}s. Nothing else red or green in view, and nothing "
          f"else within a hand's width of it.")
    pairs = []          # (pixel angle off the lens axis, true bearing from the lens)
    deadline = time.monotonic() + args.seconds
    last_note = 0.0
    try:
        while time.monotonic() < deadline:
            camera.capture_image()
            camera.transform_image()
            found = solver.detect(camera.hsv_image)
            if not found:
                continue
            detection = max(found, key=lambda d: d.height_px)
            if detection.clipped:
                continue
            want_bearing, want_range = sizer._from_lidar(detection, height)
            scan = sizer._scan()
            if scan is None:
                continue
            seen = sizer.nearest_around(scan, want_bearing, CALIBRATE_SEARCH_DEG)
            if seen is None or abs(seen[1] - want_range) > CALIBRATE_RANGE_TOLERANCE * want_range:
                continue
            # The lidar's return, as a point, then as a bearing from the lens.
            centre = seen[1] + FACE_TO_CENTER_MM
            forward = (lidar_offset[0] - camera_offset[0]
                       + centre * math.cos(math.radians(seen[0])))
            right = (lidar_offset[1] - camera_offset[1]
                     + centre * math.sin(math.radians(seen[0])))
            true_bearing = math.degrees(math.atan2(right, forward))
            pixel_angle = detection.off_axis_deg / CAMERA_BEARING_SCALE
            pairs.append((pixel_angle, true_bearing))
            if time.monotonic() - last_note > 1.0:
                last_note = time.monotonic()
                print(f"  {len(pairs):4d} samples   now: camera says "
                      f"{detection.bearing_deg:+6.1f}deg, lidar says "
                      f"{true_bearing:+6.1f}deg  ({seen[1] / 10:.0f}cm from the lidar)")
    finally:
        lidar.stop()

    if len(pairs) < CALIBRATE_MIN_SAMPLES:
        print(f"Only {len(pairs)} samples - need {CALIBRATE_MIN_SAMPLES}. Is the sign "
              f"in view, and --height right? (The camera's range for that height "
              f"has to agree with the lidar's to within "
              f"{CALIBRATE_RANGE_TOLERANCE:.0%}.)")
        return 1
    pixel = np.array([p for p, _ in pairs])
    truth = np.array([t for _, t in pairs])
    span = float(pixel.max() - pixel.min())
    scale, yaw = np.polyfit(pixel, truth, 1)
    before = pixel * CAMERA_BEARING_SCALE + CAMERA_YAW_DEG - truth
    after = pixel * scale + yaw - truth

    def rms(values):
        return float(np.sqrt(np.mean(np.square(values))))

    print(f"\n{len(pairs)} samples over {span:.0f}deg of the frame "
          f"({pixel.min():+.0f} to {pixel.max():+.0f}deg off the lens axis)")
    print(f"  bearing error now        rms {rms(before):4.1f}deg, worst "
          f"{np.max(np.abs(before)):4.1f}deg")
    print(f"  with the fitted numbers  rms {rms(after):4.1f}deg, worst "
          f"{np.max(np.abs(after)):4.1f}deg")
    print("\n  error by where it is in the frame (now -> fitted):")
    for low in range(-90, 90, 15):
        inside = (pixel >= low) & (pixel < low + 15)
        if np.count_nonzero(inside) >= 3:
            print(f"    {low:+4d} to {low + 15:+4d}deg  {np.count_nonzero(inside):4d} samples  "
                  f"{np.mean(before[inside]):+5.1f} -> {np.mean(after[inside]):+5.1f}deg")
    if span < CALIBRATE_MIN_SPAN_DEG:
        print(f"\n  ONLY {span:.0f}deg OF THE FRAME COVERED - the scale is not "
              f"trustworthy from that. Move the sign further left and right and "
              f"run it again; the yaw alone would be "
              f"{CAMERA_YAW_DEG + float(np.mean(-before)):+.1f}.")
        return 1
    print(f"\n  Put these in classes/object_solver.py:")
    print(f"    CAMERA_YAW_DEG = {yaw:.1f}          (now {CAMERA_YAW_DEG})")
    print(f"    CAMERA_BEARING_SCALE = {scale:.3f}   (now {CAMERA_BEARING_SCALE})")
    if rms(after) > 3.0:
        print("  The fitted line still leaves over 3deg rms: the error is not a "
              "straight line across the frame (a lens the linear model does not "
              "fit), or other things were being picked up. Look at the table above.")
    print("  Then set surprise.lidar_window_deg back down to about 4.")
    return 0


def run(args):
    setting, settings = load_settings(args.config, args.lidar_sees_short)
    lidar_offset = (float(setting("robot.lidar_ahead_mm")), 0.0)
    camera_offset = camera_offset_behind_lidar(lidar_offset)
    wall_mm = args.wall * 10.0 if args.wall else 1.0e6
    pose = BenchPose(0.0, 0.0, args.facing)
    print(f"[surprise] from {args.config}; lens "
          f"{camera_offset[0] - lidar_offset[0]:.0f}mm ahead of the lidar")

    camera = lidar = None
    if args.synthetic:
        if not (args.place and args.height):
            raise SystemExit("--synthetic needs --place CM and --height CM")
        sees = args.height > 5 or settings.lidar_sees_short
        lidar = None if args.no_lidar else SyntheticLidar(
            args.place * 10.0 + camera_offset[0] - lidar_offset[0], wall_mm, sees)
    else:
        from camera_manager import CameraManager
        camera = CameraManager()
        camera.start_camera()
        time.sleep(1.0)                    # let exposure settle
        if not args.no_lidar:
            from classes.lidar_manager import LidarManager
            lidar = LidarManager(port=args.lidar_port)
            lidar.start()
            if not lidar.wait_until_ready(timeout=5.0):
                print("WARNING: the lidar returned little data - is it spinning?")

    solver = ObjectSolver(debug=False)
    solver.set_surprise(SurpriseDetection(
        min_area_px=float(setting("surprise.min_area_px")),
        max_squat_ratio=float(setting("surprise.max_squat_ratio"))))
    sizer = SignSizer(lidar, BenchMap(wall_mm), lidar_offset, camera_offset, settings)

    verdicts, kept = [], []
    try:
        for _ in range(args.frames):
            if args.synthetic:
                found = [synthetic_detection(args.place, args.height, args.facing)]
            else:
                camera.capture_image()
                camera.transform_image()
                found = solver.detect(camera.hsv_image)
            if not found:
                continue
            # The sign nearest where it was put: the bench has one sign in it.
            aim = math.degrees(math.atan2(args.side, args.place or 1.0))
            detection = min(found, key=lambda d: abs(d.bearing_deg - aim))
            scan = sizer._scan()
            verdicts.append(sizer.judge(pose, detection, scan))
            detection.candidates_cm = {h: sizer._face_mm(detection, h) / 10.0
                                       for h in HEIGHTS_MM}
            kept.append(detection)
            if not args.synthetic:
                time.sleep(0.05)
    finally:
        if lidar is not None and hasattr(lidar, "stop"):
            lidar.stop()

    if not verdicts:
        print("No red or green blob in any frame. Is the sign in view, lit, and "
              "inside the colour ranges (make_color_spec.py)?")
        return 1
    winner = report(verdicts, kept, sizer, args)
    in_range = not args.place or args.place * 10.0 <= settings.shape_max_range_mm
    wanted = int(round(args.height * 10)) if args.height and in_range else STANDARD_MM
    if (args.height or not in_range) and winner != wanted:
        return 1
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Bench test of the surprise rule's size check")
    parser.add_argument("--place", type=float, metavar="CM",
                        help="true distance from the lens to the sign's near face")
    parser.add_argument("--height", type=float, choices=(5.0, 10.0, 15.0), metavar="CM",
                        help="the sign's true height: 5, 10 or 15")
    parser.add_argument("--side", type=float, default=0.0, metavar="CM",
                        help="how far the sign is to the side of the lens' "
                             "straight ahead, + right / - left (default 0)")
    parser.add_argument("--facing", type=float, default=0.0, metavar="DEG",
                        help="angle between the sightline and square-on to the "
                             "sign's face (0 = face-on, 45 = corner-on)")
    parser.add_argument("--wall", type=float, default=None, metavar="CM",
                        help="distance from the lidar to whatever is behind the "
                             "sign (default: nothing near)")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--no-lidar", action="store_true",
                        help="camera only - shows the shape cue, never a flip")
    parser.add_argument("--lidar-port", default=None)
    parser.add_argument("--lidar-sees-short", action="store_true",
                        help="judge with surprise.lidar_sees_short = true")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG),
                        help="config.toml to read [surprise] from")
    parser.add_argument("--calibrate", action="store_true",
                        help="fit CAMERA_YAW_DEG and CAMERA_BEARING_SCALE to the "
                             "lidar while one sign is moved around")
    parser.add_argument("--seconds", type=float, default=60.0,
                        help="how long --calibrate collects for")
    parser.add_argument("--synthetic", action="store_true",
                        help="no hardware: a computed blob and scan")
    arguments = parser.parse_args()
    raise SystemExit(run_calibrate(arguments) if arguments.calibrate
                     else run(arguments))
