"""
Measure the weave directly: step the line sideways and watch how the robot
gets back onto it.

A robot that "wanders down the line and never settles in the middle" is an
under-damped control loop, and the only honest way to see one is a STEP TEST -
the standard tool for it. Everything else (driving a lap and eyeballing it,
staring at the off-line number) mixes the loop's own behaviour up with the
corners, the pillars and where the line happens to go.

So: drive straight along a line that exists only in this script, and part way
down it move that line `--step` mm sideways in one tick. Nothing about the
robot changes - same pose, same speed, same everything - so what happens next
is the loop's step response and nothing else. Out of it come the four numbers
that say what is wrong:

    overshoot     how far past the line it goes on the first swing, as a % of
                  the step. Under ~15% is a well-damped loop; 40% and up is
                  the weave you can see from across the room.
    swings        how many times the error crosses zero before settling. 0-1
                  is right. 4+ is a limit cycle - it never settles at all.
    wavelength    how far the robot drives per full oscillation. Compare it
                  with the lookahead: a wavelength of a few lookaheads is the
                  geometry oscillating; a much longer one is the POSE
                  oscillating, and no lookahead value will fix that.
    residual      how far off the line it is still sitting at the end. This is
                  the "it never comes to the centre" number. Pure pursuit has
                  no steady-state error by construction, so anything here is a
                  bias - a mis-scaled max_road_wheel_deg, a servo whose centre
                  is not straight ahead, or a pose the filter is holding off
                  to one side.

WHAT TO TURN, given the answer:

    high overshoot + short wavelength   lookahead too short for the speed:
                                        raise pursuit.lookahead_base_mm
    high overshoot + long wavelength    the loop is chasing lag: get
                                        pursuit.servo_lag_s right first (from
                                        `test_steering.py --lag`), then raise
                                        pursuit.lag_compensation_s toward it
    slow, no overshoot                  over-damped: lookahead too long, or
                                        lag_compensation over-predicting
    non-zero residual                   not a damping problem at all - go and
                                        measure max_road_wheel_deg
                                        (`test_steering.py --calibrate`) and
                                        check the servo's centre

    python test_step_response.py                       one 200mm step
    python test_step_response.py --step 300 --speed 50
    python test_step_response.py --sweep pursuit.lookahead_base_mm 150,250,400
    python test_step_response.py --repeat 3            three, averaged

The step is taken in the settings of a round's config.toml (--round, default
final), so what it measures is what the round will do. Any setting can be
overridden for the run with --set, which is what makes the sweep worth having.

CLEAR SPACE NEEDED - a straight run of --pre plus --after (2m by default) with
--step of side room, so about 2.5m x 1m of clear mat. It aborts on the lidar
if something turns up in front. START BUTTON, as ever: nothing moves until it
is pressed.
"""
import argparse
import math
import sys
import time

import numpy as np

from classes.pure_pursuit import PurePursuit
from classes.robot_geometry import to_field
from classes.task_context import TaskContext
from utils.angle_utils import angle_difference
from utils.task_config import TaskConfig

SAMPLE_S = 0.02          # 50Hz, the rate the real control loop runs at
ABORT_SECTOR_DEG = 25.0
ABORT_CLEARANCE_MM = 250.0
# The room check before the run looks down the line, not across the whole
# abort cone: from a corner cell the centre block's corner sits at ~25deg and
# 1.1m, inside +/-25 but never on the path, and it refused every run there.
ROOM_SECTOR_DEG = 5.0
# Starting the step off a pose that only JUST crossed the reliable line is
# starting it off a guess. Confidence alone cannot tell a right pose from a
# wrong one - measured, a correct lock in a corner wobbles between 0.71 and
# 0.81, and a global search that settled 2m off reported 0.70 - so the start
# also checks the scan against the map FROM that pose: a correct lock fits to
# ~20mm, a wrong one does not fit at all. A pose has to pass both, and keep
# passing for START_HOLD_S.
START_CONFIDENCE = 0.5
START_FIT_MM = 60.0
START_HOLD_S = 1.0
START_TIMEOUT_S = 30.0
# A global search that has settled somewhere wrong stays there, so after this
# long without a good fit the particles are scattered again.
RESEARCH_AFTER_S = 6.0
FIT_BEAMS_DEG = np.arange(-120, 121, 5)
FIT_MAX_RANGE_MM = 2900.0
# And one bad update mid-run is not a lost pose. The round itself rides out
# safety.lost_timeout_s of it; this only gives up once it has lasted this long.
UNRELIABLE_GRACE_S = 0.5
SETTLE_FRACTION = 0.1    # "settled" = inside this fraction of the step


# ============================================================================
# THE LINE - a straight one, in the field frame, that this script invents
# ============================================================================

class StepLine:
    """
    A straight reference line through `origin` on heading `heading_deg`, which
    jumps `step_mm` sideways once the robot has driven `after_mm` along it.

    Deliberately not a RacingLine: a step has to be a STEP, and every path in
    the round proper is smooth by construction. The whole measurement is that
    discontinuity - the robot's own smoothing is what is being measured, so
    nothing here may do any of it.
    """

    def __init__(self, origin, heading_deg, step_mm, after_mm):
        self.x, self.y = float(origin[0]), float(origin[1])
        self.heading = float(heading_deg)
        self.step_mm = float(step_mm)
        self.after_mm = float(after_mm)
        radians = math.radians(self.heading)
        # Heading is degrees clockwise from +Y, so forward is (sin, cos) and
        # right is (cos, -sin) - the same convention as everything else here.
        self.forward = (math.sin(radians), math.cos(radians))
        self.right = (math.cos(radians), -math.sin(radians))

    def project(self, x, y):
        """(distance along the line, distance to the RIGHT of it) for a point."""
        dx, dy = x - self.x, y - self.y
        return (dx * self.forward[0] + dy * self.forward[1],
                dx * self.right[0] + dy * self.right[1])

    def offset_at(self, along_mm):
        """Where the line sits, at that distance along. The step itself."""
        return self.step_mm if along_mm >= self.after_mm else 0.0

    def point_at(self, along_mm):
        """A point on the line, in field mm - what pure pursuit chases."""
        lateral = self.offset_at(along_mm)
        return (self.x + along_mm * self.forward[0] + lateral * self.right[0],
                self.y + along_mm * self.forward[1] + lateral * self.right[1])

    def error_mm(self, x, y):
        """How far RIGHT of where the line now is the robot sits."""
        along, lateral = self.project(x, y)
        return lateral - self.offset_at(along), along


# ============================================================================
# DRIVING IT
# ============================================================================

def clearance_mm(context, sector_deg=ABORT_SECTOR_DEG):
    """Closest lidar return within +/- sector_deg of the nose, or None if the
    lidar is not answering."""
    if context.lidar is None:
        return None
    distance, _ = context.lidar.get_min_distance(-sector_deg, sector_deg)
    return None if math.isnan(distance) else distance


def build_pursuit(config):
    """The round's own controller, with the round's own numbers."""
    return PurePursuit(
        wheelbase_mm=config.get("pursuit.wheelbase_mm", 165.0),
        lookahead_base_mm=config.get("pursuit.lookahead_base_mm", 260.0),
        lookahead_per_speed_mm=config.get("pursuit.lookahead_per_speed_mm", 3.0),
        lookahead_min_mm=config.get("pursuit.lookahead_min_mm", 250.0),
        lookahead_max_mm=config.get("pursuit.lookahead_max_mm", 700.0),
        max_road_wheel_deg=config.get("pursuit.max_road_wheel_deg", 70.0),
        max_steer_command=config.get("pursuit.max_steer_command", 70.0),
        rear_axle_offset_mm=config.get("pursuit.rear_axle_offset_mm", 0.0),
        corner_lookahead_fraction=config.get("pursuit.corner_lookahead_fraction", 0.8),
        servo_lag_s=config.get("pursuit.servo_lag_s", 0.0))


def drive_step(context, config, speed, step_mm, pre_mm, after_mm):
    """
    One step test. Returns (samples, aborted), samples being
    [(along_mm, error_mm, steer_command, heading_error_deg), ...].

    The control loop here is the round's, cut down to the parts under test:
    project, aim a lookahead further on, steer at it, feed the odometry. The
    lateral bounds, the speed profile, the pillars and the safety stops are all
    absent on purpose - they are not what oscillates.
    """
    pursuit = build_pursuit(config)
    lead_s = float(config.get("pursuit.lag_compensation_s", 0.0) or 0.0)
    mm_per_s = float(config.get("startup.mm_per_s_at_full", 390.0))

    pose = context.nav.get_pose()
    line = StepLine((pose.x, pose.y), pose.heading, step_mm, pre_mm)

    samples = []
    context.motor.drive(0, speed)
    last = time.monotonic()
    aborted = None
    unreliable_since = None
    while True:
        now = time.monotonic()
        dt = now - last
        last = now
        if dt < SAMPLE_S:
            time.sleep(SAMPLE_S - dt)

        distance = speed / 100.0 * mm_per_s * dt
        held = pursuit.advance_servo(dt)
        turn = _yaw(pursuit, distance, held)
        context.nav.report_motion(distance, turn)
        pose = context.nav.get_pose()

        error, along = line.error_mm(pose.x, pose.y)
        samples.append((along, error, pursuit.last_command,
                        angle_difference(pose.heading, line.heading)))

        if along >= pre_mm + after_mm:
            break
        front = clearance_mm(context)
        if front is not None and front < ABORT_CLEARANCE_MM:
            aborted = f"something {front:.0f}mm ahead"
            break
        if abs(error) > max(500.0, 4.0 * abs(step_mm)):
            aborted = f"{error:+.0f}mm off the line - lost it"
            break
        if pose.is_reliable:
            unreliable_since = None
        elif unreliable_since is None:
            unreliable_since = now
        elif now - unreliable_since > UNRELIABLE_GRACE_S:
            aborted = (f"pose unreliable for over {UNRELIABLE_GRACE_S:.1f}s "
                       f"(conf={pose.confidence:.2f})")
            break

        # Steer off where the robot WILL be when the wheels arrive, exactly as
        # PathDrivingTask does - this is a measurement of that loop, so it has
        # to be the same loop.
        lookahead = pursuit.lookahead_distance(speed)
        target = line.point_at(along + lookahead)
        command = pursuit.steering(_lead(pursuit, pose, lead_s, speed, mm_per_s), target)
        context.motor.drive(command, speed)

    context.motor.drive(0, 0)
    return samples, aborted


def _yaw(pursuit, distance_mm, road_wheel_deg):
    """Bicycle-model yaw over that distance at that wheel angle."""
    if not distance_mm:
        return 0.0
    return math.degrees(distance_mm / pursuit.wheelbase_mm
                        * math.tan(math.radians(road_wheel_deg)))


def _lead(pursuit, pose, lead_s, speed, mm_per_s):
    """The pose projected forward by the actuator lag. See _lead_pose."""
    if lead_s <= 0.0 or not speed:
        return pose
    from dataclasses import replace
    distance = speed / 100.0 * mm_per_s * lead_s
    turn = _yaw(pursuit, distance, pursuit.mean_road_wheel_deg(lead_s))
    midpoint = math.radians(pose.heading + turn / 2.0)
    return replace(pose,
                   x=pose.x + distance * math.sin(midpoint),
                   y=pose.y + distance * math.cos(midpoint),
                   heading=(pose.heading + turn) % 360.0)


# ============================================================================
# READING THE ANSWER OUT
# ============================================================================

def analyse(samples, step_mm, pre_mm):
    """
    The four numbers, from the part of the track after the step.

    Sign convention: `error` is how far RIGHT of the line the robot is, and the
    step moves the line right by `step_mm`, so the error jumps to -step_mm at
    the step and the response is it coming back to zero. Normalising by the
    step makes overshoot a percentage and makes left and right steps read the
    same.
    """
    after = [(along - pre_mm, error) for along, error, _, _ in samples
             if along >= pre_mm]
    if len(after) < 20:
        return None

    distance = np.array([a for a, _ in after])
    # +1 at the moment of the step, decaying to 0 when it is back on the line.
    response = np.array([e for _, e in after]) / -float(step_mm)

    # Straight off the ERROR, not off the normalised response - the response is
    # negated by construction and a sign slip here would read a robot sitting
    # left of the line as one sitting right of it.
    error = np.array([e for _, e in after])
    residual = float(np.mean(error[-max(5, len(error) // 10):]))
    overshoot = float(max(0.0, -response.min()) * 100.0)

    # A crossing only counts if the robot actually went somewhere either side
    # of it. Once it has settled the error rattles about zero by a millimetre,
    # and counting those reads a perfectly damped run as a limit cycle.
    deadband = max(SETTLE_FRACTION, 0.05)
    crossings = _real_crossings(response, deadband)
    swings = int(len(crossings))
    wavelength = None
    if swings >= 2:
        wavelength = float(np.median(np.diff(distance[crossings])) * 2.0)

    settled = None
    inside = np.abs(response) <= SETTLE_FRACTION
    for index in range(len(inside)):
        if inside[index:].all():
            settled = float(distance[index])
            break

    return {"overshoot_pct": overshoot, "swings": swings,
            "wavelength_mm": wavelength, "settled_mm": settled,
            "residual_mm": residual, "distance": distance, "response": response}


def _real_crossings(response, deadband):
    """
    Indices where the response crosses zero having been outside `deadband` on
    the way in - i.e. crossings that are swings of the robot rather than noise
    in a settled pose.
    """
    crossings = []
    side = 0            # which side it was last properly out on
    for index, value in enumerate(response):
        if abs(value) < deadband:
            continue
        now = 1 if value > 0.0 else -1
        if side and now != side:
            crossings.append(index)
        side = now
    return np.array(crossings, dtype=int)


def report(result, lookahead_mm):
    print(f"  overshoot   {result['overshoot_pct']:5.1f}% of the step"
          + ("   <- weaving" if result["overshoot_pct"] > 30.0 else ""))
    print(f"  swings      {result['swings']} zero crossings"
          + ("   <- limit cycle, it never settles" if result["swings"] >= 4 else ""))
    if result["wavelength_mm"]:
        ratio = result["wavelength_mm"] / max(lookahead_mm, 1.0)
        blame = ("the geometry - shorten/lengthen the lookahead" if ratio < 6.0
                 else "the LAG or the POSE, not the lookahead")
        print(f"  wavelength  {result['wavelength_mm']:.0f}mm "
              f"({ratio:.1f}x the {lookahead_mm:.0f}mm lookahead) -> {blame}")
    else:
        print("  wavelength  -  (no oscillation to measure)")
    print(f"  settled     "
          + (f"{result['settled_mm']:.0f}mm after the step"
             if result["settled_mm"] is not None else
             f"NEVER - still outside {SETTLE_FRACTION:.0%} of the step at the end"))
    print(f"  residual    {result['residual_mm']:+.0f}mm off the line at the end"
          + ("   <- a BIAS, not damping: check max_road_wheel_deg and the servo centre"
             if abs(result["residual_mm"]) > 25.0 else ""))


def plot(result):
    """A rough plot of the response, in text, because there is no screen."""
    print("\n  error against distance after the step "
          "(| is the line, one row per 50mm):")
    distance, response = result["distance"], result["response"]
    width = 46
    for target in np.arange(0, distance[-1], 50.0):
        index = int(np.searchsorted(distance, target))
        if index >= len(response):
            break
        value = float(np.clip(response[index], -1.2, 1.2))
        column = int(round((1.0 - value / 1.2) * width / 2.0))
        row = [" "] * (width + 1)
        row[width // 2] = "|"
        row[max(0, min(width, column))] = "*"
        print(f"  {target:5.0f}mm {''.join(row)}")


# ============================================================================
# RUNNING
# ============================================================================

def load(round_name, overrides):
    config = TaskConfig.load(f"tasks/{round_name}/config.toml")
    for assignment in overrides:
        key, _, raw = assignment.partition("=")
        config.set(key.strip(), float(raw))
    return config


def one_run(context, config, args, sign):
    step = args.step * sign
    print(f"\nStep {step:+.0f}mm at speed {args.speed} "
          f"(lookahead {build_pursuit(config).lookahead_distance(args.speed):.0f}mm, "
          f"servo_lag {config.get('pursuit.servo_lag_s', 0.0)}s, "
          f"lag_comp {config.get('pursuit.lag_compensation_s', 0.0)}s, "
          f"full lock {config.get('pursuit.max_road_wheel_deg', 0.0)}deg)")
    samples, aborted = drive_step(context, config, args.speed, step,
                                  args.pre, args.after)
    if aborted:
        # A step test that stopped early stopped part way through the
        # response, so "settled" and the residual are facts about where it was
        # cut off, not about the robot. Overshoot and swings up to the stop
        # are still real, so they are shown - labelled as partial.
        print(f"  ABORTED: {aborted}")
        result = analyse(samples, step, args.pre)
        if result is None:
            print("  stopped before enough of the run after the step to measure")
            return None
        print("  PARTIAL - only overshoot and swings mean anything here:")
        report(result, build_pursuit(config).lookahead_distance(args.speed))
        if args.plot:
            plot(result)
        return None
    result = analyse(samples, step, args.pre)
    if result is None:
        print("  not enough of a run after the step to measure anything")
        return None
    report(result, build_pursuit(config).lookahead_distance(args.speed))
    if args.plot:
        plot(result)
    return result


def scan_fit_mm(context, pose):
    """
    How well the live scan matches the map seen from `pose`: the median gap
    between each measured range and the range the map predicts, in mm. About
    20mm when the pose is right; hundreds when it is somewhere else.
    """
    nav = context.nav
    scan = np.asarray(context.lidar.get_scan(), dtype=float)
    measured = scan[FIT_BEAMS_DEG % 360]
    usable = np.isfinite(measured) & (measured < FIT_MAX_RANGE_MM)
    if np.count_nonzero(usable) < 10:
        return float("inf")
    lidar_x, lidar_y = to_field(pose.x, pose.y, pose.heading, nav.lidar_offset_mm)
    predicted = nav.map.raycast(lidar_x, lidar_y,
                                np.radians(pose.heading + FIT_BEAMS_DEG[usable]))
    return float(np.median(np.abs(predicted - measured[usable])))


def wait_for_settled_pose(context, heading_deg=None):
    """
    Updates the filter until the pose has, for START_HOLD_S straight, both
    confidence >= START_CONFIDENCE and a scan that fits the map from it to
    within START_FIT_MM. Re-scatters the search if it settles somewhere wrong.
    Returns False, having said why, if that never happens.
    """
    started = time.monotonic()
    deadline = started + START_TIMEOUT_S
    searched_at = started
    held_since = None
    pose, fit = context.nav.update(), float("inf")
    while time.monotonic() < deadline:
        pose = context.nav.update()
        fit = scan_fit_mm(context, pose)
        now = time.monotonic()
        if pose.confidence >= START_CONFIDENCE and fit <= START_FIT_MM:
            held_since = held_since or now
            if now - held_since >= START_HOLD_S:
                print(f"  {pose}  scan fits the map to {fit:.0f}mm")
                return True
        else:
            held_since = None
            if now - searched_at > RESEARCH_AFTER_S:
                print(f"  settled at {pose} but the scan is {fit:.0f}mm off the "
                      f"map from there - searching again")
                context.nav.start(heading_deg=heading_deg)
                searched_at = now
        time.sleep(0.05)
    print(f"  {pose}  scan {fit:.0f}mm off the map")
    print(f"  No pose held conf >= {START_CONFIDENCE} with the scan within "
          f"{START_FIT_MM:.0f}mm of the map for {START_HOLD_S:.0f}s in "
          f"{START_TIMEOUT_S:.0f}s. Check nothing is lying on the mat inside "
          f"the walls (pillars, bay walls, a bag) - the map only knows the walls.")
    return False


def main(args):
    config = load(args.round, args.set or [])
    if args.sweep:
        key, raw = args.sweep
        values = [float(value) for value in raw.split(",")]
    else:
        key, values = None, [None]

    context = TaskContext(debug=False, use_lidar=True, lidar_port=args.lidar_port)
    with context:
        # The field is 90-degree symmetric, so the lidar alone can lock onto
        # the wrong quadrant - and a line followed from a pose in the wrong
        # quadrant drives straight into a wall. The compass settles it, but
        # only once told which way the robot is really facing.
        if args.heading is not None:
            context.nav.start(heading_deg=args.heading)
        else:
            print("WARNING: no --heading given, so the compass is unanchored and "
                  "the pose may land in the wrong quadrant of the field")
        # The checks before the run only warn: the run itself still stops on
        # anything in front, on losing the line, and on an unreliable pose.
        print("Waiting for the localizer...")
        if not wait_for_settled_pose(context, args.heading):
            print("  WARNING: pose not settled - running anyway")

        front = clearance_mm(context, ROOM_SECTOR_DEG)
        need = args.pre + args.after
        print(f"  {front:.0f}mm clear ahead" if front else "  no lidar reading ahead")
        if front is not None and front < need:
            print(f"  WARNING: the run is {need:.0f}mm long - it will stop "
                  f"early at the wall")

        if not args.no_wait:
            context.wait_for_start()
            # The filter is not updated while the button is waited on, and the
            # robot gets touched to press it - so settle again before moving.
            print("Re-checking the pose before driving...")
            if not wait_for_settled_pose(context):
                print("  WARNING: pose not settled - running anyway")

        try:
            for value in values:
                if key is not None:
                    config.set(key, value)
                    print(f"\n=== {key} = {value} ===")
                for index in range(args.repeat):
                    # Alternate sides, so a step test cannot walk off the mat
                    # and so a bias shows up as the same residual either way
                    # rather than cancelling.
                    one_run(context, config, args, 1 if index % 2 == 0 else -1)
                    context.motor.drive(0, 0)
                    if index + 1 < args.repeat or value is not values[-1]:
                        input("  reposition the robot down the mat, then Enter: ")
                        if not wait_for_settled_pose(context):
                            print("  WARNING: pose not settled - running anyway")
        finally:
            context.motor.drive(0, 0)
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Step the line sideways and measure how the robot recovers")
    parser.add_argument("--round", default="final", help="which config.toml to use")
    parser.add_argument("--step", type=float, default=200.0,
                        help="how far to move the line sideways, mm")
    parser.add_argument("--pre", type=float, default=500.0,
                        help="straight run before the step, mm")
    parser.add_argument("--after", type=float, default=1500.0,
                        help="run after the step, mm - this is the measurement")
    parser.add_argument("--speed", type=int, default=60)
    parser.add_argument("--heading", type=float, default=None,
                        help="which way the robot is placed, field degrees: "
                             "0=+Y, 90=+X, 180=-Y, 270=-X (anchors the compass)")
    parser.add_argument("--repeat", type=int, default=1,
                        help="steps per setting, alternating side")
    parser.add_argument("--sweep", nargs=2, metavar=("KEY", "VALUES"),
                        help="e.g. --sweep pursuit.lookahead_base_mm 150,250,400")
    parser.add_argument("--set", action="append", metavar="KEY=VALUE",
                        help="override a config value for the run")
    parser.add_argument("--plot", action="store_true", help="text plot of the response")
    parser.add_argument("--no-wait", action="store_true",
                        help="skip the start button (it will not move without it)")
    parser.add_argument("--lidar-port", default=None)
    sys.exit(main(parser.parse_args()))
