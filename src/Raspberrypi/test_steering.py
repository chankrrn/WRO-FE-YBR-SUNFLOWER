"""
Measure the robot's steering, instead of guessing at it.

Two numbers in config.toml cannot be derived from first principles and both
cause a weave when they are wrong:

    pursuit.max_road_wheel_deg     what the wheels really do at full lock
    pursuit.lag_compensation_s     how long they take to get there

`max_road_wheel_deg` is the more dangerous of the two, because it SCALES every
steering command. Set it lower than reality and the robot turns more than the
controller asked for, every single tick - which looks exactly like a controller
that over-corrects and hunts, and no amount of lookahead tuning will fix it.

This script drives the robot in the open and works both out from the pose
track, using the localizer that is already running. No protractor needed.

    python test_steering.py --calibrate     road-wheel angle per command
    python test_steering.py --lag           steering response time
    python test_steering.py --all

    python test_steering.py --sweep speed.base 45,55,65,75    tuning trade-offs
                                                              (simulated)

CLEAR SPACE NEEDED. The robot drives arcs of up to ~1m radius for a few
seconds at a time. Put it in the middle of a clear mat, not on a bench.

START BUTTON. Anything here that moves waits for the physical start button
exactly like a round does - localize first, show you the pose, then wait for the press. That
ordering is deliberate: you get to see that the robot knows where it is before
you commit to letting it move.
"""
import argparse
import math
import sys
import time

import numpy as np

from classes.task_context import TaskContext
from utils.angle_utils import angle_difference

# Speed the arcs are driven at. This has to clear stiction WITH THE WHEELS
# TURNED, which takes noticeably more torque than rolling straight - a speed
# that drives fine in a straight line can leave the robot sitting still the
# moment any steering goes on. Override with --drive-speed.
CALIBRATION_SPEED = 75

# Short burst above the measurement speed to break stiction, then settle back.
# Without it the robot can refuse to start rolling at all on a low-command arc.
KICK_BONUS = 25
KICK_S = 0.35

SETTLE_S = 0.7          # let the wheels reach the angle before measuring
MEASURE_S = 1.6
SAMPLE_S = 0.05

# These arcs are driven open-loop - nothing is steering around obstacles - so
# the lidar is watched the whole time and the run is cut the moment anything
# gets close. A calibration is not worth a broken robot. Generous because the
# DFR0592's stop lets the motor freewheel - the robot rolls on well past the
# point where the run is cut, and 280mm was not enough to stop short of a wall.
ABORT_CLEARANCE_MM = 550.0
# Narrow enough that the side walls of a ~1m lane stay out of it - at +-40deg
# a robot 35cm off a side wall saw it at ~50cm and stopped every arc before it
# began. A wall the arc is turning INTO still comes through the middle.
ABORT_SECTOR_DEG = 20.0
# The start check only asks whether the way straight ahead is open. Across the
# whole abort sector the side walls of a 1m-wide straight are only ~90cm away,
# so a wide cone would refuse every start on the field.
START_SECTOR_DEG = 15.0
# Below this many directions with a reading, the lidar is not really working
# and an empty view ahead says nothing about whether it is open.
MIN_VALID_DIRECTIONS = 180


def clearance_mm(context, sector_deg=ABORT_SECTOR_DEG):
    """Closest lidar return within +-sector_deg of ahead, or None if there is none."""
    if context.lidar is None:
        return None
    distance, _ = context.lidar.get_min_distance(-sector_deg, sector_deg)
    return None if math.isnan(distance) else distance


def wall_stop(context):
    """
    Cuts the motor if anything is within ABORT_CLEARANCE_MM ahead, and says
    what it saw - so a stop can be told apart from a false alarm.

    I/O:
        return: True if it fired
    """
    if context.lidar is None:
        return False
    distance, bearing = context.lidar.get_min_distance(-ABORT_SECTOR_DEG, ABORT_SECTOR_DEG)
    if math.isnan(distance) or distance >= ABORT_CLEARANCE_MM:
        return False
    context.motor.drive(0, 0)
    bearing = bearing - 360 if bearing > 180 else bearing
    print(f"    [wall stop: {distance:.0f}mm at {bearing:+d}deg, motor "
          f"{context.motor.rpm:.0f}rpm, pose {context.nav.get_pose()}]", flush=True)
    return True


def watch(context, seconds):
    """
    Waits `seconds` with the wall stop on. EVERY moment the robot is driving
    has to go through this or collect() - a plain sleep is a blind stretch,
    and the blind kick and settle before each arc are what ran into the wall.

    I/O:
        return: True if the wall stop fired (the motor is stopped)
    """
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if wall_stop(context):
            return True
        time.sleep(SAMPLE_S)
    return False


def kick_off(context, steer_command, speed):
    """
    Breaks stiction with a short burst above the target speed, then settles to
    it. Returns once the wheels should be rolling.

    I/O:
        return: True if the wall stop fired during the kick
    """
    context.motor.drive(steer_command, min(100, speed + KICK_BONUS))
    if watch(context, KICK_S):
        return True
    context.motor.drive(steer_command, speed)
    return False


def collect(context, steer_command, speed, seconds):
    """
    Drives at a fixed steering command and logs the pose, stopping early if
    anything comes within ABORT_CLEARANCE_MM of the nose.

    I/O:
        return: (samples, aborted) where samples is
                [(t, x_mm, y_mm, heading_deg, motor_rpm), ...]
    """
    context.motor.drive(steer_command, speed)
    samples = []
    started = time.monotonic()
    while time.monotonic() - started < seconds:
        if wall_stop(context):
            return samples, True
        pose = context.nav.get_pose()
        samples.append((time.monotonic() - started, pose.x, pose.y, pose.heading,
                        context.motor.rpm))
        time.sleep(SAMPLE_S)
    return samples, False


def arc_geometry(samples, wheelbase_mm):
    """
    Road-wheel angle implied by an arc, from the pose track alone.

    A bicycle-model vehicle turns at yaw_rate = v / L * tan(delta), so
    delta = atan(yaw_rate * L / v). Both yaw rate and speed are measured
    here rather than assumed, which is what makes this independent of the
    mm_per_s_at_full calibration - a wrong speed figure cannot bias it.

    I/O:
        return: (road_wheel_deg, speed_mm_s, yaw_rate_deg_s), signed the same
                way as the steering command
    """
    if len(samples) < 4:
        return None, 0.0, 0.0

    times = np.array([s[0] for s in samples])
    points = np.array([(s[1], s[2]) for s in samples])
    headings = np.array([s[3] for s in samples])

    span = times[-1] - times[0]
    if span <= 0.0:
        return None, 0.0, 0.0

    # Unwrap the heading so a lap past 360 does not read as a huge jump back.
    turned = 0.0
    for previous, current in zip(headings, headings[1:]):
        turned += angle_difference(current, previous)
    yaw_rate = turned / span

    travelled = float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))
    speed = travelled / span
    if speed < 20.0:
        return None, speed, yaw_rate

    road_wheel = math.degrees(math.atan(math.radians(yaw_rate) * wheelbase_mm / speed))
    return road_wheel, speed, yaw_rate


def probe_link():
    """
    Are the boards answering at all? Brings up the DFR0604 and the DFR0592,
    centres the steering and prints the encoder for a couple of seconds. The
    drive motor is held disabled - this only checks the link. Turn the wheels
    by hand while it runs and the rpm should move.
    """
    from classes.board_manager import BoardManager
    from classes.motor_manager import MotorManager

    board = BoardManager(debug=True)
    motor = MotorManager(board, debug=True, drive_enabled=False)
    board.start()
    try:
        motor.start()
        for _ in range(10):
            print(f"  rpm {motor.rpm:+7.1f}   position {motor.position_deg:+8.1f} deg",
                  flush=True)
            time.sleep(0.2)
        print(f"  write failures {motor.write_failures}, "
              f"encoder read failures {motor.encoder_failures}", flush=True)
    finally:
        motor.stop_and_close()
        board.stop()


def probe_speed(context, speeds):
    """
    Drives straight at each speed and measures how fast the robot ACTUALLY
    went, from the pose track.

    This is the test to run when a calibration comes back as noise: it
    separates "the motor is not turning" from "the motor turns but my speed
    figure is wrong", and it finds the stiction threshold - the command below
    which the robot does not move at all, which is worth knowing before
    speed.minimum is set under it.
    """
    print(f"\n{'speed':>7} {'measured':>12} {'implied full lock':>19}")
    rows = []
    for speed in speeds:
        context.motor.drive(0, speed)
        if watch(context, SETTLE_S):
            samples, aborted = [], True
        else:
            samples, aborted = collect(context, 0, speed, MEASURE_S)
        context.motor.drive(0, 0)
        time.sleep(0.8)

        _, measured, _ = arc_geometry(samples, 165.0)
        note = "  (aborted - wall ahead)" if aborted else ""
        if measured < 20.0:
            print(f"{speed:>7} {measured:>10.0f}mm/s {'-- not moving':>19}{note}")
            continue
        rows.append((speed, measured))
        print(f"{speed:>7} {measured:>10.0f}mm/s {measured / speed * 100:>17.0f}{note}")

    if not rows:
        print("\n  The robot never moved. Either the motor is not being driven at all")
        print("  (is the DFR0592 powered? try --probe-link), or every")
        print("  speed tried was below its stiction threshold - retry with --speeds 50,70,90")
        return
    moving = [speed for speed, _ in rows]
    print(f"\n  lowest speed that moved: {min(moving)}  -> keep speed.minimum above it")
    print(f"  startup.mm_per_s_at_full = "
          f"{np.median([m / s * 100 for s, m in rows]):.0f}")


def tape_measure(context, speed_command, seconds):
    """
    Drives straight for a fixed time and stops, so the distance can be read
    off a TAPE MEASURE rather than off the pose track.

    The one measurement in this file that does not trust the localizer. Every
    other number here - and probe_speed above - derives distance by summing
    |delta pose| along the pose track, so pose jitter adds up as phantom
    travel and reads as a robot that is going faster than it is. That is
    fine when the filter is healthy and worthless when it is not, and
    `mm_per_s_at_full` is exactly the number you need when it is NOT: it feeds
    the filter's motion model, so a wrong value is what makes the pose jitter
    in the first place. Measuring it from the pose is circular.

    Mark the floor at the robot's back edge, run it, mark again, measure.
    """
    print(f"\nDriving straight at speed {speed_command} for {seconds:.1f}s.")
    print("  Mark the floor at the robot's rear edge NOW, and again where it stops.")
    print("  Starting in 3s...", flush=True)
    time.sleep(3.0)

    context.motor.drive(0, speed_command)
    start = time.monotonic()
    stopped_early = watch(context, seconds)
    context.motor.drive(0, 0)
    driven_s = time.monotonic() - start
    if stopped_early:
        print(f"\n  WALL STOP after {driven_s:.2f}s - the figure below still holds "
              f"for that time, but start further back for a cleaner run.")

    print(f"\n  drove for {driven_s:.2f}s at command {speed_command}")
    print( "  measure the distance between the marks, then:")
    print(f"      startup.mm_per_s_at_full = distance_mm / {driven_s:.2f} / "
          f"{speed_command} * 100")
    for distance in (500, 1000, 1500, 2000):
        print(f"        {distance:5}mm -> {distance / driven_s / speed_command * 100:6.0f}")


def calibrate(context, wheelbase_mm, max_steer_command, commands,
              speed_command=CALIBRATION_SPEED):
    """
    Drives an arc at each steering command and reports the road-wheel angle
    each one really produced.
    """
    print(f"\nCalibrating steering at speed {speed_command} ({len(commands)} arcs, "
          f"~{len(commands) * (SETTLE_S + MEASURE_S + 1.0):.0f}s)...\n")
    print(f"{'command':>8} {'road wheel':>12} {'implied full lock':>18} "
          f"{'speed':>10} {'yaw rate':>10} {'motor rpm':>10}")

    rows = []
    walled = 0
    for command in commands:
        if kick_off(context, command, speed_command) or watch(context, SETTLE_S):
            samples, aborted = [], True
        else:
            samples, aborted = collect(context, command, speed_command, MEASURE_S)
        context.motor.drive(0, 0)
        time.sleep(0.6)

        road_wheel, speed, yaw_rate = arc_geometry(samples, wheelbase_mm)
        if road_wheel is None:
            reason = "too close to a wall" if aborted else "robot did not move enough"
            walled += aborted
            print(f"{command:>8} {'--':>12}   ({reason})")
            continue
        full_lock = road_wheel * max_steer_command / command
        rows.append((command, road_wheel, full_lock, speed))
        # The motor's own view of the same arc: every row is driven at the same
        # speed command, so one that is far off the others is a motor problem
        # (a write that landed wrong), not a steering one.
        rpm = float(np.median([sample[4] for sample in samples]))
        print(f"{command:>8} {road_wheel:>10.1f}deg {full_lock:>16.1f}deg "
              f"{speed:>8.0f}mm/s {yaw_rate:>8.1f}d/s {rpm:>10.0f}")

    if not rows and walled == len(commands):
        print("\n  No usable arcs - the wall stop cut every one. See the [wall stop]")
        print("  lines above for what it saw; start in the middle of the lane, facing")
        print("  down the straight, and put the robot back between runs.")
        return
    if not rows:
        print(f"\n  No usable arcs - the robot did not move at speed {speed_command}.")
        print( "  Steering costs more torque than driving straight, so the speed that")
        print( "  rolls fine in a line may not be enough with the wheels turned:")
        print(f"      uv run python test_steering.py --calibrate --drive-speed "
              f"{min(100, speed_command + 20)}")
        print( "  Or skip the arcs entirely - a normal lap measures the same number")
        print( "  from ordinary cornering, at whatever speed the robot really drives:")
        print( "      uv run python main.py qualification --laps 1")
        return

    full_locks = [row[2] for row in rows]
    speeds = [row[3] for row in rows]
    best = float(np.median(full_locks))
    spread = float(np.max(full_locks) - np.min(full_locks))

    print(f"\n  pursuit.max_road_wheel_deg = {best:.1f}")
    print(f"  spread across commands: {spread:.1f}deg", end="")
    if spread > 0.25 * best:
        print("  <- steering is NOT linear; trust the mid-range arcs most")
    else:
        print("  (consistent - the linear model holds)")
    print(f"  startup.mm_per_s_at_full = {np.median(speeds) / speed_command * 100:.0f}"
          f"   (measured at speed {speed_command})")


def measure_lag(context, wheelbase_mm, command=45):
    """
    Steering response time, from a step input.

    Commands a step from centre to `command` and watches how fast the yaw rate
    builds. The time to reach 63% of the final value is the first-order time
    constant - which is what pursuit.lag_compensation_s wants.
    """
    print(f"\nMeasuring steering lag (step to {command})...")
    if kick_off(context, 0, CALIBRATION_SPEED) or watch(context, 1.0):
        print("  too close to a wall before the step - start further back")
        return
    samples, aborted = collect(context, command, CALIBRATION_SPEED, 2.0)
    context.motor.drive(0, 0)
    if aborted:
        print("  (cut short by the wall stop - start with more room for the turn)")

    times = np.array([s[0] for s in samples])
    headings = np.array([s[3] for s in samples])
    rates = np.array([angle_difference(b, a) / max(dt, 1e-3)
                      for a, b, dt in zip(headings, headings[1:], np.diff(times))])
    if len(rates) < 6:
        print("  not enough samples - is the pose updating?")
        return

    # Smooth, then find when it first crosses 63% of the settled rate.
    smoothed = np.convolve(np.abs(rates), np.ones(3) / 3.0, mode="valid")
    settled = float(np.median(smoothed[-max(3, len(smoothed) // 3):]))
    if settled < 5.0:
        print(f"  yaw rate only reached {settled:.1f}deg/s - too small to time")
        return

    crossing = np.flatnonzero(smoothed >= 0.63 * settled)
    tau = times[crossing[0] + 1] if len(crossing) else float("nan")
    print(f"  settled yaw rate {settled:.1f}deg/s, 63% reached at {tau:.2f}s")
    print(f"\n  pursuit.lag_compensation_s = {min(tau, 0.15):.2f}")
    print("  (capped at 0.15 - over-predicting is worse than not predicting;"
          " if the true lag is larger, cut speed.corner instead)")


def sweep(key, values):
    """
    Simulated trade-off table for one config key. No hardware; this is for
    seeing which direction a knob moves things before trying it for real.
    """
    import argparse as _argparse
    import contextlib
    import io

    from tasks.cli import load_config
    from tasks.qualification.task import QualificationTask
    import test_driving as td

    print(f"\nSimulated sweep of {key} (2 laps, 2 placements each)\n")
    print(f"{key:>22} {'off-line rms':>13} {'worst clearance':>17}  ok")
    for raw in values:
        value = float(raw) if "." in raw or "e" in raw.lower() else int(raw)
        results = []
        for seed in (3, 9):
            config = load_config(QualificationTask, _argparse.Namespace(
                config=None, laps=None, speed=None, corner_speed=None))
            config.set("laps.goal", 2)
            config.set(key, value)
            with contextlib.redirect_stdout(io.StringIO()):
                results.append(td.simulate(QualificationTask, config, (1000.0, 0.0, 0.0),
                                           seed=seed))
        rms = np.mean([r["rms_offset_mm"] for r in results]) / 10
        clear = np.min([r["min_clearance_mm"] for r in results]) / 10
        ok = all(r["completed"] and not r["crashed"] for r in results)
        print(f"{value:>22} {rms:>11.2f}cm {clear:>15.1f}cm  {'yes' if ok else 'NO'}")


def main(args):
    if args.sweep:
        sweep(args.sweep[0], args.sweep[1].split(","))
        return 0

    if args.probe_link:
        probe_link()
        return 0

    context = TaskContext(debug=False, use_lidar=True, lidar_port=args.lidar_port)
    with context:
        print("Waiting for the localizer...")
        for _ in range(80):
            if context.nav.update().is_reliable:
                break
            time.sleep(0.05)
        pose = context.nav.get_pose()
        print(f"  {pose}")
        if not pose.is_reliable:
            print("  WARNING: pose is not reliable; the numbers below will be noisy")

        front = clearance_mm(context, START_SECTOR_DEG)
        if front is None:
            # Nothing ahead is only "open" if the lidar is seeing everything
            # else - the S2L loses the 10cm black wall at ~3m, but finds it
            # well inside the abort distance. With most of the scan empty the
            # wall stop would be blind too, so that is refused.
            valid = int(np.count_nonzero(~np.isnan(context.lidar.get_scan())))
            if valid < MIN_VALID_DIRECTIONS:
                print(f"  Lidar sees only {valid}/360 directions, so the wall stop "
                      f"cannot work. Check it is spinning and powered before driving.")
                return 1
            print(f"  nothing within lidar range straight ahead ({valid}/360 "
                  f"directions valid) - treating as open")
        else:
            print(f"  {front:.0f}mm clear ahead")
        if front is not None and front < ABORT_CLEARANCE_MM * 2:
            print(f"  Not enough room to drive - need at least "
                  f"{ABORT_CLEARANCE_MM * 2:.0f}mm ahead. Move the robot clear.")
            return 1

        if not args.no_wait:
            context.wait_for_start()

        commands = [int(v) for v in args.commands.split(",")]
        try:
            if args.tape:
                tape_measure(context, args.drive_speed, args.tape)
            if args.speeds or args.all:
                probe_speed(context, [int(v) for v in (args.speeds or "25,35,50,70").split(",")])
            if args.calibrate or args.all:
                calibrate(context, args.wheelbase, args.max_steer_command, commands,
                          args.drive_speed)
            if args.lag or args.all:
                measure_lag(context, args.wheelbase, max(commands))
        finally:
            context.motor.drive(0, 0)
            print(f"\n  motor board: {context.motor.write_failures} failed writes, "
                  f"{context.motor.write_mismatches} corrupted (and rewritten), "
                  f"{context.motor.encoder_failures} failed encoder reads")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Measure the robot's steering geometry")
    parser.add_argument("--calibrate", action="store_true",
                        help="measure the road-wheel angle each command produces")
    parser.add_argument("--lag", action="store_true",
                        help="measure the steering response time")
    parser.add_argument("--all", action="store_true", help="all of the above")
    parser.add_argument("--probe-link", action="store_true",
                        help="check the motor and expansion boards answer (no motion)")
    parser.add_argument("--tape", nargs="?", type=float, const=3.0, default=None,
                        metavar="SECONDS",
                        help="drive straight for N seconds (default 3) and stop, so "
                             "mm_per_s_at_full can be measured with a tape measure "
                             "instead of the pose track. Use this when the pose is "
                             "not trustworthy - which is exactly when this number "
                             "is wrong.")
    parser.add_argument("--speeds", nargs="?", const="25,35,50,70", default=None,
                        metavar="LIST",
                        help="drive straight at each speed and measure the real mm/s")
    parser.add_argument("--sweep", nargs=2, metavar=("KEY", "VALUES"),
                        help="simulated trade-off table, e.g. --sweep speed.base 45,55,65")
    parser.add_argument("--commands", default="20,35,50,65,80",
                        help="steering commands to test (default: 20,35,50,65,80)")
    parser.add_argument("--drive-speed", type=int, default=CALIBRATION_SPEED,
                        help=f"speed to drive the arcs at (default: {CALIBRATION_SPEED}). "
                             f"Raise it if the robot will not move with the wheels turned.")
    parser.add_argument("--wheelbase", type=float, default=165.0,
                        help="front axle to rear axle, mm (default: 165)")
    parser.add_argument("--max-steer-command", type=float, default=80.0)
    parser.add_argument("--stop-mm", type=float, default=ABORT_CLEARANCE_MM,
                        help=f"wall stop distance ahead, mm (default: {ABORT_CLEARANCE_MM:.0f}). "
                             f"Lower it to fit the arcs into a 1m field lane.")
    parser.add_argument("--no-wait", action="store_true",
                        help="skip the start button")
    parser.add_argument("--lidar-port", default=None)
    args = parser.parse_args()
    ABORT_CLEARANCE_MM = args.stop_mm

    if not (args.calibrate or args.lag or args.all or args.sweep
            or args.probe_link or args.speeds or args.tape):
        parser.print_help()
        sys.exit(0)
    sys.exit(main(args))
