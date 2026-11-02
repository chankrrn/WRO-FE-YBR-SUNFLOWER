"""
Measure the steering with a tape instead of the pose: drive a circle at a fixed
steering command and measure how big it is.

    python test_circle.py --steer -80          left
    python test_circle.py --steer 80           right

No lidar, no localizer - the pose was too unreliable to read a turn off. The
circle's radius gives the road-wheel angle directly (bicycle model, rear axle):

    road_wheel = atan(wheelbase / R)

HOW TO MEASURE: put a piece of tape on the floor under the OUTSIDE rear wheel
before starting, and another where that wheel is half a circle later (the
point furthest from the first). That distance is the outer rear wheel's
diameter; R is half of it minus half the track.

Put the robot in the middle of a corner square or on open floor - a circle at
full lock is about 70cm across. Nothing stops it but the timer and Ctrl+C.
"""
import argparse
import math
import sys
import time

from classes.board_manager import BoardManager
from classes.motor_manager import MotorManager

WHEELBASE_MM = 165.0
TRACK_MM = 120.0
MAX_STEER_COMMAND = 90.0


def main(args):
    board = BoardManager(debug=True)
    motor = MotorManager(board, debug=True)
    board.start()
    try:
        motor.start()
        side = "LEFT" if args.steer < 0 else "RIGHT"
        input(f"\nCircle {side} at steer {args.steer:+.0f}, speed {args.speed}, "
              f"{args.seconds:.1f}s. Tape under the outside rear wheel, then Enter: ")
        print("  Starting in 2s...")
        time.sleep(2.0)
        motor.drive(args.steer, args.speed)
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        print("\n  stopped")
    finally:
        motor.stop_and_close()
        board.stop()

    print("\nMeasure the outer rear wheel's circle diameter, then:")
    print(f"  {'diameter':>9} {'road wheel':>11} {'full lock (x' + str(int(MAX_STEER_COMMAND)) + '/' + str(abs(int(args.steer))) + ')':>20}")
    for diameter in range(500, 1501, 100):
        radius = diameter / 2.0 - TRACK_MM / 2.0
        wheel = math.degrees(math.atan(WHEELBASE_MM / radius))
        print(f"  {diameter:>7}mm {wheel:>9.1f}deg {wheel * MAX_STEER_COMMAND / abs(args.steer):>18.1f}deg")
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drive a fixed-steering circle to measure with a tape")
    parser.add_argument("--steer", type=float, required=True,
                        help="steering command, - left / + right (max 90)")
    parser.add_argument("--speed", type=int, default=90)
    parser.add_argument("--seconds", type=float, default=4.0,
                        help="how long to drive (default 4s, about one circle at full lock)")
    sys.exit(main(parser.parse_args()))
