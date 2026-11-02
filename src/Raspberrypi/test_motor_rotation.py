"""
Check the drive motor's encoder and the rotation control on the DFR0592.

The board only reports rpm; MotorManager integrates it into position_deg
(output-shaft degrees). Run these in order on a new build:

    python test_motor_rotation.py --hand 5        motor OFF: turn the wheel 5
                                                  times by hand, press Enter,
                                                  get ENCODER_CALIBRATION
    python test_motor_rotation.py --spin 80       spin at speed 80 for 2s and
                                                  print rpm / position
    python test_motor_rotation.py --rotate 720    closed-loop: 2 turns forward
    python test_motor_rotation.py --distance 720 --speed 80
                                                  open-loop: drive until 720 deg

--spin, --rotate and --distance turn the motor - put the robot on a stand with
the wheels off the ground. If --spin forward makes position go DOWN, flip
ENCODER_SIGN in classes/motor_manager.py; if the wheels turn backwards, flip
FORWARD_ORIENTATION.
"""
import argparse
import sys
import threading
import time

from classes.board_manager import BoardManager
from classes import motor_manager
from classes.motor_manager import MotorManager


def print_state(motor, label=""):
    print(f"  {label}rpm {motor.rpm:+7.1f}   position {motor.position_deg:+9.1f} deg "
          f"({motor.rotations:+.2f} turns)", flush=True)


def hand_test(motor, turns):
    """
    No motor power: the user turns the output shaft `turns` revolutions and
    the integrated position is compared against it.
    """
    print(f"\nTurn the wheel FORWARD exactly {turns:g} full turns (mark it first), "
          f"then press Enter.")
    motor.reset_position()
    done = threading.Event()
    threading.Thread(target=lambda: (input(), done.set()), daemon=True).start()
    while not done.is_set():
        print_state(motor)
        done.wait(0.5)

    measured = motor.position_deg
    expected = turns * 360.0
    print(f"\n  measured {measured:+.1f} deg, expected {expected:.1f} deg")
    if abs(measured) < 10:
        print("  The encoder saw (almost) nothing. Is it plugged into the board's "
              "encoder port for the drive motor (M2), and is the board powered?")
        return
    if measured < 0:
        print("  Position went DOWN for a forward turn: set ENCODER_SIGN = "
              f"{-motor_manager.ENCODER_SIGN}")
    calibration = motor_manager.ENCODER_CALIBRATION * expected / abs(measured)
    print(f"  ENCODER_CALIBRATION = {calibration:.4f}   "
          f"(currently {motor_manager.ENCODER_CALIBRATION})")


def spin_test(motor, speed, seconds):
    motor.reset_position()
    motor.forward(speed)
    try:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            print_state(motor)
            time.sleep(0.2)
    finally:
        motor.stop()
    time.sleep(0.5)
    print_state(motor, "stopped: ")


def main(args):
    driving = args.spin is not None or args.rotate is not None or args.distance is not None
    board = BoardManager(debug=True)
    if board.address == motor_manager.MOTOR_BOARD_ADDRESS:
        # Only the motor board is on the bus: test the drive motor, no steering.
        print("[board] DFR0604 shares the motor board's address - steering skipped")
        board = None
    motor = MotorManager(board, debug=True, drive_enabled=driving)
    if board is not None:
        board.start()
    try:
        motor.start()
        if args.hand is not None:
            hand_test(motor, args.hand)
        if args.spin is not None:
            spin_test(motor, args.spin, args.seconds)
        if args.rotate is not None:
            start = motor.position_deg
            began = time.monotonic()
            arrived = motor.rotate(args.rotate, max_speed=args.speed or motor_manager.ROTATE_MAX_SPEED)
            took = time.monotonic() - began
            time.sleep(0.5)
            print(f"  rotate({args.rotate:g}): {'arrived' if arrived else 'TIMED OUT'} "
                  f"in {took:.2f}s, moved {motor.position_deg - start:+.1f} deg "
                  f"(after settling)")
        if args.distance is not None:
            start = motor.position_deg
            result = motor.drive_distance(args.speed or 80, args.distance)
            at_stop = motor.position_deg - start
            speed_at_stop = motor.rpm * 6.0
            time.sleep(0.5)
            coast = motor.position_deg - start - at_stop
            print(f"  drive_distance({args.distance:g}): {'done' if result else 'TIMED OUT'}, "
                  f"{at_stop:+.1f} deg at cut-off, {motor.position_deg - start:+.1f} deg "
                  f"after coasting")
            if abs(speed_at_stop) > 1:
                print(f"  coasted {coast:+.1f} deg from {speed_at_stop:+.0f} deg/s: "
                      f"ROTATE_COAST_S = {coast / speed_at_stop:.3f}   "
                      f"(currently {motor_manager.ROTATE_COAST_S})")
        print(f"  write failures {motor.write_failures}, "
              f"encoder read failures {motor.encoder_failures}")
    finally:
        motor.stop_and_close()
        if board is not None:
            board.stop()
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Encoder + rotation control check")
    parser.add_argument("--hand", type=float, nargs="?", const=5.0, default=None,
                        metavar="TURNS",
                        help="motor off; turn the wheel by hand and calibrate the encoder")
    parser.add_argument("--spin", type=int, default=None, metavar="SPEED",
                        help="spin at SPEED (PWM -255..255) and print rpm")
    parser.add_argument("--seconds", type=float, default=2.0)
    parser.add_argument("--rotate", type=float, default=None, metavar="DEG",
                        help="closed-loop move of DEG output-shaft degrees")
    parser.add_argument("--distance", type=float, default=None, metavar="DEG",
                        help="open-loop drive_distance of DEG output-shaft degrees")
    parser.add_argument("--speed", type=int, default=None,
                        help="speed for --distance (default 80) / max speed for --rotate")
    args = parser.parse_args()
    if all(v is None for v in (args.hand, args.spin, args.rotate, args.distance)):
        parser.print_help()
        sys.exit(0)
    sys.exit(main(args))
