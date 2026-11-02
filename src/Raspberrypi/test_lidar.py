"""
Manual test script for classes/lidar_manager.py (RPLidar S2L on the Pi's UART).

Modes:
    python test_lidar.py                 top-down OpenCV visualizer, ESC to quit
    python test_lidar.py --ascii         text radar in the terminal, no display
                                          needed (use this over plain SSH)
    python test_lidar.py --ports         list the UARTs and the port it would use

Useful flags:
    --port /dev/ttyAMA0   override the port (default: lidar_manager.DEFAULT_PORT;
                           NOT /dev/serial0, which is the debug UART on a Pi 5)
    --offset 90           rotate the sensor's zero onto the robot's forward axis
    --mirror              sensor mounted upside down (angles run the other way)
    --duration 10         stop after N seconds instead of running until ESC

The visualizer draws the robot at the center, forward pointing up, bearings
increasing clockwise, with range rings every 50cm and the closest reading in
the front/right/back/left sectors listed in the corner.
"""
import argparse
import glob
import os
import time

import cv2

from classes.lidar_manager import DEFAULT_PORT, LidarManager


def list_ports():
    print("UARTs:")
    for path in sorted(glob.glob("/dev/ttyAMA*") + glob.glob("/dev/serial[0-9]")):
        target = f" -> {os.path.realpath(path)}" if os.path.islink(path) else ""
        print(f"  {path}{target}")
    print(f"\nLidarManager would use: {DEFAULT_PORT}")


def build_lidar(args):
    return LidarManager(
        port=args.port,
        debug=True,
        angle_offset_deg=args.offset,
        mirror=args.mirror,
        render_range_mm=args.range * 10.0,
    )


def run_window(args):
    lidar = build_lidar(args)
    with lidar:
        if not lidar.wait_until_ready(timeout=5.0):
            print("Warning: little or no data after 5s - is the motor spinning?")

        print("Showing lidar view. Press ESC (with the window focused) to quit.")
        deadline = time.monotonic() + args.duration if args.duration else None
        while lidar.is_running:
            try:
                lidar.show_debug()
            except cv2.error as e:
                print(f"Cannot open a window ({e.err.strip()}). Re-run with --ascii.")
                break

            if cv2.waitKey(30) == 27:   # ESC
                break
            if deadline and time.monotonic() > deadline:
                break

        cv2.destroyAllWindows()
        report(lidar)


def run_ascii(args):
    lidar = build_lidar(args)
    with lidar:
        if not lidar.wait_until_ready(timeout=5.0):
            print("Warning: little or no data after 5s - is the motor spinning?")

        print("Printing lidar sectors. Press Ctrl-C to quit.")
        deadline = time.monotonic() + args.duration if args.duration else None
        try:
            while lidar.is_running:
                print("\033[2J\033[H" + lidar.debug_text(), flush=True)
                time.sleep(0.5)
                if deadline and time.monotonic() > deadline:
                    break
        except KeyboardInterrupt:
            print()
        report(lidar)


def report(lidar):
    if lidar.error is not None:
        print(f"Scan thread error: {lidar.error!r}")
        return
    distance, bearing = lidar.get_min_distance(-20, 20)
    if bearing is None:
        print("Front sector: nothing in range")
    else:
        print(f"Front sector: closest {distance / 10:.1f}cm at {bearing}deg")
    print(f"Directions with data: {len(lidar.get_points())}/360")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test the RPLidar S2L LidarManager")
    parser.add_argument("--port", default=None, help=f"serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--offset", type=float, default=0.0,
                        help="degrees to rotate the sensor's zero onto robot forward")
    parser.add_argument("--mirror", action="store_true",
                        help="sensor is mounted upside down")
    parser.add_argument("--range", type=float, default=400.0,
                        help="centimeters covered from the center to the edge of the view")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="stop after N seconds (0 = run until quit)")
    parser.add_argument("--ascii", action="store_true",
                        help="print a text radar instead of opening a window")
    parser.add_argument("--ports", action="store_true",
                        help="list serial ports and exit")
    args = parser.parse_args()

    if args.ports:
        list_ports()
    elif args.ascii:
        run_ascii(args)
    else:
        run_window(args)
