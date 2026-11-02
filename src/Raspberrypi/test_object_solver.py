"""
Manual test script for classes/object_solver.py

Modes:
    python test_object_solver.py               live camera loop, debug visualizer on, ESC to quit
    python test_object_solver.py --synthetic    generates a fake frame with a green + red box,
                                                 no camera/hardware required
    python test_object_solver.py --fov-at 50    live, with ONE pillar placed 50cm straight ahead
                                                 of the lens: prints the CAPTURED_HORIZONTAL_FOV_DEG
                                                 that pillar implies (for a new lens)
    python test_object_solver.py --fov-at 50 --headless
                                                 the same with no windows (over SSH): averages
                                                 --frames frames, saves fov_check.png, and
                                                 prints one number to put in object_solver.py
    python test_object_solver.py --place 50 20 --headless
                                                 a pillar 50cm ahead and 20cm to the RIGHT
                                                 (-20 = left): also checks the bearing, and
                                                 shows what the solver reports vs the truth

The debug visualizer opens two windows: the camera frame with detected boxes
outlined, and a top-down "bird's eye" view of the robot with each detected
box plotted at its solved distance/bearing.
"""
import argparse
import math
import time

import cv2
import numpy as np

from classes.object_solver import BOX_HEIGHT_CM, CAPTURED_FRAME_WIDTH_PX, ObjectSolver
from utils.image_transform_utils import ImageTransformUtils


def hsv_to_bgr(h, s, v):
    pixel = np.uint8([[[h, s, v]]])
    return tuple(int(c) for c in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0][0])


def run_synthetic_check():
    solver = ObjectSolver(debug=True)
    width, height = solver.image_width, solver.image_height
    image = np.zeros((height, width, 3), dtype=np.uint8)

    # colors picked from the middle of the GREEN/RED HSV ranges in
    # utils/image_color_utils.py so they're reliably detected
    green_bgr = hsv_to_bgr(60, 200, 180)
    red_bgr = hsv_to_bgr(5, 150, 150)

    # small box near top-center -> far away; big box lower-left -> close
    cv2.rectangle(image, (300, 60), (340, 90), green_bgr, -1)
    cv2.rectangle(image, (150, 180), (230, 260), red_bgr, -1)

    hsv_image = ImageTransformUtils.bgr_to_hsv(image)
    objects = solver.detect(hsv_image, display_image=image)
    # detect() only renders - putting the windows up is the caller's job now,
    # because in a round detect() runs on the vision thread. See ObjectSolver.
    solver.show_debug()

    print(f"Detected {len(objects)} object(s):")
    for obj in objects:
        print(f"  {obj.color.value:>5}: distance={obj.distance_cm:6.1f}cm  bearing={obj.bearing_deg:6.1f}deg "
              f"forward={obj.forward_cm:6.1f}cm  lateral={obj.lateral_cm:6.1f}cm")

    print("\nPress any key on an OpenCV window to exit...")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def implied_fov_deg(height_px, forward_cm):
    """The full-frame FOV that makes a pillar height_px tall read forward_cm."""
    angular_height_deg = math.degrees(2.0 * math.atan((BOX_HEIGHT_CM / 2.0) / forward_cm))
    return angular_height_deg / height_px * CAPTURED_FRAME_WIDTH_PX


def run_headless_fov(forward_cm, lateral_cm, frames, image_path):
    """
    No windows: one pillar at a KNOWN place, `frames` frames, and three
    answers - the FOV its height implies, the FOV its sideways position
    implies (only when it is off to the side), and what the solver says
    with the FOV it has now, against the truth. Saves a picture of the last
    detection so it can be checked it was the right pillar.

    I/O:
        forward_cm: lens to the pillar's near face, along the camera axis
        lateral_cm: camera axis to the pillar's centre line, + right / - left
    """
    from camera_manager import CameraManager
    from classes.object_solver import CAMERA_YAW_DEG, CAPTURED_HORIZONTAL_FOV_DEG

    true_bearing = math.degrees(math.atan2(lateral_cm, forward_cm))
    camera_manager = CameraManager()
    solver = ObjectSolver(debug=False)
    camera_manager.start_camera()
    time.sleep(1.0)                        # let exposure settle
    centre = CAPTURED_FRAME_WIDTH_PX / 2.0
    seen = []
    print(f"Measuring: pillar {forward_cm:g}cm ahead, {lateral_cm:+g}cm to the side "
          f"(bearing {true_bearing:+.1f} deg), {frames} frames...")
    for _ in range(frames):
        camera_manager.capture_image()
        camera_manager.transform_image()
        objects = solver.detect(camera_manager.hsv_image, display_image=camera_manager.display_image)
        if objects:
            # the one nearest where it should be with the FOV as it stands
            seen.append(min(objects, key=lambda o: abs(o.bearing_deg - true_bearing)))
    cv2.imwrite(image_path, camera_manager.display_image)
    if not seen:
        print(f"No pillar seen in any frame. Check {image_path}: is a red or green pillar "
              f"in view, well lit, and nothing in front of the lens?")
        return

    def med(values):
        return float(np.median(values))

    height = med([o.height_px for o in seen])
    # pixels from the middle of the FULL frame (the solver crops it)
    offset = med([o.pixel_center[0] + solver.horizontal_crop_offset_px - centre for o in seen])
    print(f"  pillar seen in {len(seen)}/{frames} frames: {height:.1f}px tall "
          f"(spread {min(o.height_px for o in seen)}-{max(o.height_px for o in seen)}px), "
          f"{offset:+.0f}px from the middle")

    print(f"\n  with the FOV it has now ({CAPTURED_HORIZONTAL_FOV_DEG}):")
    for name, got, want in (("forward", med([o.forward_cm for o in seen]), forward_cm),
                            ("side", med([o.lateral_cm for o in seen]), lateral_cm),
                            ("bearing", med([o.bearing_deg for o in seen]), true_bearing)):
        unit = "deg" if name == "bearing" else "cm"
        print(f"    {name:8s} {got:+7.1f}{unit}  (true {want:+.1f}, off by {got - want:+.1f})")

    bearing_now = med([o.bearing_deg for o in seen])
    print(f"\n  CAMERA_YAW_DEG should be {CAMERA_YAW_DEG + true_bearing - bearing_now:+.1f}"
          f"   (now {CAMERA_YAW_DEG:+.1f}) - the bearing it reads is off by "
          f"{bearing_now - true_bearing:+.1f}")

    by_height = implied_fov_deg(height, forward_cm)
    print(f"\n  CAPTURED_HORIZONTAL_FOV_DEG from its height:   {by_height:6.1f}")
    if abs(lateral_cm) >= 5 and abs(offset) >= 10:
        # the pixel offset includes the camera's own yaw, so take it back out
        by_side = (true_bearing - CAMERA_YAW_DEG) / offset * CAPTURED_FRAME_WIDTH_PX
        print(f"  CAPTURED_HORIZONTAL_FOV_DEG from its position: {by_side:6.1f}")
        gap = abs(by_side - by_height) / by_height
        if gap > 0.1:
            print(f"  The two disagree by {gap:.0%}: the lens is not one fixed degrees-per-"
                  f"pixel across the image (or the pillar is not where it was measured "
                  f"to be). Distances will be right near the middle and off at the sides.")
        else:
            print(f"  They agree within {gap:.0%} - use about {(by_side + by_height) / 2:.1f}.")
    elif abs(lateral_cm) >= 5:
        print("  (too close to the middle to measure from its position)")
    print(f"\n  picture of the detection: {image_path}")


def run_live(fov_at_cm=None):
    from camera_manager import CameraManager

    camera_manager = CameraManager()
    solver = ObjectSolver(debug=True)
    camera_manager.start_camera()

    print("Running live object solver. Press ESC (with an OpenCV window focused) to quit.")
    try:
        while True:
            camera_manager.capture_image()
            camera_manager.transform_image()

            objects = solver.detect(camera_manager.hsv_image, display_image=camera_manager.display_image)
            solver.show_debug()
            for obj in objects:
                print(f"{obj.color.value:>5}: dist={obj.distance_cm}cm bearing={obj.bearing_deg}deg "
                      f"fwd={obj.forward_cm}cm lat={obj.lateral_cm}cm "
                      f"h={obj.height_px}px x={obj.pixel_center[0]}px")
                if fov_at_cm:
                    print(f"      -> CAPTURED_HORIZONTAL_FOV_DEG = "
                          f"{implied_fov_deg(obj.height_px, fov_at_cm):.1f} "
                          f"(if this pillar is {fov_at_cm:g}cm straight ahead)")

            key = cv2.waitKey(1)
            if key == 27:  # ESC
                break
            time.sleep(0.01)
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Test ObjectSolver GREEN/RED box detection")
    parser.add_argument("--synthetic", action="store_true",
                         help="Run against a generated test frame instead of the live camera")
    parser.add_argument("--fov-at", type=float, metavar="CM",
                        help="A pillar is this many cm straight ahead: print the FOV it implies")
    parser.add_argument("--place", type=float, nargs=2, metavar=("FORWARD_CM", "SIDE_CM"),
                        help="with --headless: a pillar this far ahead and to the side "
                             "(+ right, - left) - checks distance AND bearing")
    parser.add_argument("--headless", action="store_true",
                        help="with --fov-at or --place: no windows, average over --frames")
    parser.add_argument("--frames", type=int, default=30,
                        help="frames to average with --headless (default 30)")
    parser.add_argument("--image", default="fov_check.png",
                        help="where --headless saves the detection picture")
    args = parser.parse_args()

    if args.synthetic:
        run_synthetic_check()
    elif args.headless:
        if args.place:
            forward, side = args.place
        elif args.fov_at:
            forward, side = args.fov_at, 0.0
        else:
            parser.error("--headless needs --fov-at CM or --place FORWARD_CM SIDE_CM")
        run_headless_fov(forward, side, args.frames, args.image)
    else:
        run_live(args.fov_at)
