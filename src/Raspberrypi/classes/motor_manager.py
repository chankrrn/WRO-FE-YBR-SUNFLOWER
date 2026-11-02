import threading
import time
from collections import deque

# Vendored DFRobot driver (libraries/) rather than a pip dependency - upstream
# does not publish it to PyPI.
from libraries.DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as MotorBoard
from utils.angle_utils import clamp

# ============================================================================
# DFR0592 DC motor HAT - the drive motor and its encoder
# ============================================================================
MOTOR_I2C_BUS = 1
# Both DFRobot HATs ship at 0x10; the DFR0604 kept it, this one was moved.
# A new DFR0592 is moved with set_addr(0x11) and a power cycle.
MOTOR_BOARD_ADDRESS = 0x11
DRIVE_MOTOR = MotorBoard.M2   # moved off M1: it ran ~full speed at 21.6% duty
MOTOR_PWM_FREQUENCY_HZ = 1000
BEGIN_TIMEOUT_S = 10.0

# Which way the board has to turn the motor to drive the robot FORWARD. Flip
# to MotorBoard.CCW if a positive speed reverses.
FORWARD_ORIENTATION = MotorBoard.CW

# A speed is the RAW PWM duty the Arduino wrote (0-255), not a percentage -
# every tuned speed in config.toml is in these units. The board wants 0-100%.
SPEED_FULL_SCALE = 255

# ============================================================================
# Encoder: CHP-20GP-180, 11 pulses per motor revolution, 19:1 gearbox
# ============================================================================
# The board only ever reports a SPEED (rpm), never a count, so position is the
# integral of it, polled here on a thread.
#
# Its rpm assumes a 16-line encoder: 16 * 2 * reduction_ratio pulses per
# revolution. Telling it reduction_ratio = 1 makes it report something close
# to MOTOR-shaft rpm, which is 19x finer than letting it divide by the gearbox
# itself - the reading is an integer, and at low speed that resolution is the
# difference between a position and a guess. The conversion back to
# output-shaft rpm is done here instead.
GEAR_RATIO = 19
ENCODER_PPR = 11
BOARD_REDUCTION_RATIO = 1
# The library documents 16 * 2 * ratio pulses per turn, but on this motor the
# board counts both edges of BOTH channels (x4, not x2) - with the documented
# figure every rotation read exactly double. Measured with --hand.
OUTPUT_RPM_PER_COUNT = 8.0 * BOARD_REDUCTION_RATIO / (ENCODER_PPR * GEAR_RATIO)
# Measured correction on top of the above - `test_motor_rotation.py --hand`
# prints the value to put here. 1.0 means "the datasheet numbers are right".
ENCODER_CALIBRATION = 1.0
# Flip to -1 if driving forward makes position_deg go DOWN.
ENCODER_SIGN = -1

ENCODER_POLL_S = 0.01

# ============================================================================
# Rotation control - ported from the Arduino's motor_degree()
# ============================================================================
# kp is PWM (0-255) per degree of output-shaft error, kd PWM per deg/s.
ROTATE_KP = 0.33
ROTATE_KD = 0.007
ROTATE_MAX_SPEED = 100        # the Arduino capped the correction at 100/255
ROTATE_MIN_SPEED = 60         # below this the motor stalls instead of turning
ROTATE_MIN_SPEED_ZONE_DEG = 5  # ...so it is forced up to it outside this error
ROTATE_TOLERANCE_DEG = 2
# How long the drivetrain keeps rolling once the motor is cut: the move aims
# at where the shaft WILL stop (position + velocity * this), not where it is,
# or every arrival overshoots by the coast and the next correction hunts.
# `test_motor_rotation.py --distance` prints the measured value.
ROTATE_COAST_S = 0.4         # --distance 720 --speed 80, wheels off the ground
# Inside the min-speed zone the PD output is too weak to move the motor, so
# once the shaft has sat still there this long it is as close as it gets.
ROTATE_SETTLE_S = 0.2
ROTATE_STILL_RPM = 0.5
ROTATE_TIMEOUT_S = 10.0

# ============================================================================
# DFR0604 expansion board - the steering servo
# ============================================================================
STEER_SERVO_CHANNEL = 3       # PWM header 0-3; the ultrasonic head is on 0
# Arduino Servo.h's pulse range: kept, so a steering command lands the wheels
# exactly where it did when the Arduino drove the servo.
SERVO_MIN_US = 544
SERVO_MAX_US = 2400

# Hard limit on any steering command (servo 45-135deg). On the DFR0604 build
# command 50 only reaches ~13deg at the road wheel, with ~10deg of linkage
# travel left - so the old cap of 50 stopped the steering well short of its
# stops. Set this to the command where the wheels just reach the mechanical
# stop (find it with tasks.servo_test), so nothing can push the servo into it.
# The rounds clamp lower still, at pursuit.max_steer_command.
MAX_STEER_DEG = 90
STEER_SMOOTHING_WINDOW = 3

# drive() skips the I2C writes when nothing has meaningfully changed. The
# control loop runs far faster than the robot can respond, so most ticks ask
# for the same thing as the last one.
STEER_DEADBAND_DEG = 1.0
SPEED_DEADBAND = 1

DRIVE_DISTANCE_TIMEOUT_S = 30.0

# Every speed write is read back from the board and rewritten if it did not
# land. The I2C bus is shared with the BNO055, and a write that is corrupted on
# the wire still "succeeds" - the robot just drives at whatever arrived.
SPEED_WRITE_ATTEMPTS = 3
# The board's registers for motor 1: orientation, then duty as [whole, tenths].
# Motor 2's sit 3 further on. (The library writes them but never reads them.)
_REG_MOTOR1_ORIENTATION = 0x0f

# Runaway check. Measured on the floor: speed 75 -> ~210 output rpm, so about
# 2.8 rpm per speed unit, and true full speed (255) would be ~700. The motor
# counts as running away when it turns faster than the command can explain -
# above RPM_PER_SPEED x |speed| x RUNAWAY_FACTOR + RUNAWAY_SLACK_RPM - for
# RUNAWAY_S straight. Then it is reported with what the board holds, and the
# command re-sent. A light, unloaded wheel spins a bit faster than on the
# floor, which the factor and slack absorb.
RPM_PER_SPEED = 2.8
RUNAWAY_FACTOR = 1.6
RUNAWAY_SLACK_RPM = 80
RUNAWAY_S = 0.5
DONE_REPLY = "t"          # what drive_distance returns on success, as it always has


def steer_to_servo_deg(angle):
    """A steering command (- left / + right) to a 0-180 servo angle - the Arduino's cal_head()."""
    return (angle + 180) / 2


def servo_deg_to_pulse_us(servo_deg):
    return SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * clamp(servo_deg, 0, 180) / 180.0


class MotorManager:
    """
    Drive motor + steering. The motor and its encoder are on the DFR0592 HAT,
    the steering servo on the DFR0604 expansion board (via BoardManager).

    Units are the ones the Arduino used, so nothing above this class changes:
        angle     steering in degrees, - left / + right
        speed     PWM -255..255, sign = direction
        distance  degrees of rotation of the gearbox OUTPUT shaft

    Rotation: the board reports rpm only, so a thread polls it and integrates
    position_deg. On top of that:
        drive_distance(speed, deg)   drive at a fixed speed until that far
        rotate(deg) / rotate_to(deg) closed-loop move to a position, PD like
                                     the Arduino's motor_degree()

    With `drive_enabled=False` every motor write goes out as stop (see
    _apply_speed), which is the one place all of them pass through - steering
    still does everything it normally would.
    """

    def __init__(self, board_manager, debug=False, drive_enabled=True,
                 bus_id=MOTOR_I2C_BUS, address=MOTOR_BOARD_ADDRESS,
                 steer_channel=STEER_SERVO_CHANNEL):
        self.board_manager = board_manager
        self.bus_id = bus_id
        self.address = address
        self.steer_channel = steer_channel
        self.debug = debug
        # False (--no-drive) means the steering servo still does everything it
        # normally would, but the motor never turns - so the round can be
        # walked through by hand, pushing the robot along while watching the
        # wheels answer the real control loop.
        self.drive_enabled = drive_enabled

        self.board = None
        self.current_speed = 0
        self.current_angle = 0
        self.steering_history = deque(maxlen=STEER_SMOOTHING_WINDOW)
        self.write_failures = 0
        self.write_mismatches = 0
        self.runaways = 0
        self._runaway_since = None
        self.encoder_failures = 0

        # The odometry thread and the control loop share one I2C handle.
        self._bus_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._position_deg = 0.0
        self._rpm = 0.0
        self._odometry_thread = None
        self._stop_event = threading.Event()

    # ========================================================================
    # LIFECYCLE
    # ========================================================================

    def start(self):
        """
        Brings up the motor board and the encoder thread and centres the
        steering. Needs board_manager started first. Raises TimeoutError if the
        motor board never answers.
        """
        board = MotorBoard(self.bus_id, self.address)
        deadline = time.monotonic() + BEGIN_TIMEOUT_S
        while board.begin() != board.STA_OK:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Motor board at {hex(self.address)} on i2c-{self.bus_id} never came up")
            time.sleep(0.5)
        # Only rewritten when the board does not already hold it: rewriting it
        # restarts the board's PWM timer, and runs where that happened at
        # start() sometimes drove the motor at ~full speed for the whole run
        # while the registers still read the commanded duty.
        held_pwm = board._read_bytes(board._REG_MOTOR_PWM, 1)[0]
        if (board.last_operate_status != board.STA_OK
                or held_pwm != MOTOR_PWM_FREQUENCY_HZ // 50):
            board.set_moter_pwm_frequency(MOTOR_PWM_FREQUENCY_HZ)
        board.set_encoder_enable([DRIVE_MOTOR])
        board.set_encoder_reduction_ratio([DRIVE_MOTOR], BOARD_REDUCTION_RATIO)
        board.motor_stop(board.ALL)
        self.board = board

        if self.debug:
            print(f"[motor] DFR0592 at {hex(self.address)} on i2c-{self.bus_id}, "
                  f"steering on DFR0604 PWM {self.steer_channel}")
        if not self.drive_enabled:
            print("  DRIVE MOTOR DISABLED - steering only, push the robot by hand")

        self.current_speed = 0
        self.steering_history.clear()
        self.write_failures = 0
        self.write_mismatches = 0
        self.runaways = 0
        self._runaway_since = None
        self.encoder_failures = 0
        self.reset_position()
        self._stop_event.clear()
        self._odometry_thread = threading.Thread(target=self._odometry_loop,
                                                 name="motor-odometry", daemon=True)
        self._odometry_thread.start()
        self.steer(0)

    def stop_and_close(self):
        try:
            self.stop()
            self.steer_center()
            time.sleep(0.1)
        except Exception:
            pass
        self._stop_event.set()
        if self._odometry_thread is not None:
            self._odometry_thread.join(timeout=1.0)
            self._odometry_thread = None
        if self.runaways or self.write_failures or self.write_mismatches:
            print(f"[motor] this run: {self.runaways} runaway(s), {self.write_failures} failed "
                  f"writes, {self.write_mismatches} corrupted writes, "
                  f"{self.encoder_failures} failed encoder reads")
        if self.board is not None:
            try:
                with self._bus_lock:
                    self.board.motor_stop(self.board.ALL)
            except Exception:
                pass
            self.board = None

    @property
    def is_connected(self):
        return self.board is not None

    # ========================================================================
    # LOW LEVEL
    # ========================================================================

    def _require_board(self):
        if self.board is None:
            raise RuntimeError("MotorManager.start() must be called first")

    def _apply_speed(self, speed):
        """Writes `speed` (PWM -255..255) to the motor. The ONLY motor write."""
        self._require_board()
        if not self.drive_enabled:
            speed = 0
        duty = round(min(100.0, abs(speed) * 100.0 / SPEED_FULL_SCALE), 1)
        backward = MotorBoard.CCW if FORWARD_ORIENTATION == MotorBoard.CW else MotorBoard.CW
        orientation = FORWARD_ORIENTATION if speed > 0 else backward
        ok = False
        for attempt in range(SPEED_WRITE_ATTEMPTS):
            with self._bus_lock:
                if duty == 0:
                    self.board.motor_stop([DRIVE_MOTOR])
                else:
                    self.board.motor_movement([DRIVE_MOTOR], orientation, duty)
                ok = self.board.last_operate_status == self.board.STA_OK
                landed = self._read_motor_registers() if ok else None
            if landed is None or self._command_landed(landed, duty, orientation):
                break
            self.write_mismatches += 1
            if self.write_mismatches in (1, 10) or self.write_mismatches % 100 == 0:
                print(f"WARNING: motor board holds {landed} after writing "
                      f"duty {duty} - rewriting ({self.write_mismatches} so far; "
                      f"the I2C bus is corrupting commands)")
        if not ok:
            # Counted and carried on rather than raised: losing one command is
            # survivable, ending the round in the middle of a corner is not.
            self.write_failures += 1
            if self.write_failures in (1, 10) or self.write_failures % 100 == 0:
                print(f"WARNING: motor board write failed "
                      f"({self.write_failures} so far) - is the DFR0592 powered?")

    def _read_motor_registers(self):
        """(orientation, duty) the board holds for the drive motor, or None. Call holding _bus_lock."""
        reg = _REG_MOTOR1_ORIENTATION + 3 * (DRIVE_MOTOR - 1)
        raw = self.board._read_bytes(reg, 3)
        if self.board.last_operate_status != self.board.STA_OK:
            return None
        return raw[0], raw[1] + raw[2] / 10.0

    @staticmethod
    def _command_landed(landed, duty, orientation):
        held_orientation, held_duty = landed
        if duty == 0:
            return held_orientation == MotorBoard.STOP
        # Exactly what motor_movement() encodes, float rounding and all.
        sent = int(duty) + int((duty * 10) % 10) / 10.0
        return held_orientation == orientation and abs(held_duty - sent) < 0.05

    def _apply_steering(self, angle):
        if self.board_manager is None:      # no DFR0604: drive motor only
            return
        pulse = servo_deg_to_pulse_us(steer_to_servo_deg(angle))
        self.board_manager.set_servo_pulse(self.steer_channel, pulse)

    # ========================================================================
    # ENCODER / ODOMETRY
    # ========================================================================

    def _read_output_rpm(self):
        """Output-shaft rpm, signed so + is forward, or None if the read failed."""
        with self._bus_lock:
            counts = self.board.get_encoder_speed([DRIVE_MOTOR])[0]
            ok = self.board.last_operate_status == self.board.STA_OK
        if not ok:
            return None
        return counts * OUTPUT_RPM_PER_COUNT * ENCODER_CALIBRATION * ENCODER_SIGN

    def _odometry_loop(self):
        last_time = time.monotonic()
        last_rpm = 0.0
        while not self._stop_event.is_set():
            board = self.board
            rpm = None
            if board is not None:
                try:
                    rpm = self._read_output_rpm()
                except Exception:
                    rpm = None
            now = time.monotonic()
            if rpm is None:
                # Hold the last reading across a dropped one rather than
                # pretending the wheel stopped.
                self.encoder_failures += 1
                rpm = last_rpm
            # rpm * 6 = deg/s; trapezoid between this sample and the last.
            step = (rpm + last_rpm) * 3.0 * (now - last_time)
            with self._state_lock:
                self._position_deg += step
                self._rpm = rpm
            last_time, last_rpm = now, rpm
            self._check_runaway(rpm, now)
            self._stop_event.wait(ENCODER_POLL_S)

    def _check_runaway(self, rpm, now):
        """
        Flags the motor turning far faster than it was told to, and re-sends
        the command. Only while driving: after a stop the wheels coast down
        from whatever they were doing, which is not a runaway.
        """
        speed = self.current_speed if self.drive_enabled else 0
        limit = RPM_PER_SPEED * abs(speed) * RUNAWAY_FACTOR + RUNAWAY_SLACK_RPM
        if speed == 0 or abs(rpm) <= limit:
            self._runaway_since = None
            return
        if self._runaway_since is None:
            self._runaway_since = now
            return
        if now - self._runaway_since < RUNAWAY_S:
            return
        self._runaway_since = None
        self.runaways += 1
        with self._bus_lock:
            held = self._read_motor_registers()
        if self.runaways <= 5 or self.runaways % 50 == 0:
            print(f"WARNING: motor RUNAWAY #{self.runaways} - {abs(rpm):.0f} rpm at speed "
                  f"{speed} (expected ~{RPM_PER_SPEED * abs(speed):.0f}); board holds "
                  f"{held} (orientation, duty%), write failures {self.write_failures}, "
                  f"corrupted {self.write_mismatches} - re-sending")
        try:
            self._apply_speed(speed)
        except Exception:
            pass

    @property
    def position_deg(self):
        """Output-shaft rotation since start() or reset_position(), + forward."""
        with self._state_lock:
            return self._position_deg

    @property
    def rotations(self):
        return self.position_deg / 360.0

    @property
    def rpm(self):
        """Output-shaft rpm, + forward."""
        with self._state_lock:
            return self._rpm

    def reset_position(self, position_deg=0.0):
        with self._state_lock:
            self._position_deg = float(position_deg)

    # ========================================================================
    # DRIVING
    # ========================================================================

    def drive(self, angle, speed):
        """
        Sets steering and speed together, and writes nothing at all when
        neither value has moved past its deadband.

        I/O:
            return: True if a command actually went out
        """
        angle = clamp(angle, -MAX_STEER_DEG, MAX_STEER_DEG)
        steer_changed = abs(angle - self.current_angle) >= STEER_DEADBAND_DEG
        speed_changed = abs(speed - self.current_speed) >= SPEED_DEADBAND
        if not steer_changed and not speed_changed:
            return False
        if steer_changed:
            self.current_angle = angle
            self._apply_steering(angle)
        if speed_changed:
            self.current_speed = speed
            self._apply_speed(speed)
        return True

    def forward(self, speed):
        self.current_speed = speed
        self._apply_speed(speed)

    def reverse(self, speed):
        self.forward(-abs(speed))

    def stop(self):
        self.current_speed = 0
        self._apply_speed(0)

    def steer(self, angle):
        angle = clamp(angle, -MAX_STEER_DEG, MAX_STEER_DEG)
        self.current_angle = angle
        self._apply_steering(angle)

    def steer_smooth(self, target_angle):
        """
        Steers toward the rolling average of the last STEER_SMOOTHING_WINDOW
        requests, so a single noisy sensor frame cannot jerk the wheels.
        """
        self.steering_history.append(clamp(target_angle, -MAX_STEER_DEG, MAX_STEER_DEG))
        self.steer(sum(self.steering_history) / len(self.steering_history))

    def steer_center(self):
        self.steering_history.clear()
        self.steer(0)

    # ========================================================================
    # DRIVING BY ROTATION
    # ========================================================================

    def drive_distance(self, speed, distance, angle=None, max_wait=DRIVE_DISTANCE_TIMEOUT_S):
        """
        Blocking: drives at `speed` until the output shaft has turned
        `distance` degrees (either way - the direction comes from the sign of
        `speed`, like it did on the Arduino), then stops.

        Open loop on speed, so it coasts past the target by however far the
        robot rolls after the motor is cut. Use rotate() to land on it.

        I/O:
            return: "t" when the distance was covered, None on timeout
        """
        angle = self.current_angle if angle is None else angle
        self.steer(angle)
        if not self.drive_enabled:
            return DONE_REPLY

        start = self.position_deg
        self.forward(speed)
        deadline = time.monotonic() + max_wait
        try:
            while time.monotonic() < deadline:
                if abs(self.position_deg - start) >= abs(distance):
                    return DONE_REPLY
                time.sleep(ENCODER_POLL_S)
        finally:
            self.stop()

        print(f"WARNING: drive_distance timed out after {max_wait:.0f}s "
              f"({abs(self.position_deg - start):.0f} of {abs(distance):.0f} deg)")
        return None

    def rotate(self, degrees, angle=None, **kwargs):
        """
        Blocking: turns the output shaft `degrees` from where it is now, + forward.
        Takes the same keyword arguments as rotate_to().
        """
        return self.rotate_to(self.position_deg + degrees, angle=angle, **kwargs)

    def rotate_rotations(self, rotations, angle=None, **kwargs):
        return self.rotate(rotations * 360.0, angle=angle, **kwargs)

    def rotate_to(self, target_deg, angle=None, max_speed=ROTATE_MAX_SPEED,
                  min_speed=ROTATE_MIN_SPEED, tolerance_deg=ROTATE_TOLERANCE_DEG,
                  timeout=ROTATE_TIMEOUT_S):
        """
        Blocking: drives the output shaft to `target_deg` on position_deg's
        scale and stops there, correcting back if it overshoots.

        The Arduino's motor_degree() loop: PD on the error, capped at
        max_speed, pushed up to min_speed while still more than
        ROTATE_MIN_SPEED_ZONE_DEG out so it does not stall short. Inside that
        zone it counts as arrived once the shaft stops.

        I/O:
            return: True if it arrived, False on timeout
        """
        angle = self.current_angle if angle is None else angle
        self.steer(angle)
        if not self.drive_enabled:
            return True

        deadline = time.monotonic() + timeout
        still_since = None
        try:
            while True:
                error = target_deg - self.position_deg
                still = abs(self.rpm) < ROTATE_STILL_RPM
                if abs(error) <= tolerance_deg and still:
                    return True
                if abs(error) <= ROTATE_MIN_SPEED_ZONE_DEG and still:
                    still_since = still_since or time.monotonic()
                    if time.monotonic() - still_since >= ROTATE_SETTLE_S:
                        return True
                else:
                    still_since = None
                if time.monotonic() > deadline:
                    print(f"WARNING: rotate_to({target_deg:.0f}) timed out "
                          f"{error:+.0f} deg short")
                    return False

                velocity_deg_s = self.rpm * 6.0
                coast_error = error - velocity_deg_s * ROTATE_COAST_S
                if abs(coast_error) <= tolerance_deg or coast_error * error < 0:
                    # Will roll onto (or past) the target from here: cut the
                    # motor and let the next ticks see where it comes to rest.
                    # Not reversed - braking against the coast is what made it
                    # hunt; if it does end up past, it is driven back from rest.
                    self.stop()
                    time.sleep(ENCODER_POLL_S)
                    continue

                output = ROTATE_KP * coast_error - ROTATE_KD * velocity_deg_s
                command = min(abs(output), max_speed)
                # A kick to break stiction, only from rest: forcing it while
                # already turning drove every small correction far past.
                if (command < min_speed and abs(coast_error) > ROTATE_MIN_SPEED_ZONE_DEG
                        and still):
                    command = min_speed
                self.forward(command if output >= 0 else -command)
                time.sleep(ENCODER_POLL_S)
        finally:
            self.stop()
