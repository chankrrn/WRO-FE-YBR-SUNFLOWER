import threading
import time

from classes.motor_manager import MOTOR_BOARD_ADDRESS, MOTOR_I2C_BUS

# Vendored DFRobot driver (libraries/) rather than a pip dependency - upstream
# does not publish it to PyPI.
from libraries.DFRobot_RaspberryPi_Expansion_Board import (
    DFRobot_Expansion_Board_IIC as Board,
    DFRobot_Expansion_Board_Servo as Servo,
)

I2C_BUS = 1
BOARD_ADDRESS = 0x10
BEGIN_TIMEOUT_S = 10.0

SERVO_FRAME_US = 20000     # 50Hz, what servo.begin() sets the PWM to


class BoardManager:
    """
    The DFRobot DFR0604 expansion board: the I2C hub every analog sensor and
    both servos (steering and the ultrasonic head) hang off.

    Owns nothing application-specific - it just brings the board up, keeps the
    ADC enabled and hands out raw channel reads. LightSensorManager and
    UltraServoManager sit on top of it and give those channels meaning.
    """

    def __init__(self, bus_id=I2C_BUS, address=BOARD_ADDRESS, debug=False):
        self.bus_id = bus_id
        self.address = address
        self.debug = debug
        self.board = None
        self.servo = None
        # The steering is written from the control loop and the ADC read from
        # wherever a sensor is polled - one transaction at a time.
        self._lock = threading.Lock()

    def start(self):
        """
        Connects and enables the ADC. Raises TimeoutError if the board never
        reports STA_OK - usually a power or I2C wiring problem, and there is no
        point letting a task run blind past it.
        """
        # The DFR0592 answers begin() too (same PID), so without this the
        # ADC enable and servo setup would be written into the motor board.
        if (self.bus_id, self.address) == (MOTOR_I2C_BUS, MOTOR_BOARD_ADDRESS):
            raise RuntimeError(
                f"Expansion board and motor board are both set to {hex(self.address)} "
                f"on i2c-{self.bus_id} - change one board's address switch")
        board = Board(self.bus_id, self.address)
        deadline = time.monotonic() + BEGIN_TIMEOUT_S
        while board.begin() != board.STA_OK:
            if time.monotonic() > deadline:
                raise TimeoutError(
                    f"Expansion board at {hex(self.address)} on i2c-{self.bus_id} never came up")
            time.sleep(0.5)

        board.set_adc_enable()
        self.board = board
        if self.debug:
            print(f"[board] up at {hex(self.address)} on i2c-{self.bus_id}, ADC enabled")

    def read_adc(self, channel):
        """Raw ADC reading in millivolts, or None if the read failed."""
        if self.board is None:
            return None
        try:
            with self._lock:
                return self.board.get_adc_value(channel)
        except Exception:
            return None

    def create_servo(self):
        """
        Returns the servo driver bound to this board, started on first use.

        Shared, not one per caller: servo.begin() zeroes EVERY PWM channel, so
        the ultrasonic head starting after the steering would otherwise cut
        the steering servo's signal.
        """
        if self.board is None:
            raise RuntimeError("BoardManager.start() must be called first")
        if self.servo is None:
            with self._lock:
                servo = Servo(self.board)
                servo.begin()
            self.servo = servo
        return self.servo

    def set_servo_pulse(self, channel, pulse_us):
        """
        Drives a servo on PWM `channel` (0-3) with a pulse of `pulse_us`.

        Servo.move() maps 0-180 onto 500-2500us; this lets a caller keep its
        own mapping. The board takes duty in 0.1% steps, which at 50Hz is 20us
        - about 2 servo degrees - so that is the finest it can resolve.
        """
        self.create_servo()
        duty = 100.0 * pulse_us / SERVO_FRAME_US
        with self._lock:
            self.board.set_pwm_duty(channel, duty)

    def stop(self):
        if self.board is None:
            return
        try:
            with self._lock:
                self.board.set_adc_disable()
        except Exception:
            pass
        self.board = None
        self.servo = None
