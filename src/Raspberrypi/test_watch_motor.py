"""
READ-ONLY: polls the DFR0592 motor board and prints whenever the drive
motor's command, PWM frequency or speed changes. Never writes to the bus, so
it is safe to leave running in a second terminal while a task drives.

    python3 test_watch_motor.py
"""
import time

import smbus2

from classes.motor_manager import (DRIVE_MOTOR, ENCODER_SIGN, MOTOR_BOARD_ADDRESS,
                                   MOTOR_I2C_BUS, OUTPUT_RPM_PER_COUNT)

# Register layout of the DFR0592 (see libraries/DFRobot_RaspberryPi_DC_Motor.py).
REG_ENCODER_SPEED = 0x05 + 5 * (DRIVE_MOTOR - 1)
REG_ENCODER_RATIO = 0x07 + 5 * (DRIVE_MOTOR - 1)
REG_MOTOR_PWM = 0x0e
REG_ORIENTATION = 0x0f + 3 * (DRIVE_MOTOR - 1)
FIRST_REG, LAST_REG = 0x05, 0x14
POLL_S = 0.05
RPM_BUCKET = 25        # only print when rpm moves by about this much

ORIENTATION_NAMES = {1: "CW", 2: "CCW", 5: "STOP"}


def main():
    bus = smbus2.SMBus(MOTOR_I2C_BUS)
    print(f"watching M{DRIVE_MOTOR} on {hex(MOTOR_BOARD_ADDRESS)} - Ctrl+C to quit", flush=True)
    last = None
    t0 = time.monotonic()
    while True:
        try:
            raw = bus.read_i2c_block_data(MOTOR_BOARD_ADDRESS, FIRST_REG, LAST_REG - FIRST_REG + 1)
            reg = lambda r: raw[r - FIRST_REG]
            counts = (reg(REG_ENCODER_SPEED) << 8) | reg(REG_ENCODER_SPEED + 1)
            if counts & 0x8000:
                counts -= 0x10000
            ratio = (reg(REG_ENCODER_RATIO) << 8) | reg(REG_ENCODER_RATIO + 1)
            pwm_hz = reg(REG_MOTOR_PWM) * 50
            orient = reg(REG_ORIENTATION)
            duty = reg(REG_ORIENTATION + 1) + reg(REG_ORIENTATION + 2) / 10.0
            rpm = counts * OUTPUT_RPM_PER_COUNT * ENCODER_SIGN
            state = (orient, duty, ratio, pwm_hz, round(rpm / RPM_BUCKET))
            if state != last:
                name = ORIENTATION_NAMES.get(orient, f"?{orient}")
                print(f"{time.monotonic() - t0:7.2f}s  {name:4s} duty={duty:5.1f}%  "
                      f"pwm={pwm_hz}Hz  enc_ratio={ratio}  ~{rpm:6.0f} rpm", flush=True)
                last = state
        except OSError as e:
            print(f"{time.monotonic() - t0:7.2f}s  read failed: {e}", flush=True)
        time.sleep(POLL_S)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
