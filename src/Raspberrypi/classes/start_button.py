import time

# ============================================================================
# Hardware wiring
# ============================================================================
# BCM pin the start button is on ("digital 16" on the DFR0604).
# The DFR0785 button module drives the pin HIGH while pressed. The internal
# pull-DOWN keeps it LOW otherwise, so an unplugged button reads "not
# pressed" and can never start a round by itself.
START_BUTTON_PIN = 16

# The 40-pin header's GPIO chip. lgpio rather than RPi.GPIO, which cannot
# drive a Pi 5's pins at all ("Cannot determine SOC peripheral base address").
GPIO_CHIP = 0

# What the pin reads while the button is held: 1 for the DFR0785 Gravity button
# module, which drives the signal HIGH when pressed (the pull-down in start()
# holds it at 0 otherwise).
PRESSED_LEVEL = 1

# The press has to read HIGH for this long to count, so a bounce or a glitch on
# the wire cannot start a round.
DEBOUNCE_S = 0.05
POLL_S = 0.01

# Time to take the hand away from the start button before the robot moves.
RELEASE_DELAY_S = 0.8


class StartButton:
    """
    The physical button that starts a round.
    """

    def __init__(self, pin=START_BUTTON_PIN, chip=GPIO_CHIP, debug=False):
        self.pin = pin
        self.chip = chip
        self.debug = debug
        self._handle = None

    def start(self):
        import lgpio

        handle = lgpio.gpiochip_open(self.chip)
        try:
            lgpio.gpio_claim_input(handle, self.pin, lgpio.SET_PULL_DOWN)
        except Exception:
            lgpio.gpiochip_close(handle)
            raise
        self._lgpio = lgpio
        self._handle = handle
        if self.debug:
            print(f"[start] button on BCM {self.pin}, currently "
                  f"{'pressed' if self.is_pressed() else 'released'}")

    def stop(self):
        if self._handle is None:
            return
        try:
            self._lgpio.gpiochip_close(self._handle)
        except Exception:
            pass
        self._handle = None

    def is_pressed(self):
        if self._handle is None:
            raise RuntimeError("StartButton.start() must be called first")
        return self._lgpio.gpio_read(self._handle, self.pin) == PRESSED_LEVEL

    def wait(self):
        """Blocks until the button is pressed, then gives the hand time to clear."""
        print(f"Waiting for the start button (BCM {self.pin})...")
        pressed_since = None
        while True:
            if self.is_pressed():
                pressed_since = pressed_since or time.monotonic()
                if time.monotonic() - pressed_since >= DEBOUNCE_S:
                    break
            else:
                pressed_since = None
            time.sleep(POLL_S)
        time.sleep(RELEASE_DELAY_S)
        print("Start signal received.")
