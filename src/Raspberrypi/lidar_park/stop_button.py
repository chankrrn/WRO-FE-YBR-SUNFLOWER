"""
The start button doubles as an emergency stop while the parking runs.

A background thread watches it. A press cuts the drive motor at once, from
that thread - it does not wait for the control loop to come round - and
sets STOP, which every drive command in run.py checks first, so nothing can
start the motor again. The loops then end with StopRequested and the
program stops.

Only a NEW press counts: the watcher first waits for the button to be let
go, so the press that started the round cannot stop the parking.
"""
import threading
import time

DEBOUNCE_S = 0.03       # must read pressed this long to count
POLL_S = 0.005
RELEASED_FOR_S = 0.2    # must be let go this long before a press can count

# Set by the watcher (or anyone) to stop the parking.
STOP = threading.Event()


class StopRequested(Exception):
    """Raised inside the parking when STOP is set."""


def check_stop():
    if STOP.is_set():
        raise StopRequested()


class StopWatcher:
    """
    I/O:
        button: a started classes.start_button.StartButton
        motor:  the MotorManager to cut on a press
    """

    def __init__(self, button, motor):
        self.button = button
        self.motor = motor
        self._quit = threading.Event()
        self._thread = None

    def start(self):
        STOP.clear()
        self._quit.clear()
        self._thread = threading.Thread(target=self._run, name="stop-button", daemon=True)
        self._thread.start()
        print("  (press the button at any time to STOP)")

    def stop(self):
        self._quit.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def _run(self):
        try:
            # wait for the button to be let go first
            released_since = None
            while not self._quit.is_set():
                if self.button.is_pressed():
                    released_since = None
                else:
                    released_since = released_since or time.monotonic()
                    if time.monotonic() - released_since >= RELEASED_FOR_S:
                        break
                time.sleep(POLL_S)
            pressed_since = None
            while not self._quit.is_set():
                if self.button.is_pressed():
                    pressed_since = pressed_since or time.monotonic()
                    if time.monotonic() - pressed_since >= DEBOUNCE_S:
                        STOP.set()
                        try:
                            self.motor.stop()
                        except Exception:
                            pass
                        print("\n*** STOP BUTTON PRESSED - motor cut ***", flush=True)
                        return
                else:
                    pressed_since = None
                time.sleep(POLL_S)
        except Exception as e:      # a dead watcher must not take the run down
            print(f"WARNING: stop button watcher failed ({e}) - no button stop this run")
