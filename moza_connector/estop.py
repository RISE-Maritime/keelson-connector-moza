"""The Moza E-Stop's state, read on its own serial port (``base_serial.EstopSerial``).

The E-Stop cuts force feedback in the base itself and reports each change, unasked. This only
listens: it never writes to the E-Stop and never acts on the wheel; it says what the stop is
doing so the helm's operators can see it. The state is ``unknown`` until the first change
arrives (the E-Stop does not say its state when opened), and again when its port is lost.
"""

from __future__ import annotations

import logging
import threading

from . import serial_proto as proto

log = logging.getLogger(__name__)

PRESSED, RELEASED, UNKNOWN, GONE = "pressed", "released", "unknown", "gone"


class EstopWatch:
    """Reads ``port`` (EstopSerial or FakeEstop) until ``stop`` is set; ``state`` is the latest."""

    def __init__(self, port):
        self._port = port
        self.state = UNKNOWN
        self._lock = threading.Lock()

    def _set(self, state: str) -> None:
        with self._lock:
            if state != self.state:
                log.warning("E-Stop %s", state) if state == PRESSED else log.info("E-Stop %s", state)
            self.state = state

    def run(self, stop: threading.Event) -> None:
        buf = b""
        while not stop.is_set():
            try:
                data = self._port.read()
            except Exception as e:  # the E-Stop unplugged: say so, never guess its state
                log.warning("E-Stop lost (%s): its state is not known", e)
                self._set(GONE)
                return
            if not data:
                continue
            states, buf = proto.estop_states(buf + data)
            if states:
                self._set(PRESSED if states[-1] else RELEASED)

    def close(self) -> None:
        self._port.close()
