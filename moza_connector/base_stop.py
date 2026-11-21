"""The hard end stop: the base's own steering angle, over its serial port.

While the hard stop is on, the base's steering angle is set to the helm's lock (twice
``wheel_lock_deg``), so the base's own stop, far stiffer than any force-feedback spring and
not limited by the gain, sits at full lock. The decisions are ``model.base_range_for`` and
``model.base_stop_decision``; this holds the port, applies them, and puts the base back:

* The base's own angle is read at start. A narrower angle left by a run that died (the
  settings file's ``base_restore``) is put back first.
* A narrower angle is set only while the wheel is inside it; wider at once.
* ``restore()`` puts the angle read at start back. It runs on every exit and never raises.
* ``persist`` is told what to put back whenever that changes, so a crash can be undone.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable

from . import model
from . import serial_proto as proto
from .base_serial import BaseSerialError
from .model import FfbSettings

log = logging.getLogger(__name__)

#: The base's angle is both of these (Pit House sets both); both are set and put back.
COMMANDS = ("limit", "max-angle")
#: The base's value is degrees either side of centre: these bound a saved restore.
_VALUE_RANGE = (45, 1800)


def valid_restore(doc) -> dict[str, int] | None:
    """A settings file's ``base_restore``, when it is one: every command, a sane value."""
    if not isinstance(doc, dict) or set(doc) != set(COMMANDS):
        return None
    lo, hi = _VALUE_RANGE
    if not all(isinstance(v, int) and not isinstance(v, bool) and lo <= v <= hi for v in doc.values()):
        return None
    return dict(doc)


class BaseStop:
    def __init__(self, port, persist: Callable[[dict | None], None] = lambda _restore: None):
        self._port = port
        self._persist = persist
        self._lock = threading.Lock()
        self._original: dict[str, int] = {}
        self._current: dict[str, int] = {}
        self._closed = False
        self.status = "starting"

    # --- At start -------------------------------------------------------------------------

    def start(self, pending: dict | None) -> tuple[float, float]:
        """Read the base's angle (putting back one a dead run left narrower). Returns the
        base's own angle and its angle now, lock to lock. Raises BaseSerialError."""
        with self._lock:
            values = {n: self._port.read(n) for n in COMMANDS}
            pending = valid_restore(pending)
            if pending and any(pending[n] > values[n] for n in COMMANDS):
                log.warning("The base's steering angle is still narrowed by an earlier run: putting %s back", pending)
                for n in COMMANDS:
                    self._port.write(n, pending[n])
                values = dict(pending)
            self._original = dict(values)
            self._current = dict(values)
            self._persist(None)
            own = proto.range_deg(values["limit"])
            self.status = f"base at {own:g}°"
            log.info("The base's steering angle: %.0f deg lock to lock (over %s)", own, getattr(self._port, "path", "serial"))
            return own, own

    @property
    def current_deg(self) -> float:
        return proto.range_deg(self._current["limit"])

    @property
    def pending_restore(self) -> dict | None:
        """What to put back after a crash: the angle read at start, while the base is narrower."""
        if any(self._current.get(n, 0) < self._original.get(n, 0) for n in COMMANDS):
            return dict(self._original)
        return None

    # --- While running ----------------------------------------------------------------------

    def sync(self, settings: FfbSettings, wheel_deg: float | None) -> float | None:
        """Bring the base's angle in line with the settings. Returns the base's new angle (lock
        to lock) when it changed, else None."""
        with self._lock:
            if self._closed or not self._current:
                return None
            target = model.base_range_for(settings)
            action, why = model.base_stop_decision(target, self.current_deg, wheel_deg)
            if action == "wait":
                self.status = f"waiting: {why} for the hard stop at ±{target / 2:g}°"
                return None
            if action == "none":
                self._status(settings)
                return None
            value = proto.angle_value(target)
            narrower = target < self.current_deg
            try:
                if narrower:
                    # Saved first: a crash between the writes still gets the angle put back.
                    self._persist(dict(self._original))
                for n in COMMANDS:
                    self._current[n] = self._port.write(n, value)
            except BaseSerialError as e:
                self.status = f"error: {e}"
                log.warning("Setting the base's steering angle to %.0f deg failed: %s", target, e)
                return None
            log.info("The base's steering angle: %.0f deg lock to lock (hard stop %s)", target, "on" if settings.hard_stop else "off")
            self._persist(self.pending_restore)
            self._status(settings)
            return target

    def _status(self, settings: FfbSettings) -> None:
        if settings.hard_stop:
            self.status = f"hard stop at ±{self.current_deg / 2:g}°"
        else:
            self.status = f"base at {self.current_deg:g}°"

    # --- Letting go -------------------------------------------------------------------------

    def restore(self) -> bool:
        """Put the angle read at start back and close the port. Never raises: this runs on
        the way out of every exit path. Returns whether the base is as it was."""
        with self._lock:
            if self._closed:
                return True
            self._closed = True
            ok = True
            if self.pending_restore:
                for n in COMMANDS:
                    try:
                        self._current[n] = self._port.write(n, self._original[n])
                    except Exception as e:  # the base may be gone: the next start puts it back
                        ok = False
                        log.warning("Putting the base's %s back failed: %s", n, e)
                if ok:
                    log.info("The base's steering angle is back at %.0f deg", proto.range_deg(self._original["limit"]))
                    try:
                        self._persist(None)
                    except Exception as e:
                        log.debug("Settings after restore: %s", e)
            try:
                self._port.close()
            except Exception as e:
                log.debug("Closing the base's serial port: %s", e)
            return ok
