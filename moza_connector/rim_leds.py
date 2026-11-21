"""The rim's LEDs in an alarm's colour (``FfbSettings.alarm_leds``), over the base's serial port.

The decision is ``model.rim_led_state``: an alarm level's colour while one is active, flashing
(the brightness to 0 and back) for an emergency, else the operator's own colours. This holds
the port and puts the rim back:

* The rim's own colours and brightness are read at start. Colours a run that died left
  showing (the settings file's ``rim_restore``) are put back first.
* What to put back is saved before an alarm colour is shown, so a crash can be undone.
* ``restore()`` runs on every exit and never raises.
* ``sync`` never raises either: it runs on its own thread, and a rim that stops answering
  only stops the colours (said in ``status``), never the helm.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable

from . import model
from . import serial_proto as proto
from .model import FfbSettings

log = logging.getLogger(__name__)

NAMES = (*proto.RIM_COLORS, *proto.RIM_BRIGHTNESS)
_LEVEL_NAMES = {p: level for level, p in model.ALARM_PRIORITIES.items()}


def valid_restore(doc) -> dict[str, int] | None:
    """A settings file's ``rim_restore``, when it is one: every LED, a sane value."""
    if not isinstance(doc, dict) or set(doc) != set(NAMES):
        return None
    if not all(isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 0xFFFFFF for v in doc.values()):
        return None
    if any(doc[n] > 100 for n in proto.RIM_BRIGHTNESS):
        return None
    return dict(doc)


class RimLeds:
    def __init__(self, port, persist: Callable[[dict | None], None] = lambda _restore: None):
        self._port = port
        self._persist = persist
        self._lock = threading.Lock()
        self._own: dict[str, int] = {}
        self._color: int | None = None  # the alarm colour showing, None for the rim's own
        self._dark = False
        self._closed = False
        self.status = "off"

    def start(self, pending: dict | None) -> None:
        """Read the rim's own colours (putting back ones a dead run left). Raises when the rim
        does not answer (no rim, or one without these LEDs)."""
        with self._lock:
            pending = valid_restore(pending)
            if pending:
                log.warning("The rim still shows an alarm colour from an earlier run: putting its own back")
                for n in NAMES:
                    self._port.write(n, pending[n], verify=False)
            self._own = {n: self._port.read(n) for n in NAMES}
            self._persist(None)
            self.status = "ready"

    def sync(self, settings: FfbSettings, top: int | None, now: float) -> None:
        """Show what ``model.rim_led_state`` says. Never raises."""
        with self._lock:
            if self._closed or not self._own:
                return
            color, dark = model.rim_led_state(settings, top, now)
            try:
                if color != self._color:
                    if color is None:
                        self._write_own()
                    else:
                        if self._color is None:
                            self._persist(dict(self._own))  # saved first: a crash still gets them back
                        for n in proto.RIM_COLORS:
                            self._port.write(n, color, verify=False)
                        if self._dark != dark or self._color is None:
                            self._write_brightness(dark)
                    self._color = color
                elif color is not None and dark != self._dark:
                    self._write_brightness(dark)
                if color is None:
                    self.status = "ready"
                else:
                    self.status = f"showing {_LEVEL_NAMES.get(top, 'an alarm')}"
            except Exception as e:  # the rim may be gone: the helm goes on without colours
                self.status = f"error: {e}"
                log.warning("The rim's LEDs: %s", e)

    def _write_brightness(self, dark: bool) -> None:
        for n in proto.RIM_BRIGHTNESS:
            self._port.write(n, 0 if dark else self._own[n], verify=False)
        self._dark = dark

    def _write_own(self) -> None:
        for n in NAMES:
            self._port.write(n, self._own[n], verify=False)
        self._dark = False
        self._persist(None)

    def restore(self) -> bool:
        """Put the rim's own colours back. Never raises: this runs on every exit path. Does not
        close the port (the hard stop's restore does, after this)."""
        with self._lock:
            if self._closed:
                return True
            self._closed = True
            if self._color is None and not self._dark:
                return True
            try:
                self._write_own()
                self._color = None
                log.info("The rim's own colours are back")
                return True
            except Exception as e:
                log.warning("Putting the rim's colours back failed: %s", e)
                return False
