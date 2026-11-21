"""The wheel's lights in an alarm's colour, over the base's serial port: the rim's LEDs
(``FfbSettings.alarm_leds``) and the base's light bar (``alarm_base_light``).

The decision is ``model.alarm_light_color``: the highest active alarm's colour, steady, else
each target's own colours. These are the devices' stored settings, so a colour is written once
when the alarm changes and once when it ends, never flashed. This holds the port and puts the
lights back:

* Each target's own values are read at start; one that does not answer is left out (a rim
  without LEDs, a base without a light bar). Values a run that died left showing (the settings
  file's ``lights_restore``) are put back first.
* What to put back is saved before an alarm colour is shown, so a crash can be undone.
* While showing, the base bar's standby mode is constant (its colours are that mode's).
* ``restore()`` runs on every exit and never raises. ``sync`` never raises either: it runs on
  its own thread, and a device that stops answering only loses its colours, never the helm.
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable
from dataclasses import replace

from . import model
from . import serial_proto as proto
from .model import FfbSettings

log = logging.getLogger(__name__)

TARGETS = {"rim": proto.RIM_COLORS, "base": proto.BASE_LIGHT}
ALL_NAMES = (*proto.RIM_COLORS, *proto.BASE_LIGHT)
#: How long a Try shows its colour past its rumble pattern.
PREVIEW_TAIL_S = 2.0
#: A target that failed is tried again after this long, not on every tick.
RETRY_S = 10.0


def valid_restore(doc) -> dict[str, int] | None:
    """A settings file's ``lights_restore`` (or older ``rim_restore``), when it is one: known
    lights, sane values. Unknown names are dropped."""
    if not isinstance(doc, dict):
        return None
    kept = {n: v for n, v in doc.items() if n in ALL_NAMES}
    if not kept or not all(isinstance(v, int) and not isinstance(v, bool) and 0 <= v <= 0xFFFFFF for v in kept.values()):
        return None
    return kept


class AlarmLights:
    def __init__(self, port, persist: Callable[[dict | None], None] = lambda _restore: None):
        self._port = port
        self._persist = persist
        self._lock = threading.Lock()
        self._own: dict[str, dict[str, int]] = {}  # target -> name -> its own value
        self._shown: dict[str, int | None] = {}  # target -> the alarm colour showing, None for its own
        self._preview: tuple[int, float] | None = None  # (rank, until)
        self._retry_at: dict[str, float] = {}  # target -> not before
        self._closed = False
        self.status = {"rim": "off", "base": "off"}

    @property
    def targets(self) -> tuple[str, ...]:
        return tuple(self._own)

    def start(self, pending: dict | None) -> tuple[str, ...]:
        """Read each target's own values (putting back ones a dead run left). Returns the
        targets that answer."""
        with self._lock:
            pending = valid_restore(pending) or {}
            for target, names in TARGETS.items():
                try:
                    for n in names:
                        if n in pending:
                            self._port.write(n, pending[n], verify=False)
                    if any(n in pending for n in names):
                        log.warning("The %s still showed an alarm colour from an earlier run: put its own back", target)
                    self._own[target] = {n: self._port.read(n) for n in names}
                    self._shown[target] = None
                    self.status[target] = "ready"
                except Exception as e:  # no such lights: left out
                    log.info("No %s lights over the serial port (%s)", target, e)
                    self._own.pop(target, None)
                    self.status[target] = "off"
            self._persist(None)
            return self.targets

    def preview(self, kind: str, now: float, seconds: float) -> None:
        """Show a kind's colour for a while (the panel's Try), on every target, alarms or not."""
        with self._lock:
            self._preview = (model.ALARM_RANKS[kind], now + seconds)

    def sync(self, settings: FfbSettings, top: int | None, now: float) -> None:
        """Show what ``model.alarm_light_color`` says on each target. Never raises."""
        with self._lock:
            if self._closed:
                return
            if self._preview is not None and now < self._preview[1]:
                settings = replace(settings, alarm_leds=True, alarm_base_light=True)
                top = max(top or 0, self._preview[0])
            else:
                self._preview = None
            kind = model.alarm_kind(top) if top else None
            for target in self.targets:
                color = model.alarm_light_color(settings, top, target)
                if now < self._retry_at.get(target, 0.0):
                    continue
                try:
                    if color != self._shown[target]:
                        if color is None:
                            self._write_own(target)
                        else:
                            if all(c is None for c in self._shown.values()):
                                self._persist(self._all_own())  # saved first: a crash still gets them back
                            self._write_color(target, color)
                        self._shown[target] = color
                    self.status[target] = "ready" if color is None else f"showing {kind}"
                except Exception as e:  # the device may be gone: the helm goes on without colours
                    self.status[target] = f"error: {e}"
                    self._retry_at[target] = now + RETRY_S
                    log.warning("The %s's alarm lights: %s (trying again in %.0f s)", target, e, RETRY_S)
            if all(c is None for c in self._shown.values()):
                self._persist_none_once()

    def _all_own(self) -> dict[str, int]:
        return {n: v for own in self._own.values() for n, v in own.items()}

    def _write_color(self, target: str, color: int) -> None:
        for n in TARGETS[target]:
            if n == "base-standby-mode":
                if self._own[target][n] != proto.CONSTANT_MODE:
                    self._port.write(n, proto.CONSTANT_MODE, verify=False)
            else:
                self._port.write(n, color, verify=False)
        self._persisted_none = False

    def _write_own(self, target: str) -> None:
        own = self._own[target]
        for n in TARGETS[target]:
            if n != "base-standby-mode":
                self._port.write(n, own[n], verify=False)
        if "base-standby-mode" in own and own["base-standby-mode"] != proto.CONSTANT_MODE:
            self._port.write("base-standby-mode", own["base-standby-mode"], verify=False)

    _persisted_none = True

    def _persist_none_once(self) -> None:
        if not self._persisted_none:
            self._persist(None)
            self._persisted_none = True

    def restore(self) -> bool:
        """Put every target's own colours back. Never raises: this runs on every exit path.
        Does not close the port (the hard stop's restore does, after this)."""
        with self._lock:
            if self._closed:
                return True
            self._closed = True
            ok = True
            for target in self.targets:
                if self._shown.get(target) is None:
                    continue
                try:
                    self._write_own(target)
                    self._shown[target] = None
                    log.info("The %s's own colours are back", target)
                except Exception as e:
                    ok = False
                    log.warning("Putting the %s's colours back failed: %s", target, e)
            if ok:
                try:
                    self._persist_none_once()
                except Exception as e:
                    log.debug("Settings after restore: %s", e)
            return ok
