"""Force feedback: the spring (centring or rudder follow-up) and the damper on the base.

The controller owns every effect it uploads and is the only thing that touches the
base's force feedback. ``shutdown`` always leaves the wheel limp: every effect stopped
and erased, the gain at 0 -- also when the base is already gone.
"""

from __future__ import annotations

import logging
import threading

from . import codes, model
from .device import MozaDevice
from .model import FfbMode, FfbSettings, SpringTarget

log = logging.getLogger(__name__)


class FfbController:
    def __init__(self, device: MozaDevice, settings: FfbSettings):
        self._dev = device
        self._s = settings
        self._lock = threading.Lock()
        self._rudder_deg: float | None = None
        self._rudder_at: float | None = None
        self._spring_id: int | None = None
        self._damper_id: int | None = None
        self._spring: SpringTarget | None = None
        self._started_at: float | None = None
        self._gain: int | None = None

    @property
    def active(self) -> bool:
        return self._s.mode is not FfbMode.OFF and self._dev.has_ff()

    def on_rudder(self, rudder_deg: float, received_at: float) -> None:
        """A rudder reading (monotonic receive time). Called from the Zenoh thread."""
        with self._lock:
            self._rudder_deg, self._rudder_at = rudder_deg, received_at

    def start(self, now: float) -> None:
        if self._s.mode is FfbMode.OFF:
            log.info("Force feedback off")
            return
        if not self._dev.has_ff():
            log.warning("%s reports no FF_SPRING: force feedback disabled", self._dev.name)
            return
        self._started_at = now
        # Our spring is the only centring force: the base's own autocentre would fight it.
        self._dev.set_autocenter(0)
        self._set_gain(0)
        if self._s.damper_pct > 0:
            self._damper_id = self._dev.upload_condition(
                codes.FF_DAMPER,
                -1,
                center=0,
                coeff=model.coeff_level(self._s.damper_pct),
                saturation=model.saturation_level(100),
            )
            self._dev.play(self._damper_id, True)
        log.info(
            "Force feedback %s: gain %.0f%% (ramping over %.1fs), spring %.0f%%, damper %.0f%%",
            self._s.mode.value,
            min(self._s.max_gain_pct, model.MAX_GAIN_CEILING_PCT),
            model.GAIN_RAMP_S,
            self._s.spring_pct,
            self._s.damper_pct,
        )
        self.tick(now)

    def _set_gain(self, level: int) -> None:
        if level != self._gain:
            self._dev.set_gain(level)
            self._gain = level

    def tick(self, now: float) -> SpringTarget | None:
        """Bring the base in line with the latest rudder reading. Raises DeviceGone."""
        if self._started_at is None:
            return None
        self._set_gain(model.ramped_gain_level(self._s.max_gain_pct, now - self._started_at))
        with self._lock:
            rudder_deg, rudder_at = self._rudder_deg, self._rudder_at
        target = model.spring_target(self._s, now=now, rudder_deg=rudder_deg, rudder_received_at=rudder_at)
        if not model.needs_update(self._spring, target):
            return self._spring
        if self._spring is None or self._spring.source != target.source:
            log.info("Spring: %s", target.source)
        if target.enabled:
            new = self._spring_id is None
            self._spring_id = self._dev.upload_condition(
                codes.FF_SPRING,
                -1 if new else self._spring_id,
                center=target.center,
                coeff=model.coeff_level(self._s.spring_pct),
                saturation=model.saturation_level(100),
            )
            if new:
                self._dev.play(self._spring_id, True)
        elif self._spring_id is not None:
            self._dev.play(self._spring_id, False)
        self._spring = target
        return target

    def shutdown(self) -> None:
        """Leave the wheel limp. Never raises: this runs on the way out of every exit path."""
        if self._started_at is None:
            return
        self._started_at = None
        for effect_id in (self._spring_id, self._damper_id):
            if effect_id is None:
                continue
            for step in (lambda: self._dev.play(effect_id, False), lambda: self._dev.erase(effect_id)):
                try:
                    step()
                except Exception as e:  # the base may already be gone
                    log.debug("FF cleanup of effect %s: %s", effect_id, e)
        self._spring_id = self._damper_id = None
        self._spring = None
        try:
            self._dev.set_gain(0)
        except Exception as e:
            log.debug("FF cleanup of gain: %s", e)
        self._gain = None
        log.info("Force feedback released")
