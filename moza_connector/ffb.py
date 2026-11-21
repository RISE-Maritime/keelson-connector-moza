"""Force feedback: the spring (centring or rudder follow-up) and the damper on the base.

The controller owns every effect it uploads and is the only thing that touches the
base's force feedback. ``shutdown`` always leaves the wheel limp: every effect stopped
and erased, the gain at 0 -- also when the base is already gone.

With ``--ffb-remote`` the settings can change while the connector runs (``update``,
called from a Zenoh thread). One lock serialises every operation, so the 20 Hz tick
and a remote change never interleave their writes to the base. Switching on ramps the
gain up from 0; raising the gain ramps from the current level; lowering it is
immediate; switching off releases the wheel exactly as ``shutdown`` does.
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
        self._lock = threading.Lock()  # the rudder reading
        self._op = threading.RLock()  # every operation on the base
        self._rudder_deg: float | None = None
        self._rudder_at: float | None = None
        self._spring_id: int | None = None
        self._spring_coeff: int | None = None
        self._damper_id: int | None = None
        self._endstop_id: int | None = None
        self._spring_playing = False
        self._click_id: int | None = None
        self._click_level: int | None = None
        self._notch: int | None = None  # the detent the wheel's rudder order sits in
        self._center_id: int | None = None
        self._center_level: int | None = None
        self._center_side: int | None = None  # -1 port of centre, 0 at it, +1 starboard
        self._wheel_pct: float | None = None
        self._spring: SpringTarget | None = None
        self._started_at: float | None = None
        self._gain: int | None = None
        self._gain_from = 0  # where the current gain ramp started ...
        self._ramp_at: float | None = None  # ... and when
        self._closed = False  # after shutdown nothing may switch the wheel back on

    @property
    def settings(self) -> FfbSettings:
        return self._s

    @property
    def active(self) -> bool:
        return self._s.mode is not FfbMode.OFF and self._dev.has_ff()

    @property
    def running(self) -> bool:
        """Force feedback is on the wheel (started and not released)."""
        return self._started_at is not None

    @property
    def spring_source(self) -> str:
        """What the spring is doing: ``center``, ``rudder`` or ``off``."""
        spring = self._spring
        return spring.source if spring is not None and self.running else "off"

    def on_rudder(self, rudder_deg: float, received_at: float) -> None:
        """A rudder reading (monotonic receive time). Called from the Zenoh thread."""
        with self._lock:
            self._rudder_deg, self._rudder_at = rudder_deg, received_at

    def start(self, now: float) -> None:
        with self._op:
            if self._s.mode is FfbMode.OFF:
                log.info("Force feedback off")
                return
            if not self._dev.has_ff():
                log.warning("%s reports no FF_SPRING: force feedback disabled", self._dev.name)
                return
            if self._started_at is not None:
                return
            self._started_at = now
            self._gain_from, self._ramp_at = 0, now
            # Our spring is the only centring force: the base's own autocentre would fight it.
            self._dev.set_autocenter(0)
            self._set_gain(0)
            if self._s.damper_pct > 0:
                self._damper_id = self._upload_damper(-1)
                self._dev.play(self._damper_id, True)
            self._sync_endstop()
            self._sync_click()
            self._sync_center()
            log.info(
                "Force feedback %s: gain %.0f%% (ramping over %.1fs), spring %.0f%%, damper %.0f%%, "
                "full lock at %.0f deg of the base's %.0f, end stop %s",
                self._s.mode.value,
                min(self._s.max_gain_pct, model.MAX_GAIN_CEILING_PCT),
                model.GAIN_RAMP_S,
                self._s.spring_pct,
                self._s.damper_pct,
                self._s.wheel_lock_deg,
                self._s.base_range_deg,
                f"{self._s.endstop_pct:.0f}%" if model.endstop_enabled(self._s) else "none",
            )
            self._tick(now)

    def _upload_damper(self, effect_id: int) -> int:
        return self._dev.upload_condition(
            codes.FF_DAMPER,
            effect_id,
            center=0,
            coeff=model.coeff_level(self._s.damper_pct),
            saturation=model.saturation_level(100),
        )

    def _sync_endstop(self) -> None:
        """The end stop: a stiff spring with a dead band as wide as full helm lock, so it
        pushes only past it. Uploaded, updated or removed to match the settings."""
        if model.endstop_enabled(self._s):
            new = self._endstop_id is None
            self._endstop_id = self._dev.upload_condition(
                codes.FF_SPRING,
                -1 if new else self._endstop_id,
                center=0,
                coeff=model.coeff_level(self._s.endstop_pct),
                saturation=model.saturation_level(100),
                deadband=model.endstop_deadband(self._s),
            )
            if new:
                self._dev.play(self._endstop_id, True)
        elif self._endstop_id is not None:
            self._dev.play(self._endstop_id, False)
            self._dev.erase(self._endstop_id)
            self._endstop_id = None

    def _sync_click(self) -> None:
        """The detent click: uploaded while ``detent`` mode wants one, removed otherwise."""
        want = self._s.mode is FfbMode.DETENT and self._s.detent_pct > 0
        if want:
            level = model.click_level(self._s)
            if level != self._click_level:
                try:
                    self._click_id = self._dev.upload_click(
                        -1 if self._click_id is None else self._click_id,
                        magnitude=level,
                        period_ms=model.CLICK_PERIOD_MS,
                        duration_ms=model.CLICK_DURATION_MS,
                    )
                    self._click_level = level
                except NotImplementedError as e:
                    log.warning("No detent clicks: %s", e)
            # Start counting notches from where the wheel is now: no click for switching on.
            self._notch = None
            if self._wheel_pct is not None:
                self._notch = model.detent_notch(
                    model.rudder_order_deg(self._wheel_pct, self._s), None, self._s.detent_deg
                )
        elif self._click_id is not None:
            self._dev.play(self._click_id, False)
            self._dev.erase(self._click_id)
            self._click_id = self._click_level = None

    def _sync_center(self) -> None:
        """The centre mark: uploaded while force feedback is on and it is wanted."""
        if self._started_at is not None and self._s.center_mark_pct > 0:
            level = model.center_mark_level(self._s)
            if level != self._center_level:
                try:
                    self._center_id = self._dev.upload_click(
                        -1 if self._center_id is None else self._center_id,
                        magnitude=level,
                        period_ms=model.CLICK_PERIOD_MS,
                        duration_ms=model.CENTER_MARK_MS,
                    )
                    self._center_level = level
                except NotImplementedError as e:
                    log.warning("No centre mark: %s", e)
        elif self._center_id is not None:
            self._dev.play(self._center_id, False)
            self._dev.erase(self._center_id)
            self._center_id = self._center_level = None

    def on_wheel(self, wheel_pct: float, now: float) -> bool:
        """The wheel's published position. A centre mark each time it reaches centre, and in
        ``detent`` mode a click each time its rudder order passes into another notch (not
        both at once: centre wins). Returns whether anything played. Raises DeviceGone."""
        with self._op:
            self._wheel_pct = wheel_pct
            side = model.center_side(model.wheel_deg(wheel_pct, self._s), self._center_side)
            at_center = model.center_reached(self._center_side, side)
            self._center_side = side
            if self._started_at is None:
                return False
            played = False
            if at_center and self._center_id is not None:
                self._dev.play(self._center_id, True)
                played = True
            if self._s.mode is FfbMode.DETENT:
                rudder = model.rudder_order_deg(wheel_pct, self._s)
                notch = model.detent_notch(rudder, self._notch, self._s.detent_deg)
                moved = self._notch is not None and notch != self._notch
                self._notch = notch
                if moved and not played and self._click_id is not None:
                    self._dev.play(self._click_id, True)
                    played = True
            return played

    def _set_gain(self, level: int) -> None:
        if level != self._gain:
            self._dev.set_gain(level)
            self._gain = level

    def tick(self, now: float) -> SpringTarget | None:
        """Bring the base in line with the latest settings and rudder reading. Raises DeviceGone."""
        with self._op:
            return self._tick(now)

    def _tick(self, now: float) -> SpringTarget | None:
        if self._started_at is None:
            return None
        target_gain = model.gain_level(self._s.max_gain_pct)
        ramp_at = now if self._ramp_at is None else self._ramp_at
        self._set_gain(model.gain_ramp_level(self._gain_from, target_gain, now - ramp_at))
        with self._lock:
            rudder_deg, rudder_at = self._rudder_deg, self._rudder_at
        target = model.spring_target(self._s, now=now, rudder_deg=rudder_deg, rudder_received_at=rudder_at)
        coeff = model.coeff_level(self._s.spring_pct)
        if not model.needs_update(self._spring, target) and (not target.enabled or coeff == self._spring_coeff):
            return self._spring
        if self._spring is None or self._spring.source != target.source:
            log.info("Spring: %s", target.source)
        if target.enabled:
            new = self._spring_id is None
            self._spring_id = self._dev.upload_condition(
                codes.FF_SPRING,
                -1 if new else self._spring_id,
                center=target.center,
                coeff=coeff,
                saturation=model.saturation_level(100),
            )
            self._spring_coeff = coeff
            if not self._spring_playing:
                self._dev.play(self._spring_id, True)
                self._spring_playing = True
        elif self._spring_id is not None and self._spring_playing:
            self._dev.play(self._spring_id, False)
            self._spring_playing = False
        self._spring = target
        return target

    def update(self, settings: FfbSettings, now: float) -> None:
        """Apply new settings while running (remote control). Raises DeviceGone."""
        with self._op:
            if self._closed:
                raise RuntimeError("force feedback is shutting down")
            old, self._s = self._s, settings
            if settings.mode is FfbMode.OFF or not self._dev.has_ff():
                if self._started_at is not None:
                    self._release()
                    log.info("Force feedback off (remote)")
                return
            if self._started_at is None:
                self.start(now)
                return
            if settings.mode is not old.mode:
                log.info("Force feedback %s (remote)", settings.mode.value)
            if settings.max_gain_pct != old.max_gain_pct:
                # Ramp from where the gain is now; a lower target applies at once.
                self._gain_from, self._ramp_at = self._gain or 0, now
                log.info("Force feedback gain %.0f%% (remote)", settings.max_gain_pct)
            if settings.lock_fraction != old.lock_fraction:
                # The end stop and the follow-up centre move: if the wheel is now past the
                # new lock it gets pushed, so the force fades in again from 0.
                self._gain_from, self._ramp_at = 0, now
                log.info(
                    "Full lock at %.0f deg of the base's %.0f (remote): force ramps in again",
                    settings.wheel_lock_deg,
                    settings.base_range_deg,
                )
            if settings.lock_fraction != old.lock_fraction or settings.endstop_pct != old.endstop_pct:
                self._sync_endstop()
            if (settings.mode, settings.detent_pct, settings.detent_deg, settings.rudder_max_deg) != (
                old.mode,
                old.detent_pct,
                old.detent_deg,
                old.rudder_max_deg,
            ):
                self._sync_click()
            if settings.center_mark_pct != old.center_mark_pct:
                self._sync_center()
            if settings.damper_pct != old.damper_pct:
                if settings.damper_pct > 0 and self._damper_id is not None:
                    self._upload_damper(self._damper_id)
                elif settings.damper_pct > 0:
                    self._damper_id = self._upload_damper(-1)
                    self._dev.play(self._damper_id, True)
                elif self._damper_id is not None:
                    self._dev.play(self._damper_id, False)
                    self._dev.erase(self._damper_id)
                    self._damper_id = None
            self._tick(now)

    def _release(self) -> None:
        """Every effect stopped and erased, the gain at 0. Never raises."""
        self._started_at = None
        for effect_id in (self._spring_id, self._damper_id, self._endstop_id, self._click_id, self._center_id):
            if effect_id is None:
                continue
            for step in (lambda: self._dev.play(effect_id, False), lambda: self._dev.erase(effect_id)):
                try:
                    step()
                except Exception as e:  # the base may already be gone
                    log.debug("FF cleanup of effect %s: %s", effect_id, e)
        self._spring_id = self._damper_id = self._endstop_id = self._click_id = self._center_id = None
        self._center_level = None
        self._spring_playing = False
        self._click_level = None
        self._notch = None
        self._spring = None
        self._spring_coeff = None
        try:
            self._dev.set_gain(0)
        except Exception as e:
            log.debug("FF cleanup of gain: %s", e)
        self._gain = None

    def shutdown(self) -> None:
        """Leave the wheel limp. Never raises: this runs on the way out of every exit path."""
        with self._op:
            self._closed = True
            if self._started_at is None:
                return
            self._release()
            log.info("Force feedback released")
