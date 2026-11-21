"""Force feedback: the spring (centring or rudder follow-up), the damper, the end stop and the
short pulses (detent click, centre mark, hard-over warning, alarm rumble) on the base.

The controller owns every effect it uploads and is the only thing that touches the
base's force feedback. ``shutdown`` always leaves the wheel limp: every effect stopped
and erased, the gain at 0 -- also when the base is already gone.

With ``--ffb-remote`` the settings can change while the connector runs (``update``,
called from a Zenoh thread). One lock serialises every operation, so the 20 Hz tick
and a remote change never interleave their writes to the base. Switching on ramps the
gain up from 0; raising the gain ramps from the current level; lowering it is
immediate; switching off releases the wheel exactly as ``shutdown`` does.

Every input with an age -- the rudder, the speed over ground -- is used only while fresh:
a stale rudder means centring and no lag resistance, a stale speed means full weight.
"""

from __future__ import annotations

import logging
import threading

from . import codes, model
from .device import MozaDevice
from .model import FfbMode, FfbSettings, SpringTarget

log = logging.getLogger(__name__)

#: The short pulses, each its own click effect: (duration ms, square period ms).
PULSES = {
    "detent": (model.CLICK_DURATION_MS, model.CLICK_PERIOD_MS),
    "center": (model.CENTER_MARK_MS, model.CLICK_PERIOD_MS),
    "hardover": (model.HARDOVER_MS, model.CLICK_PERIOD_MS),
    "rumble": (model.RUMBLE_MS, model.RUMBLE_PERIOD_MS),
}


class FfbController:
    def __init__(self, device: MozaDevice, settings: FfbSettings):
        self._dev = device
        self._s = settings
        self._lock = threading.Lock()  # the inputs from the bus: rudder, speed
        self._op = threading.RLock()  # every operation on the base
        self._rudder_deg: float | None = None
        self._rudder_at: float | None = None
        self._sog_kn: float | None = None
        self._sog_at: float | None = None
        self._spring_ids: list[int] = []  # one per layer (spring_boost)
        self._spring_coeff: int | None = None
        self._spring_playing = False
        self._weight = 1.0  # the speed weight last applied
        self._damper_id: int | None = None
        self._damper_level: int | None = None
        self._endstop_id: int | None = None
        self._pulses: dict[str, tuple[int, int]] = {}  # name -> (effect id, level)
        self._notch: int | None = None  # the detent the wheel's rudder order sits in
        self._center_side: int | None = None  # -1 port of centre, 0 at it, +1 starboard
        self._hardover_side: int | None = None  # +-1 in a hard-over warning zone, else 0
        self._rumble: tuple[float, int] | None = None  # (started at, pulses played)
        self._alarms_felt: set[str] = set()
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
        """What the spring is doing: ``center``, ``rudder``, ``free`` or ``off``."""
        spring = self._spring
        return spring.source if spring is not None and self.running else "off"

    # --- Inputs from the bus (Zenoh threads) ------------------------------------------

    def on_rudder(self, rudder_deg: float, received_at: float) -> None:
        """A rudder reading (monotonic receive time)."""
        with self._lock:
            self._rudder_deg, self._rudder_at = rudder_deg, received_at

    def on_speed(self, sog_kn: float, received_at: float) -> None:
        """Speed over ground (monotonic receive time), for the speed-sensitive weight."""
        with self._lock:
            self._sog_kn, self._sog_at = sog_kn, received_at

    def on_alarm(self, identifier: str, priority: int, acknowledged: bool, now: float) -> bool:
        """An alarm. A new one that should be felt starts a rumble (played by the tick);
        acknowledged, it may rumble again if it is raised again. Returns whether it starts."""
        with self._op:
            if acknowledged:
                self._alarms_felt.discard(identifier)
                return False
            if not model.alarm_rumbles(self._s, priority, acknowledged) or identifier in self._alarms_felt:
                return False
            self._alarms_felt.add(identifier)
            if self._started_at is None or "rumble" not in self._pulses:
                return False
            self._rumble = (now, 0)
            log.info("Alarm %s: rumble", identifier)
            return True

    # --- Switching on -------------------------------------------------------------------

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
            self._sync_damper(None, force=True)
            self._sync_endstop()
            self._sync_pulses()
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

    # --- The effects ----------------------------------------------------------------------

    def _sync_damper(self, lag_deg: float | None, *, force: bool = False) -> None:
        """The damper: the set damper times the speed weight, plus the lag resistance.
        Present while either is wanted; re-uploaded only on a real change."""
        wanted = self._s.damper_pct > 0 or self._s.lag_resistance_pct > 0
        if not wanted:
            if self._damper_id is not None:
                self._dev.play(self._damper_id, False)
                self._dev.erase(self._damper_id)
                self._damper_id = self._damper_level = None
            return
        level = model.coeff_level(model.damper_pct_now(self._s, self._weight, lag_deg))
        if self._damper_id is not None and self._damper_level is not None:
            if level == self._damper_level:
                return
            # A small move is not worth a USB transfer; reaching or leaving 0 always is.
            small = abs(level - self._damper_level) < round(model.WEIGHT_STEP * model.S16_MAX)
            if not force and small and (level == 0) == (self._damper_level == 0):
                return
        new = self._damper_id is None
        self._damper_id = self._dev.upload_condition(
            codes.FF_DAMPER,
            -1 if new else self._damper_id,
            center=0,
            coeff=level,
            saturation=model.saturation_level(100),
        )
        self._damper_level = level
        if new:
            self._dev.play(self._damper_id, True)

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

    def _sync_pulse(self, name: str, wanted: bool, level: int) -> None:
        """One short pulse effect, uploaded while wanted and removed otherwise. A base that
        cannot (no square effect, its effect pool full) goes without it, said once."""
        current = self._pulses.get(name)
        if wanted and level > 0:
            if current is not None and current[1] == level:
                return
            duration_ms, period_ms = PULSES[name]
            try:
                effect_id = self._dev.upload_click(
                    -1 if current is None else current[0],
                    magnitude=level,
                    period_ms=period_ms,
                    duration_ms=duration_ms,
                )
            except (NotImplementedError, RuntimeError) as e:
                log.warning("No %s pulse: %s", name, e)
                return
            self._pulses[name] = (effect_id, level)
        elif current is not None:
            self._dev.play(current[0], False)
            self._dev.erase(current[0])
            del self._pulses[name]

    def _sync_pulses(self) -> None:
        s = self._s
        self._sync_pulse("detent", s.mode is FfbMode.DETENT, model.click_level(s))
        self._sync_pulse("center", True, model.center_mark_level(s))
        self._sync_pulse("hardover", s.hardover_warn_deg > 0, model.coeff_level(s.hardover_pct))
        self._sync_pulse("rumble", s.alarm_rumble_pct > 0, model.coeff_level(s.alarm_rumble_pct))
        if "rumble" not in self._pulses:
            self._rumble = None
        # Count notches and warning zones from where the wheel is now: no pulse for a setting.
        self._notch = self._hardover_side = None
        if self._wheel_pct is not None:
            order = model.rudder_order_deg(self._wheel_pct, s)
            self._notch = model.detent_notch(order, None, s.detent_deg)
            self._hardover_side = model.hardover_side(order, None, s.hardover_warn_deg)

    def _pulse(self, name: str) -> bool:
        current = self._pulses.get(name)
        if current is None:
            return False
        self._dev.play(current[0], True)
        return True

    # --- The wheel (main thread) ----------------------------------------------------------

    def on_wheel(self, wheel_pct: float, now: float) -> bool:
        """The wheel's published position. One pulse at most, the most important first:
        the centre mark on reaching centre, the hard-over warning on entering its zone, a
        detent click (``detent`` mode) on passing into another notch. Returns whether
        anything played. Raises DeviceGone."""
        with self._op:
            self._wheel_pct = wheel_pct
            s = self._s
            side = model.center_side(model.wheel_deg(wheel_pct, s), self._center_side)
            at_center = model.center_reached(self._center_side, side)
            self._center_side = side
            order = model.rudder_order_deg(wheel_pct, s)
            zone = model.hardover_side(order, self._hardover_side, s.hardover_warn_deg)
            at_hardover = model.hardover_reached(self._hardover_side, zone)
            self._hardover_side = zone
            notch = model.detent_notch(order, self._notch, s.detent_deg)
            at_notch = self._notch is not None and notch != self._notch
            self._notch = notch
            if self._started_at is None:
                return False
            if at_center and self._pulse("center"):
                return True
            if at_hardover and self._pulse("hardover"):
                return True
            return s.mode is FfbMode.DETENT and at_notch and self._pulse("detent")

    # --- The tick (20 Hz) -------------------------------------------------------------------

    def _set_gain(self, level: int) -> None:
        if level != self._gain:
            self._dev.set_gain(level)
            self._gain = level

    def tick(self, now: float) -> SpringTarget | None:
        """Bring the base in line with the latest settings and inputs. Raises DeviceGone."""
        with self._op:
            return self._tick(now)

    def _tick(self, now: float) -> SpringTarget | None:
        if self._started_at is None:
            return None
        s = self._s
        target_gain = model.gain_level(s.max_gain_pct)
        ramp_at = now if self._ramp_at is None else self._ramp_at
        self._set_gain(model.gain_ramp_level(self._gain_from, target_gain, now - ramp_at))
        with self._lock:
            rudder_deg, rudder_at = self._rudder_deg, self._rudder_at
            sog, sog_at = self._sog_kn, self._sog_at

        weight = model.speed_weight(s, sog, None if sog_at is None else now - sog_at)
        if abs(weight - self._weight) > model.WEIGHT_STEP or (weight == 1.0) != (self._weight == 1.0):
            self._weight = weight

        lag = None
        if self._wheel_pct is not None and rudder_at is not None and 0.0 <= now - rudder_at <= s.stale_s:
            lag = model.rudder_order_deg(self._wheel_pct, s) - rudder_deg
        self._sync_damper(lag)

        if self._rumble is not None:
            started, played = self._rumble
            if model.rumble_due(started, played, now):
                self._pulse("rumble")
                played += 1
            self._rumble = None if played >= model.RUMBLE_COUNT else (started, played)

        target = model.spring_target(s, now=now, rudder_deg=rudder_deg, rudder_received_at=rudder_at)
        coeff = model.coeff_level(s.spring_pct * self._weight)
        layers = model.spring_layers(s)
        same_shape = coeff == self._spring_coeff and layers == len(self._spring_ids)
        if not model.needs_update(self._spring, target) and (not target.enabled or same_shape):
            return self._spring
        if self._spring is None or self._spring.source != target.source:
            log.info("Spring: %s", target.source)
        if target.enabled:
            # Fewer layers: stop and free the extra ones first.
            while len(self._spring_ids) > layers:
                extra = self._spring_ids.pop()
                self._dev.play(extra, False)
                self._dev.erase(extra)
            # Every layer the same spring, each with its share of the saturation: the pull
            # near centre is `layers` times steeper, the pull far from it unchanged.
            saturation = model.saturation_level(100.0 / layers)
            for i in range(layers):
                existing = self._spring_ids[i] if i < len(self._spring_ids) else None
                effect_id = self._dev.upload_condition(
                    codes.FF_SPRING,
                    -1 if existing is None else existing,
                    center=target.center,
                    coeff=coeff,
                    saturation=saturation,
                )
                if existing is None:
                    self._spring_ids.append(effect_id)
                    if self._spring_playing:
                        self._dev.play(effect_id, True)
            self._spring_coeff = coeff
            if not self._spring_playing:
                for effect_id in self._spring_ids:
                    self._dev.play(effect_id, True)
                self._spring_playing = True
        elif self._spring_ids and self._spring_playing:
            for effect_id in self._spring_ids:
                self._dev.play(effect_id, False)
            self._spring_playing = False
        self._spring = target
        return target

    # --- Remote changes -----------------------------------------------------------------------

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
            self._sync_pulses()
            self._sync_damper(None, force=True)
            self._tick(now)

    # --- Letting go ---------------------------------------------------------------------------

    def _release(self) -> None:
        """Every effect stopped and erased, the gain at 0. Never raises."""
        self._started_at = None
        effects = (*self._spring_ids, self._damper_id, self._endstop_id, *(p[0] for p in self._pulses.values()))
        for effect_id in effects:
            if effect_id is None:
                continue
            for step in (lambda: self._dev.play(effect_id, False), lambda: self._dev.erase(effect_id)):
                try:
                    step()
                except Exception as e:  # the base may already be gone
                    log.debug("FF cleanup of effect %s: %s", effect_id, e)
        self._spring_ids = []
        self._damper_id = self._damper_level = self._endstop_id = None
        self._pulses = {}
        self._spring_playing = False
        self._weight = 1.0
        self._notch = self._hardover_side = None
        self._rumble = None
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
