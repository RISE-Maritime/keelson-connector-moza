"""Force feedback: the spring (centring or rudder follow-up), the damper, the end stop, the
centre notch and the short pulses (detent click, centre mark, hard-over warning, alarm rumble)
on the base.

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
from typing import NamedTuple

from . import codes, model
from .device import MozaDevice
from .model import FfbMode, FfbSettings, SpringTarget

log = logging.getLogger(__name__)

#: The short pulses, each its own click effect: (duration ms, square period ms).
PULSES = {
    "detent": (model.CLICK_DURATION_MS, model.CLICK_PERIOD_MS),
    "center": (model.CENTER_MARK_MS, model.CLICK_PERIOD_MS),
    "hardover": (model.HARDOVER_MS, model.CLICK_PERIOD_MS),
    "rumble": (model.SEGMENT_MAX_MS, model.RUMBLE_PERIOD_MS),  # re-uploaded per play, at its length
}


class _Rumble(NamedTuple):
    kind: str
    schedule: list  # [(start ms, duration ms)]
    started: float
    index: int  # the next play
    period: float | None  # seconds to the next round while the alarm lasts, None for once
    once: bool  # a Try


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
        # The centre notch: a constant force, played again only by the loop that reads the wheel.
        self._cnotch_id: int | None = None
        self._cnotch_level = 0  # the magnitude last written
        self._cnotch_playing = False
        self._cnotch_written_at = float("-inf")
        self._cnotch_played_at = float("-inf")
        self._cnotch_unavailable = False  # the base has no constant force: said once
        self._center_side: int | None = None  # -1 port of centre, 0 at it, +1 starboard
        self._hardover_side: int | None = None  # +-1 in a hard-over warning zone, else 0
        # The rumble playing (an alarm's pattern), or None.
        self._rumble: _Rumble | None = None
        self._alarms: dict = {}  # identifier -> model.AlarmSeen
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

    def on_alarm(self, identifier: str, priority: int, acknowledged: bool, now: float,
                 expires_at: float | None = None, category: int = 0) -> bool:
        """An alarm. A new (or newly higher) unacknowledged one plays its kind's pattern at its
        kind's strength (fire for keelson's fire category, whatever the level), unless a
        higher kind is playing; acknowledged, it is over (and may play again if raised again).
        Returns whether a rumble starts."""
        with self._op:
            rank = model.alarm_rank(priority, category)
            self._alarms, fresh = model.alarm_seen(self._alarms, identifier, rank, acknowledged, now, expires_at)
            kind = model.alarm_kind(rank) if priority >= model.ALARM_PRIORITIES["caution"] else None
            if not fresh or kind is None or model.alarm_pct(self._s, kind) <= 0 or self._started_at is None:
                return False
            if self._rumble is not None and model.ALARM_RANKS[self._rumble.kind] > rank:
                return False
            if not self._start_rumble(kind, now):
                return False
            log.info("Alarm %s (%s): rumble", identifier, kind)
            return True

    def top_alarm(self, now: float) -> int | None:
        """The highest rank of an active alarm (for the lights' colour), or None."""
        with self._op:
            return model.alarm_top(self._alarms, now)

    def _start_rumble(self, kind: str, now: float, once: bool = False) -> bool:
        """Start a kind's pattern; ``once`` (a Try) plays it once, whatever it would repeat."""
        if model.alarm_pct(self._s, kind) <= 0:
            return False
        self._sync_pulse("rumble", True, model.coeff_level(model.alarm_pct(self._s, kind)))
        if "rumble" not in self._pulses:
            return False
        schedule, continuous = model.alarm_schedule(self._s, kind)
        period = None if once else model.pattern_period_s(self._s, kind, schedule, continuous)
        self._rumble = _Rumble(kind, schedule, now, 0, period, once)
        self._tick(now)
        return True

    def _play_segment(self, kind: str, duration_ms: int) -> None:
        """One play of the rumble: re-uploaded at this length (at most a lease, never renewed)
        and this kind's strength."""
        current = self._pulses.get("rumble")
        if current is None:
            return
        level = model.coeff_level(model.alarm_pct(self._s, kind))
        try:
            effect_id = self._dev.upload_click(
                current[0], magnitude=level, period_ms=model.RUMBLE_PERIOD_MS,
                duration_ms=min(int(duration_ms), model.SEGMENT_MAX_MS),
            )
        except (NotImplementedError, RuntimeError) as e:
            log.warning("No rumble: %s", e)
            return
        self._pulses["rumble"] = (effect_id, level)
        self._dev.play(effect_id, True)

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
        if self._rumble is not None and model.alarm_pct(s, self._rumble.kind) <= 0:
            self._rumble = None
        pcts = [model.alarm_pct(s, kind) for kind in model.ALARM_KINDS]
        playing = model.alarm_pct(s, self._rumble.kind) if self._rumble is not None else max(pcts)
        self._sync_pulse("rumble", max(pcts) > 0, model.coeff_level(playing))
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

    def test_pulse(self, name: str, now: float) -> None:
        """Play one haptic on request (the panel's Test buttons), at its own strength: a
        click, the centre mark, the hard-over warning, or a whole alarm rumble. Only while
        force feedback runs, so the gain (and its ramp) still bounds it. A pulse whose
        feature is off is uploaded for the test and freed at the next settings change or on
        release; like every pulse it is shorter than a lease and never renewed. Raises
        ValueError saying why not."""
        level = model.test_pulse_level(self._s, name)
        with self._op:
            if self._closed:
                raise ValueError("force feedback is shutting down")
            if self._started_at is None:
                raise ValueError("force feedback is off: choose a mode first")
            if name not in model.ALARM_KINDS and name not in self._pulses:
                self._sync_pulse(name, True, level)
                if name not in self._pulses:
                    raise ValueError(f"{self._dev.name} cannot play the {name} pulse")
            log.info("Test: %s", name)
            if name in model.ALARM_KINDS:
                if not self._start_rumble(name, now, once=True):
                    raise ValueError(f"{self._dev.name} cannot play the rumble")
            else:
                self._pulse(name)

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

    # --- The centre notch -----------------------------------------------------------------

    def on_axis(self, axis_pct: float, now: float) -> int:
        """The centre notch, from where the wheel really is: the steering axis, -100..100 of
        the base's half travel, unsnapped (``InputPublisher.steering_axis_pct``). Called on
        every pass of the loop that reads the wheel, and only by it: each play ends by itself
        after ``NOTCH_MS``, so if that loop stops, so does the force. Returns the magnitude
        playing (s16, 0 for none). Raises DeviceGone."""
        with self._op:
            if self._started_at is None or self._closed or self._cnotch_unavailable:
                return 0
            level = model.notch_level(axis_pct, self._s)
            if level == 0:
                self._stop_notch()
                return 0
            moved = abs(level - self._cnotch_level) > model.NOTCH_EPSILON
            if self._cnotch_id is None or (moved and now - self._cnotch_written_at >= model.NOTCH_MIN_INTERVAL_S):
                try:
                    self._cnotch_id = self._dev.upload_constant(
                        -1 if self._cnotch_id is None else self._cnotch_id,
                        magnitude=level,
                        duration_ms=model.NOTCH_MS,
                    )
                except (NotImplementedError, RuntimeError) as e:
                    log.warning("No centre notch: %s", e)
                    self._cnotch_unavailable = True
                    return 0
                self._cnotch_level, self._cnotch_written_at = level, now
                self._cnotch_playing = False  # started again with it, in case the base takes a new magnitude only then
            if not self._cnotch_playing or now - self._cnotch_played_at >= model.NOTCH_RENEW_S:
                self._dev.play(self._cnotch_id, True)
                self._cnotch_playing, self._cnotch_played_at = True, now
            return self._cnotch_level

    def _stop_notch(self) -> None:
        if self._cnotch_id is not None and self._cnotch_playing:
            self._dev.play(self._cnotch_id, False)
        self._cnotch_playing = False

    def _drop_notch(self) -> None:
        """The notch's effect stopped and erased (switched off, or follow-up)."""
        if self._cnotch_id is None:
            return
        self._stop_notch()
        self._dev.erase(self._cnotch_id)
        self._cnotch_id, self._cnotch_level = None, 0

    # --- The tick (20 Hz) -------------------------------------------------------------------

    def _rumble_tick(self, now: float) -> None:
        """Play the pattern's plays as they come due; then again while its alarm lasts (a Try
        once). A repeating alarm stops when it is over: acknowledged, expired, not repeated,
        or no longer the highest."""
        r = self._rumble
        if not r.once and r.period is not None and model.alarm_top(self._alarms, now) != model.ALARM_RANKS[r.kind]:
            self._rumble = None
            return
        elapsed_ms = (now - r.started) * 1000.0
        due = [i for i in range(r.index, len(r.schedule)) if r.schedule[i][0] <= elapsed_ms]
        if due:
            last = due[-1]  # behind (a slow tick): the latest play, not a burst of old ones
            _start, duration = r.schedule[last]
            self._play_segment(r.kind, duration)
            r = self._rumble = r._replace(index=last + 1)
        if r.index < len(r.schedule):
            return
        if r.period is None:
            if elapsed_ms >= model.pattern_length_ms(r.schedule):
                self._rumble = None
        elif now - r.started >= r.period:
            self._rumble = r._replace(started=now, index=0)

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
            self._rumble_tick(now)

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
            if not model.notch_active(settings):
                self._drop_notch()
            self._sync_pulses()
            self._sync_damper(None, force=True)
            self._tick(now)

    # --- Letting go ---------------------------------------------------------------------------

    def _release(self) -> None:
        """Every effect stopped and erased, the gain at 0. Never raises."""
        self._started_at = None
        effects = (
            *self._spring_ids, self._damper_id, self._endstop_id, self._cnotch_id, *(p[0] for p in self._pulses.values()),
        )
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
        self._cnotch_id, self._cnotch_level, self._cnotch_playing = None, 0, False
        self._cnotch_written_at = self._cnotch_played_at = float("-inf")
        self._cnotch_unavailable = False
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
