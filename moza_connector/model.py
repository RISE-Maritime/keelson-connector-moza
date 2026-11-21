"""Pure decisions: no evdev, no Zenoh, no clock. Every time is passed in.

Everything that decides a number the operator feels or the vessel acts on lives here,
so it is tested without a wheel, a bus or a sleep.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from enum import Enum

# --- Axes -----------------------------------------------------------------------------


def normalize(raw: int, lo: int, hi: int, *, bipolar: bool, invert: bool = False) -> float:
    """A raw evdev axis value as a percentage of its travel.

    Bipolar (the wheel): -100 at ``lo``, +100 at ``hi``, 0 halfway. Unipolar (a pedal):
    0 at ``lo``, 100 at ``hi``. ``invert`` flips the travel before scaling. Values outside
    [lo, hi] are clamped: the kernel can report one count past absinfo on some firmware.
    """
    if hi <= lo:
        raise ValueError(f"axis range must be increasing, got [{lo}, {hi}]")
    frac = (min(max(raw, lo), hi) - lo) / (hi - lo)  # 0..1
    if invert:
        frac = 1.0 - frac
    return (frac * 2.0 - 1.0) * 100.0 if bipolar else frac * 100.0


def snap(value: float, center_snap_pct: float) -> float:
    """Exactly 0 (the wheel at centre, a pedal at rest) inside ``center_snap_pct`` of it."""
    if center_snap_pct > 0 and abs(value) < center_snap_pct:
        return 0.0
    return value


def should_publish(
    now: float,
    value: float,
    last_value: float | None,
    last_published_at: float | None,
    *,
    max_hz: float,
    deadband_pct: float,
) -> bool:
    """Publish a changed axis now, or leave it for the next change / the backstop?

    The first reading always goes out. Otherwise a change is held to ``max_hz``, except a
    jump larger than ``deadband_pct``, which goes out at once so a fast hard-over is never
    delayed. An unchanged value is never re-sent here -- that is the backstop's job.
    """
    if last_value is None or last_published_at is None:
        return True
    if value == last_value:
        return False
    if abs(value - last_value) > deadband_pct:
        return True
    return max_hz <= 0 or (now - last_published_at) >= 1.0 / max_hz


# --- Force feedback -------------------------------------------------------------------

#: The highest overall force-feedback gain the connector will ever set, whatever the
#: command line says. The R21 Ultra peaks at 21 Nm; a helm needs a fraction of that.
MAX_GAIN_CEILING_PCT = 60.0

S16_MAX = 0x7FFF
U16_MAX = 0xFFFF


class FfbMode(str, Enum):
    OFF = "off"
    CENTER = "center"
    FOLLOW_UP = "follow-up"
    #: No spring: the wheel stays where it is left, and clicks each ``detent_deg`` of
    #: rudder order it passes.
    DETENT = "detent"


@dataclass(frozen=True)
class FfbSettings:
    mode: FfbMode = FfbMode.FOLLOW_UP
    max_gain_pct: float = 30.0
    spring_pct: float = 60.0
    damper_pct: float = 20.0
    stale_s: float = 1.0
    rudder_max_deg: float = 35.0
    rudder_invert: bool = False
    #: The steering axis' ``invert`` from the profile: the spring centre is in device
    #: coordinates, so an inverted wheel needs an inverted centre too.
    wheel_invert: bool = False
    #: The helm's range, shared by the published wheel position and force feedback. The
    #: base's axis spans ``base_range_deg`` lock to lock (its steering angle in Pit House
    #: or Boxflat; the USB reports do not say). Full helm lock, +-100 % on
    #: ``wheel_position_pct``, is ``wheel_lock_deg`` either side of centre.
    base_range_deg: float = 900.0
    wheel_lock_deg: float = 450.0
    #: The end stop past full helm lock, 0 for none.
    endstop_pct: float = 100.0
    #: ``detent`` mode: a click each this many degrees of rudder order, this strong.
    detent_deg: float = 5.0
    detent_pct: float = 60.0
    #: The centre mark: a distinct pulse each time the wheel reaches centre, 0 for none.
    center_mark_pct: float = 80.0
    #: Wheel degrees either side of centre that order exactly 0: a spring never brings the
    #: wheel exactly to centre against the base's friction, and a ship needs a true 0.
    center_deadband_deg: float = 0.0
    #: How many times steeper the centring spring is near centre (1..3): that many springs,
    #: each with that share of the saturation, so the far pull is unchanged.
    spring_boost: float = 1.0
    #: A distinct pulse when the rudder order reaches this many degrees either way (0: none).
    hardover_warn_deg: float = 0.0
    hardover_pct: float = 80.0
    #: Speed-sensitive weight: spring and damper at ``speed_min_pct`` alongside, full from
    #: ``speed_full_kn`` over the ground (0: off).
    speed_full_kn: float = 0.0
    speed_min_pct: float = 40.0
    #: Alarms, per keelson.Alarm priority: a rumble this strong (0: none) for a new,
    #: unacknowledged alarm (caution once, warning three times, emergency every 2 s while it
    #: lasts), and, with ``alarm_leds``, the rim's LEDs in that level's colour (emergency
    #: flashing) while it lasts.
    alarm_caution_pct: float = 0.0
    alarm_warning_pct: float = 0.0
    alarm_emergency_pct: float = 0.0
    alarm_fire_pct: float = 0.0
    alarm_leds: bool = False
    alarm_base_light: bool = False
    alarm_caution_color: str = "#ffd400"
    alarm_warning_color: str = "#ff7a00"
    alarm_emergency_color: str = "#ff0000"
    alarm_fire_color: str = "#ff0000"
    #: Each kind's rumble pattern (``ALARM_PATTERNS``, or ``custom``: its ``_custom`` S/L
    #: string) and how often it repeats while the alarm lasts (0: once per alarm).
    alarm_caution_pattern: str = "one-short"
    alarm_warning_pattern: str = "two-short"
    alarm_emergency_pattern: str = "one-long"
    alarm_fire_pattern: str = "fire"
    alarm_caution_custom: str = "S"
    alarm_warning_custom: str = "SS"
    alarm_emergency_custom: str = "L"
    alarm_fire_custom: str = "L"
    alarm_caution_repeat_s: float = 0.0
    alarm_warning_repeat_s: float = 0.0
    alarm_emergency_repeat_s: float = 5.0
    alarm_fire_repeat_s: float = 0.0
    #: How long a short and a long rumble are, and the pause between them.
    alarm_short_ms: float = 250.0
    alarm_long_ms: float = 1500.0
    alarm_gap_ms: float = 300.0
    #: Extra damping while the rudder trails the wheel's order (0: none).
    lag_resistance_pct: float = 0.0
    #: The hard end stop: the base's own steering angle set to the helm's lock, over its serial
    #: port (``--base-serial``). ``base_max_deg`` is the base's own angle before any narrowing
    #: (0 when there is no serial port): the widest ``base_range_deg`` may be, and what full lock
    #: means. ``base_range_deg`` stays the angle the base has now, which the axis spans.
    hard_stop: bool = False
    base_max_deg: float = 0.0

    @property
    def lock_limit_deg(self) -> float:
        """The widest full helm lock: half the base's own angle (before any narrowing)."""
        return (self.base_max_deg or self.base_range_deg) / 2.0

    @property
    def deadband_pct(self) -> float:
        """The helm deadband as a percentage of full helm lock."""
        return self.center_deadband_deg / self.wheel_lock_deg * 100.0

    @property
    def lock_fraction(self) -> float:
        """Full helm lock as a fraction of the axis' half travel, in (0, 1]."""
        return lock_fraction(self.wheel_lock_deg, self.base_range_deg)


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


# --- The helm's range ------------------------------------------------------------------


def lock_fraction(wheel_lock_deg: float, base_range_deg: float) -> float:
    """Full helm lock as a fraction of the axis' half travel, in (0, 1]."""
    if base_range_deg <= 0 or wheel_lock_deg <= 0:
        raise ValueError("base_range_deg and wheel_lock_deg must be positive")
    return clamp(wheel_lock_deg / (base_range_deg / 2.0), 1e-3, 1.0)


def steering_pct(axis_pct: float, fraction: float) -> float:
    """The published wheel position for the axis at ``axis_pct`` of its half travel: full
    helm lock (+-100) at ``fraction`` of it, and held there past it."""
    return clamp(axis_pct / fraction, -100.0, 100.0)


#: A condition's dead band is half its width: the force starts this far either side of
#: the centre, on the same scale where the full deadband (0xFFFF) is the axis' half travel.
#: hid-pidff and the PID spec read it so; the live check is that the end stop bites where
#: ``wheel_position_pct`` reaches +-100.
def spring_layers(settings: FfbSettings) -> int:
    """How many centring springs to stack for ``spring_boost`` (1..3)."""
    return int(clamp(round(settings.spring_boost), 1, 3))


def helm_deadband(pct: float, band_pct: float) -> float:
    """The wheel position as ordered: exactly 0 within ``band_pct`` of centre, and rescaled
    outside it so the order grows from 0 at the band's edge to +-100 at full lock, no jump."""
    if band_pct <= 0:
        return pct
    if abs(pct) <= band_pct:
        return 0.0
    return math.copysign((abs(pct) - band_pct) / (100.0 - band_pct) * 100.0, pct)


def endstop_deadband(settings: FfbSettings) -> int:
    """The end stop's dead band (u16): no force inside full helm lock."""
    return round(settings.lock_fraction * U16_MAX)


def endstop_enabled(settings: FfbSettings) -> bool:
    """An end stop only when the helm's lock is short of the axis' own end."""
    return settings.endstop_pct > 0 and settings.lock_fraction < 1.0


def gain_level(max_gain_pct: float) -> int:
    """``FF_GAIN`` (0..0xFFFF) for a percentage, never above ``MAX_GAIN_CEILING_PCT``."""
    pct = clamp(max_gain_pct, 0.0, MAX_GAIN_CEILING_PCT)
    return round(pct / 100.0 * U16_MAX)


#: How long the overall gain takes to rise from 0 to its setting when force feedback
#: starts. The wheel may be anywhere when the connector comes up; a spring switched on at
#: full strength would whip it to centre.
GAIN_RAMP_S = 2.0


def gain_ramp_level(from_level: int, to_level: int, elapsed_s: float, ramp_s: float = GAIN_RAMP_S) -> int:
    """``FF_GAIN`` ``elapsed_s`` seconds after the target changed from ``from_level`` to
    ``to_level``. A rise is a linear ramp over ``ramp_s``; a drop is immediate, since less
    force is always safe. Never above the ceiling, whatever is asked."""
    ceiling = gain_level(MAX_GAIN_CEILING_PCT)
    to_level = int(clamp(to_level, 0, ceiling))
    from_level = int(clamp(from_level, 0, ceiling))
    if to_level <= from_level or ramp_s <= 0 or elapsed_s >= ramp_s:
        return to_level
    return round(from_level + (to_level - from_level) * max(0.0, elapsed_s) / ramp_s)


def ramped_gain_level(max_gain_pct: float, elapsed_s: float, ramp_s: float = GAIN_RAMP_S) -> int:
    """``FF_GAIN`` ``elapsed_s`` seconds after force feedback started: a linear ramp."""
    return gain_ramp_level(0, gain_level(max_gain_pct), elapsed_s, ramp_s)


# --- Remote force-feedback settings (configurable/v1, --ffb-remote) ---------------------

#: The settings Crowsnest may change, with their bounds. ``max_gain_pct`` is bounded by
#: the command line's cap instead (see ``apply_config``).
CONFIG_BOUNDS = {
    "spring_pct": (0.0, 100.0),
    "damper_pct": (0.0, 100.0),
    "rudder_max_deg": (1.0, 90.0),
    "stale_s": (0.1, 10.0),
    "base_range_deg": (90.0, 3600.0),
    "wheel_lock_deg": (5.0, 1800.0),  # and never past half the base's range
    "endstop_pct": (0.0, 100.0),
    "detent_deg": (0.5, 30.0),
    "detent_pct": (0.0, 100.0),
    "center_mark_pct": (0.0, 100.0),
    "center_deadband_deg": (0.0, 30.0),  # and under half the wheel lock
    "spring_boost": (1.0, 3.0),
    "hardover_warn_deg": (0.0, 89.0),  # and under rudder_max_deg
    "hardover_pct": (0.0, 100.0),
    "speed_full_kn": (0.0, 60.0),
    "speed_min_pct": (0.0, 100.0),
    "alarm_caution_pct": (0.0, 100.0),
    "alarm_warning_pct": (0.0, 100.0),
    "alarm_emergency_pct": (0.0, 100.0),
    "alarm_fire_pct": (0.0, 100.0),
    "alarm_caution_repeat_s": (0.0, 60.0),
    "alarm_warning_repeat_s": (0.0, 60.0),
    "alarm_emergency_repeat_s": (0.0, 60.0),
    "alarm_fire_repeat_s": (0.0, 60.0),
    "alarm_short_ms": (80.0, 600.0),
    "alarm_long_ms": (400.0, 4000.0),
    "alarm_gap_ms": (100.0, 1500.0),
    "lag_resistance_pct": (0.0, 100.0),
}
#: keelson.Alarm.Priority values a rumble may start from.
ALARM_PRIORITIES = {"caution": 2, "warning": 3, "emergency": 4}
ALARM_LEVELS = tuple(ALARM_PRIORITIES)
#: What an alarm is felt and shown as: its level, or fire (keelson's fire-alarm category, any
#: level), each with a rank: the higher wins.
ALARM_RANKS = {**ALARM_PRIORITIES, "fire": 5}
ALARM_KINDS = tuple(ALARM_RANKS)
ALARM_COLOR_KEYS = tuple(f"alarm_{k}_color" for k in ALARM_KINDS)
ALARM_PATTERN_KEYS = tuple(f"alarm_{k}_pattern" for k in ALARM_KINDS)
ALARM_CUSTOM_KEYS = tuple(f"alarm_{k}_custom" for k in ALARM_KINDS)
ALARM_TEXT_KEYS = (*ALARM_COLOR_KEYS, *ALARM_PATTERN_KEYS, *ALARM_CUSTOM_KEYS)
#: Every alarm setting, all of them a profile's.
ALARM_SETTING_KEYS = (
    *(f"alarm_{k}_pct" for k in ALARM_KINDS), "alarm_leds", "alarm_base_light", *ALARM_TEXT_KEYS,
    *(f"alarm_{k}_repeat_s" for k in ALARM_KINDS), "alarm_short_ms", "alarm_long_ms", "alarm_gap_ms",
)
CONFIG_WRITABLE = (
    "mode", "max_gain_pct", *CONFIG_BOUNDS, "rudder_invert", "hard_stop", "alarm_leds", "alarm_base_light",
    *ALARM_TEXT_KEYS,
)


def config_of(settings: FfbSettings) -> dict:
    """The writable settings as JSON-ready values."""
    return {
        "mode": settings.mode.value,
        "max_gain_pct": settings.max_gain_pct,
        "spring_pct": settings.spring_pct,
        "damper_pct": settings.damper_pct,
        "rudder_max_deg": settings.rudder_max_deg,
        "stale_s": settings.stale_s,
        "rudder_invert": settings.rudder_invert,
        "hard_stop": settings.hard_stop,
        "base_range_deg": settings.base_range_deg,
        "wheel_lock_deg": settings.wheel_lock_deg,
        "endstop_pct": settings.endstop_pct,
        "detent_deg": settings.detent_deg,
        "detent_pct": settings.detent_pct,
        "center_mark_pct": settings.center_mark_pct,
        "center_deadband_deg": settings.center_deadband_deg,
        "spring_boost": settings.spring_boost,
        "hardover_warn_deg": settings.hardover_warn_deg,
        "hardover_pct": settings.hardover_pct,
        "speed_full_kn": settings.speed_full_kn,
        "speed_min_pct": settings.speed_min_pct,
        **{k: getattr(settings, k) for k in ALARM_SETTING_KEYS},
        "lag_resistance_pct": settings.lag_resistance_pct,
    }


def apply_config(
    settings: FfbSettings, patch: dict, *, gain_cap_pct: float, read_only: dict
) -> FfbSettings:
    """``settings`` with a remote ``set_config`` patch applied, or ValueError.

    The patch may be partial. ``max_gain_pct`` may not exceed ``gain_cap_pct`` (the command
    line's ``--ffb-max-gain-pct``), which never exceeds the ceiling. A key from
    ``read_only`` is accepted only unchanged, so a get-edit-set round trip works; any
    other key is refused. Nothing is clipped: a value out of bounds is an error.
    """
    if not isinstance(patch, dict):
        raise ValueError("set_config expects a JSON object")
    cap = min(gain_cap_pct, MAX_GAIN_CEILING_PCT)
    changes: dict = {}
    for key, value in patch.items():
        if key in read_only:
            if value != read_only[key]:
                raise ValueError(f"{key} is read-only ({read_only[key]!r})")
            continue
        if key not in CONFIG_WRITABLE:
            raise ValueError(f"unknown setting {key!r} (settable: {', '.join(CONFIG_WRITABLE)})")
        if key == "mode":
            try:
                changes["mode"] = FfbMode(value)
            except ValueError:
                raise ValueError(f"mode must be one of {[m.value for m in FfbMode]}, got {value!r}") from None
        elif key in ALARM_COLOR_KEYS:
            changes[key] = color_hex(value)
        elif key in ALARM_PATTERN_KEYS:
            if value not in (*ALARM_PATTERNS, "custom"):
                raise ValueError(f"{key} must be one of {[*ALARM_PATTERNS, 'custom']}, got {value!r}")
            changes[key] = value
        elif key in ALARM_CUSTOM_KEYS:
            changes[key] = pattern_text(value)
        elif key in ("rudder_invert", "hard_stop", "alarm_leds", "alarm_base_light"):
            if not isinstance(value, bool):
                raise ValueError(f"{key} must be true or false, got {value!r}")
            changes[key] = value
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value != value:
                raise ValueError(f"{key} must be a number, got {value!r}")
            lo, hi = (0.0, cap) if key == "max_gain_pct" else CONFIG_BOUNDS[key]
            if not lo <= value <= hi:
                what = f" (the cap is {cap:g}%)" if key == "max_gain_pct" else ""
                raise ValueError(f"{key} {value:g} outside {lo:g}..{hi:g}{what}")
            changes[key] = float(value)
    new = replace(settings, **changes)
    if new.hardover_warn_deg > 0 and new.hardover_warn_deg >= new.rudder_max_deg:
        raise ValueError(
            f"hardover_warn_deg {new.hardover_warn_deg:g} must be under rudder_max_deg {new.rudder_max_deg:g}"
        )
    if new.center_deadband_deg >= new.wheel_lock_deg / 2.0:
        raise ValueError(
            f"center_deadband_deg {new.center_deadband_deg:g} must be under half of wheel_lock_deg "
            f"{new.wheel_lock_deg:g}"
        )
    if new.wheel_lock_deg > new.lock_limit_deg:
        raise ValueError(
            f"wheel_lock_deg {new.wheel_lock_deg:g} is past the base's own end "
            f"(half of {new.base_max_deg or new.base_range_deg:g})"
        )
    return new


def coeff_level(pct: float) -> int:
    """A condition effect's coefficient (s16, 0..0x7FFF used) for a percentage."""
    return round(clamp(pct, 0.0, 100.0) / 100.0 * S16_MAX)


def saturation_level(pct: float) -> int:
    """A condition effect's saturation (u16) for a percentage."""
    return round(clamp(pct, 0.0, 100.0) / 100.0 * U16_MAX)


def rudder_to_wheel_fraction(rudder_deg: float, settings: FfbSettings) -> float:
    """Where the wheel should sit for this rudder angle, -1 (port lock) .. +1 (starboard lock).

    The wheel is a rudder-order helm: full lock is hard over. So the follow-up position is
    the rudder angle as a fraction of ``rudder_max_deg``, not the angle itself.
    """
    if settings.rudder_max_deg <= 0:
        raise ValueError("rudder_max_deg must be positive")
    frac = clamp(rudder_deg / settings.rudder_max_deg, -1.0, 1.0)
    return -frac if settings.rudder_invert else frac


def center_level(wheel_fraction: float, settings: FfbSettings) -> int:
    """The spring ``center`` (s16, device coordinates) for a fraction of full helm ORDER:
    the wheel position that orders it, through the helm deadband and the lock."""
    frac = clamp(wheel_fraction, -1.0, 1.0)
    band = settings.deadband_pct / 100.0
    if frac != 0.0:
        frac = math.copysign(band + abs(frac) * (1.0 - band), frac)
    frac *= settings.lock_fraction
    if settings.wheel_invert:
        frac = -frac
    return round(frac * S16_MAX)


@dataclass(frozen=True)
class SpringTarget:
    """What the spring should do now. ``source`` says why, for the log and the tests."""

    enabled: bool
    center: int = 0
    source: str = "off"


def spring_target(
    settings: FfbSettings,
    *,
    now: float,
    rudder_deg: float | None,
    rudder_received_at: float | None,
) -> SpringTarget:
    """The spring for this instant.

    ``follow-up`` pulls the wheel to the rudder's real position -- but only on a fresh
    reading. A rudder reading older than ``stale_s`` (or none yet) falls back to plain
    centring: the wheel never holds an offset nobody is reporting any more.
    """
    if settings.mode is FfbMode.OFF:
        return SpringTarget(enabled=False)
    if settings.mode is FfbMode.DETENT:
        return SpringTarget(enabled=False, source="free")
    if settings.mode is FfbMode.FOLLOW_UP and rudder_deg is not None and rudder_received_at is not None:
        if 0.0 <= now - rudder_received_at <= settings.stale_s:
            frac = rudder_to_wheel_fraction(rudder_deg, settings)
            return SpringTarget(enabled=True, center=center_level(frac, settings), source="rudder")
    return SpringTarget(enabled=True, center=0, source="center")


#: Move the spring centre only when it shifts by more than this many s16 counts (~0.1 %):
#: re-uploading an effect is a USB transfer, and a twitching rudder sensor must not
#: saturate the bus to the base.
CENTER_EPSILON = 32


def needs_update(current: SpringTarget | None, target: SpringTarget) -> bool:
    if current is None:
        return True
    if current.enabled != target.enabled:
        return True
    return target.enabled and abs(current.center - target.center) > CENTER_EPSILON


# --- Behaviours: named presets --------------------------------------------------------


@dataclass(frozen=True)
class Preset:
    label: str
    description: str
    #: The settings it sets. ``wheel_lock_deg`` may be ``"full"``, the base's own end, and
    #: a number is pulled in to it. Never the gain: that stays the operator's, under the cap.
    settings: dict


#: How the wheel behaves, one name each. A preset is a set_config patch, so every value in
#: it passes the same checks as one sent by hand; ``tests/test_model.py`` holds them to that.
PRESETS = {
    "ship": Preset(
        "Ship's wheel",
        "The wheel steers: it stays where it is put and the rudder follows it; well damped, over the "
        "base's full range.",
        {"mode": "detent", "detent_pct": 0.0, "damper_pct": 40.0, "endstop_pct": 100.0,
         "wheel_lock_deg": "full", "rudder_max_deg": 35.0},
    ),
    "hydraulic": Preset(
        "Hydraulic helm",
        "The wheel steers, heavy and slow: it stays where it is put, with a lot of damping.",
        {"mode": "detent", "detent_pct": 0.0, "damper_pct": 80.0, "endstop_pct": 100.0,
         "wheel_lock_deg": "full", "rudder_max_deg": 35.0},
    ),
    "follow-rudder": Preset(
        "Follow rudder",
        "Something else steers (an autopilot): the wheel is pulled to where the rudder really is. "
        "Not while this wheel steers that rudder: the two would hold each other where they are.",
        {"mode": "follow-up", "spring_pct": 80.0, "damper_pct": 40.0, "endstop_pct": 100.0,
         "wheel_lock_deg": "full", "rudder_max_deg": 35.0},
    ),
    "car": Preset(
        "Car",
        "A firm centring spring and light damping, 900 degrees lock to lock where the base allows.",
        {"mode": "center", "spring_pct": 100.0, "damper_pct": 15.0, "endstop_pct": 100.0,
         "wheel_lock_deg": 450.0, "rudder_max_deg": 35.0},
    ),
    "small-boat": Preset(
        "Small boat",
        "Centring and quick: full rudder a quarter turn either side, an end stop past it.",
        {"mode": "center", "spring_pct": 60.0, "damper_pct": 10.0, "endstop_pct": 100.0,
         "wheel_lock_deg": 90.0, "rudder_max_deg": 30.0},
    ),
    "jetski": Preset(
        "Jet ski",
        "Handlebar steering: a short +-30 degree throw to a firm end stop, a light pull to centre, "
        "full nozzle (25 degrees) at full lock.",
        {"mode": "center", "spring_pct": 50.0, "damper_pct": 20.0, "endstop_pct": 100.0,
         "wheel_lock_deg": 30.0, "rudder_max_deg": 25.0},
    ),
    "notched": Preset(
        "Notched",
        "No spring: the wheel stays where it is left and clicks each 5 degrees of rudder.",
        {"mode": "detent", "damper_pct": 0.0, "endstop_pct": 100.0, "wheel_lock_deg": 90.0,
         "rudder_max_deg": 35.0, "detent_deg": 5.0, "detent_pct": 70.0},
    ),
    "free": Preset(
        "Free",
        "No spring and no clicks: the wheel turns freely up to the end stop at full lock.",
        {"mode": "detent", "damper_pct": 5.0, "endstop_pct": 100.0, "wheel_lock_deg": 90.0,
         "rudder_max_deg": 35.0, "detent_pct": 0.0},
    ),
}


def preset_patch(name: str, base_range_deg: float) -> dict:
    """The settings a built-in preset sets, on a base of ``base_range_deg``."""
    try:
        preset = PRESETS[name]
    except KeyError:
        raise ValueError(f"unknown preset {name!r} (known: {', '.join(PRESETS)})") from None
    patch = dict(preset.settings)
    lock = patch.get("wheel_lock_deg")
    if lock is not None:
        patch["wheel_lock_deg"] = base_range_deg / 2.0 if lock == "full" else min(float(lock), base_range_deg / 2.0)
    return patch


# --- Profiles: how the wheel behaves, built in or the operator's own --------------------

#: What a profile holds: everything that shapes how the wheel behaves, its mode included.
#: Not the gain (the operator's strength, under the cap) nor the wheel's setup.
BEHAVIOUR_KEYS = (
    "mode", "spring_pct", "spring_boost", "damper_pct", "endstop_pct", "wheel_lock_deg",
    "center_deadband_deg", "rudder_max_deg", "detent_deg", "detent_pct", "center_mark_pct",
    "hardover_warn_deg", "hardover_pct", "speed_full_kn", "speed_min_pct", *ALARM_SETTING_KEYS,
    "lag_resistance_pct",
)
#: The wheel's setup: it belongs to this wheel, not to a profile, and is always remembered.
SETUP_KEYS = ("max_gain_pct", "base_range_deg", "rudder_invert", "stale_s", "hard_stop")
#: The built-in profile made of the command line's behaviour flags.
COMMAND_LINE = "command-line"
COMMAND_LINE_LABEL = "Command line"
#: Where a deleted default or selected profile falls back to.
FALLBACK_PROFILE = "ship"
#: Keys get_config reports that set_config ignores: they follow from the rest.
DERIVED_KEYS = ("profiles", "modified")
#: What set_config does besides changing settings.
PROFILE_ACTIONS = ("enabled", "profile", "revert", "save_profile", "save_profile_as", "delete_profile", "default_profile")


def behaviour_of(settings: FfbSettings) -> dict:
    """A profile's values: the behaviour keys of ``settings``."""
    current = config_of(settings)
    return {k: current[k] for k in BEHAVIOUR_KEYS}


def _feel(values: dict) -> dict:
    """A profile's mode is never ``off``: off is force feedback's switch, not a behaviour."""
    if values.get("mode") in (None, FfbMode.OFF.value):
        return {**values, "mode": FfbSettings().mode.value}
    return values


@dataclass(frozen=True)
class Helm:
    """The wheel's force feedback as the operator sees it: the profile being tuned (its
    values in ``settings``, mode included, never OFF), whether force feedback is on, which
    profile is selected and which is the default, the operator's own profiles, and the
    command line's. ``enabled`` is never saved: force feedback always starts off."""

    settings: FfbSettings
    profile: str = COMMAND_LINE
    default_profile: str = COMMAND_LINE
    user: dict = field(default_factory=dict)  # slug -> Preset with complete settings
    command_line: dict = field(default_factory=dict)  # the command-line profile's values
    enabled: bool = False


def builtin_profiles() -> tuple[str, ...]:
    return (*PRESETS, COMMAND_LINE)


def profile_names(helm: Helm) -> list[str]:
    """Every profile: the built-ins, then the operator's own."""
    return [*builtin_profiles(), *helm.user]


def profile_label(helm: Helm, name: str) -> str:
    if name == COMMAND_LINE:
        return COMMAND_LINE_LABEL
    if name in PRESETS:
        return PRESETS[name].label
    return helm.user[name].label if name in helm.user else name


def profile_settings(helm: Helm, name: str) -> dict:
    """Profile ``name``'s complete values, on this wheel. A built-in preset is the defaults
    with its own settings over them, so choosing one sets every behaviour key; a lock is
    pulled in to the base's own end."""
    base = helm.settings.lock_limit_deg * 2.0  # the base's own angle: "full" is all of it
    if name == COMMAND_LINE:
        values = dict(helm.command_line)
    elif name in PRESETS:
        values = {**behaviour_of(FfbSettings()), **preset_patch(name, base)}
    elif name in helm.user:
        values = dict(helm.user[name].settings)
    else:
        raise ValueError(f"unknown profile {name!r} (known: {', '.join(profile_names(helm))})")
    if "wheel_lock_deg" in values:
        values["wheel_lock_deg"] = min(float(values["wheel_lock_deg"]), base / 2.0)
    return _feel(values)


def effective(helm: Helm) -> FfbSettings:
    """What the base gets: the profile's settings, with mode OFF unless force feedback is on."""
    return helm.settings if helm.enabled else replace(helm.settings, mode=FfbMode.OFF)


def modified(helm: Helm) -> list[str]:
    """The behaviour keys that differ from the selected profile as saved."""
    try:
        saved = profile_settings(helm, helm.profile)
    except ValueError:
        return list(BEHAVIOUR_KEYS)
    current = behaviour_of(helm.settings)
    return [k for k in BEHAVIOUR_KEYS if k in saved and current[k] != saved[k]]


def describe_behaviour(settings: dict) -> str:
    """A one-line summary of a saved profile, for its tooltip."""
    mode = {"center": "Centre", "follow-up": "Follow-up", "detent": "Free"}.get(settings.get("mode"), "Mode unchanged")
    parts = [mode]
    if settings.get("mode") in ("center", "follow-up"):
        parts.append(f"spring {settings.get('spring_pct', 0):g} %")
    parts += [f"damper {settings.get('damper_pct', 0):g} %", f"full lock {settings.get('wheel_lock_deg', 0):g} deg"]
    if settings.get("mode") == "detent" and settings.get("detent_pct"):
        parts.append(f"a click each {settings.get('detent_deg', 0):g} deg")
    return "Yours: " + ", ".join(parts) + "."


def profile_list(helm: Helm) -> list[dict]:
    """What get_config offers: each profile's name, label, description, whether it is built
    in, and its saved values (so the panel can show what changed)."""
    out = []
    for name in profile_names(helm):
        if name == COMMAND_LINE:
            description = "The settings moza2keelson was started with."
        elif name in PRESETS:
            description = PRESETS[name].description
        else:
            description = helm.user[name].description
        out.append({
            "name": name,
            "label": profile_label(helm, name),
            "description": description,
            "builtin": name not in helm.user,
            "settings": profile_settings(helm, name),
        })
    return out


def helm_config(helm: Helm) -> dict:
    """get_config's document (without the read-only keys the app adds)."""
    return {
        **config_of(helm.settings),
        "enabled": helm.enabled,
        "profile": helm.profile,
        "default_profile": helm.default_profile,
        "modified": modified(helm),
        "profiles": profile_list(helm),
    }


def preset_slug(name: str) -> str:
    """A profile's name as its key: lower case, words joined by dashes."""
    slug = "-".join("".join(c if c.isalnum() else " " for c in str(name).lower()).split())
    if not slug or len(slug) > 40:
        raise ValueError("a profile's name must have 1..40 letters or digits")
    if slug in builtin_profiles() or slug == "custom":
        raise ValueError(f"{name!r} is a built-in profile's name")
    return slug


def load_profile(helm: Helm, name: str) -> Helm:
    """``helm`` with profile ``name`` selected and its values loaded: unsaved changes go."""
    values = profile_settings(helm, name)
    settings = apply_config(helm.settings, values, gain_cap_pct=MAX_GAIN_CEILING_PCT, read_only={})
    return replace(helm, settings=settings, profile=name)


def _saved(label: str, settings: FfbSettings) -> Preset:
    values = behaviour_of(settings)
    return Preset(" ".join(str(label).split())[:40], describe_behaviour(values), values)


def apply_helm(helm: Helm, patch: dict, *, gain_cap_pct: float, read_only: dict) -> Helm:
    """``helm`` after a set_config ``patch``, or ValueError (and nothing changed).

    In order: a profile is loaded (``profile``, unless it is the selected one, or ``revert``);
    settings change (``apply_config``'s checks); the profile is saved (``save_profile``, the
    operator's own only) or saved as a new one, which is then selected (``save_profile_as``);
    a profile is deleted (``delete_profile``); the default is set (``default_profile``);
    force feedback is switched (``enabled``; ``mode: off`` is the same as ``enabled: false``,
    and another mode changes the profile's mode without switching it on).
    """
    if not isinstance(patch, dict):
        raise ValueError("set_config expects a JSON object")
    patch = {k: v for k, v in patch.items() if k not in DERIVED_KEYS}
    act = {k: patch.pop(k) for k in PROFILE_ACTIONS if k in patch}
    if patch.get("mode") == FfbMode.OFF.value:
        patch.pop("mode")
        if act.get("enabled") is True:
            raise ValueError("mode off and enabled true contradict each other")
        act["enabled"] = False
    if "enabled" in act and not isinstance(act["enabled"], bool):
        raise ValueError(f"enabled must be true or false, got {act['enabled']!r}")

    new = helm
    target = act.get("profile")
    if target is not None and act.get("revert"):
        raise ValueError("profile and revert go one at a time")
    if target is not None and target != helm.profile:
        new = load_profile(new, str(target))
    elif act.get("revert"):
        new = load_profile(new, new.profile)

    if patch:
        new = replace(new, settings=apply_config(new.settings, patch, gain_cap_pct=gain_cap_pct, read_only=read_only))

    if act.get("save_profile"):
        if new.profile not in new.user:
            raise ValueError(f"{profile_label(new, new.profile)} is built in: save it as a new profile")
        new = replace(new, user={**new.user, new.profile: _saved(new.user[new.profile].label, new.settings)})
    if act.get("save_profile_as") is not None:
        name = act["save_profile_as"]
        slug = preset_slug(name)
        if slug in new.user:
            raise ValueError(f"a profile named {name!r} exists already: select it and Save")
        new = replace(new, user={**new.user, slug: _saved(name, new.settings)}, profile=slug)

    gone = act.get("delete_profile")
    if gone is not None:
        if gone in builtin_profiles():
            raise ValueError(f"{profile_label(new, gone)} is built in and cannot be deleted")
        if gone not in new.user:
            raise ValueError(f"no profile {gone!r} of yours")
        new = replace(new, user={k: v for k, v in new.user.items() if k != gone})
        if new.default_profile == gone:
            new = replace(new, default_profile=FALLBACK_PROFILE)
        if new.profile == gone:
            new = load_profile(new, new.default_profile)

    default = act.get("default_profile")
    if default is not None and default != new.default_profile:
        if default not in profile_names(new):
            raise ValueError(f"unknown profile {default!r}")
        new = replace(new, default_profile=default)

    if "enabled" in act:
        new = replace(new, enabled=act["enabled"])
    return new


def saved_state(helm: Helm) -> dict:
    """What is written to the settings file: the wheel's setup, the default profile and the
    operator's own profiles. Never ``enabled`` (force feedback always starts off) and never
    unsaved changes."""
    current = config_of(helm.settings)
    # The base's own angle, not one the hard stop narrowed it to: what it is put back to.
    current["base_range_deg"] = helm.settings.base_max_deg or helm.settings.base_range_deg
    return {
        "version": 2,
        "setup": {k: current[k] for k in SETUP_KEYS},
        "default_profile": helm.default_profile,
        "profiles": {n: {"label": p.label, "description": p.description, "settings": p.settings} for n, p in helm.user.items()},
    }


#: The name a version-1 file's tuned settings come back under, when no profile has them.
RESTORED = "restored-settings"
RESTORED_LABEL = "Restored settings"


def load_state(doc, helm: Helm, *, gain_cap_pct: float) -> tuple[Helm, list[str]]:
    """``helm`` with a settings file's setup, default and profiles, and what was dropped.

    ``enabled`` is never restored. A saved gain above today's cap comes back as the cap. A
    profile that no longer passes the checks is dropped, and said why. A version-1 file
    (``settings`` and ``presets``) is read too: its presets become profiles, and its tuned
    settings, if no profile has them, become the profile "Restored settings", the default.
    The selected profile is not loaded here; the caller loads the one to start in.
    """
    notes: list[str] = []
    if not isinstance(doc, dict):
        return helm, ["the settings file is not a JSON object: ignored"]
    v1 = doc.get("version") != 2
    defaults = behaviour_of(FfbSettings())
    user: dict = {}
    for name, p in (doc.get("presets" if v1 else "profiles") or {}).items():
        try:
            preset_slug(name)
            kept = {k: v for k, v in upgrade_alarm_keys(dict(p["settings"])).items() if k in BEHAVIOUR_KEYS}
            values = _feel({**defaults, **kept})
            apply_config(FfbSettings(), {k: v for k, v in values.items() if k != "wheel_lock_deg"},
                         gain_cap_pct=MAX_GAIN_CEILING_PCT, read_only={})
            user[name] = Preset(str(p["label"])[:40], describe_behaviour(values), values)
        except (ValueError, KeyError, TypeError) as e:
            notes.append(f"saved profile {name!r} dropped: {e}")
    helm = replace(helm, user=user)

    saved = upgrade_alarm_keys(dict(doc.get("settings" if v1 else "setup") or {}))
    setup = {k: v for k, v in saved.items() if k in SETUP_KEYS}
    gain = setup.get("max_gain_pct")
    cap = min(gain_cap_pct, MAX_GAIN_CEILING_PCT)
    if isinstance(gain, (int, float)) and not isinstance(gain, bool) and gain > cap:
        setup["max_gain_pct"] = cap
        notes.append(f"saved gain {gain:g} % is above today's cap: {cap:g} %")
    base = setup.get("base_range_deg")
    if isinstance(base, (int, float)) and not isinstance(base, bool) and helm.settings.wheel_lock_deg > base / 2.0:
        # A smaller base than the command line's lock: the lock is pulled in (a profile sets it next).
        setup["wheel_lock_deg"] = max(CONFIG_BOUNDS["wheel_lock_deg"][0], base / 2.0)
    try:
        helm = replace(helm, settings=apply_config(helm.settings, setup, gain_cap_pct=gain_cap_pct, read_only={}))
    except ValueError as e:
        notes.append(f"saved setup ignored: {e}")

    if v1:
        tuned = {k: v for k, v in saved.items() if k in BEHAVIOUR_KEYS and k != "mode"}  # v1 kept no mode
        if tuned:
            try:
                restored = apply_config(helm.settings, {**profile_settings(helm, COMMAND_LINE), **tuned},
                                        gain_cap_pct=MAX_GAIN_CEILING_PCT, read_only={})
                values = behaviour_of(restored)
                if not any(profile_settings(helm, n) == values for n in profile_names(helm)):
                    helm = replace(helm, user={**helm.user, RESTORED: Preset(RESTORED_LABEL, describe_behaviour(values), values)})
                    helm = replace(helm, default_profile=RESTORED)
                    notes.append(f"the settings tuned before profiles are kept as {RESTORED_LABEL!r}, the default")
            except ValueError as e:
                notes.append(f"saved settings ignored: {e}")
    else:
        default = doc.get("default_profile")
        if default in profile_names(helm):
            helm = replace(helm, default_profile=default)
        elif default is not None:
            notes.append(f"default profile {default!r} is gone: starting in {profile_label(helm, helm.default_profile)}")
    return helm, notes


# --- Detents (``detent`` mode) ----------------------------------------------------------

#: How far past half a step the rudder order must go before the next notch counts, as a
#: fraction of the step: a wheel resting on a boundary must not chatter.
DETENT_HYSTERESIS = 0.15
#: The click: a short square pulse. Its duration is finite and short; it is never leased.
CLICK_PERIOD_MS = 20
CLICK_DURATION_MS = 30


def rudder_order_deg(wheel_pct: float, settings: FfbSettings) -> float:
    """The rudder the wheel orders: full helm lock is ``rudder_max_deg``."""
    return clamp(wheel_pct, -100.0, 100.0) / 100.0 * settings.rudder_max_deg


def detent_notch(rudder_deg: float, notch: int | None, step_deg: float) -> int:
    """The notch (multiple of ``step_deg``) the rudder order sits in. The current notch is
    kept until the order is more than half a step plus the hysteresis away from it."""
    if step_deg <= 0:
        raise ValueError("step_deg must be positive")
    nearest = round(rudder_deg / step_deg)
    if notch is None:
        return nearest
    if abs(rudder_deg - notch * step_deg) > step_deg * (0.5 + DETENT_HYSTERESIS):
        return nearest
    return notch


def click_level(settings: FfbSettings) -> int:
    """The click's magnitude (s16, 0..0x7FFF)."""
    return coeff_level(settings.detent_pct)


# --- The centre mark --------------------------------------------------------------------

#: The wheel is "at centre" within this many degrees of wheel rotation either side, and has
#: left it only past twice that: a wheel resting on centre must not buzz.
CENTER_BAND_DEG = 1.0
#: Twice a detent click's length, so the centre feels different from a notch.
CENTER_MARK_MS = 60


def wheel_deg(wheel_pct: float, settings: FfbSettings) -> float:
    """The wheel's rotation from centre (deg) for its published position."""
    return clamp(wheel_pct, -100.0, 100.0) / 100.0 * settings.wheel_lock_deg


def center_side(deg: float, side: int | None) -> int:
    """-1 port of centre, 0 at centre, +1 starboard, with hysteresis: centre is entered
    within ``CENTER_BAND_DEG`` and left only past twice that."""
    if abs(deg) <= CENTER_BAND_DEG or (side == 0 and abs(deg) <= 2 * CENTER_BAND_DEG):
        return 0
    return 1 if deg > 0 else -1


def center_reached(side: int | None, new: int) -> bool:
    """Mark centre on arriving at it, or on passing straight over it between two readings;
    never on leaving it, and never for the first reading."""
    if side is None or new == side:
        return False
    return new == 0 or (side != 0 and new == -side)


def center_mark_level(settings: FfbSettings) -> int:
    """The centre mark's magnitude (s16, 0..0x7FFF)."""
    return coeff_level(settings.center_mark_pct)


# --- Haptics: hard-over warning, speed-sensitive weight, alarm rumble, gear lag ------------

#: The hard-over warning: longer than a click, shorter than the centre mark's double.
HARDOVER_MS = 45
#: Out of the warning zone only this far back inside it, so a helm held at the line is quiet.
HARDOVER_HYSTERESIS_DEG = 2.0


def hardover_side(order_deg: float, side: int | None, warn_deg: float) -> int:
    """+-1 while the rudder order is at or past +-``warn_deg``, else 0 (with hysteresis)."""
    if warn_deg <= 0:
        return 0
    if abs(order_deg) >= warn_deg or (side and abs(order_deg) > warn_deg - HARDOVER_HYSTERESIS_DEG
                                      and (order_deg > 0) == (side > 0)):
        return 1 if order_deg > 0 else -1
    return 0


def hardover_reached(side: int | None, new: int) -> bool:
    """Warn on entering a warning zone (never on leaving, never for the first reading)."""
    return side is not None and new != 0 and new != side


#: Speed over ground older than this counts as no speed: the wheel keeps its full weight.
SPEED_STALE_S = 2.0
#: Re-upload the spring and damper only when the weight moves by more than this.
WEIGHT_STEP = 0.02


def speed_weight(settings: FfbSettings, sog_kn: float | None, age_s: float | None) -> float:
    """How heavy the wheel is (0..1) for the speed over ground: ``speed_min_pct`` alongside,
    full at ``speed_full_kn``. Off, or without a fresh speed, it is full: never lighter than
    set on old data."""
    if settings.speed_full_kn <= 0 or sog_kn is None or age_s is None or not 0 <= age_s <= SPEED_STALE_S:
        return 1.0
    low = settings.speed_min_pct / 100.0
    return low + (1.0 - low) * clamp(abs(sog_kn) / settings.speed_full_kn, 0.0, 1.0)


#: Gear lag: the extra damping is full once the rudder trails the order by this much.
LAG_FULL_DEG = 5.0


def damper_pct_now(settings: FfbSettings, weight: float, lag_deg: float | None) -> float:
    """The damper this instant: the set damper, scaled by the speed weight, plus the lag
    resistance while the rudder trails the order (``lag_deg`` None: no fresh rudder)."""
    extra = 0.0
    if settings.lag_resistance_pct > 0 and lag_deg is not None:
        extra = settings.lag_resistance_pct * clamp(abs(lag_deg) / LAG_FULL_DEG, 0.0, 1.0)
    return clamp(settings.damper_pct * weight + extra, 0.0, 100.0)


# --- Alarms: a rumble and the rim's colour per priority ------------------------------------

#: One rumble: a quick square this long. Each is shorter than a lease and never renewed.
RUMBLE_PERIOD_MS = 25
#: One play of the rumble effect is at most this long (under a lease, never renewed): a longer
#: rumble is several plays back to back.
SEGMENT_MAX_MS = 800
#: The patterns, after the ship's signals: S short, L long. ``fire`` is continuous: it repeats
#: with only the gap between, for as long as the alarm lasts.
ALARM_PATTERNS = {
    "one-short": "S",
    "two-short": "SS",
    "three-short": "SSS",
    "one-long": "L",
    "general-alarm": "SSSSSSSL",  # the general emergency alarm: seven short, one long
    "man-overboard": "LLL",  # three prolonged (Morse O)
    "fire": "L",  # a continuous signal
}
CONTINUOUS_PATTERNS = frozenset({"fire"})
#: An alarm not repeated for this long (and not given an expiry) is over.
ALARM_HOLD_S = 30.0
#: keelson.Alarm.Category CATEGORY_FIRE_ALARM: felt and shown as fire, whatever its priority.
FIRE_CATEGORY = 4

_HEX = set("0123456789abcdef")


def color_hex(value) -> str:
    """A colour setting, checked: ``#rrggbb`` in lower case."""
    if not isinstance(value, str) or len(value) != 7 or value[0] != "#" or not set(value[1:].lower()) <= _HEX:
        raise ValueError(f"a colour must be #rrggbb, got {value!r}")
    return value.lower()


def color_rgb(value: str) -> int:
    """``#rrggbb`` as the devices' 3-byte RGB value."""
    return int(color_hex(value)[1:], 16)


def pattern_text(value) -> str:
    """A custom pattern, checked: 1..16 of S (short) and L (long)."""
    if not isinstance(value, str) or not 1 <= len(value) <= 16 or set(value.upper()) - {"S", "L"}:
        raise ValueError(f"a pattern is 1 to 16 of S and L, got {value!r}")
    return value.upper()


def upgrade_alarm_keys(d: dict) -> dict:
    """Settings from before per-level alarms: one rumble strength from a lowest priority up.
    Fire, newer still, takes the emergency's strength."""
    d = dict(d)
    if "alarm_rumble_pct" in d or "alarm_min_priority" in d:
        pct = d.pop("alarm_rumble_pct", 0.0)
        lowest = ALARM_PRIORITIES.get(d.pop("alarm_min_priority", "warning"), 3)
        for level, priority in ALARM_PRIORITIES.items():
            d.setdefault(f"alarm_{level}_pct", pct if priority >= lowest else 0.0)
    if "alarm_emergency_pct" in d:
        d.setdefault("alarm_fire_pct", d["alarm_emergency_pct"])
    return d


def alarm_kind(rank: int) -> str | None:
    """The kind for a rank (2 caution, 3 warning, 4 emergency, 5 fire), or None."""
    for kind, r in ALARM_RANKS.items():
        if r == rank:
            return kind
    return "fire" if rank > ALARM_RANKS["fire"] else None


def alarm_rank(priority: int, category: int = 0) -> int:
    """An alarm's rank: fire for keelson's fire-alarm category, else its priority."""
    if category == FIRE_CATEGORY and priority >= ALARM_PRIORITIES["caution"]:
        return ALARM_RANKS["fire"]
    return min(priority, ALARM_PRIORITIES["emergency"])


def alarm_level(priority: int) -> str | None:
    """The level of a keelson.Alarm priority (2 caution, 3 warning, 4 emergency), or None."""
    return alarm_kind(min(priority, ALARM_PRIORITIES["emergency"])) if priority >= 2 else None


def alarm_pct(settings: FfbSettings, kind: str) -> float:
    return getattr(settings, f"alarm_{kind}_pct")


def pattern_symbols(settings: FfbSettings, kind: str) -> tuple[str, bool]:
    """A kind's pattern as S/L symbols, and whether it is continuous."""
    name = getattr(settings, f"alarm_{kind}_pattern")
    if name == "custom":
        return getattr(settings, f"alarm_{kind}_custom"), False
    return ALARM_PATTERNS[name], name in CONTINUOUS_PATTERNS


def pattern_schedule(symbols: str, short_ms: float, long_ms: float, gap_ms: float) -> list[tuple[int, int]]:
    """The plays of a pattern: (start, duration) in ms from its start. Symbols are a gap apart;
    one longer than ``SEGMENT_MAX_MS`` is several plays back to back."""
    out, at = [], 0
    for i, symbol in enumerate(symbols):
        if i:
            at += int(gap_ms)
        left = int(short_ms if symbol == "S" else long_ms)
        while left > 0:
            piece = min(left, SEGMENT_MAX_MS)
            out.append((at, piece))
            at += piece
            left -= piece
    return out


def pattern_length_ms(schedule: list[tuple[int, int]]) -> int:
    return max((s + d for s, d in schedule), default=0)


def alarm_schedule(settings: FfbSettings, kind: str) -> tuple[list[tuple[int, int]], bool]:
    symbols, continuous = pattern_symbols(settings, kind)
    return pattern_schedule(symbols, settings.alarm_short_ms, settings.alarm_long_ms, settings.alarm_gap_ms), continuous


def pattern_period_s(settings: FfbSettings, kind: str, schedule: list[tuple[int, int]], continuous: bool) -> float | None:
    """When a pattern starts again while its alarm lasts (seconds after it started), or None
    for once. Continuous: after the gap. Repeating: every ``repeat_s``, never overlapping."""
    length = pattern_length_ms(schedule) + settings.alarm_gap_ms
    if continuous:
        return length / 1000.0
    repeat = getattr(settings, f"alarm_{kind}_repeat_s")
    return max(repeat, length / 1000.0) if repeat > 0 else None


@dataclass(frozen=True)
class AlarmSeen:
    rank: int
    acknowledged: bool
    seen_at: float
    expires_at: float | None = None


def alarm_active(a: AlarmSeen, now: float) -> bool:
    """Unacknowledged, repeated recently enough, and not past its own expiry."""
    if a.acknowledged or now - a.seen_at > ALARM_HOLD_S:
        return False
    return a.expires_at is None or now < a.expires_at


def alarm_seen(board: dict, identifier: str, rank: int, acknowledged: bool, now: float,
               expires_at: float | None = None) -> tuple[dict, bool]:
    """The alarm board after a reading, and whether it starts something to feel: an alarm not
    active before (or now at a higher rank) that is active now. Old entries are dropped."""
    before = board.get(identifier)
    entry = AlarmSeen(rank, acknowledged, now, expires_at)
    fresh = alarm_active(entry, now) and (before is None or not alarm_active(before, now) or rank > before.rank)
    kept = {k: v for k, v in board.items() if now - v.seen_at <= ALARM_HOLD_S and k != identifier}
    return {**kept, identifier: entry}, fresh


def alarm_top(board: dict, now: float) -> int | None:
    """The highest rank of an active alarm with a kind, or None."""
    active = [a.rank for a in board.values() if alarm_active(a, now) and alarm_kind(a.rank)]
    return max(active) if active else None


def alarm_light_color(settings: FfbSettings, rank: int | None, target: str) -> int | None:
    """The colour (RGB) a target (``rim`` or ``base``) shows for the highest active alarm, or
    None for its own colours. Steady: written once per change, never flashed (these are the
    devices' stored settings)."""
    kind = alarm_kind(rank) if rank is not None else None
    on = settings.alarm_leds if target == "rim" else settings.alarm_base_light
    if not on or kind is None:
        return None
    return color_rgb(getattr(settings, f"alarm_{kind}_color"))


# --- Trying a haptic by hand ---------------------------------------------------------------

#: The pulses the panel can play on request, each with the setting that sets its strength.
TEST_PULSES = {
    "center": ("center_mark_pct", "Centre mark"),
    "detent": ("detent_pct", "Click strength"),
    "hardover": ("hardover_pct", "Warning strength"),
    "caution": ("alarm_caution_pct", "Caution rumble"),
    "warning": ("alarm_warning_pct", "Warning rumble"),
    "emergency": ("alarm_emergency_pct", "Emergency rumble"),
    "fire": ("alarm_fire_pct", "Fire rumble"),
}


def test_pulse_level(settings: FfbSettings, name: str) -> int:
    """The strength (s16) a requested test of pulse ``name`` plays at: its own setting, so
    the test feels as the real thing will. Raises ValueError for an unknown pulse or one
    whose strength is 0 (there would be nothing to feel)."""
    if name not in TEST_PULSES:
        raise ValueError(f"no pulse {name!r} to test; one of {', '.join(TEST_PULSES)}")
    key, label = TEST_PULSES[name]
    level = coeff_level(getattr(settings, key))
    if level == 0:
        raise ValueError(f"{label} is 0 %: raise it to feel the test")
    return level


# --- The hard end stop: the base's own steering angle (--base-serial) ----------------------

#: The narrowest steering angle the connector sets on the base, lock to lock.
BASE_MIN_RANGE_DEG = 90.0
#: A narrower angle is set only while the wheel is at least this far inside it.
BASE_WHEEL_MARGIN_DEG = 5.0


def base_range_for(settings: FfbSettings) -> float:
    """The steering angle the base should have, lock to lock: twice the helm's lock (rounded
    to the base's whole degrees either side) while the hard stop is on, else its own."""
    own = settings.base_max_deg or settings.base_range_deg
    if not settings.hard_stop:
        return own
    return clamp(2.0 * round(settings.wheel_lock_deg), BASE_MIN_RANGE_DEG, own)


def base_stop_decision(target_deg: float, current_deg: float, wheel_deg: float | None) -> tuple[str, str]:
    """What to do about the base's angle: ``("none", "")`` when it is right, ``("apply", "")``
    to set it, or ``("wait", why)``. Wider is always safe. Narrower only while the wheel is
    inside the new angle by the margin: the base's stop must never snap a wheel that is
    outside it."""
    if abs(target_deg - current_deg) < 1.0:
        return "none", ""
    if target_deg > current_deg:
        return "apply", ""
    inside = target_deg / 2.0 - BASE_WHEEL_MARGIN_DEG
    if wheel_deg is None:
        return "wait", "waiting for the wheel's position"
    if abs(wheel_deg) <= inside:
        return "apply", ""
    return "wait", f"turn the wheel within ±{inside:g}°"
