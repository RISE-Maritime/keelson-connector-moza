"""Pure decisions: no evdev, no Zenoh, no clock. Every time is passed in.

Everything that decides a number the operator feels or the vessel acts on lives here,
so it is tested without a wheel, a bus or a sleep.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
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
    #: A rumble for a new, unacknowledged alarm at or above ``alarm_min_priority`` (0: none).
    alarm_rumble_pct: float = 0.0
    alarm_min_priority: str = "warning"
    #: Extra damping while the rudder trails the wheel's order (0: none).
    lag_resistance_pct: float = 0.0

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
    "alarm_rumble_pct": (0.0, 100.0),
    "lag_resistance_pct": (0.0, 100.0),
}
#: keelson.Alarm.Priority values a rumble may start from.
ALARM_PRIORITIES = {"caution": 2, "warning": 3, "emergency": 4}
CONFIG_WRITABLE = ("mode", "max_gain_pct", *CONFIG_BOUNDS, "rudder_invert", "alarm_min_priority")


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
        "alarm_rumble_pct": settings.alarm_rumble_pct,
        "alarm_min_priority": settings.alarm_min_priority,
        "lag_resistance_pct": settings.lag_resistance_pct,
    }


def apply_config(
    settings: FfbSettings, patch: dict, *, gain_cap_pct: float, read_only: dict, user_presets: dict | None = None
) -> FfbSettings:
    """``settings`` with a remote ``set_config`` patch applied, or ValueError.

    The patch may be partial. ``max_gain_pct`` may not exceed ``gain_cap_pct`` (the command
    line's ``--ffb-max-gain-pct``), which never exceeds the ceiling. A key from
    ``read_only`` is accepted only unchanged, so a get-edit-set round trip works; any
    other key is refused. Nothing is clipped: a value out of bounds is an error.
    """
    if not isinstance(patch, dict):
        raise ValueError("set_config expects a JSON object")
    patch = expand_preset(settings, patch, user_presets)
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
        elif key == "alarm_min_priority":
            if value not in ALARM_PRIORITIES:
                raise ValueError(f"alarm_min_priority must be one of {list(ALARM_PRIORITIES)}, got {value!r}")
            changes[key] = value
        elif key == "rudder_invert":
            if not isinstance(value, bool):
                raise ValueError(f"rudder_invert must be true or false, got {value!r}")
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
    if new.wheel_lock_deg > new.base_range_deg / 2.0:
        raise ValueError(
            f"wheel_lock_deg {new.wheel_lock_deg:g} is past the base's own end "
            f"(half of base_range_deg {new.base_range_deg:g})"
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


def all_presets(user: dict | None = None) -> dict:
    """The built-in presets, then the operator's own (``user``: slug -> Preset)."""
    return {**PRESETS, **(user or {})}


def preset_patch(name: str, base_range_deg: float, user: dict | None = None) -> dict:
    """The set_config patch a preset stands for, on a base of ``base_range_deg``."""
    presets = all_presets(user)
    try:
        preset = presets[name]
    except KeyError:
        raise ValueError(f"unknown preset {name!r} (known: {', '.join(presets)})") from None
    patch = dict(preset.settings)
    lock = patch.get("wheel_lock_deg")
    if lock is not None:
        patch["wheel_lock_deg"] = base_range_deg / 2.0 if lock == "full" else min(float(lock), base_range_deg / 2.0)
    return patch


def expand_preset(settings: FfbSettings, patch: dict, user: dict | None = None) -> dict:
    """A set_config patch with its ``preset`` (if any) unfolded; the patch's own keys win.

    ``"custom"``, or the preset the settings already match, changes nothing: that is a
    get_config document sent back as it came.
    """
    if "preset" not in patch:
        return patch
    rest = {k: v for k, v in patch.items() if k != "preset"}
    name = patch["preset"]
    if name in ("custom", matching_preset(settings, user)):
        return rest
    base = rest.get("base_range_deg", settings.base_range_deg)
    return {**preset_patch(name, base, user), **rest}


def matching_preset(settings: FfbSettings, user: dict | None = None) -> str:
    """The preset the settings are, or ``"custom"``. With force feedback off its mode is not
    compared, so the panel still shows which behaviour is loaded."""
    current = config_of(settings)
    for name in all_presets(user):
        patch = preset_patch(name, settings.base_range_deg, user)
        if settings.mode is FfbMode.OFF:
            patch.pop("mode", None)
        if all(current.get(k) == v for k, v in patch.items()):
            return name
    return "custom"


def preset_list(user: dict | None = None) -> list[dict]:
    """What get_config offers: each preset's name, label and description; the operator's
    own marked ``custom``."""
    return [
        {"name": n, "label": p.label, "description": p.description, "custom": n not in PRESETS}
        for n, p in all_presets(user).items()
    ]


# --- The operator's own behaviours, and what is remembered between runs ----------------

#: What a saved behaviour keeps: how the wheel behaves, not the gain (the operator's
#: strength, under the cap), the base's range or the rudder feed's details.
BEHAVIOUR_KEYS = (
    "mode", "spring_pct", "spring_boost", "damper_pct", "endstop_pct", "wheel_lock_deg",
    "center_deadband_deg", "rudder_max_deg", "detent_deg", "detent_pct", "center_mark_pct",
    "hardover_warn_deg", "hardover_pct", "speed_full_kn", "speed_min_pct", "alarm_rumble_pct",
    "alarm_min_priority", "lag_resistance_pct",
)


def preset_slug(name: str) -> str:
    """A behaviour's name as its key: lower case, words joined by dashes."""
    slug = "-".join("".join(c if c.isalnum() else " " for c in str(name).lower()).split())
    if not slug or len(slug) > 40:
        raise ValueError("a behaviour's name must have 1..40 letters or digits")
    if slug in PRESETS or slug == "custom":
        raise ValueError(f"{name!r} is a built-in behaviour's name")
    return slug


def describe_behaviour(settings: dict) -> str:
    """A one-line summary of a saved behaviour, for its tooltip."""
    mode = {"center": "Centre", "follow-up": "Follow-up", "detent": "Free"}.get(settings.get("mode"), "Mode unchanged")
    parts = [mode]
    if settings.get("mode") in ("center", "follow-up"):
        parts.append(f"spring {settings.get('spring_pct', 0):g} %")
    parts += [f"damper {settings.get('damper_pct', 0):g} %", f"full lock {settings.get('wheel_lock_deg', 0):g} deg"]
    if settings.get("mode") == "detent" and settings.get("detent_pct"):
        parts.append(f"a click each {settings.get('detent_deg', 0):g} deg")
    return "Saved: " + ", ".join(parts) + "."


def save_behaviour(name: str, settings: FfbSettings, user: dict | None) -> dict:
    """The operator's presets with the current behaviour saved as ``name``. With force
    feedback off its mode is not kept, so choosing it later does not switch the wheel on."""
    slug = preset_slug(name)
    current = config_of(settings)
    kept = {k: current[k] for k in BEHAVIOUR_KEYS if k in current}
    if settings.mode is FfbMode.OFF:
        kept.pop("mode")
    label = " ".join(str(name).split())[:40]
    return {**(user or {}), slug: Preset(label, describe_behaviour(kept), kept)}


def delete_behaviour(name: str, user: dict | None) -> dict:
    """The operator's presets without ``name``. A built-in cannot be deleted."""
    if name in PRESETS:
        raise ValueError(f"{name!r} is built in and cannot be deleted")
    if name not in (user or {}):
        raise ValueError(f"no saved behaviour {name!r}")
    return {k: v for k, v in user.items() if k != name}


def saved_state(settings: FfbSettings, user: dict | None) -> dict:
    """What is written to the settings file: every writable setting but the mode (force
    feedback always starts off), and the operator's behaviours."""
    kept = {k: v for k, v in config_of(settings).items() if k != "mode"}
    presets = {n: {"label": p.label, "description": p.description, "settings": p.settings} for n, p in (user or {}).items()}
    return {"settings": kept, "presets": presets}


def load_state(doc, settings: FfbSettings, *, gain_cap_pct: float) -> tuple[FfbSettings, dict, list[str]]:
    """Settings and behaviours from a settings file, or the given settings and a reason.

    The mode is never restored. A saved gain above today's cap comes back as the cap. A
    setting or behaviour that no longer passes the checks is dropped, and said why.
    """
    notes: list[str] = []
    if not isinstance(doc, dict):
        return settings, {}, ["the settings file is not a JSON object: ignored"]
    user: dict = {}
    for name, p in (doc.get("presets") or {}).items():
        try:
            preset_slug(name)
            kept = {k: v for k, v in dict(p["settings"]).items() if k in BEHAVIOUR_KEYS}
            apply_config(FfbSettings(), {k: v for k, v in kept.items() if k != "wheel_lock_deg"},
                         gain_cap_pct=MAX_GAIN_CEILING_PCT, read_only={})
            user[name] = Preset(str(p["label"])[:40], str(p.get("description", "")), kept)
        except (ValueError, KeyError, TypeError) as e:
            notes.append(f"saved behaviour {name!r} dropped: {e}")
    saved = {k: v for k, v in dict(doc.get("settings") or {}).items() if k != "mode"}
    gain = saved.get("max_gain_pct")
    cap = min(gain_cap_pct, MAX_GAIN_CEILING_PCT)
    if isinstance(gain, (int, float)) and not isinstance(gain, bool) and gain > cap:
        saved["max_gain_pct"] = cap
        notes.append(f"saved gain {gain:g} % is above today's cap: {cap:g} %")
    try:
        settings = apply_config(settings, saved, gain_cap_pct=gain_cap_pct, read_only={}, user_presets=user)
    except ValueError as e:
        notes.append(f"saved settings ignored: {e}")
    return settings, user, notes


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


#: The alarm rumble: a quick square for RUMBLE_MS, RUMBLE_COUNT times, RUMBLE_EVERY_S apart.
RUMBLE_PERIOD_MS = 25
RUMBLE_MS = 400
RUMBLE_COUNT = 3
RUMBLE_EVERY_S = 1.0


def alarm_rumbles(settings: FfbSettings, priority: int, acknowledged: bool) -> bool:
    """A rumble for this alarm: rumble on, not acknowledged, at or above the set priority."""
    return settings.alarm_rumble_pct > 0 and not acknowledged and priority >= ALARM_PRIORITIES[settings.alarm_min_priority]


def rumble_due(started_at: float, played: int, now: float) -> bool:
    """Whether the next pulse of a rumble that started at ``started_at`` is due."""
    return played < RUMBLE_COUNT and now - started_at >= played * RUMBLE_EVERY_S
