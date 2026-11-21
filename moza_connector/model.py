"""Pure decisions: no evdev, no Zenoh, no clock. Every time is passed in.

Everything that decides a number the operator feels or the vessel acts on lives here,
so it is tested without a wheel, a bus or a sleep.
"""

from __future__ import annotations

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
}
CONFIG_WRITABLE = ("mode", "max_gain_pct", *CONFIG_BOUNDS, "rudder_invert")


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
    }


def apply_config(settings: FfbSettings, patch: dict, *, gain_cap_pct: float, read_only: dict) -> FfbSettings:
    """``settings`` with a remote ``set_config`` patch applied, or ValueError.

    The patch may be partial. ``max_gain_pct`` may not exceed ``gain_cap_pct`` (the command
    line's ``--ffb-max-gain-pct``), which never exceeds the ceiling. A key from
    ``read_only`` is accepted only unchanged, so a get-edit-set round trip works; any
    other key is refused. Nothing is clipped: a value out of bounds is an error.
    """
    if not isinstance(patch, dict):
        raise ValueError("set_config expects a JSON object")
    patch = expand_preset(settings, patch)
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
    """The spring ``center`` (s16, device coordinates) for a fraction of full helm lock."""
    frac = clamp(wheel_fraction, -1.0, 1.0) * settings.lock_fraction
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
    """The set_config patch a preset stands for, on a base of ``base_range_deg``."""
    try:
        preset = PRESETS[name]
    except KeyError:
        raise ValueError(f"unknown preset {name!r} (known: {', '.join(PRESETS)})") from None
    patch = dict(preset.settings)
    lock = patch.get("wheel_lock_deg")
    if lock is not None:
        patch["wheel_lock_deg"] = base_range_deg / 2.0 if lock == "full" else min(float(lock), base_range_deg / 2.0)
    return patch


def expand_preset(settings: FfbSettings, patch: dict) -> dict:
    """A set_config patch with its ``preset`` (if any) unfolded; the patch's own keys win.

    ``"custom"``, or the preset the settings already match, changes nothing: that is a
    get_config document sent back as it came.
    """
    if "preset" not in patch:
        return patch
    rest = {k: v for k, v in patch.items() if k != "preset"}
    name = patch["preset"]
    if name in ("custom", matching_preset(settings)):
        return rest
    base = rest.get("base_range_deg", settings.base_range_deg)
    return {**preset_patch(name, base), **rest}


def matching_preset(settings: FfbSettings) -> str:
    """The preset the settings are, or ``"custom"``. With force feedback off its mode is not
    compared, so the panel still shows which behaviour is loaded."""
    current = config_of(settings)
    for name in PRESETS:
        patch = preset_patch(name, settings.base_range_deg)
        if settings.mode is FfbMode.OFF:
            patch.pop("mode", None)
        if all(current[k] == v for k, v in patch.items()):
            return name
    return "custom"


def preset_list() -> list[dict]:
    """What get_config offers: each preset's name, label and description."""
    return [{"name": n, "label": p.label, "description": p.description} for n, p in PRESETS.items()]


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
