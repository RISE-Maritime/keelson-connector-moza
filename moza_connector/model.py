"""Pure decisions: no evdev, no Zenoh, no clock. Every time is passed in.

Everything that decides a number the operator feels or the vessel acts on lives here,
so it is tested without a wheel, a bus or a sleep.
"""

from __future__ import annotations

from dataclasses import dataclass
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


def clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def gain_level(max_gain_pct: float) -> int:
    """``FF_GAIN`` (0..0xFFFF) for a percentage, never above ``MAX_GAIN_CEILING_PCT``."""
    pct = clamp(max_gain_pct, 0.0, MAX_GAIN_CEILING_PCT)
    return round(pct / 100.0 * U16_MAX)


#: How long the overall gain takes to rise from 0 to its setting when force feedback
#: starts. The wheel may be anywhere when the connector comes up; a spring switched on at
#: full strength would whip it to centre.
GAIN_RAMP_S = 2.0


def ramped_gain_level(max_gain_pct: float, elapsed_s: float, ramp_s: float = GAIN_RAMP_S) -> int:
    """``FF_GAIN`` ``elapsed_s`` seconds after force feedback started: a linear ramp."""
    if ramp_s <= 0 or elapsed_s >= ramp_s:
        return gain_level(max_gain_pct)
    return round(gain_level(max_gain_pct) * max(0.0, elapsed_s) / ramp_s)


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
    """The spring ``center`` (s16, device coordinates) for a wheel fraction."""
    frac = clamp(wheel_fraction, -1.0, 1.0)
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
