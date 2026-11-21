"""A rudder to test the wheel against: Slipway's steering gear, mirrored. Pure.

No clock, no Zenoh: every time is passed in, as in ``model.py``. ``rudder_sim_app`` runs
it on the bus. It mirrors -- does not import -- the gear Slipway's L1/L2 hulls use
(DRYDOCK is private and not on PyPI); ``tests/test_rudder_sim.py`` pins the numbers.

- Order (SLIPWAY services/kernel/slipway_kernel/guidance.py:198, commands.py:87-101):
  ``clamp(pct, -100, 100) / 100 * max_rudder_deg``, positive to starboard, no deadband.
- Gear (DRYDOCK src/drydock/parts/steering/steering_gear.py:79-84, docs/INTEGRATION.md):
  ``rate = clamp((order - delta) / 0.5 s, +-rate_degps)`` -- rate-limited, not a lag --
  then the stop, ``clamp(delta, +-max)``, after every sub-step.
- Integration (DRYDOCK src/drydock/integrate/plant.py:64, 242-251): a 100 ms tick of ten
  RK4 sub-steps, the order held for the tick, the angle in radians inside.
- Dead-man (SLIPWAY runtime.py:600-640, ``max_axis_age_s``): no helm sample for 1 s and
  the order is amidships. The last value is never held.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: DRYDOCK steering_gear.py:29, STEERING_SETTLE_TAU_S.
SETTLE_TAU_S = 0.5
#: DRYDOCK plant.py:64, SUB_STEPS.
SUB_STEPS = 10
#: Slipway's tick in every scenario (``dt_ms: 100``); it publishes rudder_angle_deg each tick.
TICK_S = 0.1
#: SLIPWAY ``max_axis_age_s`` default: an older helm sample is no order at all.
MAX_INPUT_AGE_S = 1.0


@dataclass(frozen=True)
class Hull:
    max_deg: float
    rate_degps: float


#: SLIPWAY vessels/<id>/vessel.yaml, and DRYDOCK's default when a package omits them
#: (catalogue/compat/slipway_v1.py:250-251).
HULLS = {
    "road-ferry-78m": Hull(max_deg=35.0, rate_degps=3.0),  # hull: max_rudder_deg / rudder_rate_degps
    "usv-6m": Hull(max_deg=30.0, rate_degps=30.0),  # steering: max_deg / rate_degps
    "workboat-12m": Hull(max_deg=30.0, rate_degps=20.0),  # steering: max_deg / rate_degps
    "drydock-default": Hull(max_deg=35.0, rate_degps=2.5),
}


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def rudder_order_deg(pct: float, hull: Hull) -> float:
    """The order for a helm at ``pct`` (-100 port .. +100 starboard)."""
    return _clamp(pct, -100.0, 100.0) / 100.0 * hull.max_deg


def gear_rate(delta_rad: float, order_rad: float, hull: Hull) -> float:
    """The gear's slew (rad/s): full rate, easing in over the last ``rate * 0.5 s``."""
    rate_max = math.radians(hull.rate_degps)
    return _clamp((order_rad - delta_rad) / SETTLE_TAU_S, -rate_max, rate_max)


def step(delta_rad: float, order_rad: float, hull: Hull, dt: float) -> float:
    """One RK4 sub-step, then the stop."""
    k1 = gear_rate(delta_rad, order_rad, hull)
    k2 = gear_rate(delta_rad + dt / 2 * k1, order_rad, hull)
    k3 = gear_rate(delta_rad + dt / 2 * k2, order_rad, hull)
    k4 = gear_rate(delta_rad + dt * k3, order_rad, hull)
    delta_rad += dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
    max_rad = math.radians(hull.max_deg)
    return _clamp(delta_rad, -max_rad, max_rad)


#: Steering-gear faults for training. ``jammed`` holds the rudder where it is, as Slipway's
#: jammed gear (SLIPWAY tests/deterministic/test_actuator_faults.py); ``slow`` runs the gear
#: at SLOW_FACTOR of its rate; ``hard_over_*`` drives it to that stop whatever the helm says.
FAULTS = ("none", "jammed", "slow", "hard_over_port", "hard_over_stbd")
SLOW_FACTOR = 0.25


def tick(delta_deg: float, order_deg: float, hull: Hull, tick_s: float = TICK_S, fault: str = "none") -> float:
    """The rudder angle (deg) one tick later, the order held for the tick, with ``fault``."""
    if fault not in FAULTS:
        raise ValueError(f"unknown fault {fault!r}")
    if fault == "jammed":
        return delta_deg
    if fault == "slow":
        hull = Hull(max_deg=hull.max_deg, rate_degps=hull.rate_degps * SLOW_FACTOR)
    elif fault == "hard_over_port":
        order_deg = -hull.max_deg
    elif fault == "hard_over_stbd":
        order_deg = hull.max_deg
    delta, order = math.radians(delta_deg), math.radians(order_deg)
    for _ in range(SUB_STEPS):
        delta = step(delta, order, hull, tick_s / SUB_STEPS)
    return math.degrees(delta)


def order_for(pct: float | None, received_at: float | None, now: float, hull: Hull) -> float:
    """The order this tick: the helm's, or amidships once it has been silent ``MAX_INPUT_AGE_S``."""
    if pct is None or received_at is None:
        return 0.0
    if not 0.0 <= now - received_at <= MAX_INPUT_AGE_S:
        return 0.0
    return rudder_order_deg(pct, hull)


# --- Changing it while it runs (configurable/v1) -----------------------------------------


@dataclass(frozen=True)
class SimState:
    hull_name: str  # a key of HULLS, or "custom" once travel or rate are set by hand
    hull: Hull
    fault: str = "none"


def sim_config(state: SimState, delta_deg: float) -> dict:
    """What get_config returns: the settings, the choices, and where the rudder is."""
    return {
        "hull": state.hull_name,
        "max_rudder_deg": state.hull.max_deg,
        "rudder_rate_degps": state.hull.rate_degps,
        "fault": state.fault,
        "hulls": [{"name": n, "max_deg": h.max_deg, "rate_degps": h.rate_degps} for n, h in HULLS.items()],
        "faults": list(FAULTS),
        "rudder_deg": round(delta_deg, 3),
    }


def apply_sim_config(state: SimState, patch: dict) -> SimState:
    """``state`` with a set_config patch applied, or ValueError. A hull sets travel and rate;
    travel or rate given as well win, and make the hull "custom". The choices and the angle
    (read-only) are ignored, so a get-edit-set round trip works."""
    if not isinstance(patch, dict):
        raise ValueError("set_config expects a JSON object")
    unknown = set(patch) - {"hull", "max_rudder_deg", "rudder_rate_degps", "fault", "hulls", "faults", "rudder_deg"}
    if unknown:
        raise ValueError(f"unknown setting(s) {sorted(unknown)}")
    name, hull, fault = state.hull_name, state.hull, state.fault
    if "hull" in patch and patch["hull"] != name:
        if patch["hull"] not in HULLS:
            raise ValueError(f"hull must be one of {list(HULLS)}, got {patch['hull']!r}")
        name, hull = patch["hull"], HULLS[patch["hull"]]
    for key, lo, hi in (("max_rudder_deg", 1.0, 90.0), ("rudder_rate_degps", 0.1, 90.0)):
        if key not in patch:
            continue
        value = patch[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not lo <= value <= hi:
            raise ValueError(f"{key} must be a number {lo:g}..{hi:g}, got {value!r}")
        if value != (hull.max_deg if key == "max_rudder_deg" else hull.rate_degps):
            hull = Hull(
                max_deg=float(value) if key == "max_rudder_deg" else hull.max_deg,
                rate_degps=float(value) if key == "rudder_rate_degps" else hull.rate_degps,
            )
            name = "custom"
    if "fault" in patch:
        if patch["fault"] not in FAULTS:
            raise ValueError(f"fault must be one of {list(FAULTS)}, got {patch['fault']!r}")
        fault = patch["fault"]
    return SimState(hull_name=name, hull=hull, fault=fault)


def clamp_to_travel(delta_deg: float, hull: Hull) -> float:
    """A rudder already past a new, shorter travel ends on its stop."""
    return _clamp(delta_deg, -hull.max_deg, hull.max_deg)
