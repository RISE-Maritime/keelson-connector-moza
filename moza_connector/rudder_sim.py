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


def tick(delta_deg: float, order_deg: float, hull: Hull, tick_s: float = TICK_S) -> float:
    """The rudder angle (deg) one tick later, the order held for the tick."""
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
