"""The simulated rudder against Slipway's own numbers: its steering gear must behave as the
L1/L2 hulls' does (SLIPWAY tests/deterministic/test_l1_dynamics.py, DRYDOCK steering_gear.py)."""

import math

import pytest

from moza_connector import rudder_sim as sim

FERRY = sim.HULLS["road-ferry-78m"]


def run(delta, order, hull, seconds):
    out = []
    for _ in range(round(seconds / sim.TICK_S)):
        delta = sim.tick(delta, order, hull)
        out.append(delta)
    return out


def test_hulls_match_slipway_vessel_yaml():
    # SLIPWAY vessels/road-ferry-78m/vessel.yaml:105-107, usv-6m/vessel.yaml:111-113,
    # workboat-12m/vessel.yaml:77-79; DRYDOCK catalogue/compat/slipway_v1.py:250-251.
    assert {k: (h.max_deg, h.rate_degps) for k, h in sim.HULLS.items()} == {
        "road-ferry-78m": (35.0, 3.0),
        "usv-6m": (30.0, 30.0),
        "workboat-12m": (30.0, 20.0),
        "drydock-default": (35.0, 2.5),
    }
    assert (sim.SETTLE_TAU_S, sim.SUB_STEPS, sim.TICK_S, sim.MAX_INPUT_AGE_S) == (0.5, 10, 0.1, 1.0)


def test_the_rudder_slews_at_its_declared_rate_and_no_faster():
    # As SLIPWAY test_l1_dynamics.py:152: hard over from amidships, 1 s later <= rate * 1 s.
    angles = run(0.0, 35.0, FERRY, 1.0)
    assert angles[-1] == pytest.approx(3.0, abs=0.01)
    assert all(b - a <= FERRY.rate_degps * sim.TICK_S + 1e-9 for a, b in zip([0.0, *angles], angles))


def test_hard_over_reaches_the_stop_and_stays():
    angles = run(0.0, 35.0, FERRY, 15.0)
    assert angles[-1] == pytest.approx(35.0, abs=0.05)
    assert max(angles) <= 35.0
    # An order past the travel (a mis-scaled helm) still ends on the stop, never past it.
    assert max(run(0.0, 50.0, FERRY, 20.0)) == pytest.approx(35.0)


def test_the_last_stretch_closes_with_the_settle_time_constant():
    # Inside rate * 0.5 s of the order the gear is a pure tau = 0.5 s approach.
    hull = sim.HULLS["usv-6m"]  # 30 deg/s: the linear zone is the last 15 deg
    start, order = 0.0, 10.0
    after = run(start, order, hull, sim.SETTLE_TAU_S)[-1]
    assert (order - after) / (order - start) == pytest.approx(math.exp(-1), rel=0.01)


def test_port_and_starboard_mirror():
    stbd = run(0.0, 20.0, FERRY, 4.0)
    port = run(0.0, -20.0, FERRY, 4.0)
    assert port == pytest.approx([-a for a in stbd])


def test_order_scaling_matches_slipway_helm():
    assert sim.rudder_order_deg(50.0, FERRY) == 17.5
    assert sim.rudder_order_deg(-100.0, FERRY) == -35.0
    assert sim.rudder_order_deg(130.0, FERRY) == 35.0  # clamped like Slipway's scale_axis_value


@pytest.mark.parametrize(
    "pct, received_at, now, want",
    [
        (50.0, 0.0, 1.0, 17.5),  # exactly 1 s old: still the order
        (50.0, 0.0, 1.01, 0.0),  # older: amidships, never the last value
        (None, None, 5.0, 0.0),  # no helm yet
        (50.0, 2.0, 1.0, 0.0),  # from the future: not trusted
    ],
)
def test_dead_man(pct, received_at, now, want):
    assert sim.order_for(pct, received_at, now, FERRY) == want


# --- Faults for training, and changing hull while it runs ---------------------------------


def test_jammed_holds_whatever_is_ordered():
    assert sim.tick(12.0, -35.0, FERRY, fault="jammed") == 12.0


def test_slow_gear_runs_at_a_quarter_rate():
    angles = run_fault(0.0, 35.0, FERRY, 4.0, "slow")
    assert angles[-1] == pytest.approx(3.0, abs=0.01)  # 4 s at 0.75 deg/s


@pytest.mark.parametrize("fault, end", [("hard_over_port", -35.0), ("hard_over_stbd", 35.0)])
def test_hard_over_drives_to_the_stop_whatever_the_helm(fault, end):
    angles = run_fault(0.0, 0.0, FERRY, 15.0, fault)
    assert angles[-1] == pytest.approx(end, abs=0.05)
    assert abs(angles[9]) == pytest.approx(3.0, abs=0.01)  # still at the gear's own rate


def test_unknown_fault_is_refused():
    with pytest.raises(ValueError):
        sim.tick(0.0, 0.0, FERRY, fault="fire")


def test_sim_config_hull_travel_rate_and_fault():
    state = sim.SimState(hull_name="road-ferry-78m", hull=FERRY)
    usv = sim.apply_sim_config(state, {"hull": "usv-6m"})
    assert (usv.hull_name, usv.hull) == ("usv-6m", sim.HULLS["usv-6m"])
    custom = sim.apply_sim_config(usv, {"rudder_rate_degps": 5})
    assert custom.hull_name == "custom" and custom.hull.rate_degps == 5 and custom.hull.max_deg == 30
    faulty = sim.apply_sim_config(state, {"fault": "jammed"})
    assert faulty.fault == "jammed" and faulty.hull == FERRY
    doc = sim.sim_config(faulty, 4.2)
    assert sim.apply_sim_config(faulty, doc) == faulty  # a get-edit-set round trip changes nothing
    for bad in ({"hull": "liner"}, {"fault": "fire"}, {"max_rudder_deg": 0}, {"colour": "red"}):
        with pytest.raises(ValueError):
            sim.apply_sim_config(state, bad)
    assert sim.clamp_to_travel(34.0, sim.HULLS["usv-6m"]) == 30.0


def run_fault(delta, order, hull, seconds, fault):
    out = []
    for _ in range(round(seconds / sim.TICK_S)):
        delta = sim.tick(delta, order, hull, fault=fault)
        out.append(delta)
    return out
