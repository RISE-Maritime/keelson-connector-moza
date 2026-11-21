from dataclasses import replace

import pytest

from moza_connector import model
from moza_connector.model import FfbMode, FfbSettings


class TestNormalize:
    def test_bipolar_ends_and_middle(self):
        assert model.normalize(0, 0, 65535, bipolar=True) == -100.0
        assert model.normalize(65535, 0, 65535, bipolar=True) == 100.0
        assert model.normalize(32767, 0, 65535, bipolar=True) == pytest.approx(0.0, abs=0.01)

    def test_signed_range(self):
        assert model.normalize(-32768, -32768, 32767, bipolar=True) == -100.0
        assert model.normalize(32767, -32768, 32767, bipolar=True) == 100.0

    def test_unipolar(self):
        assert model.normalize(0, 0, 1000, bipolar=False) == 0.0
        assert model.normalize(250, 0, 1000, bipolar=False) == 25.0

    def test_invert(self):
        assert model.normalize(0, 0, 65535, bipolar=True, invert=True) == 100.0
        assert model.normalize(1000, 0, 1000, bipolar=False, invert=True) == 0.0

    def test_out_of_range_is_clamped(self):
        assert model.normalize(70000, 0, 65535, bipolar=True) == 100.0
        assert model.normalize(-5, 0, 65535, bipolar=False) == 0.0

    def test_bad_range(self):
        with pytest.raises(ValueError):
            model.normalize(0, 10, 10, bipolar=True)


def test_snap():
    assert model.snap(0.3, 0.5) == 0.0
    assert model.snap(-0.3, 0.5) == 0.0
    assert model.snap(0.6, 0.5) == 0.6
    assert model.snap(0.3, 0) == 0.3


class TestShouldPublish:
    kw = dict(max_hz=50.0, deadband_pct=2.0)

    def test_first_reading_always(self):
        assert model.should_publish(0.0, 1.0, None, None, **self.kw)

    def test_unchanged_never(self):
        assert not model.should_publish(10.0, 1.0, 1.0, 0.0, **self.kw)

    def test_small_change_waits_for_the_interval(self):
        assert not model.should_publish(0.01, 1.5, 1.0, 0.0, **self.kw)
        assert model.should_publish(0.02, 1.5, 1.0, 0.0, **self.kw)

    def test_big_jump_bypasses_the_cap(self):
        assert model.should_publish(0.001, 5.0, 1.0, 0.0, **self.kw)

    def test_no_cap(self):
        assert model.should_publish(0.0, 1.5, 1.0, 0.0, max_hz=0, deadband_pct=2.0)


class TestGain:
    def test_ceiling_holds_whatever_is_asked(self):
        assert model.gain_level(100) == model.gain_level(model.MAX_GAIN_CEILING_PCT)
        assert model.gain_level(100) < model.U16_MAX

    def test_zero(self):
        assert model.gain_level(0) == 0
        assert model.gain_level(-10) == 0

    def test_ramp(self):
        full = model.gain_level(30)
        assert model.ramped_gain_level(30, 0.0) == 0
        assert model.ramped_gain_level(30, model.GAIN_RAMP_S / 2) == round(full / 2)
        assert model.ramped_gain_level(30, model.GAIN_RAMP_S) == full
        assert model.ramped_gain_level(30, 100.0) == full

    def test_levels(self):
        assert model.coeff_level(100) == model.S16_MAX
        assert model.coeff_level(150) == model.S16_MAX
        assert model.saturation_level(100) == model.U16_MAX


class TestFollowUp:
    s = FfbSettings(mode=FfbMode.FOLLOW_UP, rudder_max_deg=35.0, stale_s=1.0)

    def test_hard_over_is_full_lock(self):
        assert model.rudder_to_wheel_fraction(35.0, self.s) == 1.0
        assert model.rudder_to_wheel_fraction(-35.0, self.s) == -1.0
        assert model.rudder_to_wheel_fraction(50.0, self.s) == 1.0

    def test_half_rudder_is_half_wheel(self):
        assert model.rudder_to_wheel_fraction(17.5, self.s) == 0.5

    def test_rudder_invert(self):
        s = FfbSettings(rudder_invert=True)
        assert model.rudder_to_wheel_fraction(35.0, s) == -1.0

    def test_wheel_invert_flips_the_centre(self):
        assert model.center_level(0.5, FfbSettings(wheel_invert=True)) == -model.center_level(0.5, FfbSettings())

    def test_fresh_rudder_moves_the_centre(self):
        t = model.spring_target(self.s, now=10.0, rudder_deg=17.5, rudder_received_at=9.5)
        assert t.enabled and t.source == "rudder"
        assert t.center == round(0.5 * model.S16_MAX)

    def test_stale_rudder_falls_back_to_centre(self):
        t = model.spring_target(self.s, now=10.0, rudder_deg=17.5, rudder_received_at=8.9)
        assert t == model.SpringTarget(enabled=True, center=0, source="center")

    def test_no_rudder_yet_is_centre(self):
        t = model.spring_target(self.s, now=10.0, rudder_deg=None, rudder_received_at=None)
        assert t.source == "center" and t.center == 0

    def test_reading_from_the_future_is_not_trusted(self):
        t = model.spring_target(self.s, now=10.0, rudder_deg=17.5, rudder_received_at=11.0)
        assert t.source == "center"

    def test_center_mode_ignores_the_rudder(self):
        s = FfbSettings(mode=FfbMode.CENTER)
        t = model.spring_target(s, now=10.0, rudder_deg=17.5, rudder_received_at=10.0)
        assert t.source == "center" and t.center == 0

    def test_off(self):
        s = FfbSettings(mode=FfbMode.OFF)
        assert not model.spring_target(s, now=0.0, rudder_deg=None, rudder_received_at=None).enabled


def test_needs_update():
    a = model.SpringTarget(True, 1000, "rudder")
    assert model.needs_update(None, a)
    assert not model.needs_update(a, model.SpringTarget(True, 1000 + model.CENTER_EPSILON, "rudder"))
    assert model.needs_update(a, model.SpringTarget(True, 1000 + model.CENTER_EPSILON + 1, "rudder"))
    assert model.needs_update(a, model.SpringTarget(False))


# --- Remote settings (--ffb-remote) ------------------------------------------------------

RO = {"gain_cap_pct": 30.0, "ceiling_pct": 60.0, "has_ff": True}


def test_apply_config_partial_patch():
    s = model.FfbSettings(mode=model.FfbMode.OFF, max_gain_pct=30)
    new = model.apply_config(s, {"mode": "center", "max_gain_pct": 10}, gain_cap_pct=30, read_only=RO)
    assert new.mode is model.FfbMode.CENTER and new.max_gain_pct == 10.0
    assert new.spring_pct == s.spring_pct and new.wheel_invert == s.wheel_invert


@pytest.mark.parametrize(
    "patch, message",
    [
        ({"max_gain_pct": 31}, "cap is 30"),
        ({"max_gain_pct": -1}, "outside"),
        ({"spring_pct": 101}, "outside"),
        ({"rudder_max_deg": 0}, "outside"),
        ({"stale_s": 0}, "outside"),
        ({"mode": "max"}, "mode must be"),
        ({"max_gain_pct": "10"}, "must be a number"),
        ({"max_gain_pct": True}, "must be a number"),
        ({"rudder_invert": 1}, "true or false"),
        ({"wheel_invert": True}, "unknown setting"),
        ({"gain_cap_pct": 60}, "read-only"),
    ],
)
def test_apply_config_refuses(patch, message):
    with pytest.raises(ValueError, match=message):
        model.apply_config(model.FfbSettings(), patch, gain_cap_pct=30, read_only=RO)


def test_apply_config_never_above_the_ceiling():
    with pytest.raises(ValueError, match="cap is 60"):
        model.apply_config(model.FfbSettings(), {"max_gain_pct": 61}, gain_cap_pct=99, read_only={})


def test_apply_config_round_trip():
    s = model.FfbSettings()
    doc = {**model.config_of(s), **RO}
    assert model.apply_config(s, doc, gain_cap_pct=30, read_only=RO) == s


def test_gain_ramp_rises_from_where_it_is_and_drops_at_once():
    lo, hi = model.gain_level(10), model.gain_level(30)
    assert model.gain_ramp_level(lo, hi, 0.0) == lo
    assert model.gain_ramp_level(lo, hi, model.GAIN_RAMP_S / 2) == round((lo + hi) / 2)
    assert model.gain_ramp_level(lo, hi, model.GAIN_RAMP_S) == hi
    assert model.gain_ramp_level(hi, lo, 0.0) == lo
    ceiling = model.gain_level(model.MAX_GAIN_CEILING_PCT)
    assert model.gain_ramp_level(0, model.U16_MAX, 99.0) == ceiling


def test_helm_lock_fraction_and_wheel_scale():
    assert model.lock_fraction(90, 900) == pytest.approx(0.2)
    assert model.lock_fraction(450, 900) == 1.0
    assert model.lock_fraction(900, 900) == 1.0  # never past the base's own end
    assert model.steering_pct(10.0, 0.2) == pytest.approx(50.0)
    assert model.steering_pct(-60.0, 0.2) == -100.0
    with pytest.raises(ValueError):
        model.lock_fraction(0, 900)


def test_follow_up_centre_and_end_stop_use_the_lock():
    s = FfbSettings(base_range_deg=900, wheel_lock_deg=90)
    assert model.center_level(1.0, s) == round(0.2 * model.S16_MAX)
    assert model.center_level(-0.5, s) == round(-0.1 * model.S16_MAX)
    assert model.endstop_enabled(s) and model.endstop_deadband(s) == round(0.2 * model.U16_MAX)
    assert not model.endstop_enabled(FfbSettings())  # lock at the base's own end
    assert not model.endstop_enabled(FfbSettings(wheel_lock_deg=90, endstop_pct=0))


def test_apply_config_helm_range():
    s = FfbSettings()
    new = model.apply_config(s, {"wheel_lock_deg": 90}, gain_cap_pct=30, read_only={})
    assert new.lock_fraction == pytest.approx(0.2)
    with pytest.raises(ValueError, match="past the base's own end"):
        model.apply_config(s, {"wheel_lock_deg": 500}, gain_cap_pct=30, read_only={})
    with pytest.raises(ValueError, match="past the base's own end"):
        model.apply_config(new, {"base_range_deg": 120}, gain_cap_pct=30, read_only={})
    assert model.apply_config(new, {"base_range_deg": 1080}, gain_cap_pct=30, read_only={}).lock_fraction == (
        pytest.approx(90 / 540)
    )


def test_detent_notches_with_hysteresis():
    assert model.detent_notch(0.0, None, 5) == 0
    assert model.detent_notch(2.6, 0, 5) == 0  # just past half a step: not yet
    assert model.detent_notch(3.3, 0, 5) == 1  # past half a step plus hysteresis
    assert model.detent_notch(2.0, 1, 5) == 1  # back below the boundary: still 1
    assert model.detent_notch(1.6, 1, 5) == 0
    assert model.detent_notch(-12.0, 0, 5) == -2  # a fast turn lands in the right notch
    with pytest.raises(ValueError):
        model.detent_notch(1.0, 0, 0)


def test_detent_mode_has_no_spring_and_rudder_order_scale():
    s = FfbSettings(mode=model.FfbMode.DETENT, rudder_max_deg=35)
    assert model.spring_target(s, now=0, rudder_deg=10, rudder_received_at=0) == model.SpringTarget(
        enabled=False, source="free"
    )
    assert model.rudder_order_deg(50.0, s) == 17.5
    assert model.rudder_order_deg(150.0, s) == 35.0
    assert model.CLICK_DURATION_MS < 1000  # always inside an HID lease


# --- Behaviour presets ---------------------------------------------------------------------


@pytest.mark.parametrize("name", list(model.PRESETS))
def test_every_preset_is_a_valid_patch_and_is_recognised(name):
    s = FfbSettings(max_gain_pct=12)
    new = model.apply_config(s, {"preset": name}, gain_cap_pct=30, read_only={})
    assert model.matching_preset(new) == name
    assert new.max_gain_pct == 12  # a preset never touches the gain
    assert "max_gain_pct" not in model.PRESETS[name].settings


def test_preset_lock_fits_a_small_base():
    s = FfbSettings(base_range_deg=180, wheel_lock_deg=90)
    assert model.apply_config(s, {"preset": "car"}, gain_cap_pct=30, read_only={}).wheel_lock_deg == 90
    assert model.apply_config(s, {"preset": "ship"}, gain_cap_pct=30, read_only={}).wheel_lock_deg == 90  # "full"
    # A base range in the same patch is the one the lock is fitted to.
    both = model.apply_config(s, {"preset": "ship", "base_range_deg": 540}, gain_cap_pct=30, read_only={})
    assert both.wheel_lock_deg == 270


def test_preset_explicit_keys_win_and_round_trips_change_nothing():
    s = FfbSettings()
    new = model.apply_config(s, {"preset": "small-boat", "damper_pct": 33}, gain_cap_pct=30, read_only={})
    assert new.damper_pct == 33 and new.wheel_lock_deg == 90
    assert model.matching_preset(new) == "custom"
    assert model.apply_config(new, {"preset": "custom"}, gain_cap_pct=30, read_only={}) == new
    boat = model.apply_config(s, {"preset": "small-boat"}, gain_cap_pct=30, read_only={})
    doc = {**model.config_of(boat), "preset": "small-boat"}
    assert model.apply_config(boat, doc, gain_cap_pct=30, read_only={}) == boat
    with pytest.raises(ValueError, match="unknown preset"):
        model.apply_config(s, {"preset": "rocket"}, gain_cap_pct=30, read_only={})


def test_preset_is_shown_while_off():
    s = model.apply_config(FfbSettings(), {"preset": "notched"}, gain_cap_pct=30, read_only={})
    assert model.matching_preset(replace(s, mode=model.FfbMode.OFF)) == "notched"


def test_center_mark_on_arriving_or_passing_never_on_leaving():
    side = model.center_side(5.0, None)
    assert side == 1 and not model.center_reached(None, side)  # the first reading marks nothing
    new = model.center_side(0.5, side)
    assert new == 0 and model.center_reached(side, new)  # arriving at centre
    assert model.center_side(1.8, 0) == 0  # resting near centre: still at it, no buzz
    left = model.center_side(2.5, 0)
    assert left == 1 and not model.center_reached(0, left)  # leaving: nothing
    assert model.center_reached(1, model.center_side(-30.0, 1))  # straight over centre between readings
    s = FfbSettings(wheel_lock_deg=90)
    assert model.wheel_deg(50.0, s) == 45.0
    assert model.CENTER_MARK_MS == 2 * model.CLICK_DURATION_MS < 1000


# --- Zero at centre: the helm deadband ----------------------------------------------------


def test_helm_deadband_is_zero_inside_continuous_and_full_at_lock():
    assert model.helm_deadband(1.5, 2.0) == 0.0
    assert model.helm_deadband(-2.0, 2.0) == 0.0
    assert model.helm_deadband(2.0001, 2.0) == pytest.approx(0.0, abs=1e-3)  # no jump at the edge
    assert model.helm_deadband(100.0, 2.0) == 100.0
    assert model.helm_deadband(-51.0, 2.0) == pytest.approx(-50.0)
    assert model.helm_deadband(37.0, 0.0) == 37.0  # no band: unchanged


def test_follow_up_centre_inverts_the_deadband():
    s = FfbSettings(wheel_lock_deg=450, center_deadband_deg=45)  # a 10 % band
    assert s.deadband_pct == pytest.approx(10.0)
    assert model.center_level(0.0, s) == 0
    assert model.center_level(1.0, s) == model.S16_MAX  # full rudder: full lock
    half = model.center_level(0.5, s) / model.S16_MAX * 100  # the wheel that orders half rudder
    assert model.helm_deadband(half, s.deadband_pct) == pytest.approx(50.0, abs=0.01)
    assert model.center_level(-0.5, s) == -model.center_level(0.5, s)


def test_apply_config_deadband_and_boost():
    s = FfbSettings(wheel_lock_deg=90)
    new = model.apply_config(s, {"center_deadband_deg": 10, "spring_boost": 3}, gain_cap_pct=30, read_only={})
    assert (new.center_deadband_deg, model.spring_layers(new)) == (10.0, 3)
    with pytest.raises(ValueError, match="under half"):
        model.apply_config(replace(s, wheel_lock_deg=40), {"center_deadband_deg": 20}, gain_cap_pct=30, read_only={})
    with pytest.raises(ValueError, match="outside"):
        model.apply_config(s, {"spring_boost": 4}, gain_cap_pct=30, read_only={})
