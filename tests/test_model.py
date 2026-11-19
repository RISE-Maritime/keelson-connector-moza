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
