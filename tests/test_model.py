import json
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


# --- Profiles ------------------------------------------------------------------------------


def helm(**kw):
    s = FfbSettings(mode=model.FfbMode.CENTER, max_gain_pct=12, **kw)
    return model.Helm(s, command_line=model.behaviour_of(s))


def apply(h, patch, cap=30):
    return model.apply_helm(h, patch, gain_cap_pct=cap, read_only={})


@pytest.mark.parametrize("name", list(model.PRESETS))
def test_every_builtin_profile_is_complete_valid_and_leaves_the_gain(name):
    h = model.load_profile(helm(), name)
    values = model.profile_settings(h, name)
    assert set(values) == set(model.BEHAVIOUR_KEYS) and values["mode"] != "off"
    assert model.behaviour_of(h.settings) == values and model.modified(h) == []
    assert h.settings.max_gain_pct == 12  # a profile never touches the gain
    assert "max_gain_pct" not in model.PRESETS[name].settings


def test_a_builtin_profile_sets_the_haptics_too():
    h = apply(helm(alarm_warning_pct=50, alarm_leds=True, hardover_warn_deg=20), {"profile": "ship"})
    assert (h.settings.alarm_warning_pct, h.settings.alarm_leds, h.settings.hardover_warn_deg) == (0, False, 0)


def test_profile_lock_fits_a_small_base():
    h = helm(base_range_deg=180, wheel_lock_deg=90)
    assert apply(h, {"profile": "car"}).settings.wheel_lock_deg == 90
    assert apply(h, {"profile": "ship"}).settings.wheel_lock_deg == 90  # "full"
    with pytest.raises(ValueError, match="unknown profile"):
        apply(h, {"profile": "rocket"})


def test_modified_tracks_changes_and_revert_drops_them():
    h = apply(helm(), {"profile": "small-boat", "damper_pct": 33})
    assert (h.profile, h.settings.damper_pct, model.modified(h)) == ("small-boat", 33, ["damper_pct"])
    assert apply(h, {"max_gain_pct": 20}).settings.max_gain_pct == 20
    assert model.modified(apply(h, {"max_gain_pct": 20})) == ["damper_pct"]  # the gain is no profile's
    assert apply(h, {"profile": "small-boat"}) == h  # the selected profile again: a round trip, no reload
    back = apply(h, {"revert": True})
    assert model.modified(back) == [] and back.settings.damper_pct == 10
    doc = model.helm_config(h)
    assert apply(h, {k: v for k, v in doc.items()}) == h  # get_config sent back changes nothing


def test_save_as_selects_save_overwrites_only_your_own():
    h = apply(helm(), {"profile": "car", "spring_pct": 70})
    with pytest.raises(ValueError, match="built in"):
        apply(h, {"save_profile": True})
    mine = apply(h, {"save_profile_as": "Rivo  bridge"})
    assert (mine.profile, mine.user["rivo-bridge"].label, model.modified(mine)) == ("rivo-bridge", "Rivo bridge", [])
    assert mine.user["rivo-bridge"].settings["spring_pct"] == 70 and "max_gain_pct" not in mine.user["rivo-bridge"].settings
    tuned = apply(mine, {"damper_pct": 55, "save_profile": True})
    assert tuned.user["rivo-bridge"].settings["damper_pct"] == 55 and model.modified(tuned) == []
    with pytest.raises(ValueError, match="exists already"):
        apply(tuned, {"save_profile_as": "rivo bridge"})
    with pytest.raises(ValueError, match="built-in"):
        apply(tuned, {"save_profile_as": "Car"})
    listed = {p["name"]: p["builtin"] for p in model.profile_list(tuned)}
    assert listed["rivo-bridge"] is False and listed["car"] is True and listed[model.COMMAND_LINE] is True


def test_delete_falls_back_and_builtins_stay():
    h = apply(apply(helm(), {"save_profile_as": "mine"}), {"default_profile": "mine"})
    assert h.default_profile == "mine"
    gone = apply(h, {"delete_profile": "mine"})
    assert (gone.user, gone.default_profile, gone.profile) == ({}, model.FALLBACK_PROFILE, model.FALLBACK_PROFILE)
    with pytest.raises(ValueError, match="built in"):
        apply(h, {"delete_profile": "car"})
    with pytest.raises(ValueError, match="unknown profile"):
        apply(h, {"default_profile": "rocket"})


def test_on_off_is_a_switch_and_the_mode_is_the_profiles():
    h = helm()
    assert not h.enabled and model.effective(h).mode is model.FfbMode.OFF
    on = apply(h, {"enabled": True})
    assert model.effective(on).mode is model.FfbMode.CENTER
    detent = apply(h, {"mode": "detent"})
    assert not detent.enabled and detent.settings.mode is model.FfbMode.DETENT  # a mode never switches on
    off = apply(on, {"mode": "off"})  # Stop, and old clients
    assert not off.enabled and off.settings.mode is model.FfbMode.CENTER
    with pytest.raises(ValueError, match="contradict"):
        apply(h, {"mode": "off", "enabled": True})
    with pytest.raises(ValueError, match="true or false"):
        apply(h, {"enabled": "yes"})


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


# --- The settings file ---------------------------------------------------------------------


def test_saved_state_round_trip_never_restores_on_nor_unsaved_changes():
    h = apply(helm(), {"profile": "notched", "spring_pct": 33, "max_gain_pct": 14, "save_profile_as": "mine"})
    h = apply(h, {"default_profile": "mine", "enabled": True, "damper_pct": 77})  # the damper never saved
    doc = json.loads(json.dumps(model.saved_state(h)))
    assert "enabled" not in json.dumps(doc) and doc["version"] == 2
    back, notes = model.load_state(doc, helm(), gain_cap_pct=15)
    assert notes == [] and (back.default_profile, set(back.user)) == ("mine", {"mine"})
    back = model.load_profile(back, back.default_profile)
    assert not back.enabled
    assert (back.settings.spring_pct, back.settings.damper_pct, back.settings.max_gain_pct) == (33, 0, 14)


def test_load_state_clamps_gain_and_drops_what_no_longer_passes():
    doc = {"version": 2, "setup": {"max_gain_pct": 40}, "default_profile": "gone",
           "profiles": {"car": {"label": "x", "settings": {}}, "ok": {"label": "OK", "settings": {"spring_pct": 900}}}}
    back, notes = model.load_state(doc, helm(), gain_cap_pct=15)
    assert back.settings.max_gain_pct == 15 and back.user == {} and back.default_profile == model.COMMAND_LINE
    assert len(notes) == 4
    same, notes = model.load_state(["not", "a", "dict"], helm(), gain_cap_pct=15)
    assert same == helm() and notes


def test_a_version_1_file_keeps_what_was_tuned():
    doc = {"settings": {"mode": "detent", "max_gain_pct": 12, "spring_pct": 81, "alarm_rumble_pct": 36,
                        "alarm_min_priority": "caution"},
           "presets": {"rivo-bridge": {"label": "Rivo bridge", "settings": {"damper_pct": 44}}}}
    back, notes = model.load_state(doc, helm(), gain_cap_pct=15)
    assert back.default_profile == model.RESTORED and len(notes) == 1
    restored = model.load_profile(back, model.RESTORED)
    assert (restored.settings.spring_pct, restored.settings.alarm_caution_pct, restored.settings.alarm_emergency_pct) == (81, 36, 36)
    assert restored.settings.mode is model.FfbMode.CENTER and not restored.enabled  # the command line's mode
    bridge = back.user["rivo-bridge"].settings  # completed from the defaults, its mode never off
    assert set(bridge) == set(model.BEHAVIOUR_KEYS) and bridge["damper_pct"] == 44 and bridge["mode"] != "off"
    # Settings that are already a profile add nothing.
    plain, notes = model.load_state({"settings": {"spring_pct": 60}}, helm(), gain_cap_pct=15)
    assert plain.user == {} and notes == []


# --- Haptics ----------------------------------------------------------------------------


def test_hardover_zone_with_hysteresis():
    assert model.hardover_side(29.0, None, 30) == 0
    assert model.hardover_side(30.0, 0, 30) == 1 and model.hardover_reached(0, 1)
    assert model.hardover_side(28.5, 1, 30) == 1  # held near the line: still in, quiet
    assert model.hardover_side(27.5, 1, 30) == 0  # 2 deg back inside: out
    assert model.hardover_side(-31.0, 0, 30) == -1 and model.hardover_reached(0, -1)
    assert not model.hardover_reached(None, 1) and not model.hardover_reached(1, 0)
    assert model.hardover_side(35.0, None, 0) == 0  # off


def test_speed_weight_never_lighter_on_old_data():
    s = FfbSettings(speed_full_kn=10, speed_min_pct=40)
    assert model.speed_weight(s, 0.0, 0.1) == pytest.approx(0.4)
    assert model.speed_weight(s, 5.0, 0.1) == pytest.approx(0.7)
    assert model.speed_weight(s, 20.0, 0.1) == 1.0
    assert model.speed_weight(s, 0.0, model.SPEED_STALE_S + 0.1) == 1.0  # stale: full weight
    assert model.speed_weight(s, None, None) == 1.0
    assert model.speed_weight(FfbSettings(), 0.0, 0.1) == 1.0  # off


def test_damper_with_lag_resistance():
    s = FfbSettings(damper_pct=20, lag_resistance_pct=50)
    assert model.damper_pct_now(s, 1.0, None) == 20  # no fresh rudder: no extra
    assert model.damper_pct_now(s, 1.0, 2.5) == pytest.approx(45)
    assert model.damper_pct_now(s, 0.5, -10.0) == pytest.approx(60)
    assert model.damper_pct_now(FfbSettings(damper_pct=90, lag_resistance_pct=50), 1.0, 9) == 100


def test_patterns_follow_the_ships_signals():
    s = FfbSettings(alarm_short_ms=250, alarm_long_ms=1500, alarm_gap_ms=300)
    assert model.alarm_schedule(s, "caution") == ([(0, 250)], False)
    assert model.alarm_schedule(s, "warning") == ([(0, 250), (550, 250)], False)  # two short
    assert model.alarm_schedule(s, "emergency") == ([(0, 800), (800, 700)], False)  # one long, under leases
    general = model.alarm_schedule(replace(s, alarm_warning_pattern="general-alarm"), "warning")[0]
    assert len(general) == 7 + 2 and general[6] == (6 * 550, 250) and general[7] == (7 * 550, 800)
    mob, _ = model.alarm_schedule(replace(s, alarm_caution_pattern="man-overboard"), "caution")
    assert model.pattern_length_ms(mob) == 3 * 1500 + 2 * 300
    assert model.alarm_schedule(s, "fire")[1] is True  # continuous
    custom = replace(s, alarm_caution_pattern="custom", alarm_caution_custom="SLS")
    assert [d for _, d in model.alarm_schedule(custom, "caution")[0]] == [250, 800, 700, 250]
    assert all(d <= model.SEGMENT_MAX_MS < 1000 for _, d in general + mob)  # every play inside a lease


def test_repeats_and_continuous_periods():
    s = FfbSettings()
    sched, cont = model.alarm_schedule(s, "emergency")
    assert model.pattern_period_s(s, "emergency", sched, cont) == 5.0
    assert model.pattern_period_s(s, "warning", *model.alarm_schedule(s, "warning")) is None  # once
    fire, cont = model.alarm_schedule(s, "fire")
    assert model.pattern_period_s(s, "fire", fire, cont) == (1500 + 300) / 1000
    quick = replace(s, alarm_emergency_repeat_s=0.5)  # never overlapping itself
    assert model.pattern_period_s(quick, "emergency", sched, False) == (1500 + 300) / 1000


def test_kinds_ranks_and_the_board():
    assert [model.alarm_rank(p) for p in (2, 3, 4, 5)] == [2, 3, 4, 4]
    assert model.alarm_rank(3, model.FIRE_CATEGORY) == model.ALARM_RANKS["fire"]  # fire overrides the level
    assert [model.alarm_kind(r) for r in (0, 2, 3, 4, 5)] == [None, "caution", "warning", "emergency", "fire"]
    board, fresh = model.alarm_seen({}, "cpa", 3, False, 0.0)
    assert fresh and model.alarm_top(board, 0.0) == 3
    board, fresh = model.alarm_seen(board, "cpa", 3, False, 5.0)
    assert not fresh  # repeated: already felt
    board, fresh = model.alarm_seen(board, "cpa", 4, False, 6.0)
    assert fresh  # higher now
    board, _ = model.alarm_seen(board, "depth", 2, False, 7.0, expires_at=8.0)
    assert model.alarm_top(board, 7.5) == 4
    board, _ = model.alarm_seen(board, "cpa", 4, True, 9.0)
    assert model.alarm_top(board, 9.0) is None  # acknowledged, and the caution expired
    assert model.alarm_top(model.alarm_seen({}, "x", 2, False, 0.0)[0], model.ALARM_HOLD_S + 1) is None


def test_alarm_colours_and_settings_checks():
    s = FfbSettings(alarm_leds=True)
    assert model.alarm_light_color(s, 2, "rim") == 0xFFD400 and model.alarm_light_color(s, 5, "rim") == 0xFF0000
    assert model.alarm_light_color(s, 3, "base") is None  # the base bar is off
    assert model.alarm_light_color(s, None, "rim") is None
    new = model.apply_config(s, {"alarm_caution_color": "#00FF88", "alarm_warning_pattern": "man-overboard",
                                 "alarm_fire_custom": "sls"}, gain_cap_pct=30, read_only={})
    assert (new.alarm_caution_color, new.alarm_warning_pattern, new.alarm_fire_custom) == ("#00ff88", "man-overboard", "SLS")
    for bad in ({"alarm_warning_color": "orange"}, {"alarm_caution_pattern": "siren"}, {"alarm_caution_custom": "SXL"},
                {"alarm_caution_custom": "S" * 17}, {"alarm_long_ms": 9000}):
        with pytest.raises(ValueError):
            model.apply_config(s, bad, gain_cap_pct=30, read_only={})


def test_settings_from_before_per_level_alarms_are_upgraded():
    assert model.upgrade_alarm_keys({"alarm_rumble_pct": 40, "alarm_min_priority": "warning", "spring_pct": 1}) == {
        "spring_pct": 1, "alarm_caution_pct": 0.0, "alarm_warning_pct": 40, "alarm_emergency_pct": 40, "alarm_fire_pct": 40}
    assert model.upgrade_alarm_keys({"spring_pct": 1}) == {"spring_pct": 1}


def test_haptics_config_checks():
    with pytest.raises(ValueError, match="under rudder_max_deg"):
        model.apply_config(FfbSettings(rudder_max_deg=30), {"hardover_warn_deg": 30}, gain_cap_pct=30, read_only={})
    with pytest.raises(ValueError, match="unknown setting"):
        model.apply_config(FfbSettings(), {"alarm_min_priority": "warning"}, gain_cap_pct=30, read_only={})


def test_test_pulse_level_is_the_pulses_own_strength():
    s = FfbSettings(center_mark_pct=80, detent_pct=0, hardover_pct=30, alarm_emergency_pct=60)
    assert model.test_pulse_level(s, "center") == model.coeff_level(80)
    assert model.test_pulse_level(s, "hardover") == model.coeff_level(30)
    assert model.test_pulse_level(s, "emergency") == model.coeff_level(60)
    with pytest.raises(ValueError, match="Click strength is 0"):
        model.test_pulse_level(s, "detent")
    with pytest.raises(ValueError, match="no pulse"):
        model.test_pulse_level(s, "spring")


# --- The centre notch ---------------------------------------------------------------------


def test_notch_is_a_v_full_pull_in_to_its_core_and_nothing_past_it():
    s = FfbSettings(mode=FfbMode.DETENT, center_notch_pct=50, center_notch_deg=6)
    core = 6 * model.NOTCH_CORE_SHARE  # 1.5 deg
    assert model.notch_force(0.0, s) == 0.0
    assert model.notch_force(core / 2, s) == pytest.approx(-0.5)  # easing into centre
    for deg in (core, 3.0, 5.9):
        assert model.notch_force(deg, s) == pytest.approx(-1.0)  # the full pull, pulled back to port
    assert model.notch_force(6.0, s) == 0.0 and model.notch_force(40.0, s) == 0.0
    for deg in (0.5, 1.5, 3.0, 4.5):
        assert model.notch_force(-deg, s) == pytest.approx(-model.notch_force(deg, s))  # odd
    # A narrow notch keeps a core of at least NOTCH_CORE_MIN_DEG.
    narrow = replace(s, center_notch_deg=1)
    assert model.notch_force(model.NOTCH_CORE_MIN_DEG / 2, narrow) == pytest.approx(-0.5)
    assert model.notch_force(0.5, narrow) == pytest.approx(-1.0)


def test_axis_rate_over_at_least_its_window():
    w = model.NOTCH_RATE_WINDOW_S
    assert model.axis_rate([], 1.0, 5.0) == 0.0
    assert model.axis_rate([(1.0 - w / 2, 0.0)], 1.0, 5.0) == 0.0  # too recent to tell
    samples = [(0.9, 0.0), (1.0 - 2 * w, 1.0), (1.0 - w / 2, 4.0)]
    assert model.axis_rate(samples, 1.0, 2.0) == pytest.approx(1.0 / (2 * w))  # the newest old enough
    assert model.axis_rate(samples, 1.0, 0.0) == pytest.approx(-1.0 / (2 * w))


def test_notch_damping_opposes_the_wheel_inside_the_notch_only():
    s = FfbSettings(mode=FfbMode.DETENT, center_notch_pct=50, center_notch_deg=6, center_notch_damping_pct=50)
    full = model.NOTCH_DAMP_FULL_DEG_S
    # 1 deg out (two thirds of the way into the core), moving: falling in pulls less, climbing out more.
    still = model.notch_force(1.0, s)
    assert model.notch_force(1.0, s, -full / 4) == pytest.approx(still + 0.125)
    assert model.notch_force(1.0, s, full / 4) == pytest.approx(still - 0.125)
    # At centre it only brakes; never more than the notch's full pull.
    assert model.notch_force(0.0, s, full / 2) == pytest.approx(-0.25)
    assert model.notch_force(0.0, s, 100 * full) == -1.0 and model.notch_force(3.0, s, -100 * full) == 1.0
    # Undamped, it is the plain V; past the notch nothing, whatever the speed.
    assert model.notch_force(1.0, replace(s, center_notch_damping_pct=0), full) == still
    assert model.notch_force(6.0, s, full) == 0.0 and model.notch_force(-9.0, s, -full) == 0.0
    # The axis' rate in %/s becomes degrees the same way its position does (900 deg base).
    s9 = replace(s, base_range_deg=900)
    pct_per_deg = 100 / 450
    assert model.notch_level(0.0, s9, full / 2 * pct_per_deg) == round(model.NOTCH_SIGN * -0.25 * 0.5 * model.S16_MAX)


def test_notch_only_when_on_and_never_in_follow_up():
    s = FfbSettings(mode=FfbMode.CENTER, center_notch_pct=50, center_notch_deg=6)
    assert model.notch_active(s) and model.notch_active(replace(s, mode=FfbMode.DETENT))
    for off in (replace(s, center_notch_pct=0), replace(s, mode=FfbMode.FOLLOW_UP), replace(s, mode=FfbMode.OFF)):
        assert not model.notch_active(off) and model.notch_force(3.0, off) == 0.0


def test_notch_level_from_the_axis_in_degrees_and_device_coordinates():
    s = FfbSettings(mode=FfbMode.DETENT, base_range_deg=900, center_notch_pct=50, center_notch_deg=6)
    axis = 3.0 / 450 * 100  # 3 deg to starboard on a 900 deg base: the full pull
    level = model.notch_level(axis, s)
    assert level == round(model.NOTCH_SIGN * -0.5 * model.S16_MAX)
    assert model.notch_level(-axis, s) == -level
    assert model.notch_level(axis, replace(s, wheel_invert=True)) == -level  # the device's own direction
    assert model.notch_level(0.0, s) == 0 and model.notch_level(50.0, s) == 0
    assert model.notch_level(axis, replace(s, center_notch_pct=100)) == round(model.NOTCH_SIGN * -model.S16_MAX)


def test_notch_settings_are_bounded_a_profiles_and_saved():
    with pytest.raises(ValueError):
        model.apply_config(FfbSettings(), {"center_notch_pct": 101}, gain_cap_pct=15, read_only={})
    with pytest.raises(ValueError):
        model.apply_config(FfbSettings(), {"center_notch_deg": 0.5}, gain_cap_pct=15, read_only={})
    with pytest.raises(ValueError):
        model.apply_config(FfbSettings(), {"center_notch_damping_pct": 101}, gain_cap_pct=15, read_only={})
    new = model.apply_config(FfbSettings(), {"center_notch_pct": 40, "center_notch_deg": 8}, gain_cap_pct=15, read_only={})
    assert (new.center_notch_pct, new.center_notch_deg) == (40, 8)
    assert {"center_notch_pct", "center_notch_deg", "center_notch_damping_pct"} <= set(model.BEHAVIOUR_KEYS)
    h = apply(helm(), {"center_notch_pct": 40, "center_notch_deg": 8, "center_notch_damping_pct": 70, "save_profile_as": "notch"})
    back, notes = model.load_state(json.loads(json.dumps(model.saved_state(h))), helm(), gain_cap_pct=15)
    assert notes == []
    restored = model.load_profile(back, "notch").settings
    assert (restored.center_notch_pct, restored.center_notch_deg, restored.center_notch_damping_pct) == (40, 8, 70)
    # A file from before the notch: off.
    old, _ = model.load_state({"settings": {"spring_pct": 60}}, helm(), gain_cap_pct=15)
    assert old.settings.center_notch_pct == 0 and old.settings.center_notch_damping_pct == 50
