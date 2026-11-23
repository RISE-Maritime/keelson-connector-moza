import threading
from dataclasses import replace

import pytest

from moza_connector import codes, model
from moza_connector.device import DeviceGone, FakeDevice
from moza_connector.ffb import FfbController
from moza_connector.model import FfbMode, FfbSettings


def fake(ff=True):
    return FakeDevice({"axes": {"ABS_X": [0, 65535, 32767]}, "ff": ff})


def effect_ids(dev):
    """Every effect uploaded: conditions and clicks (the detent click and the centre mark)."""
    return {e[2] for e in dev.ff_log if e[0] == "upload"} | {e[1] for e in dev.ff_log if e[0] == "click"}


def uploads(dev, effect_type):
    return [e for e in dev.ff_log if e[0] == "upload" and e[1] == effect_type]


def test_off_never_touches_the_wheel():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.OFF))
    c.start(0.0)
    c.tick(1.0)
    c.shutdown()
    assert dev.ff_log == []


def test_no_ff_on_the_device_disables_it():
    dev = fake(ff=False)
    c = FfbController(dev, FfbSettings())
    assert not c.active
    c.start(0.0)
    assert dev.ff_log == []


def test_start_kills_autocentre_ramps_gain_and_plays_spring_and_damper():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.CENTER, max_gain_pct=30, damper_pct=20))
    c.start(0.0)
    assert dev.ff_log[0] == ("autocenter", 0)
    assert ("gain", 0) in dev.ff_log
    assert len(uploads(dev, codes.FF_DAMPER)) == 1
    spring = uploads(dev, codes.FF_SPRING)
    assert len(spring) == 1 and spring[0][3] == 0  # centred
    assert ("play", spring[0][2], True) in dev.ff_log

    c.tick(model.GAIN_RAMP_S / 2)
    c.tick(model.GAIN_RAMP_S + 1)
    gains = [e[1] for e in dev.ff_log if e[0] == "gain"]
    assert gains == sorted(gains) and gains[-1] == model.gain_level(30)


def test_no_damper_when_zero():
    dev = fake()
    FfbController(dev, FfbSettings(mode=FfbMode.CENTER, damper_pct=0)).start(0.0)
    assert uploads(dev, codes.FF_DAMPER) == []


def test_follow_up_moves_the_same_effect_then_falls_back_when_stale():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.FOLLOW_UP, rudder_max_deg=35, stale_s=1.0))
    c.start(0.0)
    c.on_rudder(17.5, 0.1)
    assert c.tick(0.2).source == "rudder"
    c.tick(0.5)  # unchanged: no new upload
    assert c.tick(1.5).source == "center"
    springs = uploads(dev, codes.FF_SPRING)
    assert [s[3] for s in springs] == [0, round(0.5 * model.S16_MAX), 0]
    assert len({s[2] for s in springs}) == 1  # updated in place, one effect id
    assert sum(1 for e in dev.ff_log if e[0] == "play" and e[2] is True) == 2  # damper + spring, once each


def test_shutdown_leaves_the_wheel_limp():
    dev = fake()
    c = FfbController(dev, FfbSettings())
    c.start(0.0)
    c.shutdown()
    erased = {e[1] for e in dev.ff_log if e[0] == "erase"}
    assert erased == effect_ids(dev) and erased
    assert dev.ff_log[-1] == ("gain", 0)


def test_shutdown_never_raises_on_a_vanished_base():
    dev = FakeDevice({"axes": {"ABS_X": [0, 65535, 32767]}, "events": [[0, "gone"]]})
    c = FfbController(dev, FfbSettings())
    c.start(0.0)
    with pytest.raises(DeviceGone):
        list(dev.events(0.1))
    c.shutdown()  # every call raises DeviceGone underneath; none escapes


# --- Remote changes (--ffb-remote) -------------------------------------------------------

def gains(dev):
    return [e[1] for e in dev.ff_log if e[0] == "gain"]


def test_remote_switch_on_ramps_from_zero_and_off_releases():
    dev = fake()
    off = FfbSettings(mode=FfbMode.OFF, max_gain_pct=20, damper_pct=20)
    c = FfbController(dev, off)
    c.start(0.0)
    assert dev.ff_log == [] and not c.running
    c.update(replace(off, mode=FfbMode.CENTER), 10.0)
    assert c.running and gains(dev) == [0]
    c.tick(10.0 + model.GAIN_RAMP_S)
    assert gains(dev)[-1] == model.gain_level(20)
    ids = effect_ids(dev)
    c.update(off, 20.0)
    assert not c.running and c.spring_source == "off"
    assert {e[1] for e in dev.ff_log if e[0] == "erase"} == ids
    assert dev.ff_log[-1] == ("gain", 0)
    # ... and it can come back, ramping from 0 again.
    c.update(replace(off, mode=FfbMode.CENTER), 30.0)
    assert c.running and gains(dev)[-1] == 0


def test_remote_gain_rise_ramps_from_current_and_drop_is_immediate():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, max_gain_pct=10)
    c = FfbController(dev, s)
    c.start(0.0)
    c.tick(model.GAIN_RAMP_S)
    lo, hi = model.gain_level(10), model.gain_level(30)
    assert gains(dev)[-1] == lo
    c.update(replace(s, max_gain_pct=30), 100.0)
    assert gains(dev)[-1] == lo  # not 0, not straight to 30
    c.tick(100.0 + model.GAIN_RAMP_S / 2)
    assert lo < gains(dev)[-1] < hi
    c.update(replace(s, max_gain_pct=5), 100.0 + model.GAIN_RAMP_S / 2)
    assert gains(dev)[-1] == model.gain_level(5)


def test_remote_spring_and_damper_changes_reupload():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, spring_pct=60, damper_pct=20)
    c = FfbController(dev, s)
    c.start(0.0)
    damper_id = uploads(dev, codes.FF_DAMPER)[0][2]
    spring_id = uploads(dev, codes.FF_SPRING)[0][2]
    c.update(replace(s, spring_pct=30, damper_pct=40), 1.0)
    assert uploads(dev, codes.FF_DAMPER)[-1][2:5] == (damper_id, 0, model.coeff_level(40))
    assert uploads(dev, codes.FF_SPRING)[-1][2] == spring_id
    assert uploads(dev, codes.FF_SPRING)[-1][4] == model.coeff_level(30)
    c.update(replace(s, spring_pct=30, damper_pct=0), 2.0)
    assert ("erase", damper_id) in dev.ff_log


def test_remote_change_after_shutdown_is_refused():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.CENTER))
    c.start(0.0)
    c.shutdown()
    n = len(dev.ff_log)
    with pytest.raises(RuntimeError):
        c.update(FfbSettings(mode=FfbMode.CENTER), 1.0)
    assert len(dev.ff_log) == n


def test_update_and_tick_never_interleave():
    class Watched(FakeDevice):
        inside = 0
        overlapped = False

        def _record(self, *entry):
            Watched.inside += 1
            if Watched.inside > 1:
                Watched.overlapped = True
            super()._record(*entry)
            Watched.inside -= 1

    dev = Watched({"axes": {"ABS_X": [0, 65535, 32767]}, "ff": True})
    s = FfbSettings(mode=FfbMode.FOLLOW_UP)
    c = FfbController(dev, s)
    c.start(0.0)
    stop = threading.Event()

    def ticker():
        t = 0.0
        while not stop.is_set():
            t += 0.01
            c.on_rudder((t * 100) % 35, t)
            c.tick(t)

    th = threading.Thread(target=ticker)
    th.start()
    for i in range(200):
        c.update(replace(s, max_gain_pct=10 + i % 20, damper_pct=i % 30), i * 0.01)
    stop.set()
    th.join()
    assert not Watched.overlapped


def test_end_stop_past_a_short_lock():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, base_range_deg=900, wheel_lock_deg=90, endstop_pct=80)
    c = FfbController(dev, s)
    c.start(0.0)
    stops = [e for e in uploads(dev, codes.FF_SPRING) if e[6] > 0]
    assert len(stops) == 1
    _, _, stop_id, center, coeff, _, band = stops[0]
    assert (center, coeff, band) == (0, model.coeff_level(80), model.endstop_deadband(s))
    assert ("play", stop_id, True) in dev.ff_log
    c.shutdown()
    assert ("erase", stop_id) in dev.ff_log


def test_no_end_stop_at_the_base_s_own_end():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.CENTER))
    c.start(0.0)
    assert all(e[6] == 0 for e in uploads(dev, codes.FF_SPRING))


def test_remote_lock_change_moves_the_stop_and_ramps_again():
    dev = fake()
    s = FfbSettings(mode=FfbMode.FOLLOW_UP, max_gain_pct=20, wheel_lock_deg=90)
    c = FfbController(dev, s)
    c.start(0.0)
    c.on_rudder(35.0, 9.9)
    c.tick(10.0)
    assert gains(dev)[-1] == model.gain_level(20)
    follow = [e for e in uploads(dev, codes.FF_SPRING) if e[6] == 0][-1]
    assert follow[3] == round(0.2 * model.S16_MAX)  # full rudder: the wheel at the short lock
    stop_id = next(e[2] for e in uploads(dev, codes.FF_SPRING) if e[6] > 0)

    c.on_rudder(35.0, 10.0)
    c.update(replace(s, wheel_lock_deg=180), 10.0)
    assert gains(dev)[-1] == 0  # fades in again
    moved = [e for e in uploads(dev, codes.FF_SPRING) if e[2] == stop_id][-1]
    assert moved[6] == round(0.4 * model.U16_MAX)
    follow = [e for e in uploads(dev, codes.FF_SPRING) if e[6] == 0][-1]
    assert follow[3] == round(0.4 * model.S16_MAX)

    c.update(replace(s, wheel_lock_deg=180, endstop_pct=0), 11.0)
    assert ("erase", stop_id) in dev.ff_log


def clicks_played(dev, click_id):
    return sum(1 for e in dev.ff_log if e == ("play", click_id, True))


def test_detent_mode_clicks_each_step_and_holds_no_spring():
    dev = fake()
    s = FfbSettings(mode=FfbMode.DETENT, rudder_max_deg=35, detent_deg=5, detent_pct=50, center_mark_pct=0)
    c = FfbController(dev, s)
    c.on_wheel(0.0, 0.0)
    c.start(0.0)
    assert uploads(dev, codes.FF_SPRING) == []  # free: no centring spring
    (click,) = [e for e in dev.ff_log if e[0] == "click"]
    click_id = click[1]
    assert click[2:] == (model.coeff_level(50), model.CLICK_PERIOD_MS, model.CLICK_DURATION_MS)
    assert clicks_played(dev, click_id) == 0  # no click for switching on

    pct_per_deg = 100 / 35
    assert not c.on_wheel(2.0 * pct_per_deg, 0.1)
    assert c.on_wheel(3.5 * pct_per_deg, 0.2)  # into the 5 deg notch
    assert not c.on_wheel(2.4 * pct_per_deg, 0.3)  # resting on the boundary: no chatter
    assert not c.on_wheel(3.4 * pct_per_deg, 0.4)
    assert c.on_wheel(8.5 * pct_per_deg, 0.5)  # 10 deg
    assert c.on_wheel(-1.0 * pct_per_deg, 0.6)  # back to 0 in one go: one click
    assert clicks_played(dev, click_id) == 3
    c.shutdown()
    assert ("erase", click_id) in dev.ff_log


def test_leaving_detent_mode_brings_the_spring_back():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, center_mark_pct=0)
    c = FfbController(dev, s)
    c.start(0.0)
    spring_id = uploads(dev, codes.FF_SPRING)[0][2]
    c.update(replace(s, mode=FfbMode.DETENT), 1.0)
    assert dev.ff_log.count(("play", spring_id, False)) == 1
    click_id = next(e[1] for e in dev.ff_log if e[0] == "click")
    c.on_wheel(50.0, 1.1)
    c.update(replace(s, mode=FfbMode.CENTER), 2.0)
    assert ("erase", click_id) in dev.ff_log
    assert dev.ff_log.count(("play", spring_id, True)) == 2  # playing again
    assert not c.on_wheel(90.0, 2.1)  # no clicks outside detent mode


def test_center_mark_in_every_mode_and_released_with_the_rest():
    for mode in (FfbMode.CENTER, FfbMode.FOLLOW_UP, FfbMode.DETENT):
        dev = fake()
        c = FfbController(dev, FfbSettings(mode=mode, wheel_lock_deg=90, center_mark_pct=80, detent_pct=0))
        c.on_wheel(20.0, 0.0)  # 18 deg of wheel to starboard
        c.start(0.0)
        (mark,) = [e for e in dev.ff_log if e[0] == "click" and e[4] == model.CENTER_MARK_MS]
        mark_id = mark[1]
        assert mark[2] == model.coeff_level(80)
        assert c.on_wheel(0.5, 0.1)  # reaches centre
        assert not c.on_wheel(1.5, 0.2)  # rests by it: no buzz
        assert not c.on_wheel(10.0, 0.3)  # leaves: nothing
        assert c.on_wheel(-10.0, 0.4)  # passes straight over it
        assert dev.ff_log.count(("play", mark_id, True)) == 2, mode
        c.shutdown()
        assert ("erase", mark_id) in dev.ff_log


def test_center_mark_off_and_on_remotely():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, center_mark_pct=0)
    c = FfbController(dev, s)
    c.start(0.0)
    assert not [e for e in dev.ff_log if e[0] == "click"]
    c.update(replace(s, center_mark_pct=50), 1.0)
    (mark,) = [e for e in dev.ff_log if e[0] == "click"]
    c.update(replace(s, center_mark_pct=0), 2.0)
    assert ("erase", mark[1]) in dev.ff_log


def test_center_mark_wins_over_a_detent_click():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.DETENT, wheel_lock_deg=90, rudder_max_deg=35))
    c.on_wheel(20.0, 0.0)
    c.start(0.0)
    clicks = {e[4]: e[1] for e in dev.ff_log if e[0] == "click"}
    assert c.on_wheel(0.0, 0.1)  # centre and the 0 deg notch at once: one pulse, the centre's
    assert dev.ff_log.count(("play", clicks[model.CENTER_MARK_MS], True)) == 1
    assert dev.ff_log.count(("play", clicks[model.CLICK_DURATION_MS], True)) == 0


def spring_layers(dev):
    """The centring springs uploaded (no dead band: the end stop has one)."""
    return {e[2]: e for e in uploads(dev, codes.FF_SPRING) if e[6] == 0}


def test_spring_boost_stacks_springs_with_shared_saturation():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, spring_boost=3)
    c = FfbController(dev, s)
    c.start(0.0)
    layers = spring_layers(dev)
    assert len(layers) == 3
    for e in layers.values():
        assert e[4] == model.coeff_level(s.spring_pct)  # each as steep as one spring
        assert e[5] == model.saturation_level(100 / 3)  # far pull no stronger than one spring
        assert ("play", e[2], True) in dev.ff_log
    c.shutdown()
    assert set(layers) <= {e[1] for e in dev.ff_log if e[0] == "erase"}


def test_spring_boost_changes_while_running_and_follow_up_moves_every_layer():
    dev = fake()
    s = FfbSettings(mode=FfbMode.FOLLOW_UP, spring_boost=1)
    c = FfbController(dev, s)
    c.start(0.0)
    (first,) = spring_layers(dev)
    c.update(replace(s, spring_boost=3), 1.0)
    ids = set(spring_layers(dev))
    assert len(ids) == 3 and first in ids
    c.on_rudder(17.5, 1.9)
    c.tick(2.0)
    centres = {i: [e for e in uploads(dev, codes.FF_SPRING) if e[2] == i][-1][3] for i in ids}
    assert set(centres.values()) == {model.center_level(0.5, s)}
    c.update(replace(s, spring_boost=1), 3.0)
    erased = {e[1] for e in dev.ff_log if e[0] == "erase"}
    assert len(ids - erased) == 1 and first not in erased


def pulses(dev, duration_ms):
    return [e for e in dev.ff_log if e[0] == "click" and e[4] == duration_ms]


def test_hardover_warning_pulses_on_entering_the_zone():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, rudder_max_deg=35, hardover_warn_deg=30, center_mark_pct=0)
    c = FfbController(dev, s)
    c.on_wheel(0.0, 0.0)
    c.start(0.0)
    (warn,) = pulses(dev, model.HARDOVER_MS)
    pct = 100 / 35
    assert not c.on_wheel(29.0 * pct, 0.1)
    assert c.on_wheel(30.5 * pct, 0.2)
    assert not c.on_wheel(29.0 * pct, 0.3)  # resting by the line
    assert not c.on_wheel(35.0 * pct, 0.4)  # still in the zone
    assert not c.on_wheel(20.0 * pct, 0.5)  # leaving: nothing
    assert c.on_wheel(-32.0 * pct, 0.6)
    assert dev.ff_log.count(("play", warn[1], True)) == 2


def test_speed_makes_the_wheel_heavier_and_stale_speed_is_full_weight():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, spring_pct=100, damper_pct=50, speed_full_kn=10, speed_min_pct=40)
    c = FfbController(dev, s)
    c.on_speed(0.0, 0.0)
    c.start(0.0)
    c.tick(0.1)
    spring = [e for e in uploads(dev, codes.FF_SPRING) if e[6] == 0][-1]
    assert spring[4] == model.coeff_level(40)  # alongside: 40 % of the spring
    assert uploads(dev, codes.FF_DAMPER)[-1][4] == model.coeff_level(20)
    c.on_speed(10.0, 1.0)
    c.tick(1.1)
    assert [e for e in uploads(dev, codes.FF_SPRING) if e[6] == 0][-1][4] == model.coeff_level(100)
    c.on_speed(0.0, 2.0)
    c.tick(2.0 + model.SPEED_STALE_S + 0.5)  # the speed went quiet: full weight, never lighter
    assert [e for e in uploads(dev, codes.FF_SPRING) if e[6] == 0][-1][4] == model.coeff_level(100)


def rumble_plays(dev):
    """Each play of the rumble effect: (strength, duration ms) as it was uploaded for that play."""
    uploaded, plays = {}, []
    for e in dev.ff_log:
        if e[0] == "click" and e[3] == model.RUMBLE_PERIOD_MS:
            uploaded[e[1]] = (e[2], e[4])
        elif e[0] == "play" and e[2] and e[1] in uploaded:
            plays.append(uploaded[e[1]])
    return plays


def run(c, t0, t1, step=0.05):
    t = t0
    while t < t1:
        c.tick(round(t, 3))
        t += step


def test_alarm_patterns_per_level_once_per_alarm():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, alarm_caution_pct=20, alarm_warning_pct=50, center_mark_pct=0)
    c = FfbController(dev, s)
    c.start(0.0)
    assert not c.on_alarm("fire", 4, False, 1.0)  # emergency strength is 0: nothing
    assert c.on_alarm("cpa-1", 3, False, 1.0)  # a warning: two short, at its strength
    assert not c.on_alarm("cpa-1", 3, False, 1.1)  # the same alarm again: felt already
    run(c, 1.0, 3.0)
    assert rumble_plays(dev) == [(model.coeff_level(50), 250)] * 2
    assert c.on_alarm("depth", 2, False, 5.0)  # a caution: one short, lighter
    run(c, 5.0, 6.0)
    assert rumble_plays(dev)[-1] == (model.coeff_level(20), 250) and len(rumble_plays(dev)) == 3
    c.on_alarm("cpa-1", 3, True, 8.0)  # acknowledged ...
    assert c.on_alarm("cpa-1", 3, False, 9.0)  # ... and raised again: felt again
    assert c.top_alarm(9.0) == 4  # the emergency still counts (for the lights), rumble or not
    assert c.top_alarm(9.0 + model.ALARM_HOLD_S + 1) is None  # nothing repeated: over


def test_an_emergency_repeats_its_long_rumble_until_it_is_over():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.CENTER, alarm_emergency_pct=80, center_mark_pct=0))
    c.start(0.0)
    assert c.on_alarm("grounding", 4, False, 1.0)
    run(c, 1.0, 12.0)  # one long (800 + 700 ms) at 1, 6 and 11 s
    assert rumble_plays(dev) == [(model.coeff_level(80), 800), (model.coeff_level(80), 700)] * 3
    c.on_alarm("grounding", 4, True, 12.5)  # acknowledged: over
    run(c, 12.5, 20.0)
    assert len(rumble_plays(dev)) == 6
    c.shutdown()
    assert effect_ids(dev) == {e[1] for e in dev.ff_log if e[0] == "erase"}


def test_a_fire_alarm_is_fire_whatever_its_level_and_preempts():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, alarm_warning_pct=30, alarm_fire_pct=90, center_mark_pct=0)
    c = FfbController(dev, s)
    c.start(0.0)
    assert c.on_alarm("cpa", 3, False, 1.0)
    assert c.on_alarm("engine-room", 3, False, 1.1, category=model.FIRE_CATEGORY)  # fire beats the warning
    run(c, 1.1, 5.0)  # continuous: one long, a gap, again
    plays = rumble_plays(dev)
    assert plays[-1][0] == model.coeff_level(90) and sum(d for lvl, d in plays if lvl == model.coeff_level(90)) >= 3000
    assert not c.on_alarm("cpa2", 4, False, 5.0)  # an emergency does not interrupt a fire
    c.on_alarm("engine-room", 3, True, 6.0, category=model.FIRE_CATEGORY)
    n = len(rumble_plays(dev))
    run(c, 6.0, 9.0)
    assert len(rumble_plays(dev)) == n  # over


def test_lag_resistance_damps_while_the_rudder_trails():
    dev = fake()
    s = FfbSettings(mode=FfbMode.DETENT, damper_pct=0, lag_resistance_pct=60, rudder_max_deg=35, detent_pct=0)
    c = FfbController(dev, s)
    c.start(0.0)
    damper_id = uploads(dev, codes.FF_DAMPER)[0][2]
    assert uploads(dev, codes.FF_DAMPER)[-1][4] == 0  # nothing trails yet
    c.on_wheel(100.0, 0.1)  # hard over: 35 deg ordered
    c.on_rudder(5.0, 0.1)  # the gear has 5 deg so far
    c.tick(0.2)
    assert uploads(dev, codes.FF_DAMPER)[-1][2:5] == (damper_id, 0, model.coeff_level(60))
    c.on_rudder(35.0, 0.3)  # caught up
    c.tick(0.4)
    assert uploads(dev, codes.FF_DAMPER)[-1][4] == 0
    c.tick(0.3 + s.stale_s + 1)  # the rudder went quiet: no extra damping either
    assert uploads(dev, codes.FF_DAMPER)[-1][4] == 0


def test_everything_on_fits_the_bases_ten_effects_and_all_are_released():
    dev = fake()
    s = FfbSettings(mode=FfbMode.DETENT, damper_pct=20, spring_boost=3, wheel_lock_deg=90, hardover_warn_deg=30,
                    alarm_caution_pct=20, alarm_warning_pct=50, alarm_emergency_pct=80, lag_resistance_pct=20)
    c = FfbController(dev, s)
    c.start(0.0)
    c.update(replace(s, mode=FfbMode.CENTER), 1.0)  # springs as well as the clicks of detent mode
    live = effect_ids(dev) - {e[1] for e in dev.ff_log if e[0] == "erase"}
    assert len(live) <= 10, live
    c.shutdown()
    assert effect_ids(dev) == {e[1] for e in dev.ff_log if e[0] == "erase"}


def test_test_pulse_plays_each_haptic_at_its_own_strength():
    dev = fake()
    s = FfbSettings(mode=FfbMode.CENTER, hardover_warn_deg=0, hardover_pct=70, detent_pct=50, alarm_warning_pct=40,
                    alarm_emergency_pct=90)
    c = FfbController(dev, s)
    c.start(0.0)
    assert pulses(dev, model.HARDOVER_MS) == []  # the warning is off: not uploaded ...
    c.test_pulse("hardover", 1.0)
    (hardover,) = pulses(dev, model.HARDOVER_MS)  # ... until tested, at its own strength
    assert hardover[2] == model.coeff_level(70) and ("play", hardover[1], True) in dev.ff_log
    c.test_pulse("center", 1.1)
    (center,) = pulses(dev, model.CENTER_MARK_MS)
    assert dev.ff_log[-1] == ("play", center[1], True)
    c.test_pulse("detent", 1.2)  # in centre mode, a click only on request
    (click,) = pulses(dev, model.CLICK_DURATION_MS)
    assert click[2] == model.coeff_level(50)

    c.test_pulse("warning", 2.0)  # the warning's two short, at its strength
    run(c, 2.0, 4.0)
    assert rumble_plays(dev) == [(model.coeff_level(40), 250)] * 2
    c.test_pulse("emergency", 10.0)  # would repeat while it lasts: a Try plays it once
    run(c, 10.0, 25.0)
    assert rumble_plays(dev)[2:] == [(model.coeff_level(90), 800), (model.coeff_level(90), 700)]
    c.update(replace(s, spring_pct=40), 30.0)  # a settings change frees what is off again
    assert ("erase", hardover[1]) in dev.ff_log and ("erase", click[1]) in dev.ff_log
    c.shutdown()
    assert effect_ids(dev) == {e[1] for e in dev.ff_log if e[0] == "erase"}


def test_test_pulse_refuses_when_nothing_could_be_felt():
    dev = fake()
    c = FfbController(dev, FfbSettings(mode=FfbMode.CENTER, alarm_caution_pct=0))
    with pytest.raises(ValueError, match="choose a mode"):
        c.test_pulse("center", 0.0)  # not started
    c.start(0.0)
    with pytest.raises(ValueError, match="Caution rumble is 0"):
        c.test_pulse("caution", 0.1)
    with pytest.raises(ValueError, match="no pulse"):
        c.test_pulse("boom", 0.1)
    c.shutdown()
    before = list(dev.ff_log)
    with pytest.raises(ValueError):
        c.test_pulse("center", 1.0)
    assert dev.ff_log == before


# --- The centre notch ---------------------------------------------------------------------


def constants(dev):
    return [e for e in dev.ff_log if e[0] == "constant"]


def notch_settings(**kw):
    return FfbSettings(mode=FfbMode.DETENT, base_range_deg=900, center_notch_pct=50, center_notch_deg=6,
                       center_mark_pct=0, detent_pct=0, **kw)


AT_PEAK = 3.0 / 450 * 100  # the axis 3 deg to starboard on a 900 deg base: the full pull


def test_no_notch_while_off_or_before_start():
    dev = fake()
    c = FfbController(dev, notch_settings())
    assert c.on_axis(AT_PEAK, 0.0) == 0  # not started
    c.start(0.0)
    c.update(replace(notch_settings(), center_notch_pct=0), 0.1)
    assert c.on_axis(AT_PEAK, 0.2) == 0
    assert constants(dev) == []


def test_notch_played_by_the_wheel_loop_renewed_and_stopped_outside():
    dev = fake()
    s = notch_settings()
    c = FfbController(dev, s)
    c.start(0.0)
    level = c.on_axis(AT_PEAK, 0.0)
    assert level == model.notch_level(AT_PEAK, s) != 0
    ((_, nid, mag, ms),) = constants(dev)
    assert (mag, ms) == (level, model.NOTCH_MS)
    assert dev.ff_log[-1] == ("play", nid, True)
    # A tiny move writes nothing; the play is renewed only after NOTCH_RENEW_S.
    n = len(dev.ff_log)
    c.on_axis(AT_PEAK * 0.999, 0.05)
    assert len(dev.ff_log) == n
    c.on_axis(AT_PEAK * 0.999, 0.05 + model.NOTCH_RENEW_S)
    assert dev.ff_log[n:] == [("play", nid, True)]
    # A real move rewrites the magnitude on the same effect.
    # A real move (into the core) rewrites the magnitude on the same effect, and starts it again.
    n = len(dev.ff_log)
    c.on_axis(AT_PEAK / 4, 0.2)
    rate = (AT_PEAK / 4 - AT_PEAK * 0.999) / (0.2 - (0.05 + model.NOTCH_RENEW_S))
    assert model.notch_level(AT_PEAK / 4, s, rate) != level
    assert dev.ff_log[n:] == [("constant", nid, model.notch_level(AT_PEAK / 4, s, rate), model.NOTCH_MS), ("play", nid, True)]
    # Past the notch: stopped, not erased.
    c.on_axis(50.0, 0.3)
    assert dev.ff_log[-1] == ("play", nid, False)
    n = len(dev.ff_log)
    c.on_axis(60.0, 0.4)
    assert len(dev.ff_log) == n  # stays quiet
    # Back in: played again on the same effect.
    c.on_axis(AT_PEAK, 0.5)
    assert ("play", nid, True) == dev.ff_log[-1] and len({e[1] for e in constants(dev)}) == 1
    c.shutdown()
    assert ("erase", nid) in dev.ff_log
    assert c.on_axis(AT_PEAK, 1.0) == 0  # never again after shutdown


def test_the_notch_brakes_a_wheel_falling_into_it():
    s = notch_settings(center_notch_damping_pct=60)
    still_dev, moving_dev = fake(), fake()
    still, moving = FfbController(still_dev, s), FfbController(moving_dev, s)
    still.start(0.0)
    moving.start(0.0)
    still.on_axis(AT_PEAK / 4, 0.02)
    moving.on_axis(AT_PEAK, 0.0)  # 3 deg out ...
    falling = moving.on_axis(AT_PEAK / 4, 0.02)  # ... and falling in fast
    held = constants(still_dev)[-1][2]
    assert constants(moving_dev)[-1][2] == falling
    # In the wheel's own direction (to port is -): the force moved against the motion, to starboard.
    assert model.NOTCH_SIGN * falling > model.NOTCH_SIGN * held
    assert falling == model.notch_level(AT_PEAK / 4, s, (AT_PEAK / 4 - AT_PEAK) / 0.02)


def test_the_tick_never_renews_the_notch():
    dev = fake()
    c = FfbController(dev, notch_settings())
    c.start(0.0)
    c.on_axis(AT_PEAK, 0.0)
    nid = constants(dev)[0][1]
    plays = dev.ff_log.count(("play", nid, True))
    for t in (0.05, 0.1, 0.5, 1.0, 2.0):
        c.tick(t)
    assert dev.ff_log.count(("play", nid, True)) == plays


def test_notch_erased_on_follow_up_or_off_remotely():
    for change in ({"mode": FfbMode.FOLLOW_UP}, {"center_notch_pct": 0.0}, {"mode": FfbMode.OFF}):
        dev = fake()
        s = notch_settings()
        c = FfbController(dev, s)
        c.start(0.0)
        c.on_axis(AT_PEAK, 0.0)
        nid = constants(dev)[0][1]
        c.update(replace(s, **change), 1.0)
        assert ("erase", nid) in dev.ff_log, change
        if change.get("mode") is not FfbMode.OFF:
            assert c.on_axis(AT_PEAK, 1.1) == 0


def test_a_base_without_constant_force_goes_without_a_notch():
    dev = fake()

    def refuse(*a, **kw):
        raise NotImplementedError("no constant force")

    dev.upload_constant = refuse
    c = FfbController(dev, notch_settings())
    c.start(0.0)
    assert c.on_axis(AT_PEAK, 0.0) == 0
    assert c.on_axis(AT_PEAK, 0.5) == 0  # said once, not retried
    c.tick(1.0)  # the rest goes on
