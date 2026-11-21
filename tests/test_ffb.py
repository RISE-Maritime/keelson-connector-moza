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
