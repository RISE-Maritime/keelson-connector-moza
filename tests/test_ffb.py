import pytest

from moza_connector import codes, model
from moza_connector.device import DeviceGone, FakeDevice
from moza_connector.ffb import FfbController
from moza_connector.model import FfbMode, FfbSettings


def fake(ff=True):
    return FakeDevice({"axes": {"ABS_X": [0, 65535, 32767]}, "ff": ff})


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
    uploaded = {e[2] for e in dev.ff_log if e[0] == "upload"}
    assert erased == uploaded and erased
    assert dev.ff_log[-1] == ("gain", 0)


def test_shutdown_never_raises_on_a_vanished_base():
    dev = FakeDevice({"axes": {"ABS_X": [0, 65535, 32767]}, "events": [[0, "gone"]]})
    c = FfbController(dev, FfbSettings())
    c.start(0.0)
    with pytest.raises(DeviceGone):
        list(dev.events(0.1))
    c.shutdown()  # every call raises DeviceGone underneath; none escapes
