"""The wheel's lights in an alarm's colour (the rim's LEDs, the base's light bar), with a fake
base, rim and main controller."""

from moza_connector import model, serial_proto as proto
from moza_connector.alarm_lights import ALL_NAMES, AlarmLights, valid_restore
from moza_connector.base_serial import FakeBaseSerial
from moza_connector.model import ALARM_RANKS, FfbSettings

OWN = 0x56F7FC
BOTH = FfbSettings(alarm_leds=True, alarm_base_light=True)
WARNING, EMERGENCY, FIRE = ALARM_RANKS["warning"], ALARM_RANKS["emergency"], ALARM_RANKS["fire"]

# Captured from the R21 Ultra's main controller: a light-bar colour read (mode 1, strip 0, LED 0).
READ_BASE_COLOR = bytes.fromhex("7e 07 22 12 20 00 01 00 00 00 00 e7")


def lights_with(base, pending=None):
    saved = []
    lights = AlarmLights(base, saved.append)
    lights.start(pending)
    return lights, saved


def rim(base):
    return {base.values[n] for n in proto.RIM_COLORS}


def bar(base):
    return {base.values[n] for n in proto.BASE_LIGHT if n != "base-standby-mode"}


def test_the_light_bars_commands_are_framed_as_the_base_answers_them():
    assert proto.read_request("base-color0-0") == READ_BASE_COLOR
    assert len(proto.BASE_LIGHT) == 1 + proto.BASE_STRIPS * proto.BASE_STRIP_LEDS == 19
    frame = proto.write_request("base-color1-8", 0xFF0000)
    assert frame[2:11] == bytes([0x20, 0x12, 0x20, 1, 1, 8, 0xFF, 0, 0])


def test_an_alarm_colours_rim_and_bar_steadily_and_their_own_comes_back():
    base = FakeBaseSerial()
    lights, saved = lights_with(base)
    assert lights.targets == ("rim", "base")
    lights.sync(BOTH, WARNING, 0.0)
    assert rim(base) == {0xFF7A00} and bar(base) == {0xFF7A00}
    assert lights.status == {"rim": "showing warning", "base": "showing warning"}
    assert set(saved[-1]) == set(ALL_NAMES)  # saved before anything was shown
    writes = len(base.writes)
    for t in (0.5, 1.0, 1.5):
        lights.sync(BOTH, WARNING, t)
    assert len(base.writes) == writes  # steady: written once, never flashed
    lights.sync(BOTH, EMERGENCY, 2.0)
    assert rim(base) == {0xFF0000}
    lights.sync(BOTH, None, 3.0)
    assert rim(base) == {OWN} and bar(base) == {OWN} and saved[-1] is None


def test_each_light_switches_on_its_own_and_fire_has_its_colour():
    base = FakeBaseSerial()
    lights, _ = lights_with(base)
    lights.sync(FfbSettings(alarm_leds=False, alarm_base_light=True, alarm_fire_color="#ff00aa"), FIRE, 0.0)
    assert rim(base) == {OWN} and bar(base) == {0xFF00AA}


def test_the_bars_standby_mode_is_constant_while_showing_and_put_back():
    base = FakeBaseSerial()
    base.values["base-standby-mode"] = 2  # breathing
    lights, _ = lights_with(base)
    lights.sync(BOTH, WARNING, 0.0)
    assert base.values["base-standby-mode"] == proto.CONSTANT_MODE
    assert lights.restore() and base.values["base-standby-mode"] == 2 and bar(base) == {OWN}


def test_try_previews_a_colour_then_goes_back():
    base = FakeBaseSerial()
    lights, _ = lights_with(base)
    lights.preview("caution", 10.0, 3.0)
    lights.sync(FfbSettings(), None, 10.1)  # shown even with the alarm lights off
    assert rim(base) == {0xFFD400} and bar(base) == {0xFFD400}
    lights.sync(FfbSettings(), None, 13.5)
    assert rim(base) == {OWN} and bar(base) == {OWN}


def test_a_device_without_lights_is_left_out():
    base = FakeBaseSerial()
    for n in proto.BASE_LIGHT:
        del base.values[n]
    lights, _ = lights_with(base)
    assert lights.targets == ("rim",) and lights.status["base"] == "off"


def test_a_gone_device_never_raises_waits_and_a_dead_runs_colours_come_back():
    base = FakeBaseSerial()
    lights, saved = lights_with(base)
    lights.sync(BOTH, WARNING, 0.0)
    base.gone = True
    lights.sync(BOTH, None, 1.0)  # logged, said in the status, not raised
    assert lights.status["rim"].startswith("error")
    lights.sync(BOTH, None, 2.0)  # not tried again on every tick
    assert lights.restore() is False
    left = FakeBaseSerial()
    for n in (*proto.RIM_COLORS, *proto.BASE_LIGHT[1:]):
        left.values[n] = 0xFF7A00  # what the dead run left showing
    lights_with(left, saved[-1])
    assert rim(left) == {OWN} and bar(left) == {OWN}
    assert valid_restore({"limit": 1}) is None and valid_restore({"rpm-color1": 5, "x": 1}) == {"rpm-color1": 5}


def test_colors_report_what_each_led_shows():
    base = FakeBaseSerial()
    lights = AlarmLights(base)
    lights.start(None)
    own = lights.colors()
    assert len(own["rim"]["rpm"]) == 10 and len(own["rim"]["buttons"]) == 10
    assert [len(s) for s in own["base"]] == [9, 9]
    assert own["rim"]["rpm"][0] == "#56f7fc"
    s = FfbSettings(alarm_leds=True, alarm_base_light=True)
    lights.sync(s, model.ALARM_RANKS["warning"], 0.0)
    shown = lights.colors()
    assert set(shown["rim"]["rpm"]) == {f"#{model.alarm_light_color(s, model.ALARM_RANKS['warning'], 'rim') & 0xFFFFFF:06x}"}
    lights.sync(s, None, 1.0)
    assert lights.colors() == own
