"""The rim's LEDs in an alarm's colour, with a fake base and rim."""

from moza_connector import serial_proto as proto
from moza_connector.base_serial import FakeBaseSerial
from moza_connector.model import FfbSettings
from moza_connector.rim_leds import NAMES, RimLeds, valid_restore

OWN = 0x56F7FC
ON = FfbSettings(alarm_leds=True)


def rim_with(base, pending=None):
    saved = []
    rim = RimLeds(base, saved.append)
    rim.start(pending)
    return rim, saved


def colours(base):
    return {base.values[n] for n in proto.RIM_COLORS}


def test_an_alarm_colours_the_whole_rim_and_its_own_colours_come_back():
    base = FakeBaseSerial()
    rim, saved = rim_with(base)
    rim.sync(ON, 3, 0.0)
    assert colours(base) == {0xFF7A00} and rim.status == "showing warning"
    assert saved[-1] == {n: OWN for n in proto.RIM_COLORS} | {n: 50 for n in proto.RIM_BRIGHTNESS}
    rim.sync(ON, None, 1.0)
    assert colours(base) == {OWN} and saved[-1] is None and rim.status == "ready"


def test_an_emergency_flashes_by_brightness():
    base = FakeBaseSerial()
    rim, _ = rim_with(base)
    rim.sync(ON, 4, 0.1)
    assert colours(base) == {0xFF0000} and base.values["rpm-brightness"] == 50
    writes = len(base.writes)
    rim.sync(ON, 4, 0.6)  # the dark half: two writes, not twenty
    assert base.values["rpm-brightness"] == 0 and base.values["buttons-brightness"] == 0
    assert len(base.writes) == writes + 2
    rim.sync(ON, 4, 0.7)
    assert len(base.writes) == writes + 2  # nothing changed: nothing written
    rim.sync(ON, 2, 1.1)  # down to a caution: lit, yellow
    assert colours(base) == {0xFFD400} and base.values["rpm-brightness"] == 50


def test_no_colours_when_switched_off_and_restore_on_exit():
    base = FakeBaseSerial()
    rim, _ = rim_with(base)
    rim.sync(FfbSettings(alarm_leds=False), 4, 0.0)
    assert base.writes == []
    rim.sync(ON, 4, 0.6)
    assert rim.restore() and colours(base) == {OWN} and base.values["rpm-brightness"] == 50
    rim.sync(ON, 4, 2.0)  # after restore nothing more
    assert colours(base) == {OWN}


def test_a_gone_rim_never_raises_and_a_dead_runs_colours_come_back():
    base = FakeBaseSerial()
    rim, saved = rim_with(base)
    rim.sync(ON, 3, 0.0)
    base.gone = True
    rim.sync(ON, None, 1.0)  # logged, said in the status, not raised
    assert rim.status.startswith("error")
    assert rim.restore() is False
    left = FakeBaseSerial()
    for n in proto.RIM_COLORS:
        left.values[n] = 0xFF7A00  # what the dead run left showing
    rim2, saved2 = rim_with(left, saved[-1])
    assert colours(left) == {OWN} and saved2[-1] is None
    assert valid_restore({n: 0 for n in NAMES}) is not None and valid_restore({"limit": 1}) is None


def test_the_rims_commands_are_writable_but_the_motor_is_not():
    frame = proto.write_request("rpm-color1", 0xFF0000)
    assert frame[:9] == bytes([0x7E, 7, 63, 0x17, 31, 0, 255, 0, 0xFF]) and proto.checksum(frame[:-1]) == frame[-1]
    assert proto.read_request("button-color3")[2:8] == bytes([64, 0x17, 31, 1, 255, 2])
    for name in ("soft-limit-strength", "soft-limit-stiffness"):
        try:
            proto.write_request(name, 1)
        except ValueError:
            continue
        raise AssertionError(f"{name} must not be writable")


def test_show_previews_a_colour_then_goes_back():
    base = FakeBaseSerial()
    rim, _ = rim_with(base)
    rim.preview("caution", 10.0)
    rim.sync(FfbSettings(alarm_leds=False), None, 10.1)  # shown even with the alarm colours off
    assert colours(base) == {0xFFD400}
    rim.sync(FfbSettings(alarm_leds=False), None, 15.5)
    assert colours(base) == {OWN}
