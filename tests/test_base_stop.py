"""The hard end stop: the Moza serial framing (pinned to frames captured from an R21 Ultra),
the decisions, and BaseStop with a fake base."""

import pytest

from moza_connector import model
from moza_connector import serial_proto as proto
from moza_connector.base_serial import FakeBaseSerial
from moza_connector.base_stop import BaseStop, valid_restore
from moza_connector.model import FfbSettings

# Captured from an R21 Ultra (Pit House at 900 deg), with the base's log frames between.
READ_STIFFNESS = bytes.fromhex("7e 03 28 13 1f 00 01 e9")
REPLY_LIMIT = bytes.fromhex("7e 03 a8 31 01 01 c2 2b")
REPLY_STIFFNESS = bytes.fromhex("7e 03 a8 31 1f 00 64 ea")
REPLY_STRENGTH = bytes.fromhex("7e 03 a8 31 1b 00 38 ba")
LOG_FRAME = bytes.fromhex("7e 05 0e 71 03 00 32 00 01 45")


def test_a_read_request_is_framed_as_the_base_answers_it():
    assert proto.read_request("soft-limit-stiffness") == READ_STIFFNESS
    assert proto.checksum(REPLY_LIMIT[:-1]) == REPLY_LIMIT[-1]


def test_replies_are_found_among_the_bases_own_frames():
    buf = LOG_FRAME + REPLY_STIFFNESS + LOG_FRAME
    assert proto.find_reply(buf, "soft-limit-stiffness")[0] == 100
    assert proto.find_reply(LOG_FRAME + REPLY_LIMIT, "limit")[0] == 450  # 900 deg in Pit House
    assert proto.find_reply(REPLY_STRENGTH, "soft-limit-strength")[0] == 56
    assert proto.find_reply(REPLY_LIMIT[:5], "limit") == (None, 0)  # partial: kept
    bad = REPLY_LIMIT[:-1] + bytes([REPLY_LIMIT[-1] ^ 1])
    assert proto.find_reply(bad, "limit")[0] is None  # a bad checksum is no reply
    assert proto.find_reply(REPLY_LIMIT, "limit", write=True)[0] is None  # the write's reply is A9


def test_only_the_steering_angle_may_be_written():
    frame = proto.write_request("limit", 90)
    assert frame[:7] == bytes([0x7E, 3, 41, 0x13, 1, 0, 90]) and proto.checksum(frame[:-1]) == frame[-1]
    for name in ("torque", "ffb-strength", "soft-limit-strength", "ffb-disable"):
        with pytest.raises(ValueError):
            proto.write_request(name, 1)
    with pytest.raises(ValueError):
        proto.write_request("limit", 70000)
    assert (proto.angle_value(180), proto.range_deg(450)) == (90, 900.0)


def test_the_base_angle_follows_the_lock_within_bounds():
    s = FfbSettings(hard_stop=True, base_max_deg=900, base_range_deg=900, wheel_lock_deg=90)
    assert model.base_range_for(s) == 180
    assert model.base_range_for(FfbSettings(hard_stop=True, base_max_deg=900, wheel_lock_deg=30)) == 90  # never under 90
    assert model.base_range_for(FfbSettings(hard_stop=True, base_max_deg=900, wheel_lock_deg=450)) == 900
    assert model.base_range_for(FfbSettings(hard_stop=False, base_max_deg=900, base_range_deg=180, wheel_lock_deg=90)) == 900


def test_narrower_only_with_the_wheel_inside_wider_at_once():
    assert model.base_stop_decision(180, 900, 10.0) == ("apply", "")
    assert model.base_stop_decision(180, 900, 88.0) == ("wait", "turn the wheel within ±85°")
    assert model.base_stop_decision(180, 900, -120.0)[0] == "wait"
    assert model.base_stop_decision(180, 900, None) == ("wait", "waiting for the wheel's position")
    assert model.base_stop_decision(900, 180, 89.0) == ("apply", "")  # wider: always safe
    assert model.base_stop_decision(180, 180, 300.0) == ("none", "")


def test_the_lock_may_widen_past_a_narrowed_base():
    s = FfbSettings(hard_stop=True, base_max_deg=900, base_range_deg=180, wheel_lock_deg=90)
    wider = model.apply_config(s, {"wheel_lock_deg": 270}, gain_cap_pct=15, read_only={})
    assert wider.wheel_lock_deg == 270 and model.base_range_for(wider) == 540
    with pytest.raises(ValueError, match="past the base's own end"):
        model.apply_config(s, {"wheel_lock_deg": 460}, gain_cap_pct=15, read_only={})


def angle(base):
    return {n: base.values[n] for n in ("limit", "max-angle")}


def stop_with(base, pending=None):
    saved = []
    stop = BaseStop(base, saved.append)
    own, now = stop.start(pending)
    return stop, saved, own


def test_base_stop_narrows_waits_and_restores():
    base = FakeBaseSerial(450)
    stop, saved, own = stop_with(base)
    assert own == 900
    s = FfbSettings(hard_stop=True, base_max_deg=900, base_range_deg=900, wheel_lock_deg=90)
    assert stop.sync(s, 120.0) is None and stop.status.startswith("waiting: turn the wheel within ±85°")
    assert base.writes == []  # never narrower than where the wheel is
    assert stop.sync(s, 3.0) == 180
    assert angle(base) == {"limit": 90, "max-angle": 90} and stop.status == "hard stop at ±90°"
    assert saved[-1] == {"limit": 450, "max-angle": 450}  # what to put back after a crash
    assert stop.restore() and angle(base) == {"limit": 450, "max-angle": 450} and saved[-1] is None


def test_restore_never_raises_with_the_base_gone():
    base = FakeBaseSerial(450)
    stop, saved, _ = stop_with(base)
    stop.sync(FfbSettings(hard_stop=True, base_max_deg=900, base_range_deg=900, wheel_lock_deg=90), 0.0)
    base.gone = True
    assert stop.restore() is False  # logged, not raised; the next start puts it back
    assert saved[-1] == {"limit": 450, "max-angle": 450}


def test_a_dead_runs_narrowing_is_put_back_at_start():
    base = FakeBaseSerial(90)  # left at 180 deg by a run that died
    stop, saved, own = stop_with(base, {"limit": 450, "max-angle": 450})
    assert own == 900 and angle(base) == {"limit": 450, "max-angle": 450} and saved[-1] is None
    assert valid_restore({"limit": 450}) is None and valid_restore({"limit": 5, "max-angle": 5}) is None
