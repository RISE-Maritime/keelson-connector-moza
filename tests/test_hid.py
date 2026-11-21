"""The macOS HID backend, against the R21 Ultra's real report descriptor and a recording
fake of the hidapi handle. Every byte written to the base is checked here."""

import threading
from pathlib import Path

import pytest

from moza_connector import codes, hid_device, hid_report, model
from moza_connector.device import DeviceGone, resolve_backend
from moza_connector.ffb import FfbController
from moza_connector.hid_device import EFFECT_LEASE_MS, KEEPALIVE_S, HidDevice, button_code
from moza_connector.model import FfbMode, FfbSettings

DESCRIPTOR = (Path(__file__).parent / "data" / "r21_ultra_descriptor.bin").read_bytes()

# An input report captured from the base: wheel near centre, no buttons, no pedals.
IDLE = bytes.fromhex("01c57f0180008001800180008001800080" + "08" + "00" * 16 + "e22a0143ba7c81c3")


def report(*, wheel=None, byte17=None, buttons_at=None, telemetry=None) -> list[int]:
    r = bytearray(IDLE)
    if wheel is not None:
        r[1:3] = wheel.to_bytes(2, "little")
    if byte17 is not None:
        r[17] = byte17
    for i, v in (buttons_at or {}).items():
        r[i] = v
    if telemetry is not None:
        r[34:42] = telemetry
    return list(r)


class FakeHandle:
    def __init__(self, reports=(), block_load=None):
        self.reports = [report()] + list(reports)
        self.writes: list[bytes] = []
        self.features: list[bytes] = []
        self.feature_gets: list[int] = []
        self.next_block = 1
        self.block_load = block_load  # override the reply to a block-load request
        self.closed = False
        self.fail_reads = False
        self.fail_writes = False
        self.lock = threading.Lock()

    def read(self, max_length, timeout_ms=0):
        if self.fail_reads:
            raise OSError("read error")
        return self.reports.pop(0) if self.reports else []

    def write(self, buff):
        if self.fail_writes:
            raise OSError("write error")
        with self.lock:
            self.writes.append(bytes(buff))
        return len(buff)

    def send_feature_report(self, data):
        self.features.append(bytes(data))
        return len(data)

    def get_feature_report(self, report_num, max_length):
        self.feature_gets.append(report_num)
        if self.block_load is not None:
            return list(self.block_load)
        idx = self.next_block
        self.next_block += 1
        return [18, idx, 1, 0xFF, 0xFF]  # BlockLoadSuccess

    def close(self):
        self.closed = True


def device(**kw) -> tuple[HidDevice, FakeHandle]:
    h = FakeHandle(**kw)
    return HidDevice(h, DESCRIPTOR, name="MOZA R21 Ultra Base", path="fake"), h


# --- The descriptor ------------------------------------------------------------------


def test_descriptor_reports_found_by_usage():
    d = hid_report.parse(DESCRIPTOR)
    ids = {name: d.find(kind, name).id for kind, name in hid_device._PID_REPORTS}
    assert ids == {
        "SetEffectReport": 1,
        "SetConditionReport": 3,
        "EffectOperationReport": 10,
        "PIDBlockFreeReport": 11,
        "PIDDeviceControlReport": 12,
        "DeviceGainReport": 13,
        "CreateNewEffectReport": 17,
        "PIDBlockLoadReport": 18,
    }
    inp = d.find_with("Input", "X")
    assert inp.id == 1 and inp.nbytes == 41
    x, _ = inp.slot("X")
    assert (x.bit_offset, x.size, x.lmin, x.lmax) == (0, 16, 0, 65535)
    b1, i = inp.slot("Button1")
    assert b1.bit_offset + i == 16 * 8 + 4  # after eight 16-bit axes and the 4-bit hat


def test_encode_refuses_out_of_range_and_unknown():
    gain = hid_report.parse(DESCRIPTOR).find("Output", "DeviceGainReport")
    assert gain.encode({"DeviceGain": 255}) == bytes([13, 255])
    with pytest.raises(ValueError):
        gain.encode({"DeviceGain": 256})
    with pytest.raises(KeyError):
        gain.encode({"Gain": 1})


def test_ambiguous_usage_needs_its_collection():
    eff = hid_report.parse(DESCRIPTOR).find("Output", "SetEffectReport")
    assert not eff.has("Ordinal1")
    assert eff.has("Direction/Ordinal1") and eff.has("TypeSpecificBlockOffset/Ordinal1")


# --- Inputs --------------------------------------------------------------------------


def test_axes_mirror_linux_codes_and_ranges():
    dev, _ = device()
    axes = dev.axes()
    assert axes[codes.ABS["ABS_X"]].lo == 0 and axes[codes.ABS["ABS_X"]].hi == 65535
    assert axes[codes.ABS["ABS_X"]].value == 0x7FC5
    # The pedal axes are signed and park at their minimum with nothing plugged in.
    z = axes[codes.ABS["ABS_Z"]]
    assert (z.lo, z.hi, z.value) == (-32768, 32767, -32768)
    assert axes[codes.ABS["ABS_Y"]].value == -32767
    assert codes.ABS["ABS_THROTTLE"] in axes  # Slider
    assert codes.ABS["ABS_RUDDER"] in axes  # Dial


def test_buttons_numbered_as_linux_does():
    dev, _ = device()
    keys = dev.keys()
    assert len(keys) == 128
    assert keys[0] == button_code(1) == 0x120
    assert button_code(16) == 0x12F and button_code(17) == 0x2C0
    assert keys == sorted(keys) and keys.index(button_code(17)) == 16


def test_events_wheel_and_buttons():
    dev, _ = device(
        reports=[
            report(wheel=40000),
            report(wheel=40000, byte17=0x18),  # Button1 lives in byte 17's high nibble
            report(wheel=40000, byte17=0x08, buttons_at={18: 0x01}),  # Button5
        ]
    )
    evs = [(e.type, e.code, e.value) for e in dev.events(0.01)]
    assert evs == [
        (codes.EV_ABS, codes.ABS["ABS_X"], 40000),
        (codes.EV_SYN, 0, 0),
        (codes.EV_KEY, button_code(1), 1),
        (codes.EV_SYN, 0, 0),
        (codes.EV_KEY, button_code(1), 0),
        (codes.EV_KEY, button_code(5), 1),
        (codes.EV_SYN, 0, 0),
    ]


def test_telemetry_alone_is_not_an_event():
    dev, _ = device(reports=[report(telemetry=bytes(range(8))), report(telemetry=bytes(8))])
    assert list(dev.events(0.01)) == []


def test_other_report_ids_are_skipped():
    dev, _ = device(reports=[[2, 1, 0x02], report(wheel=1000)])
    assert [(e.code, e.value) for e in dev.events(0.01) if e.type == codes.EV_ABS] == [(codes.ABS["ABS_X"], 1000)]


def test_read_error_is_device_gone():
    dev, h = device()
    h.fail_reads = True
    with pytest.raises(DeviceGone):
        list(dev.events(0.01))


def test_silent_base_is_not_found():
    h = FakeHandle()
    h.reports = []
    with pytest.raises(FileNotFoundError):
        HidDevice(h, DESCRIPTOR, name="x", path="fake")


# --- Force feedback ------------------------------------------------------------------


def test_has_ff_needs_every_pid_report():
    dev, _ = device()
    assert dev.has_ff()
    # The joystick collection alone: inputs, no PID.
    inputs_only = DESCRIPTOR[: DESCRIPTOR.index(bytes([0x06, 0x00, 0xFF]))] + bytes([0xC0])
    h = FakeHandle()
    assert not HidDevice(h, inputs_only, name="x", path="fake").has_ff()


def test_first_ff_write_resets_and_holds_gain_at_zero():
    dev, h = device()
    dev.set_autocenter(0)
    assert h.writes == [
        bytes([12, 0x08]),  # DCDeviceReset: clears effects an earlier run left behind
        bytes([12, 0x04]),  # DCStopAllEffects
        bytes([13, 0]),  # DeviceGain 0
        bytes([12, 0x01]),  # DCEnableActuators, only once the gain is 0
    ]
    dev.set_gain(0)
    assert h.writes[-1] == bytes([13, 0]) and len(h.writes) == 5


def test_gain_scaled_and_capped_at_the_ceiling():
    dev, h = device()
    dev.set_gain(model.gain_level(30))
    assert h.writes[-1] == bytes([13, round(model.gain_level(30) * 255 / 0xFFFF)])
    ceiling = model.gain_level(model.MAX_GAIN_CEILING_PCT)
    dev.set_gain(ceiling)
    with pytest.raises(ValueError):
        dev.set_gain(ceiling + 1)
    assert h.writes[-1] == bytes([13, round(ceiling * 255 / 0xFFFF)])


def test_autocentre_other_than_zero_is_refused():
    dev, _ = device()
    with pytest.raises(NotImplementedError):
        dev.set_autocenter(0x4000)


def test_spring_upload_bytes():
    dev, h = device()
    idx = dev.upload_condition(codes.FF_SPRING, -1, center=-1000, coeff=0x4000, saturation=0xFFFF)
    assert idx == 1
    assert h.features == [bytes([17, 8, 0, 0])]  # CreateNewEffect: ETSpring is the 8th type
    assert h.feature_gets == [18]
    lease = EFFECT_LEASE_MS.to_bytes(2, "little")
    assert h.writes[-2] == bytes(
        [1, 1, 8, *lease, 0, 0, 0, 0, 0, 0, 255, 0, 0b101, *(9000).to_bytes(2, "little"), 0, 0, 0, 0, 0, 0]
    )
    assert h.writes[-1] == bytes(
        [3, 1, 0, *(-1000).to_bytes(2, "little", signed=True), 0x00, 0x40, 0x00, 0x40, 0xFF, 0x7F, 0xFF, 0x7F, 0, 0]
    )
    # Moving the centre re-sends the condition only.
    n = len(h.writes)
    assert dev.upload_condition(codes.FF_SPRING, idx, center=2000, coeff=0x4000, saturation=0xFFFF) == idx
    assert len(h.writes) == n + 1 and h.writes[-1][0] == 3
    assert len(h.features) == 1


def test_update_of_wrong_effect_is_refused():
    dev, _ = device()
    idx = dev.upload_condition(codes.FF_DAMPER, -1, center=0, coeff=1, saturation=1)
    with pytest.raises(ValueError):
        dev.upload_condition(codes.FF_SPRING, idx, center=0, coeff=1, saturation=1)
    with pytest.raises(ValueError):
        dev.play(99, True)


@pytest.mark.parametrize("reply", [[18, 0, 2, 0, 0], [18, 0, 3, 0, 0]])
def test_block_load_refusal_raises(reply):
    dev, _ = device(block_load=reply)
    with pytest.raises(RuntimeError):
        dev.upload_condition(codes.FF_SPRING, -1, center=0, coeff=1, saturation=1)


def test_play_stop_erase_bytes():
    dev, h = device()
    idx = dev.upload_condition(codes.FF_SPRING, -1, center=0, coeff=1, saturation=1)
    dev.play(idx, True)
    assert h.writes[-1] == bytes([10, idx, 1, 1])  # OpEffectStart
    dev.play(idx, False)
    assert h.writes[-1] == bytes([10, idx, 3, 1])  # OpEffectStop
    dev.erase(idx)
    assert h.writes[-1] == bytes([11, idx])
    dev.close()


def test_leases_renew_only_playing_effects():
    dev, h = device()
    a = dev.upload_condition(codes.FF_SPRING, -1, center=0, coeff=1, saturation=1)
    b = dev.upload_condition(codes.FF_DAMPER, -1, center=0, coeff=1, saturation=1)
    dev.play(a, True)
    dev.play(b, True)
    dev.play(b, False)
    n = len(h.writes)
    dev.renew_leases()
    assert h.writes[n:] == [bytes([10, a, 1, 1])]
    dev.close()


def test_keepalive_is_well_inside_the_lease():
    assert KEEPALIVE_S * 1000 * 3 <= EFFECT_LEASE_MS


def test_close_after_ff_stops_everything():
    dev, h = device()
    idx = dev.upload_condition(codes.FF_SPRING, -1, center=0, coeff=1, saturation=1)
    dev.play(idx, True)
    n = len(h.writes)
    dev.close()
    assert h.writes[n:][-3:] == [bytes([12, 0x04]), bytes([13, 0]), bytes([12, 0x08])]
    assert h.closed


def test_close_without_ff_writes_nothing():
    dev, h = device(reports=[report(wheel=1)])
    list(dev.events(0.01))
    dev.close()
    assert h.writes == [] and h.features == [] and h.closed


def test_close_never_raises_on_a_gone_base():
    dev, h = device()
    dev.set_gain(0)
    h.fail_writes = True
    dev.close()
    assert h.closed


def test_write_error_is_device_gone():
    dev, h = device()
    h.fail_writes = True
    with pytest.raises(DeviceGone):
        dev.set_gain(0)


def test_controller_through_hid_ramps_and_releases():
    dev, h = device()
    c = FfbController(dev, FfbSettings(mode=FfbMode.CENTER, max_gain_pct=30, damper_pct=20))
    c.start(0.0)
    gains = [w[1] for w in h.writes if w[0] == 13]
    assert gains[-1] == 0  # starts at 0
    c.tick(model.GAIN_RAMP_S / 2)
    c.tick(model.GAIN_RAMP_S + 1)
    gains = [w[1] for w in h.writes if w[0] == 13]
    assert 0 < gains[-2] < gains[-1] == round(model.gain_level(30) * 255 / 0xFFFF)
    c.shutdown()
    assert h.writes[-1] == bytes([13, 0])
    freed = {w[1] for w in h.writes if w[0] == 11}
    assert freed == {1, 2, 3}  # damper, centre mark and spring
    dev.close()


def test_backend_by_platform(monkeypatch):
    monkeypatch.setattr("sys.platform", "darwin")
    assert resolve_backend("auto") == "hid"
    monkeypatch.setattr("sys.platform", "linux")
    assert resolve_backend("auto") == "evdev"
    assert resolve_backend("hid") == "hid"
    with pytest.raises(ValueError):
        resolve_backend("serial")


def test_end_stop_dead_band_bytes():
    dev, h = device()
    dev.upload_condition(codes.FF_SPRING, -1, center=0, coeff=0x7FFF, saturation=0xFFFF, deadband=model.U16_MAX // 5)
    assert h.writes[-1][-2:] == round((model.U16_MAX // 5) * 32767 / model.U16_MAX).to_bytes(2, "little")


def test_click_bytes_and_it_is_never_leased():
    dev, h = device()
    idx = dev.upload_click(-1, magnitude=0x4000, period_ms=20, duration_ms=30)
    assert h.features[-1] == bytes([17, 3, 0, 0])  # CreateNewEffect: ETSquare is the 3rd type
    assert h.writes[-2][:5] == bytes([1, idx, 3, 30, 0])  # its own 30 ms duration, no lease
    assert h.writes[-1] == bytes([4, idx, 0x00, 0x40, 0, 0, 0, 0, 20, 0, 0, 0])  # SetPeriodic
    dev.play(idx, True)
    n = len(h.writes)
    dev.renew_leases()
    assert len(h.writes) == n  # nothing re-started it
    with pytest.raises(ValueError):
        dev.upload_click(-1, magnitude=1, period_ms=20, duration_ms=EFFECT_LEASE_MS + 1)
    dev.close()
