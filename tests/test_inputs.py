import keelson
import pytest
from keelson.payloads.Primitives_pb2 import TimestampedFloat, TimestampedInt

from moza_connector import codes, profile
from moza_connector.device import AxisRange
from moza_connector.inputs import InputPublisher, RateSettings

AXES = {
    codes.ABS["ABS_X"]: AxisRange(0, 65535, 32767),
    codes.ABS["ABS_Z"]: AxisRange(0, 1000, 0),
    codes.ABS["ABS_RZ"]: AxisRange(0, 1000, 0),
}
KEYS = [289, 288]


class Sink:
    def __init__(self):
        self.puts = []

    def __call__(self, subject, source, payload):
        _, _, inner = keelson.uncover(payload)
        cls = TimestampedInt if subject == "button_state_change" else TimestampedFloat
        self.puts.append((subject, source, cls.FromString(inner).value))

    def of(self, subject, source):
        return [v for s, src, v in self.puts if s == subject and src == source]


def make(axes=AXES, keys=KEYS, prof=None, **rate):
    sink = Sink()
    pub = InputPublisher(sink, prof or profile.load("r21-ultra"), axes, keys, "moza_r21", RateSettings(**rate))
    return pub, sink


def test_surface():
    pub, _ = make()
    assert pub.subjects == ["button_state_change", "joystick_lt_pct", "joystick_rt_pct", "wheel_position_pct"]
    assert ("wheel_position_pct", "moza_r21/wheel") in pub.sources
    assert ("button_state_change", "moza_r21/btn_0") in pub.sources


def test_missing_pedals_are_skipped_but_missing_steering_is_fatal():
    pub, _ = make(axes={codes.ABS["ABS_X"]: AxisRange(0, 65535, 32767)})
    assert "joystick_rt_pct" not in pub.subjects
    with pytest.raises(ValueError, match="steering"):
        make(axes={codes.ABS["ABS_Z"]: AxisRange(0, 1000, 0)})


def test_prime_publishes_current_positions():
    pub, sink = make()
    pub.prime(0.0)
    assert sink.of("wheel_position_pct", "moza_r21/wheel") == [0.0]  # snapped
    assert sink.of("joystick_rt_pct", "moza_r21/throttle") == [0.0]


def test_wheel_full_lock():
    pub, sink = make()
    pub.prime(0.0)
    pub.handle(codes.EV_ABS, codes.ABS["ABS_X"], 65535, 1.0)
    pub.handle(codes.EV_ABS, codes.ABS["ABS_X"], 0, 2.0)
    assert sink.of("wheel_position_pct", "moza_r21/wheel")[-2:] == [100.0, -100.0]


def test_rate_cap_holds_small_changes_until_flush():
    pub, sink = make(max_hz=10.0, deadband_pct=5.0)
    pub.prime(0.0)
    x = codes.ABS["ABS_X"]
    pub.handle(codes.EV_ABS, x, 32767 + 655, 0.01)  # ~+2 %: held
    assert len(sink.of("wheel_position_pct", "moza_r21/wheel")) == 1
    pub.flush(0.05)
    assert len(sink.of("wheel_position_pct", "moza_r21/wheel")) == 1
    pub.flush(0.11)
    assert len(sink.of("wheel_position_pct", "moza_r21/wheel")) == 2


def test_backstop_resends_everything():
    pub, sink = make()
    pub.prime(0.0)
    n = len(sink.puts)
    pub.backstop(1.0)
    assert len(sink.puts) == 2 * n


def test_buttons_press_release_and_no_autorepeat():
    pub, sink = make(prof=profile.parse({**_builtin(), "buttons": {"names": {0: "ptt"}}}))
    pub.handle(codes.EV_KEY, 288, 1, 0.0)
    pub.handle(codes.EV_KEY, 288, 2, 0.1)
    pub.handle(codes.EV_KEY, 288, 0, 0.2)
    pub.handle(codes.EV_KEY, 289, 1, 0.3)
    pub.handle(codes.EV_KEY, 999, 1, 0.4)  # not a key the device reported
    assert sink.of("button_state_change", "moza_r21/ptt") == [1, 0]
    assert sink.of("button_state_change", "moza_r21/btn_1") == [1]


def _builtin():
    import yaml

    return yaml.safe_load(profile.builtin_path("r21-ultra").read_text())


def test_short_helm_lock_scales_the_wheel_and_holds_past_it():
    sink = Sink()
    pub = InputPublisher(
        sink, profile.load("r21-ultra"), AXES, KEYS, "moza_r21", RateSettings(), lock_fraction=0.2
    )
    wheel = ("wheel_position_pct", "moza_r21/wheel")
    pub.prime(0.0)
    # A tenth of the axis' half travel to starboard is half of a lock at a fifth of it.
    pub.handle(codes.EV_ABS, codes.ABS["ABS_X"], round(32767.5 + 0.1 * 32767.5), 1.0)
    assert sink.of(*wheel)[-1] == pytest.approx(50.0, abs=0.1)
    pub.handle(codes.EV_ABS, codes.ABS["ABS_X"], 0, 2.0)  # the base's own port end
    assert sink.of(*wheel)[-1] == -100.0
    # The pedals are not scaled.
    pub.handle(codes.EV_ABS, codes.ABS["ABS_Z"], 500, 3.0)
    assert sink.of("joystick_rt_pct", "moza_r21/throttle")[-1] == 50.0


def test_moving_the_lock_republishes_the_wheel_at_once():
    pub, sink = make()
    wheel = ("wheel_position_pct", "moza_r21/wheel")
    pub.prime(0.0)
    pub.handle(codes.EV_ABS, codes.ABS["ABS_X"], round(32767.5 + 0.25 * 32767.5), 1.0)
    assert sink.of(*wheel)[-1] == pytest.approx(25.0, abs=0.1)
    n = len(sink.of(*wheel))
    pub.set_lock_fraction(0.5, 1.001)  # inside the rate cap: still sent
    assert len(sink.of(*wheel)) == n + 1 and sink.of(*wheel)[-1] == pytest.approx(50.0, abs=0.1)
