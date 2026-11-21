"""The CLI as a real process against a scripted base, on an ISOLATED loopback Zenoh pair:
multicast and gossip scouting off on both sides, so nothing here reaches a shared network."""

import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path

import keelson
import pytest
import zenoh
from keelson.interfaces.ErrorResponse_pb2 import ErrorResponse
from keelson.payloads.Primitives_pb2 import TimestampedFloat, TimestampedInt, TimestampedString

from moza_connector import codes, model

pytestmark = pytest.mark.e2e

ROOT = Path(__file__).resolve().parent.parent
REALM, V = "e2e", "testvessel"
BASE = f"{REALM}/@v0/{V}/pubsub"


def effect_ids(ff):
    """Every effect the base was given: conditions and clicks (detent, centre mark)."""
    return {e[2] for e in ff if e[0] == "upload"} | {e[1] for e in ff if e[0] == "click"}


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def isolated(**kv):
    conf = zenoh.Config()
    conf.insert_json5("scouting/multicast/enabled", "false")
    conf.insert_json5("scouting/gossip/enabled", "false")
    for k, v in kv.items():
        conf.insert_json5(k, json.dumps(v))
    return conf


def wait(pred, timeout=10.0, what="condition"):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return
        time.sleep(0.05)
    raise AssertionError(f"timed out waiting for {what}")


class Harness:
    def __init__(self, tmp: Path):
        self.tmp = tmp
        self.port = free_port()
        self.proc = None
        self.session = zenoh.open(
            isolated(mode="peer", **{"listen/endpoints": [f"tcp/127.0.0.1:{self.port}"], "connect/endpoints": []})
        )
        self.lock = threading.Lock()
        self.wheel: list[tuple[float, float]] = []
        self.buttons: list[tuple[str, int]] = []
        self._subs = [
            self.session.declare_subscriber(f"{BASE}/wheel_position_pct/**", self._on_wheel),
            self.session.declare_subscriber(f"{BASE}/button_state_change/**", self._on_button),
        ]
        self.rudder = self.session.declare_publisher(f"{BASE}/rudder_angle_deg/sim/rudder")
        self.ff_log = tmp / "ff.jsonl"

    def _on_wheel(self, s):
        v = TimestampedFloat.FromString(keelson.uncover(s.payload.to_bytes())[2]).value
        with self.lock:
            self.wheel.append((time.monotonic(), v))

    def _on_button(self, s):
        v = TimestampedInt.FromString(keelson.uncover(s.payload.to_bytes())[2]).value
        with self.lock:
            self.buttons.append((str(s.key_expr).rsplit("/", 1)[-1], v))

    def put_rudder(self, deg):
        p = TimestampedFloat()
        p.timestamp.FromNanoseconds(time.time_ns())
        p.value = deg
        self.rudder.put(keelson.enclose(p.SerializeToString()))

    def start(self, events, *extra):
        script = self.tmp / "base.json"
        script.write_text(
            json.dumps(
                {
                    "name": "Fake MOZA R21 Ultra Base",
                    "axes": {"ABS_X": [0, 65535, 32767], "ABS_Z": [0, 1000, 0]},
                    "keys": [288, 289],
                    "ff": True,
                    "ff_log_path": str(self.ff_log),
                    "events": events,
                }
            )
        )
        zcfg = self.tmp / "zenoh.json5"
        zcfg.write_text(
            json.dumps(
                {
                    "mode": "peer",
                    "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
                    "connect": {"endpoints": [f"tcp/127.0.0.1:{self.port}"]},
                    "listen": {"endpoints": []},
                }
            )
        )
        self.proc = subprocess.Popen(
            [sys.executable, str(ROOT / "bin/moza2keelson.py"), "-r", REALM, "-e", V, "--zenoh-config", str(zcfg),
             "--fake-device", str(script), *extra],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            env={**os.environ, "MOZA_CONNECTOR_TESTING": "1"},
        )
        token = keelson.construct_source_liveliness_key(REALM, V, "moza_r21")
        wait(lambda: any(True for _ in self.session.liveliness().get(token, timeout=0.5)), what="liveliness")

    def ff(self):
        if not self.ff_log.exists():
            return []
        return [tuple(json.loads(line)) for line in self.ff_log.read_text().splitlines()]

    def close(self):
        if self.proc and self.proc.poll() is None:
            self.proc.kill()
            self.proc.communicate()
        for s in self._subs:
            s.undeclare()
        self.session.close()


@pytest.fixture
def h(tmp_path):
    harness = Harness(tmp_path)
    yield harness
    harness.close()


def test_helm_on_the_bus_follow_up_and_silence_when_unplugged(h):
    h.start(
        [[0.5, "EV_ABS", "ABS_X", 65535], [0.1, "EV_KEY", 288, 1], [0.1, "EV_KEY", 288, 0], [2.5, "gone"]],
        "--axis-min-hz", "10", "--ffb", "follow-up", "--ffb-rudder-max-deg", "35",
    )
    # The wheel: centred at start, full starboard lock after the script turns it.
    wait(lambda: any(v == 100.0 for _, v in h.wheel), what="full lock on wheel_position_pct")
    assert h.wheel[0][1] == 0.0
    wait(lambda: h.buttons[-2:] == [("btn_0", 1), ("btn_0", 0)], what="the button press")

    # Follow-up: half rudder pulls the spring centre to half lock.
    half = round(0.5 * model.S16_MAX)

    def followed():
        h.put_rudder(17.5)
        return any(e[0] == "upload" and e[1] == codes.FF_SPRING and e[3] == half for e in h.ff())

    wait(followed, what="the spring to follow the rudder")

    # A still wheel keeps talking (the backstop): vehicle_control/v1 reads silence as stop.
    t0 = time.monotonic()
    time.sleep(1.0)
    with h.lock:
        assert sum(1 for t, _ in h.wheel if t >= t0) >= 7

    # Unplugged: the process exits EXIT_DEVICE_LOST and the helm goes silent.
    assert h.proc.wait(timeout=10) == 3
    out = h.proc.stdout.read()
    assert "The base is gone" in out
    with h.lock:
        n = len(h.wheel)
    time.sleep(0.5)
    assert len(h.wheel) == n


def test_sigterm_leaves_the_wheel_limp(h):
    h.start([], "--ffb", "center")
    wait(lambda: any(e[0] == "upload" and e[1] == codes.FF_SPRING for e in h.ff()), what="the centring spring")
    h.proc.send_signal(signal.SIGTERM)
    assert h.proc.wait(timeout=10) == 0
    ff = h.ff()
    erased = {e[1] for e in ff if e[0] == "erase"}
    assert effect_ids(ff) and erased == effect_ids(ff)
    assert ff[-1] == ("gain", 0)


def test_fake_device_refused_outside_the_tests(tmp_path):
    env = {k: v for k, v in os.environ.items() if k != "MOZA_CONNECTOR_TESTING"}
    r = subprocess.run(
        [sys.executable, str(ROOT / "bin/moza2keelson.py"), "-e", V, "--fake-device", str(tmp_path / "x.json")],
        capture_output=True,
        text=True,
        env=env,
    )
    assert r.returncode == 2 and "for the tests only" in r.stderr


def rpc(h, procedure, payload=None):
    key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", procedure, "moza_r21")
    replies = list(h.session.get(key, payload=payload, timeout=5.0))
    assert len(replies) == 1, f"{procedure}: {len(replies)} replies"
    r = replies[0]
    if r.ok is not None:
        return True, r.ok.payload.to_bytes()
    return False, ErrorResponse.FromString(r.err.payload.to_bytes()).error_description


def test_remote_force_feedback_over_configurable(h):
    published = []
    sub = h.session.declare_subscriber(f"{BASE}/configuration_json/moza_r21", lambda s: published.append(s))
    try:
        h.start([], "--ffb", "follow-up", "--ffb-remote", "--ffb-max-gain-pct", "20")
        key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "get_config", "moza_r21")
        wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1")

        ok, body = rpc(h, "get_config")
        cfg = json.loads(body)
        assert ok and cfg["enabled"] is False and cfg["active"] is False and cfg["gain_cap_pct"] == 20
        time.sleep(0.3)
        assert h.ff() == []  # --ffb-remote starts off whatever --ffb says

        ok, _ = rpc(h, "set_config", json.dumps({"mode": "center", "max_gain_pct": 10}).encode())
        assert ok
        time.sleep(0.3)
        assert h.ff() == []  # a mode is the profile's: it does not switch force feedback on
        assert rpc(h, "set_config", json.dumps({"enabled": True}).encode())[0]
        wait(lambda: any(e[0] == "upload" and e[1] == codes.FF_SPRING for e in h.ff()), what="the spring")
        wait(lambda: ("gain", model.gain_level(10)) in h.ff(), what="the gain to ramp to 10%")
        g = [e[1] for e in h.ff() if e[0] == "gain"]
        assert g[0] == 0 and g == sorted(g)
        assert json.loads(rpc(h, "get_config")[1])["active"] is True

        ok, err = rpc(h, "set_config", json.dumps({"max_gain_pct": 99}).encode())
        assert not ok and "cap is 20" in err
        assert json.loads(rpc(h, "get_config")[1])["max_gain_pct"] == 10

        ok, _ = rpc(h, "set_config", json.dumps({"mode": "off"}).encode())
        assert ok
        ff = h.ff()
        assert effect_ids(ff) == {e[1] for e in ff if e[0] == "erase"}
        assert ff[-1] == ("gain", 0)
        wait(lambda: len(published) >= 3, what="configuration_json")
        last = json.loads(TimestampedString.FromString(keelson.uncover(published[-1].payload.to_bytes())[2]).value)
        assert last["enabled"] is False and last["mode"] == "center" and last["active"] is False
    finally:
        sub.undeclare()


def test_no_remote_control_without_the_flag(h):
    h.start([], "--ffb", "off")
    key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "get_config", "moza_r21")
    time.sleep(0.5)
    assert list(h.session.get(key, timeout=1.0)) == []


def test_rudder_sim_follows_the_wheel_and_returns_when_it_goes_silent(h):
    zcfg = h.tmp / "sim-zenoh.json5"
    zcfg.write_text(
        json.dumps(
            {
                "mode": "peer",
                "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
                "connect": {"endpoints": [f"tcp/127.0.0.1:{h.port}"]},
                "listen": {"endpoints": []},
            }
        )
    )
    rudder: list[tuple[float, float]] = []
    sub = h.session.declare_subscriber(
        f"{BASE}/rudder_angle_deg/rudder_sim",
        lambda s: rudder.append(
            (time.monotonic(), TimestampedFloat.FromString(keelson.uncover(s.payload.to_bytes())[2]).value)
        ),
    )
    wheel = h.session.declare_publisher(f"{BASE}/wheel_position_pct/moza_r21/wheel")
    proc = subprocess.Popen(
        [sys.executable, str(ROOT / "bin/moza-rudder-sim.py"), "-r", REALM, "-e", V, "--zenoh-config", str(zcfg),
         "--hull", "usv-6m"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    try:
        wait(lambda: len(rudder) >= 3, what="the simulated rudder")

        def steer(pct):
            p = TimestampedFloat()
            p.timestamp.FromNanoseconds(time.time_ns())
            p.value = pct
            wheel.put(keelson.enclose(p.SerializeToString()))

        # Half starboard helm on a usv (30 deg travel): the rudder settles at 15 deg.
        def at_15():
            steer(50.0)
            return rudder and abs(rudder[-1][1] - 15.0) < 0.5

        wait(at_15, timeout=5.0, what="15 deg of rudder")
        t0 = time.monotonic()
        time.sleep(1.0)
        assert 8 <= sum(1 for t, _ in rudder if t >= t0) <= 12  # 10 Hz, like Slipway's tick

        # Jammed from the panel: the rudder holds at 15 deg whatever the wheel orders.
        key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "set_config", "rudder_sim")
        wait(lambda: any(r.ok for r in h.session.get(key.replace("set_config", "get_config"), timeout=0.5)),
             what="the sim's configurable/v1")
        (reply,) = list(h.session.get(key, payload=json.dumps({"fault": "jammed"}).encode(), timeout=5.0))
        assert reply.ok is not None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.0:
            steer(-100.0)
            time.sleep(0.1)
        assert abs(rudder[-1][1] - 15.0) < 0.5
        (reply,) = list(h.session.get(key, payload=json.dumps({"fault": "none"}).encode(), timeout=5.0))
        assert reply.ok is not None

        # The wheel goes silent: after 1 s the order is amidships, never the last value. (Wait
        # the second out first: un-jammed, the rudder swings through 0 on its way to -30.)
        time.sleep(1.5)
        wait(lambda: abs(rudder[-1][1]) < 0.5, timeout=5.0, what="the rudder back amidships")
    finally:
        proc.terminate()
        out, _ = proc.communicate(timeout=10)
        sub.undeclare()
        wheel.undeclare()
    assert proc.returncode == 0, out
    assert "returns to amidships" in out


def test_remote_profile_switches_behaviour(h):
    h.start([], "--ffb-remote", "--ffb-max-gain-pct", "20", "--ffb-profile", "car")
    key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "get_config", "moza_r21")
    wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1")
    cfg = json.loads(rpc(h, "get_config")[1])
    assert (cfg["profile"], cfg["mode"], cfg["enabled"], cfg["modified"]) == ("car", "center", False, [])
    assert [p["name"] for p in cfg["profiles"]] == [*model.PRESETS, model.COMMAND_LINE]

    assert rpc(h, "set_config", json.dumps({"enabled": True, "max_gain_pct": 10}).encode())[0]
    wait(lambda: ("gain", model.gain_level(10)) in h.ff(), what="the gain to ramp to 10%")
    ok, _ = rpc(h, "set_config", json.dumps({"profile": "notched"}).encode())
    assert ok
    cfg = json.loads(rpc(h, "get_config")[1])
    assert (cfg["profile"], cfg["mode"], cfg["active"], cfg["wheel_lock_deg"]) == ("notched", "detent", True, 90.0)
    wait(lambda: any(e[0] == "click" for e in h.ff()), what="the detent click uploaded")
    gains = [e[1] for e in h.ff() if e[0] == "gain"]
    assert 0 in gains[gains.index(model.gain_level(10)):]  # the lock changed while running: ramps in again
    assert rpc(h, "set_config", json.dumps({"enabled": False}).encode())[0]
    ff = h.ff()
    assert effect_ids(ff) == {e[1] for e in ff if e[0] == "erase"} and ff[-1] == ("gain", 0)


def test_profiles_and_the_default_survive_a_restart(h):
    path = h.tmp / "settings.json"
    h.start([], "--ffb-remote", "--ffb-max-gain-pct", "20", "--settings-file", str(path))
    key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "get_config", "moza_r21")
    wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1")
    assert rpc(h, "set_config", json.dumps({"profile": "car", "spring_pct": 33, "max_gain_pct": 12}).encode())[0]
    assert json.loads(rpc(h, "get_config")[1])["modified"] == ["spring_pct"]
    ok, err = rpc(h, "set_config", json.dumps({"save_profile": True}).encode())
    assert not ok and "built in" in err
    assert rpc(h, "set_config", json.dumps({"save_profile_as": "Rivo bridge", "enabled": True}).encode())[0]
    assert rpc(h, "set_config", json.dumps({"default_profile": "rivo-bridge"}).encode())[0]
    assert rpc(h, "set_config", json.dumps({"damper_pct": 70}).encode())[0]  # never saved: lost on restart
    saved = json.loads(path.read_text())
    assert saved["version"] == 2 and saved["default_profile"] == "rivo-bridge"
    assert saved["profiles"]["rivo-bridge"]["settings"]["spring_pct"] == 33 and "enabled" not in json.dumps(saved)

    h.proc.send_signal(signal.SIGTERM)
    assert h.proc.wait(timeout=10) == 0
    h.start([], "--ffb-remote", "--ffb-max-gain-pct", "20", "--settings-file", str(path))
    wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1 again")
    cfg = json.loads(rpc(h, "get_config")[1])
    assert (cfg["profile"], cfg["spring_pct"], cfg["damper_pct"], cfg["max_gain_pct"]) == ("rivo-bridge", 33, 15, 12)
    assert (cfg["enabled"], cfg["active"], cfg["modified"]) == (False, False, [])
    assert any(p["name"] == "rivo-bridge" and not p["builtin"] for p in cfg["profiles"])


def test_remote_test_pulse_plays_and_saves_nothing(h):
    path = h.tmp / "settings.json"
    h.start([], "--ffb-remote", "--ffb-max-gain-pct", "20", "--settings-file", str(path))
    key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "get_config", "moza_r21")
    wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1")
    ok, err = rpc(h, "set_config", json.dumps({"test": "hardover"}).encode())
    assert not ok and "choose a mode" in err  # off: nothing to feel

    assert rpc(h, "set_config", json.dumps({"mode": "center", "enabled": True, "max_gain_pct": 10}).encode())[0]
    path.unlink()
    assert rpc(h, "set_config", json.dumps({"test": "hardover"}).encode())[0]
    wait(lambda: any(e[0] == "click" and e[4] == model.HARDOVER_MS for e in h.ff()), what="the hard-over pulse")
    assert not path.exists()  # trying a haptic is not a setting
    ok, err = rpc(h, "set_config", json.dumps({"test": "center", "spring_pct": 50}).encode())
    assert not ok and "goes alone" in err


def base_angle(path):
    doc = json.loads(path.read_text())
    return {n: doc[n] for n in ("limit", "max-angle")}


def test_hard_stop_sets_the_bases_angle_and_puts_it_back(h, monkeypatch):
    base_file = h.tmp / "base_angle.json"
    monkeypatch.setenv("MOZA_FAKE_BASE_FILE", str(base_file))
    path = h.tmp / "settings.json"
    args = ("--ffb-remote", "--ffb-max-gain-pct", "20", "--settings-file", str(path), "--base-serial", "fake",
            "--ffb-profile", "small-boat")
    h.start([], *args)
    key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "get_config", "moza_r21")
    wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1")
    cfg = json.loads(rpc(h, "get_config")[1])
    assert (cfg["base_serial"], cfg["base_angle_deg"], cfg["base_max_deg"], cfg["hard_stop"]) == ("base at 900°", 900.0, 900.0, False)
    ok, err = rpc(h, "set_config", json.dumps({"base_range_deg": 540}).encode())
    assert not ok and "read from the base" in err

    assert rpc(h, "set_config", json.dumps({"hard_stop": True}).encode())[0]  # small boat: lock 90
    wait(lambda: base_angle(base_file) == {"limit": 90, "max-angle": 90}, what="the base at 180 deg")
    wait(lambda: json.loads(rpc(h, "get_config")[1])["base_serial"] == "hard stop at ±90°", what="the status")
    cfg = json.loads(rpc(h, "get_config")[1])
    assert (cfg["base_range_deg"], cfg["wheel_lock_deg"]) == (180.0, 90.0)
    assert json.loads(path.read_text())["base_restore"] == {"limit": 450, "max-angle": 450}

    h.proc.send_signal(signal.SIGTERM)
    assert h.proc.wait(timeout=10) == 0
    assert base_angle(base_file) == {"limit": 450, "max-angle": 450}  # put back on exit
    assert json.loads(path.read_text())["base_restore"] is None

    # A run that dies with the base narrowed: the next start puts it back first.
    h.start([], *args)
    wait(lambda: base_angle(base_file) == {"limit": 90, "max-angle": 90}, what="narrowed again")
    h.proc.kill()
    h.proc.communicate()
    h.start([], *(a for a in args if a not in ("--ffb-profile", "small-boat")), "--ffb-profile", "ship")
    wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1 after the crash")
    assert rpc(h, "set_config", json.dumps({"hard_stop": False}).encode())[0]
    wait(lambda: base_angle(base_file) == {"limit": 450, "max-angle": 450}, what="put back")
    h.proc.send_signal(signal.SIGTERM)
    out = h.proc.communicate(timeout=10)[0]
    assert "still narrowed by an earlier run" in out


def test_an_alarm_colours_the_rim_until_acknowledged(h, monkeypatch):
    from keelson.payloads.Alarm_pb2 import Alarm

    base_file = h.tmp / "base_angle.json"
    monkeypatch.setenv("MOZA_FAKE_BASE_FILE", str(base_file))
    h.start([], "--ffb-remote", "--ffb-max-gain-pct", "20", "--base-serial", "fake")
    key = keelson.construct_rpc_key(REALM, V, "configurable", "v1", "get_config", "moza_r21")
    wait(lambda: any(r.ok for r in h.session.get(key, timeout=0.5)), what="configurable/v1")
    assert json.loads(rpc(h, "get_config")[1])["rim_leds"] == "ready"
    assert json.loads(rpc(h, "get_config")[1])["base_light"] == "ready"
    assert rpc(h, "set_config", json.dumps({"alarm_leds": True, "alarm_base_light": True, "alarm_warning_color": "#ff8800",
                                            "alarm_fire_color": "#ff00aa"}).encode())[0]
    pub = h.session.declare_publisher(f"{BASE}/alarm/test/cpa")

    def alarm(acknowledged, ident="cpa-1", category=Alarm.CATEGORY_NAVIGATION):
        a = Alarm(identifier=ident, priority=Alarm.PRIORITY_WARNING, acknowledged=acknowledged, category=category)
        pub.put(keelson.enclose(a.SerializeToString()))

    bar = lambda: json.loads(base_file.read_text())["base-color0-0"]  # noqa: E731

    colour = lambda: json.loads(base_file.read_text())["rpm-color1"]  # noqa: E731
    wait(lambda: alarm(False) or colour() == 0xFF8800, what="the rim in the warning's colour")
    cfg = json.loads(rpc(h, "get_config")[1])
    assert (cfg["alarm_active"], cfg["rim_leds"]) == ("warning", "showing warning")
    wait(lambda: alarm(True) or colour() == 0x56F7FC, what="the rim's own colour back")
    alarm(False)
    wait(lambda: colour() == 0xFF8800, what="raised again")
    alarm(True)
    wait(lambda: colour() == 0x56F7FC, what="acknowledged again")
    # A fire alarm, sent at warning level: fire, on the rim and the base's light bar.
    fire = lambda ack: alarm(ack, "engine-room", Alarm.CATEGORY_FIRE_ALARM)  # noqa: E731
    wait(lambda: fire(False) or (colour() == 0xFF00AA and bar() == 0xFF00AA), what="both in the fire colour")
    assert json.loads(rpc(h, "get_config")[1])["alarm_active"] == "fire"
    wait(lambda: fire(True) or (colour() == 0x56F7FC and bar() == 0x56F7FC), what="both back")
    # Try: an emergency as it would come, nothing on the bus. Force feedback is off here:
    # the colour only, said in the log, not refused.
    assert rpc(h, "set_config", json.dumps({"test": "alarm-emergency"}).encode())[0]
    wait(lambda: colour() == 0xFF0000, what="the emergency's colour on Try")
    h.proc.send_signal(signal.SIGTERM)
    assert h.proc.wait(timeout=10) == 0
    assert colour() == 0x56F7FC  # put back on exit
