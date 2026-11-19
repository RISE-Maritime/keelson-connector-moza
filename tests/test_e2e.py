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
from keelson.payloads.Primitives_pb2 import TimestampedFloat, TimestampedInt

from moza_connector import codes, model

pytestmark = pytest.mark.e2e

ROOT = Path(__file__).resolve().parent.parent
REALM, V = "e2e", "testvessel"
BASE = f"{REALM}/@v0/{V}/pubsub"


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
    uploaded = {e[2] for e in ff if e[0] == "upload"}
    erased = {e[1] for e in ff if e[0] == "erase"}
    assert uploaded and erased == uploaded
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
