"""Linux-only check of the real EvdevDevice path against a uinput virtual Moza base.

Run inside the connector image, privileged, with the host /dev mounted:
  docker run --rm --privileged -u root -v /dev:/dev -v "$PWD/scripts":/t:ro keelson-connector-moza "python -I /t/uinput_check.py"

The kernel's own uinput force-feedback plumbing stands in for hid-universal-pidff: it proves
discovery, absinfo, events, effect upload/update/erase and gain, not the Moza firmware.
"""

import json
import signal
import subprocess
import sys
import threading
import time

import keelson
import zenoh
from evdev import AbsInfo, UInput, ecodes as e
from keelson.payloads.Primitives_pb2 import TimestampedFloat

PORT = 7499
REALM, V = "chk", "v"


class VirtualBase:
    def __init__(self):
        caps = {
            e.EV_ABS: [
                (e.ABS_X, AbsInfo(value=32767, min=0, max=65535, fuzz=0, flat=0, resolution=0)),
                (e.ABS_Z, AbsInfo(value=0, min=0, max=1000, fuzz=0, flat=0, resolution=0)),
            ],
            e.EV_KEY: [e.BTN_TRIGGER, e.BTN_THUMB],
            e.EV_FF: [e.FF_SPRING, e.FF_DAMPER, e.FF_GAIN, e.FF_AUTOCENTER],
        }
        self.ui = UInput(caps, name="Gudsen MOZA R21 Ultra Base", vendor=0x346E, product=0x0000, max_effects=16)
        self.log = []
        self.stop = False
        self.t = threading.Thread(target=self._ff_loop, daemon=True)
        self.t.start()

    def _ff_loop(self):
        import select

        while not self.stop:
            r, _, _ = select.select([self.ui.fd], [], [], 0.1)
            if not r:
                continue
            for ev in self.ui.read():
                if ev.type == e.EV_UINPUT and ev.code == e.UI_FF_UPLOAD:
                    up = self.ui.begin_upload(ev.value)
                    up.retval = 0
                    c = up.effect.u.ff_condition_effect[0]
                    self.log.append(("upload", up.effect.type, up.effect.id, c.center, c.right_coeff))
                    self.ui.end_upload(up)
                elif ev.type == e.EV_UINPUT and ev.code == e.UI_FF_ERASE:
                    er = self.ui.begin_erase(ev.value)
                    er.retval = 0
                    self.log.append(("erase", er.effect_id))
                    self.ui.end_erase(er)
                elif ev.type == e.EV_FF:
                    self.log.append(("ff", ev.code, ev.value))

    def move(self, code, value):
        self.ui.write(e.EV_ABS, code, value)
        self.ui.syn()

    def close(self):
        self.stop = True
        self.t.join()
        self.ui.close()


def zconf(**kv):
    c = zenoh.Config()
    c.insert_json5("scouting/multicast/enabled", "false")
    c.insert_json5("scouting/gossip/enabled", "false")
    for k, v in kv.items():
        c.insert_json5(k, json.dumps(v))
    return c


def main():
    session = zenoh.open(zconf(mode="peer", **{"listen/endpoints": [f"tcp/127.0.0.1:{PORT}"]}))
    wheel = []
    session.declare_subscriber(
        f"{REALM}/@v0/{V}/pubsub/wheel_position_pct/**",
        lambda s: wheel.append(TimestampedFloat.FromString(keelson.uncover(s.payload.to_bytes())[2]).value),
    )
    rudder = session.declare_publisher(f"{REALM}/@v0/{V}/pubsub/rudder_angle_deg/sim/rudder")

    def put_rudder(deg):
        p = TimestampedFloat()
        p.timestamp.FromNanoseconds(time.time_ns())
        p.value = deg
        rudder.put(keelson.enclose(p.SerializeToString()))

    zfile = "/tmp/z.json5"
    with open(zfile, "w") as f:
        json.dump({"mode": "peer", "scouting": {"multicast": {"enabled": False}, "gossip": {"enabled": False}},
                   "connect": {"endpoints": [f"tcp/127.0.0.1:{PORT}"]}, "listen": {"endpoints": []}}, f)

    def run(base, *extra):
        return subprocess.Popen(["moza2keelson", "-r", REALM, "-e", V, "--zenoh-config", zfile, *extra],
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

    results = {}

    # --- 1: discovery, absinfo, events, follow-up, SIGTERM cleanup
    base = VirtualBase()
    time.sleep(0.5)
    proc = run(base, "--ffb", "follow-up")
    deadline = time.time() + 10
    while not wheel and time.time() < deadline:
        time.sleep(0.05)
    results["primed_centre"] = wheel[:1]
    base.move(e.ABS_X, 65535)
    time.sleep(0.3)
    results["full_lock_seen"] = 100.0 in wheel
    for _ in range(20):
        put_rudder(17.5)
        time.sleep(0.05)
    time.sleep(0.3)
    half = round(0.5 * 0x7FFF)
    springs = [x for x in base.log if x[0] == "upload" and x[1] == e.FF_SPRING]
    results["spring_centres"] = [x[3] for x in springs]
    results["follow_up_reached_half"] = any(x[3] == half for x in springs)
    results["autocenter_off"] = ("ff", e.FF_AUTOCENTER, 0) in base.log
    gains = [x[2] for x in base.log if x[0] == "ff" and x[1] == e.FF_GAIN]
    results["gain_ramp_monotonic_start0"] = gains[:1] == [0] and gains == sorted(gains)
    time.sleep(2.2)
    gains = [x[2] for x in base.log if x[0] == "ff" and x[1] == e.FF_GAIN]
    results["gain_final"] = gains[-1]
    proc.send_signal(signal.SIGTERM)
    results["sigterm_exit"] = proc.wait(timeout=10)
    time.sleep(0.3)
    uploaded = {x[2] for x in base.log if x[0] == "upload"}
    erased = {x[1] for x in base.log if x[0] == "erase"}
    results["all_erased"] = bool(uploaded) and uploaded == erased
    results["gain_zero_last"] = [x for x in base.log if x[0] == "ff" and x[1] == e.FF_GAIN][-1][2] == 0
    out1 = proc.stdout.read()
    base.close()

    # --- 2: unplug -> exit 3 and silence
    wheel.clear()
    base = VirtualBase()
    time.sleep(0.5)
    proc = run(base, "--ffb", "center")
    deadline = time.time() + 10
    while not wheel and time.time() < deadline:
        time.sleep(0.05)
    time.sleep(0.5)
    base.close()  # unplug
    results["unplug_exit"] = proc.wait(timeout=10)
    n = len(wheel)
    time.sleep(0.5)
    results["silent_after_unplug"] = len(wheel) == n
    out2 = proc.stdout.read()

    print(json.dumps(results, indent=1))
    ok = (results["primed_centre"] == [0.0] and results["full_lock_seen"] and results["follow_up_reached_half"]
          and results["autocenter_off"] and results["gain_ramp_monotonic_start0"] and results["gain_final"] > 0
          and results["sigterm_exit"] == 0 and results["all_erased"] and results["gain_zero_last"]
          and results["unplug_exit"] == 3 and results["silent_after_unplug"])
    if not ok:
        print("---- run 1 ----\n" + out1 + "\n---- run 2 ----\n" + out2)
    print("PASS" if ok else "FAIL")
    session.close()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
