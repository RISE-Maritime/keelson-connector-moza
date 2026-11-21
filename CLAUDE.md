# CLAUDE.md

A Moza direct-drive wheelbase (R21 Ultra) as a keelson helm (README). It publishes the wheel, pedals and buttons, and drives force feedback, either centring or rudder follow-up.

## Rules that must not be broken

- **The motor can hurt someone.** The R21 Ultra peaks at 21 Nm.
  - The overall gain has a hard ceiling, `model.MAX_GAIN_CEILING_PCT`. The CLI refuses anything above it. Never raise the ceiling as a side effect of another change.
  - Force feedback ramps its gain up from 0 (`model.GAIN_RAMP_S`). It never switches on at full strength.
  - Every exit path ends in `FfbController.shutdown()`: effects stopped and erased, gain at 0. It never raises, and it runs even when the base is already gone.
- **Remote force feedback is opt-in and capped.** Only with `--ffb-remote` does the connector serve `configurable/v1`. It then starts with force feedback off, refuses a gain above `--ffb-max-gain-pct` (never clips it), ramps every gain rise from the current level, applies drops at once, and refuses every change after `shutdown()`.
- **The simulated rudder is Slipway's.** `rudder_sim.py` mirrors Slipway/DRYDOCK's steering gear. Its constants and equations change only with a cited Slipway or DRYDOCK source, and `tests/test_rudder_sim.py` pins them. It publishes as `rudder_sim`, never as `slipway`.
- **Saved settings never switch the motor on.** The settings file never restores the mode, and a saved gain above today's cap comes back as the cap.
- **Stale rudder data is never followed.** Follow-up falls back to plain centring after `--ffb-stale-s`, or when no rudder reading has arrived yet. A rudder timestamp in the future counts as stale.
- **Silence means stop.** The backstop republishes every axis at `--axis-min-hz` (`vehicle_control/v1`). When the base goes away, publishing stops and the process exits with code 3. It never republishes a last-known value from a device that is gone.
- **Pure decisions.** `moza_connector/model.py` has no clock, no evdev and no Zenoh, and every decision is tested there.
- **evdev and hidapi stay at the edge.** Only `EvdevDevice` and `device.find()` import evdev, and only on Linux. Only `hid_device.open_hid()` and `find_hid()` import hidapi. Everything else must import on both platforms. `codes.py` spells out the event codes, and `EvdevDevice` checks them against `evdev.ecodes`.
- **HID reports are found by usage.** `HidDevice` never writes to a hard-coded report id or offset; it looks each field up in the base's own descriptor (`hid_report.py`), and an out-of-range value raises instead of being clipped.
- **HID effects are leased.** macOS doesn't erase a dead process's effects, so every HID effect has a finite duration (`EFFECT_LEASE_MS`) and is renewed while it plays. Never make one infinite. A click (`upload_click`) is the exception to renewal: it is never renewed, and its own duration may not exceed a lease.
- **The fake device is for the tests only.** `--fake-device` is refused unless `MOZA_CONNECTOR_TESTING=1`.
- **No loopback.** The connector subscribes only to `rudder_angle_deg`, which it never publishes. If it ever needs to subscribe to a subject it also publishes, filter out its own source id first.
- **No control authority.** The connector publishes operator input; who holds the conn is decided elsewhere.
- **Existing subjects only.** The connector publishes `wheel_position_pct`, `joystick_rt/lt_pct` and `button_state_change`. A new subject belongs in keelson's `messages/subjects.yaml`, not here.
- **keelson is an exact published pin.** Never use a path or a local build.
- **Tests never touch a network.** The e2e uses a loopback peer with multicast and gossip scouting off.
- **`tests/` has no `__init__.py`.**
- **Boxflat is GPL-3.0.** Its protocol notes may inform a serial feature, but its code must never be copied here.

## Layout

| Module | Role |
|---|---|
| `model.py` | Normalisation, rate cap, gain, follow-up, staleness (pure) |
| `profile.py` | YAML profile mapping evdev codes to subjects, validated against keelson's subject registry |
| `device.py` | The `MozaDevice` protocol, `EvdevDevice` (Linux), `FakeDevice` (scripted, records force-feedback calls), backend choice |
| `hid_report.py` | HID report descriptor parsing; reports encoded and decoded by usage name (pure) |
| `hid_device.py` | `HidDevice` (macOS): inputs and PID force feedback over hidapi, with effect leases |
| `ffb.py` | `FfbController`: spring, damper, gain ramp, safe shutdown |
| `inputs.py` | `InputPublisher`: device events to enveloped payloads through a `put(subject, source, bytes)` callback |
| `rudder_sim.py` | Slipway's steering gear, mirrored: the rudder order, the rate-limited gear, the dead-man (pure) |
| `rudder_sim_app.py` | `moza-rudder-sim`: that rudder on the bus, steered by the wheel |
| `store.py` | The settings file: saved settings and behaviours, read at start, written after each remote change (I/O only; the rules are `model.load_state`) |
| `app.py` | CLI, Zenoh session, liveliness, rudder subscriber, backstop and force-feedback threads |

## Commands

```bash
uv sync --locked --extra dev
.venv/bin/python -m pytest -q        # all
.venv/bin/python -m pytest -m e2e    # the CLI against a scripted base on an isolated loopback peer
.venv/bin/ruff check .
docker build -t keelson-connector-moza . && docker run --rm keelson-connector-moza "moza2keelson --help"
# the real evdev path, Linux kernel via Docker, against a uinput virtual base:
docker run --rm --privileged -u root -v /dev:/dev -v "$PWD/scripts":/t:ro keelson-connector-moza "python -I /t/uinput_check.py"
```
