# CLAUDE.md

A Moza direct-drive wheelbase (R21 Ultra) as a keelson helm (README). It publishes the wheel, pedals and buttons, and drives force feedback, either centring or rudder follow-up.

## Rules that must not be broken

- **The motor can hurt someone.** The R21 Ultra peaks at 21 Nm.
  - The overall gain has a hard ceiling, `model.MAX_GAIN_CEILING_PCT`. The CLI refuses anything above it. Never raise the ceiling as a side effect of another change.
  - Force feedback ramps its gain up from 0 (`model.GAIN_RAMP_S`). It never switches on at full strength.
  - Every exit path ends in `FfbController.shutdown()`: effects stopped and erased, gain at 0. It never raises, and it runs even when the base is already gone.
- **Stale rudder data is never followed.** Follow-up falls back to plain centring after `--ffb-stale-s`, or when no rudder reading has arrived yet. A rudder timestamp in the future counts as stale.
- **Silence means stop.** The backstop republishes every axis at `--axis-min-hz` (`vehicle_control/v1`). When the base goes away, publishing stops and the process exits with code 3. It never republishes a last-known value from a device that is gone.
- **Pure decisions.** `moza_connector/model.py` has no clock, no evdev and no Zenoh, and every decision is tested there.
- **evdev stays at the edge.** Only `EvdevDevice` and `device.find()` import evdev, and only on Linux. Everything else must import on macOS. `codes.py` spells out the event codes, and `EvdevDevice` checks them against `evdev.ecodes`.
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
| `device.py` | The `MozaDevice` protocol, `EvdevDevice` (real), `FakeDevice` (scripted, records force-feedback calls) |
| `ffb.py` | `FfbController`: spring, damper, gain ramp, safe shutdown |
| `inputs.py` | `InputPublisher`: device events to enveloped payloads through a `put(subject, source, bytes)` callback |
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
