# keelson-connector-moza

A Moza direct-drive wheelbase as a keelson **helm**. The connector puts the wheel, any pedals connected to the base, and the buttons onto the bus. It also gives the operator **rudder follow-up** as force feedback: the wheel is pulled toward the position that matches where the rudder actually is.

It is built and tested for the **R21 Ultra**. The R16 and R21 have the same USB IDs (`346e:0000`, `346e:0010`), so the bundled profile should cover them too.

## What it publishes

The keys follow the pattern `{realm}/@v0/{entity}/pubsub/{subject}/{source-id}/{input}`, where `--source-id` defaults to `moza_r21`. Every subject is an existing keelson subject on the `realtime` QoS profile.

| Input | Subject | Type | Range |
|---|---|---|---|
| Wheel (`ABS_X`) | `wheel_position_pct` / `moza_r21/wheel` | `TimestampedFloat` | −100 port lock … +100 starboard lock |
| Throttle pedal (`ABS_Z`) | `joystick_rt_pct` / `moza_r21/throttle` | `TimestampedFloat` | 0 … 100 |
| Brake pedal (`ABS_RZ`) | `joystick_lt_pct` / `moza_r21/brake` | `TimestampedFloat` | 0 … 100 |
| Every button | `button_state_change` / `moza_r21/btn_<n>` | `TimestampedInt` | 1 pressed, 0 released |

How it publishes:
- **Axes** are sent when they change, capped at `--axis-max-hz`. A jump bigger than `--axis-deadband-pct` bypasses the cap.
- **A backstop** re-sends every axis at `--axis-min-hz` (default 10 Hz) even when the wheel is still. `vehicle_control/v1` treats silence as *stop*.
- **When the base disappears**, the connector stops publishing and exits with code **3**, so the helm goes silent and the restart policy brings it back.
- **The mapping** comes from a profile, [`moza_connector/profiles/r21-ultra.yaml`](moza_connector/profiles/r21-ultra.yaml). Copy it and pass `-p path.yaml` to rename buttons or remap axes.
- **Liveliness:** the connector declares a source token for `moza_r21` and a token for each subject. It serves no RPC.

Wiring `wheel_position_pct/moza_r21/wheel` to a vehicle is the consumer's job; for example, `vehicle_control/v1 set_control_mapping` on mavlink2keelson. **This connector never takes control authority.** Who holds the conn is decided elsewhere.

## Force feedback

| `--ffb` | Behaviour |
|---|---|
| `follow-up` (default) | A spring pulls the wheel toward `rudder_angle_deg / --ffb-rudder-max-deg` of full lock. The rudder data comes from `rudder_angle_deg` on `--ffb-entity` (default: the same entity) and `--ffb-source` (default `**`). If the rudder data is older than `--ffb-stale-s` (default 1 s), or none has arrived yet, the spring **falls back to plain centring**. It never holds an offset that nobody is reporting. |
| `center` | A centring spring and nothing else. |
| `off` | The base's force feedback is left untouched. |

In every mode except `off`, the connector also adds a damper (`--ffb-damper-pct`, 0 to disable) and switches off the base's own autocentre so the two don't fight.

The rudder is assumed to report **positive to starboard**, the same as `wheel_position_pct`. If yours reports positive to port, pass `--ffb-rudder-invert`.

### Safety

The R21 Ultra peaks at 21 Nm, which is enough to injure a wrist.

- **Gain:** the overall gain defaults to `--ffb-max-gain-pct 30`. A **hard ceiling of 60 %** is enforced in code, and the CLI refuses any value above it.
- **Ramp-up:** at start-up, the gain rises from 0 over 2 s. The wheel may be anywhere when the connector starts, and this keeps it from being whipped to centre.
- **On exit,** whether from SIGTERM, Ctrl-C, an error or the base being unplugged, every effect is stopped and erased and the gain is set to 0. The wheel is left **limp**.
- **First power-on:** clamp the base down and keep your hands off the rim.

## Requirements

- **A Linux host with kernel ≥ 6.15**, or 6.12.24+, 6.13.12+ or 6.14.3+. These kernels ship `hid-universal-pidff`, the in-tree driver for Moza force feedback.
- **macOS can't run this against hardware.** Moza has no force-feedback support there, and Docker Desktop can't pass USB through. The tests run on macOS against a fake base.
- **Set the physical rotation range on the base**, using Pit House or [Boxflat](https://github.com/Lawstorant/boxflat). The connector normalises over whatever range the base reports, so ±100 % is always full lock.

## First run on a new base

```bash
lsusb | grep -i 346e                  # 346e:0000 or 346e:0010
sudo evtest                           # pick "... MOZA R21 Ultra Base"; turn the wheel and press the pedals
```

Check that `ABS_X` moves with the wheel and that the pedals move `ABS_Z` and `ABS_RZ`. Note which button numbers you want to name. If anything differs, copy the profile and adjust it.

```bash
sudo cp udev/99-moza.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
moza2keelson -e <vessel_id> --ffb off                                  # 1. inputs only
moza2keelson -e <vessel_id> --ffb center --ffb-max-gain-pct 10         # 2. gentle centring; Ctrl-C leaves it limp
moza2keelson -e <vessel_id> --ffb follow-up --ffb-rudder-max-deg 35    # 3. follow-up
```

## Running

The example below uses Docker on a Linux host. Edit `<vessel_id>` in [`docker-compose.moza.yml`](docker-compose.moza.yml) first.

```bash
docker compose -f docker-compose.moza.yml up -d
```

To run it natively:

```bash
uv sync --locked
.venv/bin/moza2keelson -e <vessel_id>    # or: .venv/bin/python bin/moza2keelson.py
```

The connector exits with one of these codes:

| Code | Meaning |
|---|---|
| 0 | Stopped by a signal |
| 1 | A worker thread failed |
| 2 | No base found and `--wait-s 0` |
| 3 | The base went away |

```
usage: moza2keelson [-h] [--log-level LOG_LEVEL] [--mode {peer,client}]
                    [--connect CONNECT] [--listen LISTEN]
                    [--zenoh-config ZENOH_CONFIG] [-r REALM] -e ENTITY_ID
                    [-s SOURCE_ID] [-p PROFILE] [-d DEVICE] [--wait-s WAIT_S]
                    [--axis-max-hz AXIS_MAX_HZ] [--axis-min-hz AXIS_MIN_HZ]
                    [--axis-deadband-pct AXIS_DEADBAND_PCT]
                    [--axis-center-snap-pct AXIS_CENTER_SNAP_PCT]
                    [--ffb {off,center,follow-up}]
                    [--ffb-max-gain-pct FFB_MAX_GAIN_PCT]
                    [--ffb-spring-pct FFB_SPRING_PCT]
                    [--ffb-damper-pct FFB_DAMPER_PCT]
                    [--ffb-entity FFB_ENTITY] [--ffb-source FFB_SOURCE]
                    [--ffb-rudder-max-deg FFB_RUDDER_MAX_DEG]
                    [--ffb-rudder-invert] [--ffb-stale-s FFB_STALE_S]
```

Run `moza2keelson --help` for the full list of options and their defaults.

## The keelson pin

keelson is pinned **exactly** (`keelson==0.6.0rc19`, which is tag `0.6.0-pre.19`) in [`pyproject.toml`](pyproject.toml) and `uv.lock`. Never point it at a path or a local build. To move it, change the pin and run `uv lock`.

## Development

```bash
uv sync --locked --extra dev
.venv/bin/ruff check .
.venv/bin/python -m pytest -q -m "not e2e"   # pure model, profile, force feedback, input mapping
.venv/bin/python -m pytest -q -m e2e         # the CLI against a scripted base on an isolated loopback Zenoh peer

# The real evdev path against a uinput virtual base (any Linux kernel, incl. Docker Desktop's):
docker build -t keelson-connector-moza .
docker run --rm --privileged -u root -v /dev:/dev -v "$PWD/scripts":/t:ro keelson-connector-moza "python -I /t/uinput_check.py"
```

The uinput check exercises discovery, absinfo, events, and effect upload, update, erase and gain against the kernel's force-feedback layer. It does not test the Moza firmware itself; only the hardware steps under "First run on a new base" do that.

## Not yet

- **Base settings over Moza's serial protocol** (`/dev/ttyACM*`): rotation range, strength, temperatures. This would be reimplemented from the protocol notes in Boxflat. Boxflat is GPL-3.0, so its code must not be copied into this Apache-2.0 repo.
- **A macOS relay** for reading inputs only.
- **Live tuning of force feedback** through `configurable/v1`.
