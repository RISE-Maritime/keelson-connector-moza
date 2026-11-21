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
- **The mapping** comes from a profile, [`moza_connector/profiles/r21-ultra.yaml`](moza_connector/profiles/r21-ultra.yaml). Copy it and pass `-p path.yaml` to rename buttons or remap axes. **Without pedals, use `-p r21-ultra-no-pedals`.** The base reports its pedal axes even when nothing is plugged in, parked at their minimum, so `r21-ultra` would publish a steady 0 % throttle and brake from pedals that don't exist.
- **Button numbers:** `btn_<n>` is HID Button n+1, the same physical button on Linux and macOS.
- **Liveliness:** the connector declares a source token for `moza_r21` and a token for each subject. It serves no RPC, except `configurable/v1` under `--ffb-remote` (see [From Crowsnest](#from-crowsnest---ffb-remote)).

Wiring `wheel_position_pct/moza_r21/wheel` to a vehicle is the consumer's job; for example, `vehicle_control/v1 set_control_mapping` on mavlink2keelson. **This connector never takes control authority.** Who holds the conn is decided elsewhere.

## Force feedback

| `--ffb` | Behaviour |
|---|---|
| `follow-up` (default) | A spring pulls the wheel toward `rudder_angle_deg / --ffb-rudder-max-deg` of full lock. The rudder data comes from `rudder_angle_deg` on `--ffb-entity` (default: the same entity) and `--ffb-source` (default `**`). If the rudder data is older than `--ffb-stale-s` (default 1 s), or none has arrived yet, the spring **falls back to plain centring**. It never holds an offset that nobody is reporting. |
| `center` | A centring spring and nothing else. |
| `detent` | No spring: the wheel turns freely and stays where it is left. It gives a short click (`--ffb-detent-pct`) each time its rudder order crosses another `--ffb-detent-deg` (default 5°). The rudder order is the wheel position scaled so full helm lock is `--ffb-rudder-max-deg`. A wheel resting on a boundary doesn't chatter. |
| `off` | The base's force feedback is left untouched. |

In every mode except `off`, the connector also adds a damper (`--ffb-damper-pct`, 0 to disable) and switches off the base's own autocentre so the two don't fight.

### Behaviour presets

A preset sets the wheel's whole behaviour in one go: the mode, spring, damper, end stop, full lock, rudder at full lock and clicks. Choose one with `--ffb-preset NAME`, or from Crowsnest. It never sets the gain, which stays the overall strength under the cap. Anything set by hand afterwards makes the behaviour `custom`.

| Preset | Name | Behaviour |
|---|---|---|
| `ship` | Ship's wheel | Follow-up: pulled to where the rudder really is, well damped, over the base's full range. |
| `hydraulic` | Hydraulic helm | Follow-up with a heavy, slow wheel: a soft pull to the rudder and a lot of damping. |
| `car` | Car | A firm centring spring and light damping, 900 degrees lock to lock where the base allows. |
| `small-boat` | Small boat | Centring and quick: full rudder a quarter turn either side, an end stop past it. |
| `notched` | Notched | No spring: the wheel stays where it is left and clicks each 5 degrees of rudder. |
| `free` | Free | No spring and no clicks: the wheel turns freely up to the end stop at full lock. |

The full lock of a preset is pulled in to fit the base: `ship` and `hydraulic` use the base's whole range, and `car` 450° where the base has it.

### From Crowsnest (`--ffb-remote`)

With `--ffb-remote`, the connector serves keelson's `configurable/v1` on its source id (`moza_r21`). Crowsnest's "Force feedback test" panel uses it to switch force feedback on and off and to tune it while the connector runs.

- **Starts off.** Force feedback is off at start-up, whatever `--ffb` says, until someone switches it on.
- **The command line sets the cap.** A remote gain can't exceed `--ffb-max-gain-pct`, which can't exceed the 60 % ceiling. A higher request is refused with an error reply, not clipped.
- **Gain changes are gentle.** Every rise ramps from the current gain over 2 s; a drop applies at once. Switching on ramps from 0.
- **Off releases the wheel.** Mode `off` stops and erases every effect and sets the gain to 0, as on exit.
- **Without the flag** nothing is served, and nobody on the bus can touch the motor.

`get_config` returns these, and `set_config` takes any subset of the writable ones:

| Key | Writable | Values |
|---|---|---|
| `mode` | yes | `off`, `center`, `follow-up`, `detent` |
| `preset` | yes | a preset's name, applied at once; reads back as the matching preset or `custom` |
| `presets` | no | every preset's `name`, `label` and `description` |
| `max_gain_pct` | yes | 0 … `gain_cap_pct` |
| `spring_pct`, `damper_pct` | yes | 0 … 100 |
| `rudder_max_deg` | yes | 1 … 90 |
| `stale_s` | yes | 0.1 … 10 |
| `rudder_invert` | yes | `true` / `false` |
| `base_range_deg` | yes | 90 … 3600: the base's own steering angle, lock to lock |
| `wheel_lock_deg` | yes | 5 … half of `base_range_deg`: centre to full helm lock |
| `endstop_pct` | yes | 0 … 100: end-stop stiffness past full helm lock, 0 for none |
| `detent_deg` | yes | 0.5 … 30: degrees of rudder order between clicks in `detent` mode |
| `detent_pct` | yes | 0 … 100: click strength in `detent` mode |
| `gain_cap_pct`, `ceiling_pct`, `has_ff`, `backend` | no | the cap, 60, whether the base has force feedback, `hid` / `evdev` |
| `active`, `spring_source` | no | force feedback on the wheel; `center`, `rudder` or `off` |

Every change is republished on `configuration_json/moza_r21`.

```bash
.venv/bin/moza2keelson -e <vessel_id> -p r21-ultra-no-pedals --ffb-remote --ffb-max-gain-pct 15
```

### Helm range and end stop

By default, ±100 % on `wheel_position_pct` is the base's own end, which on a sim base can be one or more full turns. `--wheel-lock-deg` (or `wheel_lock_deg` from Crowsnest) puts full helm lock nearer, for example 90° either side of centre:

- **The wheel position** reaches ±100 % at that angle and stays there past it. The follow-up spring uses the same scale, so full rudder pulls the wheel to that angle.
- **An end stop** (`--ffb-endstop-pct`, default 100) pushes back past it, harder the further you turn: a stiff spring whose dead band is as wide as full helm lock. It is under the overall gain like every other effect, so its strength follows the gain slider. It works only while force feedback is on.
- **The base's own range** must be given as `--base-range-deg` (default 900), the steering angle set in Pit House or Boxflat, lock to lock. The USB reports don't say it. To check it: set a lock of 90° and turn the wheel exactly a quarter turn. `wheel_position_pct` should read ±100. If it reads ±50, the base range is twice what you entered.
- **Moving the lock remotely** while force feedback is on fades the force in again from 0 over 2 s, since the wheel may suddenly be past the new stop. The wheel position jumps to the new scale at once: don't change it while the helm is steering a vessel.

The rudder is assumed to report **positive to starboard**, the same as `wheel_position_pct`. If yours reports positive to port, pass `--ffb-rudder-invert`.

### Safety

The R21 Ultra peaks at 21 Nm, which is enough to injure a wrist.

- **Gain:** the overall gain defaults to `--ffb-max-gain-pct 30`. A **hard ceiling of 60 %** is enforced in code, and the CLI refuses any value above it.
- **Ramp-up:** at start-up, the gain rises from 0 over 2 s. The wheel may be anywhere when the connector starts, and this keeps it from being whipped to centre.
- **On exit,** whether from SIGTERM, Ctrl-C, an error or the base being unplugged, every effect is stopped and erased and the gain is set to 0. The wheel is left **limp**.
- **First power-on:** clamp the base down and keep your hands off the rim.

## Testing against a simulated rudder

`moza-rudder-sim` is a rudder the wheel steers. It subscribes to the wheel's `wheel_position_pct/moza_r21/wheel` and publishes `rudder_angle_deg/rudder_sim` every 100 ms, so the wheel's follow-up has a realistic rudder to follow. It mirrors the steering gear of Slipway's L1/L2 hulls ([`rudder_sim.py`](moza_connector/rudder_sim.py)):

- **Order:** helm % × the hull's travel; positive to starboard, no deadband.
- **Gear:** slews at the hull's rate, easing in only over the last `rate × 0.5 s`: rate-limited, not a lag. The travel is a hard stop.
- **Integration:** 100 ms ticks of ten RK4 sub-steps.
- **Dead-man:** no wheel sample for 1 s and the rudder returns to amidships. It never holds the last order.

| `--hull` | Travel | Rate |
|---|---|---|
| `road-ferry-78m` (default; Road Ferry Rivo) | ±35° | 3 °/s |
| `usv-6m` | ±30° | 30 °/s |
| `workboat-12m` | ±30° | 20 °/s |
| `drydock-default` | ±35° | 2.5 °/s |

`--max-rudder-deg` and `--rudder-rate-degps` override the hull.

```bash
.venv/bin/moza-rudder-sim --mode client --connect tcp/localhost:7447 -e <vessel_id> --hull road-ferry-78m
```

- **Match the wheel to the hull.** Set the wheel's "Rudder at full lock" (`rudder_max_deg`) to the hull's travel: 35 for the ferry, 30 for the others.
- **One rudder at a time.** Switch Crowsnest's "Rudder test signal" off, or the wheel follows two rudders. Or start the wheel with `--ffb-source rudder_sim`.
- **The real kernel can be the rudder too.** Map `wheel_position_pct/moza_r21/wheel` with `vehicle_control/v1 set_control_mapping` in an L1/L2 scenario in commanded mode. L0 has no steering gear.

## Requirements

- **Linux** (`--backend evdev`, the default there): kernel ≥ 6.15, or 6.12.24+, 6.13.12+ or 6.14.3+. These kernels ship `hid-universal-pidff`, the in-tree driver for Moza force feedback.
- **macOS** (`--backend hid`, the default there): the connector talks to the base directly over USB HID through hidapi, natively. Docker Desktop can't pass USB through. See [macOS](#macos).
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

## macOS

The base speaks the standard USB HID force-feedback class (PID), the same one Linux's driver uses. On macOS the connector does the driver's part itself ([`hid_device.py`](moza_connector/hid_device.py)):

- **Reports by usage.** Every report is found by its usage in the base's own report descriptor, never by a hard-coded id or offset. A base without a needed report runs inputs only and logs why.
- **Same mapping as Linux.** Axes get the evdev codes Linux would give them (X → `ABS_X`, Slider → `ABS_THROTTLE`, …), and buttons are numbered the same way, so one profile serves both.
- **Same force-feedback rules.** The gain ceiling, the ramp, the stale-rudder fallback and the shutdown are the shared `FfbController`'s. The HID device also refuses a gain above the ceiling itself.
- **Effect leases.** Linux erases a process's effects when it dies, even on `kill -9`; macOS doesn't. So every effect plays for **1 s** at a time and is re-started every 250 ms. If the connector stops for any reason, the wheel goes limp within a second.
- **One program at a time.** The connector opens the base exclusively. Quit Pit House first.

```bash
uv sync --locked
.venv/bin/moza2keelson -e <vessel_id> -p r21-ultra-no-pedals --ffb off                         # 1. inputs only
.venv/bin/moza2keelson -e <vessel_id> -p r21-ultra-no-pedals --ffb center --ffb-max-gain-pct 10 # 2. gentle centring
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
                    [-s SOURCE_ID] [-p PROFILE] [--backend {auto,evdev,hid}]
                    [-d DEVICE] [--wait-s WAIT_S] [--axis-max-hz AXIS_MAX_HZ]
                    [--axis-min-hz AXIS_MIN_HZ]
                    [--axis-deadband-pct AXIS_DEADBAND_PCT]
                    [--axis-center-snap-pct AXIS_CENTER_SNAP_PCT]
                    [--base-range-deg BASE_RANGE_DEG]
                    [--wheel-lock-deg WHEEL_LOCK_DEG]
                    [--ffb {off,center,follow-up,detent}]
                    [--ffb-preset {ship,hydraulic,car,small-boat,notched,free}]
                    [--ffb-max-gain-pct FFB_MAX_GAIN_PCT]
                    [--ffb-spring-pct FFB_SPRING_PCT]
                    [--ffb-damper-pct FFB_DAMPER_PCT]
                    [--ffb-entity FFB_ENTITY] [--ffb-source FFB_SOURCE]
                    [--ffb-rudder-max-deg FFB_RUDDER_MAX_DEG]
                    [--ffb-rudder-invert] [--ffb-stale-s FFB_STALE_S]
                    [--ffb-endstop-pct FFB_ENDSTOP_PCT]
                    [--ffb-detent-deg FFB_DETENT_DEG]
                    [--ffb-detent-pct FFB_DETENT_PCT] [--ffb-remote]
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
