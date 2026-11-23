"""moza2keelson: the CLI, the Zenoh session and the threads that tie the parts together."""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import keelson
import zenoh
from keelson.payloads.Alarm_pb2 import Alarm
from keelson.payloads.Primitives_pb2 import TimestampedFloat
from keelson.scaffolding import (
    GracefulShutdown,
    add_common_arguments,
    create_zenoh_config,
    make_configurable,
    setup_logging,
)
from keelson.scaffolding.liveliness import declare_liveliness
from keelson.scaffolding.qos_zenoh import declare_publisher

from . import base_serial, codes
from . import device as devices
from . import model, profile as profiles, store
from .base_serial import BaseSerialError, FakeBaseSerial
from .base_stop import BaseStop
from .alarm_lights import PREVIEW_TAIL_S, AlarmLights
from .device import DeviceGone, FakeDevice, MozaDevice
from .ffb import FfbController
from .inputs import InputPublisher, RateSettings
from .model import FfbMode, FfbSettings

log = logging.getLogger("moza2keelson")

#: The fake device exists for the tests. An operational deployment that somehow passed
#: --fake-device would publish a scripted helm onto a vessel's bus.
TESTING_ENV = "MOZA_CONNECTOR_TESTING"

EXIT_OK, EXIT_FAILED, EXIT_NO_DEVICE, EXIT_DEVICE_LOST = 0, 1, 2, 3
#: The haptics' settings, each from --ffb-<key with dashes>.
HAPTIC_KEYS = (
    "center_notch_pct", "center_notch_deg", "center_notch_damping_pct", "hardover_warn_deg", "hardover_pct", "speed_full_kn", "speed_min_pct",
    "alarm_caution_pct", "alarm_warning_pct", "alarm_emergency_pct", "lag_resistance_pct",
)
FFB_HZ = 20.0
#: How often the alarm lights follow the alarms (a write only when the colour changes).
LIGHTS_HZ = 8.0
POLL_S = 0.05


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="moza2keelson",
        description="A Moza wheelbase (R21 Ultra) as a keelson helm: wheel, pedals and buttons onto the bus, "
        "rudder follow-up as force feedback.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    add_common_arguments(p)
    p.add_argument("-r", "--realm", default="rise", help="Keelson base path (realm)")
    p.add_argument("-e", "--entity-id", required=True, help="The entity the helm belongs to")
    p.add_argument("-s", "--source-id", default="moza_r21", help="Source id base; each input is <source-id>/<input>")
    p.add_argument("-p", "--profile", default="r21-ultra", help="Built-in profile name or path to a profile YAML")
    p.add_argument(
        "--backend",
        choices=devices.BACKENDS,
        default="auto",
        help="How to reach the base: evdev (Linux kernel driver) or hid (hidapi, macOS); auto picks by platform",
    )
    p.add_argument("-d", "--device", help="evdev node of the base (default: find it by the profile's vendor and name)")
    p.add_argument("--wait-s", type=float, default=2.0, help="Retry interval while the base is absent; 0 exits instead")
    p.add_argument("--fake-device", help=argparse.SUPPRESS)

    g = p.add_argument_group("axis publishing")
    g.add_argument("--axis-max-hz", type=float, default=50.0, help="Cap on change-driven publishes per axis")
    g.add_argument("--axis-min-hz", type=float, default=10.0, help="Backstop: every axis re-sent at least this often")
    g.add_argument("--axis-deadband-pct", type=float, default=2.0, help="A jump bigger than this bypasses the cap")
    g.add_argument("--axis-center-snap-pct", type=float, default=0.5, help="Within this of 0, publish exactly 0")
    g.add_argument(
        "--base-range-deg",
        type=float,
        default=900.0,
        help="The base's steering angle, lock to lock, as set in Pit House or Boxflat (the USB reports do not say)",
    )
    g.add_argument(
        "--center-deadband-deg",
        type=float,
        default=0.0,
        help="Wheel degrees either side of centre that order exactly 0 (a helm deadband, rescaled outside it)",
    )
    g.add_argument(
        "--wheel-lock-deg",
        type=float,
        help="Wheel rotation from centre to full helm lock (+-100 %%); past it force feedback adds an end stop. "
        "Default: half of --base-range-deg, the base's own end",
    )

    f = p.add_argument_group("force feedback")
    f.add_argument(
        "--ffb",
        choices=[m.value for m in FfbMode],
        default=FfbMode.FOLLOW_UP.value,
        help="Mode of the command-line profile; 'off' keeps force feedback off, any other switches it on "
        "(not with --ffb-remote, which always starts off)",
    )
    f.add_argument(
        "--ffb-profile",
        "--ffb-preset",
        dest="ffb_profile",
        help="The profile to start in: a built-in (%s), command-line, or one saved in the settings file. "
        "Default: the settings file's default profile, else command-line (the --ffb-* flags)" % ", ".join(model.PRESETS),
    )
    f.add_argument(
        "--ffb-max-gain-pct",
        type=float,
        default=30.0,
        help=f"Overall force-feedback gain (hard ceiling {model.MAX_GAIN_CEILING_PCT:.0f})",
    )
    f.add_argument("--ffb-spring-pct", type=float, default=60.0, help="Spring stiffness, under the gain")
    f.add_argument("--ffb-damper-pct", type=float, default=20.0, help="Damper, under the gain; 0 for none")
    f.add_argument("--ffb-entity", help="Entity whose rudder_angle_deg drives follow-up (default: --entity-id)")
    f.add_argument("--ffb-source", default="**", help="Source id (glob) of that rudder_angle_deg")
    f.add_argument("--ffb-rudder-max-deg", type=float, default=35.0, help="Rudder angle at full wheel lock")
    f.add_argument(
        "--ffb-rudder-invert",
        action="store_true",
        help="The rudder reports positive to port (default: positive is starboard, as wheel_position_pct)",
    )
    f.add_argument("--ffb-stale-s", type=float, default=1.0, help="Older rudder data falls back to plain centring")
    f.add_argument(
        "--ffb-endstop-pct", type=float, default=100.0, help="End-stop stiffness past --wheel-lock-deg, under the gain"
    )
    f.add_argument("--ffb-detent-deg", type=float, default=5.0, help="detent mode: a click each this much rudder order")
    f.add_argument("--ffb-detent-pct", type=float, default=60.0, help="detent mode: click strength, under the gain")
    f.add_argument(
        "--ffb-spring-boost",
        type=float,
        default=1.0,
        help="Centring spring this many times steeper near centre (1..3); the pull far from centre is unchanged",
    )
    f.add_argument(
        "--ffb-center-mark-pct",
        type=float,
        default=80.0,
        help="A distinct pulse each time the wheel reaches centre, in every mode; 0 for none",
    )
    h = p.add_argument_group("haptics (each off by default)")
    h.add_argument(
        "--ffb-center-notch-pct",
        type=float,
        default=0.0,
        help="Centre and detent modes: near centre the wheel is pulled into it, a V it falls into; its strength",
    )
    h.add_argument("--ffb-center-notch-deg", type=float, default=5.0, help="How far either side of centre the notch reaches (wheel degrees)")
    h.add_argument(
        "--ffb-center-notch-damping-pct",
        type=float,
        default=50.0,
        help="Inside the notch, its pull also opposes the wheel's speed, so it settles instead of swinging",
    )
    h.add_argument("--ffb-hardover-warn-deg", type=float, default=0.0, help="A pulse at this rudder order, either way")
    h.add_argument("--ffb-hardover-pct", type=float, default=80.0, help="The hard-over pulse's strength")
    h.add_argument("--ffb-speed-full-kn", type=float, default=0.0, help="Full wheel weight from this speed over ground")
    h.add_argument("--ffb-speed-min-pct", type=float, default=40.0, help="The wheel's weight alongside, of full")
    h.add_argument("--ffb-alarm-caution-pct", type=float, default=0.0, help="One rumble for a new caution alarm")
    h.add_argument("--ffb-alarm-warning-pct", type=float, default=0.0, help="Three rumbles for a new warning alarm")
    h.add_argument(
        "--ffb-alarm-emergency-pct", type=float, default=0.0, help="A rumble every 2 s while an emergency alarm lasts"
    )
    h.add_argument("--ffb-lag-resistance-pct", type=float, default=0.0, help="Extra damping while the rudder trails")
    f.add_argument(
        "--settings-file",
        default="auto",
        help="Where the settings and saved behaviours are kept between runs; 'none' keeps nothing. "
        "auto: the platform's app-settings folder (none with --fake-device)",
    )
    f.add_argument(
        "--base-serial",
        default="none",
        help="The base's serial port, for the hard end stop (the base's own steering angle set to the helm's "
        "lock, put back on exit): 'auto' finds a Moza base by its USB vendor id, or a port path; 'none' never "
        "touches it. Pit House must be closed",
    )
    f.add_argument(
        "--ffb-remote",
        action="store_true",
        help="Let configurable/v1 (Crowsnest) switch force feedback and tune it, never above --ffb-max-gain-pct. "
        "Force feedback then starts off, whatever --ffb says",
    )
    return p


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = parser()
    args = p.parse_args(argv)
    if args.fake_device and os.environ.get(TESTING_ENV) != "1":
        p.error(f"--fake-device is for the tests only (set {TESTING_ENV}=1)")
    if args.base_serial == "fake" and os.environ.get(TESTING_ENV) != "1":
        p.error(f"--base-serial fake is for the tests only (set {TESTING_ENV}=1)")
    if args.device and devices.resolve_backend(args.backend) != "evdev":
        p.error("--device names an evdev node: it needs --backend evdev")
    if args.axis_min_hz < 0 or args.axis_max_hz < 0:
        p.error("--axis-min-hz and --axis-max-hz must be >= 0")
    if args.axis_max_hz and args.axis_min_hz > args.axis_max_hz:
        p.error("--axis-min-hz must not exceed --axis-max-hz")
    if args.ffb_max_gain_pct > model.MAX_GAIN_CEILING_PCT:
        p.error(f"--ffb-max-gain-pct is capped at {model.MAX_GAIN_CEILING_PCT:.0f}")
    if args.ffb_rudder_max_deg <= 0:
        p.error("--ffb-rudder-max-deg must be positive")
    lo, hi = model.CONFIG_BOUNDS["base_range_deg"]
    if not lo <= args.base_range_deg <= hi:
        p.error(f"--base-range-deg must be {lo:g}..{hi:g}")
    if args.wheel_lock_deg is None:
        args.wheel_lock_deg = args.base_range_deg / 2.0
    if not model.CONFIG_BOUNDS["wheel_lock_deg"][0] <= args.wheel_lock_deg <= args.base_range_deg / 2.0:
        p.error("--wheel-lock-deg must be at least 5 and at most half of --base-range-deg (the base's own end)")
    if not 0 <= args.ffb_endstop_pct <= 100:
        p.error("--ffb-endstop-pct must be 0..100")
    lo, hi = model.CONFIG_BOUNDS["detent_deg"]
    if not lo <= args.ffb_detent_deg <= hi:
        p.error(f"--ffb-detent-deg must be {lo:g}..{hi:g}")
    if not 0 <= args.ffb_detent_pct <= 100:
        p.error("--ffb-detent-pct must be 0..100")
    if not 0 <= args.ffb_center_mark_pct <= 100:
        p.error("--ffb-center-mark-pct must be 0..100")
    try:
        model.apply_config(
            model.FfbSettings(rudder_max_deg=args.ffb_rudder_max_deg),
            {k: getattr(args, f"ffb_{k}") for k in HAPTIC_KEYS},
            gain_cap_pct=model.MAX_GAIN_CEILING_PCT,
            read_only={},
        )
    except ValueError as e:
        p.error(f"haptics: {e}")
    if not 1 <= args.ffb_spring_boost <= 3:
        p.error("--ffb-spring-boost must be 1..3")
    if not 0 <= args.center_deadband_deg <= model.CONFIG_BOUNDS["center_deadband_deg"][1] or (
        args.center_deadband_deg >= args.wheel_lock_deg / 2.0
    ):
        p.error("--center-deadband-deg must be 0..30 and under half of --wheel-lock-deg")
    return args


def open_base_serial(args):
    """The base's serial port for ``--base-serial``, or None. Raises BaseSerialError or OSError."""
    if args.base_serial == "none":
        return None
    if args.base_serial == "fake":
        return FakeBaseSerial(int(os.environ.get("MOZA_FAKE_BASE_ANGLE", "450")), os.environ.get("MOZA_FAKE_BASE_FILE"))
    path = base_serial.find_base_serial() if args.base_serial == "auto" else args.base_serial
    if path is None:
        raise BaseSerialError("no Moza base serial port found")
    return base_serial.BaseSerial(path)


def open_device(args, profile: profiles.Profile, shutdown: GracefulShutdown) -> MozaDevice | None:
    if args.fake_device:
        return FakeDevice.from_file(args.fake_device)
    announced = False
    while not shutdown.is_requested():
        try:
            return devices.find(profile, args.device, args.backend)
        except (FileNotFoundError, PermissionError) as e:
            if args.wait_s <= 0:
                log.error("%s", e)
                return None
            if not announced:
                log.warning("%s; retrying every %.1fs", e, args.wait_s)
                announced = True
            shutdown.wait(args.wait_s)
    return None


def _periodic(stop: threading.Event, hz: float, fn, name: str, on_error) -> threading.Thread:
    def loop():
        period = 1.0 / hz
        while not stop.wait(period):
            try:
                fn(time.monotonic())
            except Exception as e:
                on_error(name, e)
                return

    t = threading.Thread(target=loop, name=name, daemon=True)
    t.start()
    return t


def run(args: argparse.Namespace) -> int:
    prof = profiles.load(args.profile)
    with GracefulShutdown() as shutdown:
        dev = open_device(args, prof, shutdown)
        if dev is None:
            return EXIT_OK if shutdown.is_requested() else EXIT_NO_DEVICE
        stops: list[BaseStop] = []
        try:
            return _run_with_device(args, prof, dev, shutdown, stops)
        finally:
            # After force feedback has let go (in _run_with_device): put the base's own
            # steering angle back. Never raises.
            for stop in reversed(stops):  # the alarm lights, then the hard stop (which closes the port)
                stop.restore()
            dev.close()


def _run_with_device(
    args, prof: profiles.Profile, dev: MozaDevice, shutdown: GracefulShutdown, stops: list[BaseStop]
) -> int:
    conf = create_zenoh_config(mode=args.mode, connect=args.connect, listen=args.listen, zenoh_config=args.zenoh_config)
    publishers: dict[tuple[str, str], zenoh.Publisher] = {}

    with zenoh.open(conf) as session:

        def put(subject: str, source_id: str, payload: bytes) -> None:
            pub = publishers.get((subject, source_id))
            if pub is None:
                key = keelson.construct_pubsub_key(args.realm, args.entity_id, subject, source_id)
                pub = publishers[(subject, source_id)] = declare_publisher(session, key)
            pub.put(payload)

        steering = prof.steering
        if args.ffb_remote and args.ffb != FfbMode.OFF.value:
            log.info("--ffb-remote: force feedback starts off until it is switched on remotely")
        # The command-line profile: the --ffb-* flags, its mode never off (off is the switch).
        feel = FfbMode.FOLLOW_UP if args.ffb == FfbMode.OFF.value else FfbMode(args.ffb)
        settings = FfbSettings(
            mode=feel,
            max_gain_pct=args.ffb_max_gain_pct,
            spring_pct=args.ffb_spring_pct,
            damper_pct=args.ffb_damper_pct,
            stale_s=args.ffb_stale_s,
            rudder_max_deg=args.ffb_rudder_max_deg,
            rudder_invert=args.ffb_rudder_invert,
            wheel_invert=steering.invert,
            base_range_deg=args.base_range_deg,
            wheel_lock_deg=args.wheel_lock_deg,
            endstop_pct=args.ffb_endstop_pct,
            detent_deg=args.ffb_detent_deg,
            detent_pct=args.ffb_detent_pct,
            center_mark_pct=args.ffb_center_mark_pct,
            center_deadband_deg=args.center_deadband_deg,
            spring_boost=args.ffb_spring_boost,
            **{k: getattr(args, f"ffb_{k}") for k in HAPTIC_KEYS},
        )
        settings_path = None
        if args.settings_file == "auto" and not args.fake_device:
            settings_path = store.default_path(args.realm, args.entity_id, args.source_id)
        elif args.settings_file not in ("auto", "none", ""):
            settings_path = Path(args.settings_file).expanduser()
        helm = model.Helm(
            settings,
            command_line=model.behaviour_of(settings),
            enabled=not args.ffb_remote and args.ffb != FfbMode.OFF.value,
        )
        doc = None
        if settings_path is not None:
            doc = store.read(settings_path)
            if doc is not None:
                helm, notes = model.load_state(doc, helm, gain_cap_pct=args.ffb_max_gain_pct)
                for note in notes:
                    log.warning("Settings file %s: %s", settings_path, note)
                log.info("Settings from %s (%d profiles of yours)", settings_path, len(helm.user))
        # The hard end stop: the base's own steering angle, over its serial port.
        base_restore: dict | None = None
        lights_restore: dict | None = None

        def save() -> None:
            if settings_path is not None:
                store.write(settings_path, {**model.saved_state(helm), "base_restore": base_restore, "lights_restore": lights_restore})

        def persist_restore(restore: dict | None) -> None:
            nonlocal base_restore
            base_restore = restore
            save()

        def persist_lights(restore: dict | None) -> None:
            nonlocal lights_restore
            lights_restore = restore
            save()

        base_stop = None
        try:
            port = open_base_serial(args)
            if port is not None:
                base_stop = BaseStop(port, persist_restore)
                stops.append(base_stop)
                own, now_deg = base_stop.start((doc or {}).get("base_restore"))
                helm = replace(helm, settings=replace(helm.settings, base_max_deg=own, base_range_deg=now_deg))
        except (BaseSerialError, OSError, ImportError) as e:
            log.warning("--base-serial %s: %s; no hard end stop", args.base_serial, e)
            base_stop = None
        # The rim's LEDs and the base's light bar, in an alarm's colour: on the same port, each
        # when it answers.
        lights = None
        if base_stop is not None:
            lights = AlarmLights(port, persist_lights)
            saved = (doc or {}).get("lights_restore") or (doc or {}).get("rim_restore")
            if lights.start(saved):
                stops.append(lights)
                log.info("Alarm lights over %s: %s", args.base_serial, ", ".join(lights.targets))
            else:
                lights = None
        lit = set(lights.targets) if lights is not None else set()
        if "rim" not in lit and helm.settings.alarm_leds:
            helm = replace(helm, settings=replace(helm.settings, alarm_leds=False))
        if "base" not in lit and helm.settings.alarm_base_light:
            helm = replace(helm, settings=replace(helm.settings, alarm_base_light=False))
        if base_stop is None and helm.settings.hard_stop:
            helm = replace(helm, settings=replace(helm.settings, hard_stop=False))
        start_profile = args.ffb_profile or helm.default_profile
        try:
            helm = model.load_profile(helm, start_profile)
        except ValueError as e:
            if args.ffb_profile:
                log.error("--ffb-profile: %s", e)
                return EXIT_FAILED
            log.warning("Default profile %s: %s; starting in %s", start_profile, e, model.COMMAND_LINE_LABEL)
            helm = model.load_profile(helm, model.COMMAND_LINE)
        log.info("Profile: %s", model.profile_label(helm, helm.profile))
        settings = model.effective(helm)
        inputs = InputPublisher(
            put,
            prof,
            dev.axes(),
            dev.keys(),
            args.source_id,
            RateSettings(args.axis_max_hz, args.axis_min_hz, args.axis_deadband_pct, args.axis_center_snap_pct),
            lock_fraction=settings.lock_fraction,
            deadband_pct=settings.deadband_pct,
        )
        log.info("Full helm lock at %.0f deg of the base's %.0f lock to lock", args.wheel_lock_deg, args.base_range_deg)
        ffb = FfbController(dev, settings)
        helm_lock = threading.Lock()  # set_config (a Zenoh thread) and the base's angle (here)

        def sync_base_stop() -> None:
            """The hard stop: the base's angle follows the helm's lock (when the wheel allows);
            once it has moved, the axis spans a new range, so the steering and force feedback
            are rescaled (a lock change ramps the force in again)."""
            nonlocal helm
            axis = inputs.steering_axis_pct
            wheel_deg = None if axis is None else axis / 100.0 * base_stop.current_deg / 2.0
            new_deg = base_stop.sync(helm.settings, wheel_deg)
            if new_deg is None:
                return
            with helm_lock:
                helm = replace(helm, settings=replace(helm.settings, base_range_deg=new_deg))
                now = time.monotonic()
                ffb.update(model.effective(helm), now)
                inputs.set_steering(helm.settings.lock_fraction, helm.settings.deadband_pct, now)

        lost = threading.Event()  # the base is gone: exit EXIT_DEVICE_LOST
        failed = threading.Event()  # a worker thread died: stop rather than run half a helm

        def on_thread_error(name: str, e: Exception) -> None:
            if isinstance(e, (DeviceGone, OSError)):
                log.error("%s: the base stopped answering (%s)", name, e)
                lost.set()
            else:
                log.error("%s failed: %r", name, e, exc_info=e)
                failed.set()

        remote = args.ffb_remote and dev.has_ff()
        if args.ffb_remote and not remote:
            log.warning("--ffb-remote: %s has no force feedback; serving its (off) settings only", dev.name)

        subscribers = []
        if remote or ffb.active:
            rudder_key = keelson.construct_pubsub_key(
                args.realm, args.ffb_entity or args.entity_id, "rudder_angle_deg", args.ffb_source
            )

            def on_rudder(sample) -> None:
                try:
                    _, _, payload = keelson.uncover(sample.payload.to_bytes())
                    msg = TimestampedFloat.FromString(payload)
                except Exception as e:
                    log.warning("Unreadable rudder_angle_deg on %s: %s", sample.key_expr, e)
                    return
                ffb.on_rudder(msg.value, time.monotonic())

            subscribers.append(session.declare_subscriber(rudder_key, on_rudder))
            log.info("Follow-up from %s", rudder_key)

            # The speed-sensitive weight and the alarm rumble read the vessel the wheel steers.
            vessel = args.ffb_entity or args.entity_id

            def on_speed(sample) -> None:
                try:
                    _, _, payload = keelson.uncover(sample.payload.to_bytes())
                    ffb.on_speed(TimestampedFloat.FromString(payload).value, time.monotonic())
                except Exception as e:
                    log.warning("Unreadable speed_over_ground_knots on %s: %s", sample.key_expr, e)

            def on_alarm(sample) -> None:
                try:
                    _, _, payload = keelson.uncover(sample.payload.to_bytes())
                    alarm = Alarm.FromString(payload)
                except Exception as e:
                    log.warning("Unreadable alarm on %s: %s", sample.key_expr, e)
                    return
                ident = alarm.identifier or str(sample.key_expr)
                now = time.monotonic()
                expires_at = None
                if alarm.HasField("expiration_time_utc") and alarm.expiration_time_utc.seconds > 0:
                    expires_at = now + (alarm.expiration_time_utc.ToNanoseconds() - time.time_ns()) / 1e9
                ffb.on_alarm(ident, alarm.priority, alarm.acknowledged, now, expires_at, alarm.category)

            for subject, handler in (("speed_over_ground_knots", on_speed), ("alarm", on_alarm)):
                key = keelson.construct_pubsub_key(args.realm, vessel, subject, "**")
                subscribers.append(session.declare_subscriber(key, handler))

        subjects = list(inputs.subjects)
        if args.ffb_remote:
            def read_only() -> dict:
                return {
                    "gain_cap_pct": args.ffb_max_gain_pct,
                    "ceiling_pct": model.MAX_GAIN_CEILING_PCT,
                    "has_ff": dev.has_ff(),
                    "active": ffb.running,
                    "spring_source": ffb.spring_source,
                    "backend": "fake" if args.fake_device else devices.resolve_backend(args.backend),
                    "base_serial": "off" if base_stop is None else base_stop.status,
                    "base_angle_deg": None if base_stop is None else base_stop.current_deg,
                    "base_max_deg": helm.settings.base_max_deg,
                    # With the base's serial port, its angle is read, not set by hand; without
                    # it there is no hard stop to switch on.
                    **({"base_range_deg": helm.settings.base_range_deg} if base_stop is not None else {"hard_stop": False}),
                    "rim_leds": lights.status["rim"] if lights is not None else "off",
                    "base_light": lights.status["base"] if lights is not None else "off",
                    "alarm_active": model.alarm_kind(ffb.top_alarm(time.monotonic()) or 0),
                    **({} if "rim" in lit else {"alarm_leds": False}),
                    **({} if "base" in lit else {"alarm_base_light": False}),
                }

            def get_config() -> dict:
                return {**model.helm_config(helm), **read_only()}

            def set_config(patch: dict) -> None:
                # Read-only keys are compared, not trusted. Those that move on their own (the
                # wheel's state, the base's angle) are ignored, so a stale round trip works.
                nonlocal helm
                if not isinstance(patch, dict):
                    raise ValueError("set_config expects a JSON object")
                moving = ("active", "spring_source", "base_serial", "base_angle_deg", "base_range_deg", "rim_leds", "base_light", "alarm_active")
                current = read_only()
                fixed = {k: v for k, v in current.items() if k not in moving}
                if base_stop is not None and "base_range_deg" in patch and patch["base_range_deg"] != current["base_range_deg"]:
                    raise ValueError("base_range_deg is read from the base (--base-serial)")
                patch = {k: v for k, v in patch.items() if k not in moving or (k == "base_range_deg" and base_stop is None)}
                test = patch.pop("test", None)
                if test is not None:
                    # Trying a haptic (or showing an alarm colour) changes no setting: nothing saved.
                    if patch:
                        raise ValueError("'test' goes alone, not with settings")
                    test = str(test)
                    if test.startswith("alarm-"):
                        # An alarm as it would come, without one on the bus: the lights in its
                        # colour, and its pattern once while force feedback is on.
                        kind = test[len("alarm-"):]
                        if kind not in model.ALARM_KINDS:
                            raise ValueError(f"no alarm kind {kind!r}")
                        now = time.monotonic()
                        if lights is not None:
                            schedule, _ = model.alarm_schedule(helm.settings, kind)
                            lights.preview(kind, now, model.pattern_length_ms(schedule) / 1000.0 + PREVIEW_TAIL_S)
                        try:
                            ffb.test_pulse(kind, now)
                        except ValueError as e:
                            if lights is None:
                                raise ValueError(f"nothing to show or feel: {e}") from None
                            log.info("Try %s alarm: colour only (%s)", kind, e)
                        return
                    ffb.test_pulse(test, time.monotonic())
                    return
                with helm_lock:
                    new = model.apply_helm(helm, patch, gain_cap_pct=args.ffb_max_gain_pct, read_only=fixed)
                    if new.enabled and not dev.has_ff():
                        raise ValueError(f"{dev.name} has no force feedback")
                    log.info("set_config %s", patch)
                    if new.profile != helm.profile:
                        log.info("Profile: %s", model.profile_label(new, new.profile))
                    ffb.update(model.effective(new), time.monotonic())
                    inputs.set_steering(new.settings.lock_fraction, new.settings.deadband_pct, time.monotonic())
                    helm = new
                    save()

            make_configurable(session, args.realm, args.entity_id, args.source_id, get_config, set_config)
            subjects.append("configuration_json")
            log.info("Force feedback settable over configurable/v1 (cap %.0f%%)", args.ffb_max_gain_pct)

        with declare_liveliness(session, args.realm, args.entity_id, args.source_id, pubsub_subjects=sorted(subjects)):
            log.info("%s on %s: publishing %s", dev.name, dev.path, ", ".join(f"{s}/{src}" for s, src in inputs.sources))
            stop = threading.Event()
            threads = []
            try:
                now = time.monotonic()
                inputs.prime(now)
                ffb.start(now)
                if args.axis_min_hz > 0:
                    threads.append(_periodic(stop, args.axis_min_hz, inputs.backstop, "axis-backstop", on_thread_error))
                if ffb.active or remote:
                    threads.append(_periodic(stop, FFB_HZ, ffb.tick, "ffb", on_thread_error))
                if lights is not None:
                    threads.append(
                        _periodic(stop, LIGHTS_HZ, lambda now: lights.sync(helm.settings, ffb.top_alarm(now), now), "lights", on_thread_error)
                    )

                def notch() -> None:
                    axis = inputs.steering_axis_pct
                    if axis is not None:
                        ffb.on_axis(axis, time.monotonic())  # the centre notch

                last_wheel = None
                while not (shutdown.is_requested() or lost.is_set() or failed.is_set()):
                    try:
                        for ev in dev.events(POLL_S):
                            inputs.handle(ev.type, ev.code, ev.value, time.monotonic())
                            if ev.type == codes.EV_SYN:
                                notch()  # with every report, so it keeps up while the wheel turns
                        notch()  # and on every pass, so it is renewed while the wheel is still
                        wheel = inputs.steering_value
                        if wheel is not None and wheel != last_wheel:
                            last_wheel = wheel
                            ffb.on_wheel(wheel, time.monotonic())  # detent clicks
                        if base_stop is not None:
                            sync_base_stop()
                    except DeviceGone as e:
                        log.error("The base is gone (%s): stopping, so the helm goes silent", e)
                        lost.set()
                        break
                    inputs.flush(time.monotonic())
            finally:
                # Silence first (no more backstop), then let go of the wheel.
                stop.set()
                for t in threads:
                    t.join(timeout=2.0)
                ffb.shutdown()
                for sub in subscribers:
                    sub.undeclare()
    if lost.is_set():
        return EXIT_DEVICE_LOST
    return EXIT_FAILED if failed.is_set() else EXIT_OK


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    setup_logging(level=args.log_level)
    zenoh.init_log_from_env_or("error")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
