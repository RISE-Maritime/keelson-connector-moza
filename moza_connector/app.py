"""moza2keelson: the CLI, the Zenoh session and the threads that tie the parts together."""

from __future__ import annotations

import argparse
import logging
import os
import sys
import threading
import time

import keelson
import zenoh
from keelson.payloads.Primitives_pb2 import TimestampedFloat
from keelson.scaffolding import GracefulShutdown, add_common_arguments, create_zenoh_config, setup_logging
from keelson.scaffolding.liveliness import declare_liveliness
from keelson.scaffolding.qos_zenoh import declare_publisher

from . import device as devices
from . import model, profile as profiles
from .device import DeviceGone, FakeDevice, MozaDevice
from .ffb import FfbController
from .inputs import InputPublisher, RateSettings
from .model import FfbMode, FfbSettings

log = logging.getLogger("moza2keelson")

#: The fake device exists for the tests. An operational deployment that somehow passed
#: --fake-device would publish a scripted helm onto a vessel's bus.
TESTING_ENV = "MOZA_CONNECTOR_TESTING"

EXIT_OK, EXIT_FAILED, EXIT_NO_DEVICE, EXIT_DEVICE_LOST = 0, 1, 2, 3
FFB_HZ = 20.0
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
    p.add_argument("-d", "--device", help="evdev node of the base (default: find it by the profile's vendor and name)")
    p.add_argument("--wait-s", type=float, default=2.0, help="Retry interval while the base is absent; 0 exits instead")
    p.add_argument("--fake-device", help=argparse.SUPPRESS)

    g = p.add_argument_group("axis publishing")
    g.add_argument("--axis-max-hz", type=float, default=50.0, help="Cap on change-driven publishes per axis")
    g.add_argument("--axis-min-hz", type=float, default=10.0, help="Backstop: every axis re-sent at least this often")
    g.add_argument("--axis-deadband-pct", type=float, default=2.0, help="A jump bigger than this bypasses the cap")
    g.add_argument("--axis-center-snap-pct", type=float, default=0.5, help="Within this of 0, publish exactly 0")

    f = p.add_argument_group("force feedback")
    f.add_argument("--ffb", choices=[m.value for m in FfbMode], default=FfbMode.FOLLOW_UP.value, help="Mode")
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
    return p


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = parser()
    args = p.parse_args(argv)
    if args.fake_device and os.environ.get(TESTING_ENV) != "1":
        p.error(f"--fake-device is for the tests only (set {TESTING_ENV}=1)")
    if args.axis_min_hz < 0 or args.axis_max_hz < 0:
        p.error("--axis-min-hz and --axis-max-hz must be >= 0")
    if args.axis_max_hz and args.axis_min_hz > args.axis_max_hz:
        p.error("--axis-min-hz must not exceed --axis-max-hz")
    if args.ffb_max_gain_pct > model.MAX_GAIN_CEILING_PCT:
        p.error(f"--ffb-max-gain-pct is capped at {model.MAX_GAIN_CEILING_PCT:.0f}")
    if args.ffb_rudder_max_deg <= 0:
        p.error("--ffb-rudder-max-deg must be positive")
    return args


def open_device(args, profile: profiles.Profile, shutdown: GracefulShutdown) -> MozaDevice | None:
    if args.fake_device:
        return FakeDevice.from_file(args.fake_device)
    announced = False
    while not shutdown.is_requested():
        try:
            return devices.find(profile, args.device)
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
        try:
            return _run_with_device(args, prof, dev, shutdown)
        finally:
            dev.close()


def _run_with_device(args, prof: profiles.Profile, dev: MozaDevice, shutdown: GracefulShutdown) -> int:
    conf = create_zenoh_config(mode=args.mode, connect=args.connect, listen=args.listen, zenoh_config=args.zenoh_config)
    publishers: dict[tuple[str, str], zenoh.Publisher] = {}

    with zenoh.open(conf) as session:

        def put(subject: str, source_id: str, payload: bytes) -> None:
            pub = publishers.get((subject, source_id))
            if pub is None:
                key = keelson.construct_pubsub_key(args.realm, args.entity_id, subject, source_id)
                pub = publishers[(subject, source_id)] = declare_publisher(session, key)
            pub.put(payload)

        inputs = InputPublisher(
            put,
            prof,
            dev.axes(),
            dev.keys(),
            args.source_id,
            RateSettings(args.axis_max_hz, args.axis_min_hz, args.axis_deadband_pct, args.axis_center_snap_pct),
        )
        steering = prof.steering
        ffb = FfbController(
            dev,
            FfbSettings(
                mode=FfbMode(args.ffb),
                max_gain_pct=args.ffb_max_gain_pct,
                spring_pct=args.ffb_spring_pct,
                damper_pct=args.ffb_damper_pct,
                stale_s=args.ffb_stale_s,
                rudder_max_deg=args.ffb_rudder_max_deg,
                rudder_invert=args.ffb_rudder_invert,
                wheel_invert=steering.invert,
            ),
        )

        lost = threading.Event()  # the base is gone: exit EXIT_DEVICE_LOST
        failed = threading.Event()  # a worker thread died: stop rather than run half a helm

        def on_thread_error(name: str, e: Exception) -> None:
            if isinstance(e, (DeviceGone, OSError)):
                log.error("%s: the base stopped answering (%s)", name, e)
                lost.set()
            else:
                log.error("%s failed: %r", name, e, exc_info=e)
                failed.set()

        subscriber = None
        if ffb.active and args.ffb == FfbMode.FOLLOW_UP.value:
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

            subscriber = session.declare_subscriber(rudder_key, on_rudder)
            log.info("Follow-up from %s", rudder_key)

        with declare_liveliness(session, args.realm, args.entity_id, args.source_id, pubsub_subjects=inputs.subjects):
            log.info("%s on %s: publishing %s", dev.name, dev.path, ", ".join(f"{s}/{src}" for s, src in inputs.sources))
            stop = threading.Event()
            threads = []
            try:
                now = time.monotonic()
                inputs.prime(now)
                ffb.start(now)
                if args.axis_min_hz > 0:
                    threads.append(_periodic(stop, args.axis_min_hz, inputs.backstop, "axis-backstop", on_thread_error))
                if ffb.active:
                    threads.append(_periodic(stop, FFB_HZ, ffb.tick, "ffb", on_thread_error))

                while not (shutdown.is_requested() or lost.is_set() or failed.is_set()):
                    try:
                        for ev in dev.events(POLL_S):
                            inputs.handle(ev.type, ev.code, ev.value, time.monotonic())
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
                if subscriber is not None:
                    subscriber.undeclare()
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
