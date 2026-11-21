"""moza-rudder-sim: a simulated rudder the wheel steers, on the bus.

Subscribes to the wheel's ``wheel_position_pct`` and publishes ``rudder_angle_deg`` every
100 ms, with Slipway's steering gear in between (``rudder_sim``). For testing the wheel's
follow-up without a vessel; it never publishes as ``slipway``, so it is never mistaken for
the real kernel.
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time

import keelson
import zenoh
from keelson.payloads.Primitives_pb2 import TimestampedFloat
from keelson.scaffolding import GracefulShutdown, add_common_arguments, create_zenoh_config, setup_logging
from keelson.scaffolding.liveliness import declare_liveliness
from keelson.scaffolding.qos_zenoh import declare_publisher

from . import rudder_sim as sim

log = logging.getLogger("moza-rudder-sim")


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="moza-rudder-sim",
        description="A simulated rudder with Slipway's steering gear, steered by the wheel's wheel_position_pct.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    add_common_arguments(p)
    p.add_argument("-r", "--realm", default="rise", help="Keelson base path (realm)")
    p.add_argument("-e", "--entity-id", required=True, help="The entity the rudder belongs to")
    p.add_argument("-s", "--source-id", default="rudder_sim", help="Source id of the published rudder_angle_deg")
    p.add_argument("--hull", choices=sorted(sim.HULLS), default="road-ferry-78m", help="Slipway hull to mirror")
    p.add_argument("--max-rudder-deg", type=float, help="Override the hull's rudder travel each side")
    p.add_argument("--rudder-rate-degps", type=float, help="Override the hull's steering-gear rate")
    p.add_argument("--input-entity", help="Entity of the helm (default: --entity-id)")
    p.add_argument("--input-source", default="moza_r21/wheel", help="Source id of the helm's wheel_position_pct")
    return p


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = parser()
    args = p.parse_args(argv)
    base = sim.HULLS[args.hull]
    max_deg = base.max_deg if args.max_rudder_deg is None else args.max_rudder_deg
    rate = base.rate_degps if args.rudder_rate_degps is None else args.rudder_rate_degps
    if not 0 < max_deg <= 90:
        p.error("--max-rudder-deg must be in (0, 90]")
    if rate <= 0:
        p.error("--rudder-rate-degps must be positive")
    args.hull_params = sim.Hull(max_deg=max_deg, rate_degps=rate)
    return args


def run(args: argparse.Namespace) -> int:
    hull: sim.Hull = args.hull_params
    conf = create_zenoh_config(mode=args.mode, connect=args.connect, listen=args.listen, zenoh_config=args.zenoh_config)
    lock = threading.Lock()
    latest: dict = {"pct": None, "at": None}

    with GracefulShutdown() as shutdown, zenoh.open(conf) as session:
        helm_key = keelson.construct_pubsub_key(
            args.realm, args.input_entity or args.entity_id, "wheel_position_pct", args.input_source
        )

        def on_helm(sample) -> None:
            try:
                _, _, payload = keelson.uncover(sample.payload.to_bytes())
                value = TimestampedFloat.FromString(payload).value
            except Exception as e:
                log.warning("Unreadable wheel_position_pct on %s: %s", sample.key_expr, e)
                return
            with lock:
                latest["pct"], latest["at"] = value, time.monotonic()

        sub = session.declare_subscriber(helm_key, on_helm)
        pub = declare_publisher(
            session, keelson.construct_pubsub_key(args.realm, args.entity_id, "rudder_angle_deg", args.source_id)
        )
        log.info(
            "Rudder %s: +-%.0f deg at %.1f deg/s, steered by %s; publishing rudder_angle_deg/%s at %.0f Hz",
            args.hull,
            hull.max_deg,
            hull.rate_degps,
            helm_key,
            args.source_id,
            1 / sim.TICK_S,
        )

        delta = 0.0
        steered = None  # whether the last tick had a live helm, for the log
        with declare_liveliness(
            session, args.realm, args.entity_id, args.source_id, pubsub_subjects=["rudder_angle_deg"]
        ):
            deadline = time.monotonic()
            while not shutdown.is_requested():
                deadline += sim.TICK_S
                now = time.monotonic()
                with lock:
                    pct, at = latest["pct"], latest["at"]
                order = sim.order_for(pct, at, now, hull)
                live = pct is not None and at is not None and 0.0 <= now - at <= sim.MAX_INPUT_AGE_S
                if live != steered:
                    if live:
                        log.info("Helm live: the rudder follows it")
                    elif steered is not None:
                        log.info("No helm for %.0f s: the rudder returns to amidships", sim.MAX_INPUT_AGE_S)
                    steered = live
                delta = sim.tick(delta, order, hull)
                msg = TimestampedFloat()
                msg.timestamp.FromNanoseconds(time.time_ns())
                msg.value = delta
                pub.put(keelson.enclose(msg.SerializeToString()))
                # An absolute deadline: the tick does not drift with the work done in it.
                shutdown.wait(max(0.0, deadline - time.monotonic()))
        sub.undeclare()
    return 0


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    setup_logging(level=args.log_level)
    zenoh.init_log_from_env_or("error")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
