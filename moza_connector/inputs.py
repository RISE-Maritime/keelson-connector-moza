"""Device events to keelson messages: the wheel, the pedals and the buttons.

``InputPublisher`` never touches Zenoh itself. It calls ``put(subject, source_id,
envelope_bytes)``, so the tests hand it a list and the app hands it real publishers.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable

import keelson
from keelson.payloads.Primitives_pb2 import TimestampedFloat, TimestampedInt

from . import codes, model
from .device import AxisRange
from .profile import BUTTON_SUBJECT, AxisMap, Profile

log = logging.getLogger(__name__)

Put = Callable[[str, str, bytes], None]


@dataclass(frozen=True)
class RateSettings:
    max_hz: float = 50.0
    min_hz: float = 10.0
    deadband_pct: float = 2.0
    center_snap_pct: float = 0.5


def enclose_float(value: float, ts_ns: int) -> bytes:
    p = TimestampedFloat()
    p.timestamp.FromNanoseconds(ts_ns)
    p.value = value
    return keelson.enclose(p.SerializeToString())


def enclose_int(value: int, ts_ns: int) -> bytes:
    p = TimestampedInt()
    p.timestamp.FromNanoseconds(ts_ns)
    p.value = value
    return keelson.enclose(p.SerializeToString())


@dataclass
class _Axis:
    map: AxisMap
    range: AxisRange
    source_id: str
    raw: int | None = None
    value: float | None = None
    published_value: float | None = None
    published_at: float | None = None


class InputPublisher:
    def __init__(
        self,
        put: Put,
        profile: Profile,
        axes: dict[int, AxisRange],
        keys: list[int],
        source_base: str,
        rate: RateSettings,
        wall_ns: Callable[[], int] = time.time_ns,
        lock_fraction: float = 1.0,
        deadband_pct: float = 0.0,
    ):
        self._put = put
        self._lock_fraction = lock_fraction
        self._deadband_pct = deadband_pct
        self._rate = rate
        self._wall_ns = wall_ns
        self._lock = threading.Lock()
        self._axes: dict[int, _Axis] = {}
        for m in profile.axes:
            if m.code not in axes:
                if m.steering:
                    raise ValueError(f"the device has no {m.code_name}, the profile's steering axis")
                log.warning("The device has no %s: %s/%s is not published", m.code_name, m.subject, m.input)
                continue
            self._axes[m.code] = _Axis(map=m, range=axes[m.code], source_id=f"{source_base}/{m.input}")
        self._buttons = {code: f"{source_base}/{profile.button_name(i)}" for i, code in enumerate(sorted(keys))}

    @property
    def subjects(self) -> list[str]:
        subjects = {a.map.subject for a in self._axes.values()}
        if self._buttons:
            subjects.add(BUTTON_SUBJECT)
        return sorted(subjects)

    @property
    def sources(self) -> list[tuple[str, str]]:
        """Every (subject, source_id) this publisher can put on."""
        out = [(a.map.subject, a.source_id) for a in self._axes.values()]
        out += [(BUTTON_SUBJECT, s) for s in self._buttons.values()]
        return sorted(out)

    @property
    def steering_value(self) -> float | None:
        """The wheel's latest position as published (percent of full helm lock)."""
        with self._lock:
            for axis in self._axes.values():
                if axis.map.steering:
                    return axis.value
        return None

    @property
    def steering_axis_pct(self) -> float | None:
        """The steering axis itself, -100..100 of the base's own half range, before the helm's
        lock: where the wheel really is, also past full lock (where the published value stays
        at 100)."""
        with self._lock:
            for axis in self._axes.values():
                if axis.map.steering and axis.raw is not None:
                    m, r = axis.map, axis.range
                    return model.normalize(axis.raw, r.lo, r.hi, bipolar=m.bipolar, invert=m.invert)
        return None

    def _scaled(self, axis: _Axis, raw: int) -> float:
        m, r = axis.map, axis.range
        axis.raw = raw
        v = model.normalize(raw, r.lo, r.hi, bipolar=m.bipolar, invert=m.invert)
        if m.steering:
            v = model.helm_deadband(model.steering_pct(v, self._lock_fraction), self._deadband_pct)
        return model.snap(v, self._rate.center_snap_pct)

    def set_steering(self, fraction: float, deadband_pct: float, now: float) -> None:
        """Full helm lock or the helm deadband moved (``FfbSettings.lock_fraction`` /
        ``deadband_pct``): re-scale the wheel and publish it at once, so the bus never carries
        the old scale for a backstop period."""
        with self._lock:
            if (fraction, deadband_pct) == (self._lock_fraction, self._deadband_pct):
                return
            self._lock_fraction, self._deadband_pct = fraction, deadband_pct
            for axis in self._axes.values():
                if axis.map.steering and axis.raw is not None:
                    axis.value = self._scaled(axis, axis.raw)
                    self._publish_axis(axis, now)

    def prime(self, now: float) -> None:
        """Publish every axis' current position (from absinfo) once, at start-up."""
        with self._lock:
            for axis in self._axes.values():
                axis.value = self._scaled(axis, axis.range.value)
                self._publish_axis(axis, now)

    def handle(self, etype: int, code: int, value: int, now: float) -> None:
        if etype == codes.EV_ABS:
            with self._lock:
                axis = self._axes.get(code)
                if axis is None:
                    return
                axis.value = self._scaled(axis, value)
                if model.should_publish(
                    now,
                    axis.value,
                    axis.published_value,
                    axis.published_at,
                    max_hz=self._rate.max_hz,
                    deadband_pct=self._rate.deadband_pct,
                ):
                    self._publish_axis(axis, now)
        elif etype == codes.EV_KEY:
            source = self._buttons.get(code)
            if source is not None and value in (0, 1):  # 2 is autorepeat: not a state change
                self._put(BUTTON_SUBJECT, source, enclose_int(value, self._wall_ns()))

    def _publish_axis(self, axis: _Axis, now: float) -> None:
        self._put(axis.map.subject, axis.source_id, enclose_float(axis.value, self._wall_ns()))
        axis.published_value, axis.published_at = axis.value, now

    def flush(self, now: float) -> None:
        """Send any change the rate cap held back, once its interval has passed."""
        with self._lock:
            for axis in self._axes.values():
                if axis.value is not None and axis.value != axis.published_value:
                    if model.should_publish(
                        now,
                        axis.value,
                        axis.published_value,
                        axis.published_at,
                        max_hz=self._rate.max_hz,
                        deadband_pct=self._rate.deadband_pct,
                    ):
                        self._publish_axis(axis, now)

    def backstop(self, now: float) -> None:
        """Re-send every axis. ``vehicle_control/v1`` reads silence as stop, so a still
        wheel must keep talking at ``min_hz``."""
        with self._lock:
            for axis in self._axes.values():
                if axis.value is not None:
                    self._publish_axis(axis, now)
