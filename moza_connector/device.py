"""The wheelbase: reading it and driving its force feedback.

``MozaDevice`` is what the rest of the connector sees. ``EvdevDevice`` is the real base on
Linux; ``FakeDevice`` replays a scripted session for the tests (and records every
force-feedback call), so nothing above this module ever imports evdev.
"""

from __future__ import annotations

import errno
import json
import logging
import select
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Protocol

from . import codes
from .profile import Profile

log = logging.getLogger(__name__)


class DeviceGone(Exception):
    """The base was unplugged, powered off or otherwise stopped answering."""


@dataclass(frozen=True)
class AxisRange:
    lo: int
    hi: int
    value: int


@dataclass(frozen=True)
class Event:
    type: int
    code: int
    value: int


class MozaDevice(Protocol):
    name: str
    path: str

    def axes(self) -> dict[int, AxisRange]: ...

    def keys(self) -> list[int]: ...

    def has_ff(self) -> bool: ...

    def events(self, timeout: float) -> Iterator[Event]:
        """Events available within ``timeout`` seconds (possibly none). Raises DeviceGone."""
        ...

    def set_gain(self, level: int) -> None: ...

    def set_autocenter(self, level: int) -> None: ...

    def upload_condition(self, effect_type: int, effect_id: int, *, center: int, coeff: int, saturation: int) -> int:
        """Upload (``effect_id`` -1) or update a spring/damper; returns the effect id."""
        ...

    def play(self, effect_id: int, on: bool) -> None: ...

    def erase(self, effect_id: int) -> None: ...

    def close(self) -> None: ...


# --- The real base --------------------------------------------------------------------


def _gone(e: OSError) -> bool:
    return e.errno in (errno.ENODEV, errno.ENXIO, errno.EIO, errno.EBADF)


class EvdevDevice:
    def __init__(self, path: str):
        import evdev  # Linux only: see pyproject.toml
        from evdev import ecodes

        for name, value in (
            ("EV_KEY", codes.EV_KEY),
            ("EV_ABS", codes.EV_ABS),
            ("EV_FF", codes.EV_FF),
            ("FF_SPRING", codes.FF_SPRING),
            ("FF_DAMPER", codes.FF_DAMPER),
            ("FF_GAIN", codes.FF_GAIN),
            ("FF_AUTOCENTER", codes.FF_AUTOCENTER),
            *codes.ABS.items(),
        ):
            if getattr(ecodes, name) != value:
                raise RuntimeError(f"codes.{name}={value:#x} but evdev says {getattr(ecodes, name):#x}")

        self._evdev = evdev
        self._dev = evdev.InputDevice(path)
        self.path = path
        self.name = self._dev.name
        self._caps = self._dev.capabilities(absinfo=True)

    def axes(self) -> dict[int, AxisRange]:
        out = {}
        for code, info in self._caps.get(codes.EV_ABS, []):
            out[code] = AxisRange(lo=info.min, hi=info.max, value=info.value)
        return out

    def keys(self) -> list[int]:
        return sorted(self._caps.get(codes.EV_KEY, []))

    def has_ff(self) -> bool:
        ff = self._caps.get(codes.EV_FF, [])
        return codes.FF_SPRING in ff

    def events(self, timeout: float) -> Iterator[Event]:
        try:
            r, _, _ = select.select([self._dev.fd], [], [], timeout)
            if not r:
                return
            for e in self._dev.read():
                yield Event(e.type, e.code, e.value)
        except BlockingIOError:
            return
        except OSError as e:
            if _gone(e):
                raise DeviceGone(str(e)) from e
            raise

    def _write_ff(self, code: int, value: int) -> None:
        self._dev.write(codes.EV_FF, code, value)

    def set_gain(self, level: int) -> None:
        self._write_ff(codes.FF_GAIN, level)

    def set_autocenter(self, level: int) -> None:
        self._write_ff(codes.FF_AUTOCENTER, level)

    def upload_condition(self, effect_type: int, effect_id: int, *, center: int, coeff: int, saturation: int) -> int:
        ff = self._evdev.ff
        cond = ff.Condition(
            right_saturation=saturation,
            left_saturation=saturation,
            right_coeff=coeff,
            left_coeff=coeff,
            deadband=0,
            center=center,
        )
        effect = ff.Effect(
            effect_type,
            effect_id,
            0x4000,  # direction: irrelevant to a condition on one axis
            ff.Trigger(0, 0),
            ff.Replay(0, 0),  # length 0: play until stopped
            ff.EffectType(ff_condition_effect=(ff.Condition * 2)(cond, cond)),
        )
        return self._dev.upload_effect(effect)

    def play(self, effect_id: int, on: bool) -> None:
        self._write_ff(effect_id, 1 if on else 0)

    def erase(self, effect_id: int) -> None:
        self._dev.erase_effect(effect_id)

    def close(self) -> None:
        self._dev.close()


def find(profile: Profile, path: str | None = None) -> EvdevDevice:
    """The base: ``path`` when given, else the first evdev device the profile matches."""
    if path:
        return EvdevDevice(path)
    import evdev

    for candidate in sorted(evdev.list_devices()):
        try:
            d = evdev.InputDevice(candidate)
        except OSError:
            continue
        try:
            if d.info.vendor == profile.vendor and profile.name_regex.search(d.name):
                ff = d.capabilities().get(codes.EV_FF)
                abs_codes = [c for c, _ in d.capabilities(absinfo=True).get(codes.EV_ABS, [])]
                if profile.steering.code in abs_codes:
                    log.info("Found %s at %s (product %04x, force feedback: %s)", d.name, candidate, d.info.product, bool(ff))
                    return EvdevDevice(candidate)
        finally:
            d.close()
    raise FileNotFoundError(f"no {profile.name} found (vendor {profile.vendor:04x}, name /{profile.name_regex.pattern}/)")


# --- The scripted base ----------------------------------------------------------------


class FakeDevice:
    """A scripted base, for the tests only (``--fake-device``, refused outside them).

    The script is JSON::

        {"name": "Fake MOZA R21 Ultra Base",
         "axes": {"ABS_X": [0, 65535, 32767], "ABS_Z": [0, 65535, 0]},
         "keys": [288, 289],
         "ff": true,
         "events": [[0.1, "EV_ABS", "ABS_X", 65535], [0.1, "EV_KEY", 288, 1], [0.5, "gone"]]}

    Each event waits its delay (seconds, after the previous one). ``"gone"`` unplugs the
    base. After the last event the base stays plugged in and quiet. Every force-feedback
    call is appended to ``ff_log`` -- and, with ``"ff_log_path"`` in the script, written
    there as JSON lines, so a test can read what a separate process did to the wheel.
    """

    def __init__(self, script: dict, path: str = "<fake>"):
        self.path = path
        self.name = script.get("name", "Fake MOZA R21 Ultra Base")
        self._axes = {codes.ABS[k]: AxisRange(*v) for k, v in script.get("axes", {}).items()}
        self._keys = sorted(script.get("keys", []))
        self._ff = bool(script.get("ff", True))
        self._script = list(script.get("events", []))
        self._next_at: float | None = None
        self._gone = False
        self.closed = False
        self.ff_log: list[tuple] = []
        self._next_id = 0
        self._lock = threading.Lock()
        self._ff_log_path = script.get("ff_log_path")

    @classmethod
    def from_file(cls, path: str | Path) -> "FakeDevice":
        return cls(json.loads(Path(path).read_text()), path=str(path))

    def axes(self) -> dict[int, AxisRange]:
        return dict(self._axes)

    def keys(self) -> list[int]:
        return list(self._keys)

    def has_ff(self) -> bool:
        return self._ff

    def events(self, timeout: float) -> Iterator[Event]:
        if self._gone:
            raise DeviceGone("fake device unplugged")
        deadline = time.monotonic() + timeout
        while self._script:
            if self._next_at is None:
                self._next_at = time.monotonic() + float(self._script[0][0])
            wait = self._next_at - time.monotonic()
            if wait > 0:
                if time.monotonic() + wait > deadline:
                    time.sleep(max(0.0, deadline - time.monotonic()))
                    return
                time.sleep(wait)
            step = self._script.pop(0)
            self._next_at = None
            if step[1] == "gone":
                self._gone = True
                raise DeviceGone("fake device unplugged")
            etype = codes.EV_NAMES[step[1]]
            code = codes.ABS[step[2]] if isinstance(step[2], str) else int(step[2])
            yield Event(etype, code, int(step[3]))
            yield Event(codes.EV_SYN, 0, 0)
        time.sleep(max(0.0, deadline - time.monotonic()))

    def _record(self, *entry) -> None:
        if self._gone:
            raise DeviceGone("fake device unplugged")
        with self._lock:
            self.ff_log.append(entry)
            if self._ff_log_path:
                with open(self._ff_log_path, "a") as f:
                    f.write(json.dumps(entry) + "\n")

    def set_gain(self, level: int) -> None:
        self._record("gain", level)

    def set_autocenter(self, level: int) -> None:
        self._record("autocenter", level)

    def upload_condition(self, effect_type: int, effect_id: int, *, center: int, coeff: int, saturation: int) -> int:
        if effect_id == -1:
            effect_id = self._next_id
            self._next_id += 1
        self._record("upload", effect_type, effect_id, center, coeff, saturation)
        return effect_id

    def play(self, effect_id: int, on: bool) -> None:
        self._record("play", effect_id, on)

    def erase(self, effect_id: int) -> None:
        self._record("erase", effect_id)

    def close(self) -> None:
        self.closed = True
