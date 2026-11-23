"""The base over USB HID: the macOS backend, through hidapi.

The base speaks the standard USB HID Physical Interface Device (PID) class, the same one
Linux's hid-universal-pidff drives. ``HidDevice`` presents it as a ``MozaDevice``, so
the input mapping and every force-feedback rule (gain ceiling, ramp, stale rudder,
shutdown) are the ones the evdev path uses, unchanged.

It mirrors what Linux would show:
- axes get the evdev codes hid-input gives them (X -> ABS_X, Slider -> ABS_THROTTLE, ...),
  with the descriptor's logical ranges;
- HID Button n gets BTN_JOYSTICK + n-1 (n <= 16), else BTN_TRIGGER_HAPPY + n-17, so a
  profile's ``btn_<k>`` is the same physical button on both backends;
- effects are uploaded the way hid-pidff does it (create, block load, set effect, set
  condition), with Moza's fixed 90 degree direction.

One thing Linux gives for free is added here. The kernel erases a process's effects when
it dies, even on SIGKILL; macOS does not, and the base would keep playing the last
spring. So every effect is uploaded with a finite ``EFFECT_LEASE_MS`` duration and
re-started every ``KEEPALIVE_S`` while it plays: if this process stops for any reason,
the wheel goes limp within a lease.

Only ``open_hid`` and ``find_hid`` import hidapi.
"""

from __future__ import annotations

import logging
import threading
from typing import Iterator, Protocol

from . import codes, model
from .device import AxisRange, DeviceGone, Event
from .hid_report import NULL, Descriptor, Report, parse
from .profile import Profile

log = logging.getLogger(__name__)

#: An uploaded effect stops by itself this long after its last (re)start.
EFFECT_LEASE_MS = 1000
#: How often a playing effect is re-started; well inside the lease.
KEEPALIVE_S = 0.25

#: hid-input's joystick button codes (linux/input-event-codes.h).
BTN_JOYSTICK = 0x120
BTN_TRIGGER_HAPPY = 0x2C0

#: hid-input maps Generic Desktop usage 0x30..0x38 to ABS (usage & 0xF).
AXIS_CODES = {
    "X": codes.ABS["ABS_X"],
    "Y": codes.ABS["ABS_Y"],
    "Z": codes.ABS["ABS_Z"],
    "Rx": codes.ABS["ABS_RX"],
    "Ry": codes.ABS["ABS_RY"],
    "Rz": codes.ABS["ABS_RZ"],
    "Slider": codes.ABS["ABS_THROTTLE"],
    "Dial": codes.ABS["ABS_RUDDER"],
    "Wheel": codes.ABS["ABS_WHEEL"],
}

#: hid-pidff's direction for every Moza effect (HID_PIDFF_QUIRK_FIX_WHEEL_DIRECTION):
#: 0x4000 of a turn, in the descriptor's hundredths of a degree.
MOZA_DIRECTION = 9000

_EFFECT_TYPES = {codes.FF_SPRING: "ETSpring", codes.FF_DAMPER: "ETDamper"}
#: Effects that play until stopped, so are leased; a click ends by itself.
_LEASED = frozenset(_EFFECT_TYPES.values())
_CLICK = "ETSquare"
#: The centre notch's force: like a click it ends by itself, and is never leased.
_CONSTANT = "ETConstantForce"
_PID_REPORTS = (
    ("Output", "SetEffectReport"),
    ("Output", "SetConditionReport"),
    ("Output", "EffectOperationReport"),
    ("Output", "PIDBlockFreeReport"),
    ("Output", "PIDDeviceControlReport"),
    ("Output", "DeviceGainReport"),
    ("Feature", "CreateNewEffectReport"),
    ("Feature", "PIDBlockLoadReport"),
)
_BLOCK_LOAD_TRIES = 20


class HidHandle(Protocol):
    """The slice of ``hid.device`` used here (the tests pass a recording fake)."""

    def read(self, max_length: int, timeout_ms: int = 0) -> list[int]: ...

    def write(self, buff: bytes) -> int: ...

    def send_feature_report(self, data: bytes) -> int: ...

    def get_feature_report(self, report_num: int, max_length: int) -> list[int]: ...

    def close(self) -> None: ...


def button_code(n: int) -> int:
    """The evdev key code hid-input gives a joystick's HID Button ``n`` (1-based)."""
    return BTN_JOYSTICK + n - 1 if n <= 16 else BTN_TRIGGER_HAPPY + n - 17


class _Pid:
    """The descriptor's PID reports, all present, or None."""

    def __init__(self, desc: Descriptor):
        self.reports = {name: desc.find(kind, name) for kind, name in _PID_REPORTS}
        missing = [name for name, r in self.reports.items() if r is None]
        if missing:
            raise LookupError(", ".join(missing))
        types = self.reports["SetEffectReport"].slot("EffectType")[0].usages
        for t in _EFFECT_TYPES.values():
            if t not in types:
                raise LookupError(f"effect type {t}")

    def __getitem__(self, name: str) -> Report:
        return self.reports[name]


class HidDevice:
    def __init__(self, handle: HidHandle, descriptor: bytes, *, name: str, path: str):
        self._h = handle
        self.name = name
        self.path = path
        desc = parse(descriptor)
        inp = desc.find_with("Input", "X")
        if inp is None:
            raise ValueError(f"{name}: no input report with an X axis")
        self._in = inp
        self._axes: list[tuple[int, str]] = []  # (evdev code, slot name)
        for usage, code in AXIS_CODES.items():
            if inp.has(usage):
                self._axes.append((code, usage))
        self._buttons: list[tuple[int, str]] = []  # (evdev code, slot name)
        for f in inp.fields:
            if f.variable and not f.constant:
                for i in range(f.count):
                    u = f.slot_usage(i)
                    if u.startswith("Button"):
                        self._buttons.append((button_code(int(u[6:])), u))
        try:
            self._pid: _Pid | None = _Pid(desc)
        except LookupError as e:
            log.warning("%s: no usable HID force feedback (missing %s)", name, e)
            self._pid = None
        # Clicks are optional: a base without a square periodic effect still does the rest.
        self._periodic: Report | None = None
        if self._pid is not None and _CLICK in self._pid["SetEffectReport"].slot("EffectType")[0].usages:
            self._periodic = desc.find("Output", "SetPeriodicReport")
        # The constant force (the centre notch) is optional too.
        self._constant: Report | None = None
        if self._pid is not None and _CONSTANT in self._pid["SetEffectReport"].slot("EffectType")[0].usages:
            self._constant = desc.find("Output", "SetConstantForceReport")

        # One lock for every write and for the effect bookkeeping, so the lease renewal can
        # never re-start an effect between shutdown's stop and its erase.
        self._lock = threading.RLock()
        self._effects: dict[int, str] = {}  # block index -> effect type
        self._playing: set[int] = set()
        self._ff_ready = False
        self._stop = threading.Event()
        self._keepalive: threading.Thread | None = None

        # Only the bytes holding the axes and buttons decide whether a report is news: the
        # base also streams telemetry that changes in every report.
        used = [inp.slot(s)[0] for _, s in self._axes + self._buttons]
        start = 1 if inp.id else 0
        self._span = slice(
            start + min(f.bit_offset for f in used) // 8,
            start + (max(f.bit_offset + f.size * f.count for f in used) + 7) // 8,
        )
        first = None
        for _ in range(50):  # the base reports at ~1 kHz; give it up to a second
            first = self._read(20)
            if first is not None:
                break
        if first is None:
            raise FileNotFoundError(f"{name} sends no input reports")
        self._raw = first[self._span]
        self._state = self._in.decode(first)

    # --- reading ------------------------------------------------------------------

    def _read(self, timeout_ms: int) -> list[int] | None:
        """One input report, or None when none arrived within ``timeout_ms``."""
        while True:
            try:
                data = self._h.read(64, timeout_ms)
            except (OSError, ValueError) as e:
                raise DeviceGone(f"HID read: {e}") from e
            if not data:
                return None
            if not self._in.id or data[0] == self._in.id:
                return list(data)
            timeout_ms = 0  # another report (PID state): look for the next one

    def axes(self) -> dict[int, AxisRange]:
        out = {}
        for code, slot in self._axes:
            f, _ = self._in.slot(slot)
            out[code] = AxisRange(lo=f.lmin, hi=f.lmax, value=self._state[slot])
        return out

    def keys(self) -> list[int]:
        return sorted(code for code, _ in self._buttons)

    def events(self, timeout: float) -> Iterator[Event]:
        """Every input report waiting (the first within ``timeout``), as evdev-style events."""
        wait_ms = max(1, round(timeout * 1000))
        for _ in range(256):  # bounded, so a flooding base cannot starve the caller
            data = self._read(wait_ms)
            if data is None:
                return
            wait_ms = 0
            raw = data[self._span]
            if raw == self._raw:
                continue
            self._raw = raw
            new = self._in.decode(data)
            old, self._state = self._state, new
            changed = False
            for code, slot in self._axes:
                if new[slot] != old[slot]:
                    changed = True
                    yield Event(codes.EV_ABS, code, new[slot])
            for code, slot in self._buttons:
                if new[slot] != old[slot]:
                    changed = True
                    yield Event(codes.EV_KEY, code, new[slot])
            if changed:
                yield Event(codes.EV_SYN, 0, 0)

    # --- force feedback -----------------------------------------------------------

    def has_ff(self) -> bool:
        return self._pid is not None

    def _send(self, report: Report, values: dict) -> None:
        data = report.encode(values)
        try:
            with self._lock:
                if report.kind == "Feature":
                    n = self._h.send_feature_report(data)
                else:
                    n = self._h.write(data)
        except (OSError, ValueError) as e:
            raise DeviceGone(f"HID {report.name}: {e}") from e
        if n is not None and n < 0:
            raise DeviceGone(f"HID {report.name}: write failed")

    def _pid_reports(self) -> _Pid:
        if self._pid is None:
            raise RuntimeError(f"{self.name} has no HID force feedback")
        return self._pid

    def _control(self, command: str) -> None:
        self._send(self._pid_reports()["PIDDeviceControlReport"], {command: 1})

    def _prepare(self) -> None:
        """Before the first force-feedback write: clear whatever an earlier run left in
        the base, hold the gain at 0, then enable the actuators (hid-pidff's reset)."""
        if self._ff_ready:
            return
        pid = self._pid_reports()
        self._control("DCDeviceReset")
        self._control("DCStopAllEffects")
        self._send(pid["DeviceGainReport"], {"DeviceGain": 0})
        self._control("DCEnableActuators")
        self._ff_ready = True

    def set_gain(self, level: int) -> None:
        ceiling = model.gain_level(model.MAX_GAIN_CEILING_PCT)
        if not 0 <= level <= ceiling:
            raise ValueError(f"gain {level} outside 0..{ceiling} (the {model.MAX_GAIN_CEILING_PCT:.0f}% ceiling)")
        self._prepare()
        report = self._pid_reports()["DeviceGainReport"]
        f, _ = report.slot("DeviceGain")
        self._send(report, {"DeviceGain": round(level * f.lmax / model.U16_MAX)})

    def set_autocenter(self, level: int) -> None:
        # PID has no autocentre: the only centring force is the spring this connector plays.
        if level != 0:
            raise NotImplementedError("HID force feedback has no autocentre; use a spring")
        self._prepare()

    def _create(self, effect: str) -> int:
        pid = self._pid_reports()
        self._send(pid["CreateNewEffectReport"], {"EffectType": effect})
        load = pid["PIDBlockLoadReport"]
        for _ in range(_BLOCK_LOAD_TRIES):
            try:
                with self._lock:
                    data = self._h.get_feature_report(load.id, load.nbytes + 1)
            except (OSError, ValueError) as e:
                raise DeviceGone(f"HID block load: {e}") from e
            if not data:
                continue
            if data[0] != load.id:  # some hidapi builds drop the report id
                data = [load.id, *data]
            values = load.decode(data)
            status = load.selected("BlockLoadStatus", values["BlockLoadStatus"])
            if status == "BlockLoadSuccess":
                return values["EffectBlockIndex"]
            if status in ("BlockLoadFull", "BlockLoadError"):
                raise RuntimeError(f"{self.name} refused a {effect}: {status}")
        raise RuntimeError(f"{self.name} never confirmed the {effect} upload")

    def upload_condition(
        self, effect_type: int, effect_id: int, *, center: int, coeff: int, saturation: int, deadband: int = 0
    ) -> int:
        effect = _EFFECT_TYPES.get(effect_type)
        if effect is None:
            raise ValueError(f"effect type {effect_type:#x} is not a spring or a damper")
        pid = self._pid_reports()
        self._prepare()
        if effect_id == -1:
            effect_id = self._create(effect)
            self._effects[effect_id] = effect
            self._send(
                pid["SetEffectReport"],
                {
                    "EffectBlockIndex": effect_id,
                    "EffectType": effect,
                    "Duration": EFFECT_LEASE_MS,
                    "TriggerRepeatInterval": 0,
                    "SamplePeriod": 0,
                    "StartDelay": 0,
                    "Gain": pid["SetEffectReport"].slot("Gain")[0].lmax,
                    "TriggerButton": NULL,
                    "AxesEnable/X": 1,
                    "DirectionEnable": 1,
                    "Direction/Ordinal1": MOZA_DIRECTION,
                },
            )
        elif self._effects.get(effect_id) != effect:
            raise ValueError(f"effect {effect_id} is not an uploaded {effect}")
        cond = pid["SetConditionReport"]
        sat_max = cond.slot("PositiveSaturation")[0].lmax
        sat = round(min(max(saturation, 0), model.U16_MAX) * sat_max / model.U16_MAX)
        band_max = cond.slot("DeadBand")[0].lmax
        band = round(min(max(deadband, 0), model.U16_MAX) * band_max / model.U16_MAX)
        self._send(
            cond,
            {
                "EffectBlockIndex": effect_id,
                "ParameterBlockOffset": 0,
                "CPOffset": center,
                "PositiveCoefficient": coeff,
                "NegativeCoefficient": coeff,
                "PositiveSaturation": sat,
                "NegativeSaturation": sat,
                "DeadBand": band,
            },
        )
        return effect_id

    def upload_click(self, effect_id: int, *, magnitude: int, period_ms: int, duration_ms: int) -> int:
        """A short square pulse. It is never leased: its own duration ends it, and that may
        not exceed a lease, so no click outlives a dead process by more than a lease."""
        if not 0 < duration_ms <= EFFECT_LEASE_MS:
            raise ValueError(f"a click lasts 1..{EFFECT_LEASE_MS} ms, not {duration_ms}")
        pid = self._pid_reports()
        periodic = self._periodic
        if periodic is None:
            raise NotImplementedError(f"{self.name} has no square periodic effect")
        self._prepare()
        if effect_id == -1:
            effect_id = self._create(_CLICK)
            self._effects[effect_id] = _CLICK
        elif self._effects.get(effect_id) != _CLICK:
            raise ValueError(f"effect {effect_id} is not an uploaded click")
        self._send(
            pid["SetEffectReport"],
            {
                "EffectBlockIndex": effect_id,
                "EffectType": _CLICK,
                "Duration": duration_ms,
                "TriggerRepeatInterval": 0,
                "SamplePeriod": 0,
                "StartDelay": 0,
                "Gain": pid["SetEffectReport"].slot("Gain")[0].lmax,
                "TriggerButton": NULL,
                "AxesEnable/X": 1,
                "DirectionEnable": 1,
                "Direction/Ordinal1": MOZA_DIRECTION,
            },
        )
        mag_max = periodic.slot("Magnitude")[0].lmax
        self._send(
            periodic,
            {
                "EffectBlockIndex": effect_id,
                "Magnitude": min(max(magnitude, 0), mag_max),
                "Offset": 0,
                "Phase": 0,
                "Period": max(1, period_ms),
            },
        )
        return effect_id

    def upload_constant(self, effect_id: int, *, magnitude: int, duration_ms: int) -> int:
        """A constant force for ``duration_ms`` each time it is started. Never leased, like a
        click: its own duration ends it, and that may not exceed a lease. A magnitude outside
        the descriptor's range raises: never clipped."""
        if not 0 < duration_ms <= EFFECT_LEASE_MS:
            raise ValueError(f"a constant force lasts 1..{EFFECT_LEASE_MS} ms, not {duration_ms}")
        pid = self._pid_reports()
        constant = self._constant
        if constant is None:
            raise NotImplementedError(f"{self.name} has no constant force effect")
        self._prepare()
        if effect_id == -1:
            effect_id = self._create(_CONSTANT)
            self._effects[effect_id] = _CONSTANT
            self._send(
                pid["SetEffectReport"],
                {
                    "EffectBlockIndex": effect_id,
                    "EffectType": _CONSTANT,
                    "Duration": duration_ms,
                    "TriggerRepeatInterval": 0,
                    "SamplePeriod": 0,
                    "StartDelay": 0,
                    "Gain": pid["SetEffectReport"].slot("Gain")[0].lmax,
                    "TriggerButton": NULL,
                    "AxesEnable/X": 1,
                    "DirectionEnable": 1,
                    "Direction/Ordinal1": MOZA_DIRECTION,
                },
            )
        elif self._effects.get(effect_id) != _CONSTANT:
            raise ValueError(f"effect {effect_id} is not an uploaded constant force")
        self._send(constant, {"EffectBlockIndex": effect_id, "Magnitude": magnitude})
        return effect_id

    def _operate(self, effect_id: int, on: bool) -> None:
        self._send(
            self._pid_reports()["EffectOperationReport"],
            {
                "EffectBlockIndex": effect_id,
                "EffectOperation": "OpEffectStart" if on else "OpEffectStop",
                "LoopCount": 1,
            },
        )

    def play(self, effect_id: int, on: bool) -> None:
        with self._lock:
            if effect_id not in self._effects:
                raise ValueError(f"effect {effect_id} was never uploaded")
            if not on:
                self._playing.discard(effect_id)
            self._operate(effect_id, on)
            leased = on and self._effects[effect_id] in _LEASED
            if leased:
                self._playing.add(effect_id)
        if leased:
            self._start_keepalive()

    def erase(self, effect_id: int) -> None:
        with self._lock:
            self._playing.discard(effect_id)
            self._effects.pop(effect_id, None)
            self._send(self._pid_reports()["PIDBlockFreeReport"], {"EffectBlockIndex": effect_id})

    def renew_leases(self) -> None:
        """Re-start every playing effect, so each runs another ``EFFECT_LEASE_MS``."""
        with self._lock:
            for effect_id in sorted(self._playing):
                self._operate(effect_id, True)

    def _start_keepalive(self) -> None:
        if self._keepalive is not None:
            return

        def loop():
            while not self._stop.wait(KEEPALIVE_S):
                try:
                    self.renew_leases()
                except Exception as e:  # the base is gone; the read side reports it
                    log.debug("Effect keepalive stopped: %s", e)
                    return

        self._keepalive = threading.Thread(target=loop, name="ffb-lease", daemon=True)
        self._keepalive.start()

    def close(self) -> None:
        """Release the base. After any force feedback: stop and free every effect and
        leave the gain at 0, whatever ``FfbController.shutdown`` already did. Never raises."""
        self._stop.set()
        if self._keepalive is not None:
            self._keepalive.join(timeout=2 * KEEPALIVE_S)
        if self._ff_ready:
            with self._lock:
                self._playing.clear()
            for step in (
                lambda: self._control("DCStopAllEffects"),
                lambda: self._send(self._pid_reports()["DeviceGainReport"], {"DeviceGain": 0}),
                lambda: self._control("DCDeviceReset"),
            ):
                try:
                    step()
                except Exception as e:
                    log.debug("HID FF release: %s", e)
        try:
            self._h.close()
        except Exception as e:
            log.debug("HID close: %s", e)


# --- Opening the real base -----------------------------------------------------------


def open_hid(info: dict) -> HidDevice:
    """Open the hidapi device ``info`` (one entry of ``hid.enumerate()``)."""
    import hid  # macOS: see pyproject.toml

    h = hid.device()
    try:
        h.open_path(info["path"])
    except OSError as e:
        raise PermissionError(
            f"cannot open {info.get('product_string')!r}: {e} (is another program, such as Pit House or a "
            "second moza2keelson, holding it?)"
        ) from e
    try:
        descriptor = bytes(h.get_report_descriptor())
        return HidDevice(
            h,
            descriptor,
            name=info.get("product_string") or "Moza base",
            path=info["path"].decode(errors="replace") if isinstance(info["path"], bytes) else str(info["path"]),
        )
    except BaseException:
        h.close()
        raise


def find_hid(profile: Profile) -> HidDevice:
    """The first HID joystick the profile matches, with the profile's steering axis."""
    import hid

    for info in hid.enumerate(profile.vendor, 0):
        # macOS lists one entry per top-level usage; the joystick one carries the inputs.
        if (info.get("usage_page"), info.get("usage")) != (0x01, 0x04):
            continue
        if not profile.name_regex.search(info.get("product_string") or ""):
            continue
        dev = open_hid(info)
        if profile.steering.code not in dev.axes():
            dev.close()
            continue
        log.info(
            "Found %s over HID (product %04x, force feedback: %s)", dev.name, info.get("product_id", 0), dev.has_ff()
        )
        return dev
    raise FileNotFoundError(f"no {profile.name} found over HID (vendor {profile.vendor:04x})")
