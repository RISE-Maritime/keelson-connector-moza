"""HID report descriptors: parse one, then read and write its reports by usage name.

Pure: no hidapi, no clock. ``hid_device.HidDevice`` finds every report it touches here by
its usage -- never by a hard-coded report id or byte offset -- so a base whose firmware
lays its reports out differently is read and driven correctly, and a base that lacks a
report fails loudly instead of having bytes written to the wrong field.
"""

from __future__ import annotations

from dataclasses import dataclass, field

PAGE_GD, PAGE_BUTTON, PAGE_ORDINAL, PAGE_PID = 0x01, 0x09, 0x0A, 0x0F

_GD = {
    0x04: "Joystick",
    0x05: "GamePad",
    0x30: "X",
    0x31: "Y",
    0x32: "Z",
    0x33: "Rx",
    0x34: "Ry",
    0x35: "Rz",
    0x36: "Slider",
    0x37: "Dial",
    0x38: "Wheel",
    0x39: "HatSwitch",
    0x3B: "ByteCount",
}

# The USB HID Physical Interface Device usages (HID Usage Tables, page 0x0F) this
# connector reads or writes, plus the collections that name its reports.
_PID = {
    0x21: "SetEffectReport",
    0x22: "EffectBlockIndex",
    0x23: "ParameterBlockOffset",
    0x25: "EffectType",
    0x26: "ETConstantForce",
    0x27: "ETRamp",
    0x30: "ETSquare",
    0x31: "ETSine",
    0x32: "ETTriangle",
    0x33: "ETSawtoothUp",
    0x34: "ETSawtoothDown",
    0x40: "ETSpring",
    0x41: "ETDamper",
    0x42: "ETInertia",
    0x43: "ETFriction",
    0x50: "Duration",
    0x51: "SamplePeriod",
    0x52: "Gain",
    0x53: "TriggerButton",
    0x54: "TriggerRepeatInterval",
    0x55: "AxesEnable",
    0x56: "DirectionEnable",
    0x57: "Direction",
    0x58: "TypeSpecificBlockOffset",
    0x5A: "SetEnvelopeReport",
    0x5F: "SetConditionReport",
    0x60: "CPOffset",
    0x61: "PositiveCoefficient",
    0x62: "NegativeCoefficient",
    0x63: "PositiveSaturation",
    0x64: "NegativeSaturation",
    0x65: "DeadBand",
    0x6E: "SetPeriodicReport",
    0x6F: "Offset",
    0x70: "Magnitude",
    0x71: "Phase",
    0x72: "Period",
    0x73: "SetConstantForceReport",
    0x74: "SetRampForceReport",
    0x77: "EffectOperationReport",
    0x78: "EffectOperation",
    0x79: "OpEffectStart",
    0x7A: "OpEffectStartSolo",
    0x7B: "OpEffectStop",
    0x7C: "LoopCount",
    0x7D: "DeviceGainReport",
    0x7E: "DeviceGain",
    0x7F: "PIDPoolReport",
    0x89: "PIDBlockLoadReport",
    0x8B: "BlockLoadStatus",
    0x8C: "BlockLoadSuccess",
    0x8D: "BlockLoadFull",
    0x8E: "BlockLoadError",
    0x90: "PIDBlockFreeReport",
    0x92: "PIDStateReport",
    0x95: "PIDDeviceControlReport",
    0x96: "PIDDeviceControl",
    0x97: "DCEnableActuators",
    0x98: "DCDisableActuators",
    0x99: "DCStopAllEffects",
    0x9A: "DCDeviceReset",
    0x9B: "DCDevicePause",
    0x9C: "DCDeviceContinue",
    0xA7: "StartDelay",
    0xAB: "CreateNewEffectReport",
    0xAC: "RAMPoolAvailable",
}


def usage_name(page: int, usage: int) -> str:
    if page == PAGE_GD and usage in _GD:
        return _GD[usage]
    if page == PAGE_PID and usage in _PID:
        return _PID[usage]
    if page == PAGE_BUTTON:
        return f"Button{usage}"
    if page == PAGE_ORDINAL:
        return f"Ordinal{usage}"
    return f"{page:#06x}:{usage:#06x}"


class _Null:
    """A PID "null" value: out of the field's logical range, so the device ignores it
    (``Duration`` null is "infinite", ``TriggerButton`` null is "no trigger")."""

    def __repr__(self) -> str:
        return "NULL"


NULL = _Null()


@dataclass(frozen=True)
class Field:
    usages: tuple[str, ...]  # a variable's usage per slot; an array's selectable usages
    parent: str  # the innermost enclosing collection's usage
    bit_offset: int  # from the first byte after the report id
    size: int
    count: int
    lmin: int
    lmax: int
    variable: bool
    constant: bool

    @property
    def signed(self) -> bool:
        return self.lmin < 0

    def slot_usage(self, i: int) -> str:
        # A variable with more slots than usages repeats its last usage (HID 1.11, 6.2.2.8).
        return self.usages[min(i, len(self.usages) - 1)] if self.usages else ""

    def extract(self, payload: int, i: int = 0) -> int:
        raw = payload >> (self.bit_offset + i * self.size) & ((1 << self.size) - 1)
        if self.signed and raw >> (self.size - 1) & 1:
            raw -= 1 << self.size
        return raw


@dataclass(frozen=True)
class Report:
    kind: str  # "Input", "Output" or "Feature"
    id: int  # 0 when the device uses no report ids
    name: str  # the usage of the collection the report sits in
    fields: tuple[Field, ...]
    _slots: dict[str, tuple[Field, int]] = field(default_factory=dict, repr=False, compare=False)

    @property
    def nbytes(self) -> int:
        """Payload length, without the report id."""
        return (sum(f.size * f.count for f in self.fields) + 7) // 8

    def slot(self, name: str) -> tuple[Field, int]:
        """A value by usage (``"Gain"``), by ``"Parent/Usage"`` when the usage alone is
        ambiguous (``"Direction/Ordinal1"``), or an array by its collection (``"EffectType"``)."""
        try:
            return self._slots[name]
        except KeyError:
            raise KeyError(f"{self.kind} report {self.id} ({self.name}) has no {name!r}") from None

    def has(self, name: str) -> bool:
        return name in self._slots

    def encode(self, values: dict[str, int | str | _Null]) -> bytes:
        """The report, id first, with every named value set and everything else 0.

        A value outside the field's logical range raises: nothing is ever silently
        clipped on its way to the motor. ``NULL`` writes the PID null value on purpose.
        """
        payload = 0
        for name, value in values.items():
            f, i = self.slot(name)
            if f.variable:
                if isinstance(value, str):
                    raise TypeError(f"{name} is a value, not a selector")
                v = _null_for(f) if value is NULL else int(value)
                if value is not NULL and not f.lmin <= v <= f.lmax:
                    raise ValueError(f"{name}={v} outside its logical range [{f.lmin}, {f.lmax}]")
            else:
                if not isinstance(value, str) or value not in f.usages:
                    raise ValueError(f"{name} selects one of {f.usages}, got {value!r}")
                v = f.lmin + f.usages.index(value)
            payload |= (v & ((1 << f.size) - 1)) << (f.bit_offset + i * f.size)
        body = payload.to_bytes(self.nbytes, "little")
        return (bytes([self.id]) if self.id else b"") + body

    def decode(self, data: bytes | list[int]) -> dict[str, int]:
        """Every named value of a received report (id first when the device uses ids)."""
        body = bytes(data[1:] if self.id else data)
        payload = int.from_bytes(body[: self.nbytes], "little")
        return {name: f.extract(payload, i) for name, (f, i) in self._slots.items()}

    def selected(self, name: str, value: int) -> str | None:
        """The usage an array value selects, or None for a null selection."""
        f, _ = self.slot(name)
        i = value - f.lmin
        return f.usages[i] if 0 <= i < len(f.usages) else None


@dataclass(frozen=True)
class Descriptor:
    reports: tuple[Report, ...]

    def find(self, kind: str, name: str) -> Report | None:
        """A report by its collection usage (``"SetEffectReport"``)."""
        return next((r for r in self.reports if r.kind == kind and r.name == name), None)

    def find_with(self, kind: str, usage: str) -> Report | None:
        """The first report of ``kind`` carrying ``usage``."""
        return next((r for r in self.reports if r.kind == kind and r.has(usage)), None)


def _null_for(f: Field) -> int:
    if f.lmin > 0:
        return 0
    if f.lmax < (1 << f.size) - 1 and not f.signed:
        return (1 << f.size) - 1
    raise ValueError(f"field {f.slot_usage(0)} has no null value (its logical range fills it)")


def _signed(raw: int, nbytes: int) -> int:
    bits = nbytes * 8
    return raw - (1 << bits) if nbytes and raw >> (bits - 1) & 1 else raw


def parse(data: bytes) -> Descriptor:
    """Parse a report descriptor (HID 1.11, section 6.2.2)."""
    g = {"page": 0, "lmin": 0, "lmax": 0, "size": 0, "count": 0, "id": 0}
    stack: list[dict] = []
    usages: list[tuple[int, int]] = []
    umin: int | None = None
    umax: int | None = None
    collections: list[str] = []
    fields: dict[tuple[str, int], list[Field]] = {}
    names: dict[tuple[str, int], str] = {}
    offsets: dict[tuple[str, int], int] = {}

    def full(u: int, size: int) -> tuple[int, int]:
        return (u >> 16, u & 0xFFFF) if size == 4 else (g["page"], u)

    i = 0
    while i < len(data):
        b = data[i]
        if b == 0xFE:  # long item: skip
            i += 3 + data[i + 1]
            continue
        size = (0, 1, 2, 4)[b & 3]
        typ, tag = (b >> 2) & 3, b >> 4
        raw = int.from_bytes(data[i + 1 : i + 1 + size], "little")
        i += 1 + size
        if typ == 0:  # main
            if tag in (0x8, 0x9, 0xB):
                kind = {0x8: "Input", 0x9: "Output", 0xB: "Feature"}[tag]
                key = (kind, g["id"])
                if usages:
                    us = usages
                elif umin is not None and umax is not None:
                    page = umin >> 16 if umin >> 16 else g["page"]
                    us = [(page, u) for u in range(umin & 0xFFFF, (umax & 0xFFFF) + 1)]
                else:
                    us = []
                f = Field(
                    usages=tuple(usage_name(p, u) for p, u in us),
                    parent=collections[-1] if collections else "",
                    bit_offset=offsets.get(key, 0),
                    size=g["size"],
                    count=g["count"],
                    lmin=g["lmin"],
                    lmax=g["lmax"],
                    variable=bool(raw & 0x02),
                    constant=bool(raw & 0x01),
                )
                fields.setdefault(key, []).append(f)
                offsets[key] = f.bit_offset + f.size * f.count
                if key not in names:
                    names[key] = collections[1] if len(collections) > 1 else (collections[0] if collections else "")
            elif tag == 0xA:  # collection
                collections.append(usage_name(*usages[0]) if usages else "")
            elif tag == 0xC:  # end collection
                collections.pop()
            usages, umin, umax = [], None, None
        elif typ == 1:  # global
            if tag == 0x0:
                g["page"] = raw
            elif tag == 0x1:
                g["lmin"] = _signed(raw, size)
            elif tag == 0x2:
                # Logical Maximum is signed only when the minimum is (HID 1.11, 6.2.2.7).
                g["lmax"] = _signed(raw, size) if g["lmin"] < 0 else raw
            elif tag == 0x7:
                g["size"] = raw
            elif tag == 0x8:
                g["id"] = raw
            elif tag == 0x9:
                g["count"] = raw
            elif tag == 0xA:
                stack.append(dict(g))
            elif tag == 0xB:
                g = stack.pop()
        elif typ == 2:  # local
            if tag == 0x0:
                usages.append(full(raw, size))
            elif tag == 0x1:
                umin = raw
            elif tag == 0x2:
                umax = raw

    reports = []
    for key, fs in fields.items():
        slots: dict[str, tuple[Field, int]] = {}
        plain: dict[str, int] = {}
        for f in fs:
            if f.constant:
                continue
            if f.variable:
                for n in range(f.count):
                    u = f.slot_usage(n)
                    if not u:
                        continue
                    slots.setdefault(f"{f.parent}/{u}", (f, n))
                    slots.setdefault(u, (f, n))
                for u in set(f.usages):
                    plain[u] = plain.get(u, 0) + 1
            else:
                slots.setdefault(f.parent, (f, 0))
        for u, n in plain.items():  # an ambiguous bare usage must be qualified
            if n > 1:
                del slots[u]
        reports.append(Report(kind=key[0], id=key[1], name=names[key], fields=tuple(fs), _slots=slots))
    return Descriptor(reports=tuple(reports))
