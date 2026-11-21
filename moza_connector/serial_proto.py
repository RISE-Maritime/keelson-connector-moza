"""The Moza serial protocol, as far as the connector needs it (pure): the base's steering
angle (the hard stop) and the rim's LED colours and brightness (alarms).

Written from protocol notes (Boxflat's moza-protocol.md and serial.yml, read as notes only:
Boxflat is GPL-3.0 and none of its code is here) and checked against an R21 Ultra and its rim:

    0x7E | n | group | device | command id ... | value (big-endian) | checksum

* ``n`` counts the command id and value bytes.
* ``checksum`` is (13 + the sum of every byte before it, 0x7E included) mod 256.
* The reply has the group with 0x80 set, the device's nibbles swapped (0x13 -> 0x31), the
  same command id, and the stored value. A read sends a value of the field's size (1, or
  zeros for a colour).
* The base also streams frames of its own (logs, group 0x0E) on the same port: a reader
  skips them.

Measured on the R21 Ultra (Pit House at 900 deg): ``limit`` and ``max-angle`` both read 450,
the angle either side of centre. Its rim (device 0x17) has 10 RPM LEDs and 10 button LEDs,
each an RGB colour (read 56 f7 fc, a cyan), and a brightness per group (read 50).

The base's own light bar is not the base's (0x13) but the main controller's (0x12), read in
group 0x22 and written in 0x20 (from AZOM's protocol notes, read as notes only, and checked on
the R21 Ultra): a standby mode (1 = constant) and, per mode, an RGB colour for each LED of its
two strips, 9 each (read 56 f7 fc in mode 1).
"""

from __future__ import annotations

from dataclasses import dataclass

START = 0x7E
MAGIC = 13
BASE = 0x13
WHEEL = 0x17
MAIN = 0x12
#: The base's groups (read, write), and the rim's.
BASE_GROUPS = (40, 41)
WHEEL_GROUPS = (64, 63)
MAIN_LIGHT_GROUPS = (0x22, 0x20)
RPM_LEDS = 10
BUTTON_LEDS = 10
BASE_STRIPS = 2
BASE_STRIP_LEDS = 9
#: The base light bar's constant standby mode: its colours are this mode's palette.
CONSTANT_MODE = 1


@dataclass(frozen=True)
class Command:
    device: int
    groups: tuple[int, int]  # (read, write)
    id: tuple[int, ...]
    size: int  # value bytes
    writable: bool
    read_value: int = 1  # what a read sends (the devices answer with the stored value)


def _base(cid, writable=False):
    return Command(BASE, BASE_GROUPS, cid, 2, writable)


def _wheel(cid, size, writable=True):
    return Command(WHEEL, WHEEL_GROUPS, cid, size, writable, 1 if size < 3 else 0)


def _light(cid, size):
    return Command(MAIN, MAIN_LIGHT_GROUPS, cid, size, True, 0)


#: Every command the connector sends. ``writable`` is the whole allow-list of writes: the
#: base's steering angle, the rim's LED colours, the base light bar's mode and colours. Never
#: the torque, the FFB strength or the soft-limit strength (CLAUDE.md).
COMMANDS: dict[str, Command] = {
    "limit": _base((1,), writable=True),
    "max-angle": _base((23,), writable=True),
    "soft-limit-stiffness": _base((31,)),
    "soft-limit-strength": _base((27,)),
    **{f"rpm-color{n + 1}": _wheel((31, 0, 255, n), 3) for n in range(RPM_LEDS)},
    **{f"button-color{n + 1}": _wheel((31, 1, 255, n), 3) for n in range(BUTTON_LEDS)},
    "base-standby-mode": _light((0x1D,), 1),
    **{
        f"base-color{s}-{led}": _light((0x20, s, CONSTANT_MODE, led), 3)
        for s in range(BASE_STRIPS)
        for led in range(BASE_STRIP_LEDS)
    },
}
#: The rim's LED colours, all of them.
RIM_COLORS = tuple(n for n in COMMANDS if n.startswith(("rpm-color", "button-color")))
#: The base light bar: its mode, and its constant mode's colours.
BASE_LIGHT = ("base-standby-mode", *(n for n in COMMANDS if n.startswith("base-color")))


def checksum(head: bytes) -> int:
    return (MAGIC + sum(head)) & 0xFF


def command(name: str, *, write: bool = False) -> Command:
    try:
        c = COMMANDS[name]
    except KeyError:
        raise ValueError(f"{name!r} is not a command the connector sends") from None
    if write and not c.writable:
        raise ValueError(f"{name!r} is not a command the connector may write")
    return c


def encode(group: int, device: int, cid: tuple[int, ...], value: int, size: int) -> bytes:
    if not 0 <= value < 1 << (8 * size):
        raise ValueError(f"value {value} does not fit {size} bytes")
    head = bytes([START, len(cid) + size, group, device, *cid]) + value.to_bytes(size, "big")
    return head + bytes([checksum(head)])


def read_request(name: str) -> bytes:
    c = command(name)
    return encode(c.groups[0], c.device, c.id, c.read_value, c.size)


def write_request(name: str, value: int) -> bytes:
    c = command(name, write=True)
    return encode(c.groups[1], c.device, c.id, value, c.size)


def _swap(device: int) -> int:
    return ((device & 0x0F) << 4) | (device >> 4)


def find_reply(buf: bytes, name: str, *, write: bool = False) -> tuple[int | None, int]:
    """The value in the device's reply to ``name`` (to a read, or a write), if ``buf`` holds
    one with a good checksum, and how many bytes of ``buf`` are consumed (up to the end of that
    reply, or all but a possible partial frame). Other frames are skipped."""
    c = COMMANDS[name]
    group = c.groups[1 if write else 0]
    prefix = bytes([START, len(c.id) + c.size, group | 0x80, _swap(c.device), *c.id])
    size = len(prefix) + c.size + 1
    at = 0
    while True:
        i = buf.find(prefix, at)
        if i < 0:
            # Keep a tail that could be the start of the reply.
            return None, max(0, len(buf) - size)
        if len(buf) < i + size:
            return None, i
        frame = buf[i : i + size]
        if checksum(frame[:-1]) == frame[-1]:
            return int.from_bytes(frame[-1 - c.size : -1], "big"), i + size
        at = i + 1


def angle_value(range_deg: float) -> int:
    """The base's value for a steering angle given lock to lock: degrees either side."""
    return int(round(range_deg / 2.0))


def range_deg(value: int) -> float:
    """The steering angle lock to lock for the base's value."""
    return 2.0 * value
