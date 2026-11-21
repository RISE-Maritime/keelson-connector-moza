"""The Moza serial protocol, as far as the base's steering angle needs it (pure).

Written from protocol notes (Boxflat's moza-protocol.md and serial.yml, read as notes only:
Boxflat is GPL-3.0 and none of its code is here) and checked against an R21 Ultra:

    0x7E | n | group | device | command id ... | value (big-endian) | checksum

* ``n`` counts the command id and value bytes.
* ``checksum`` is (13 + the sum of every byte before it, 0x7E included) mod 256.
* The base is device 0x13. A read is group 40 (value 1), a write group 41. The reply has the
  group with 0x80 set and the device's nibbles swapped (0x31), the same command id, and the
  stored value.
* The base also streams frames of its own (logs, group 0x0E) on the same port, which a reader
  skips.

Measured on the R21 Ultra with Pit House at 900 deg: ``limit`` and ``max-angle`` both read 450,
the angle either side of centre.
"""

from __future__ import annotations

START = 0x7E
MAGIC = 13
BASE = 0x13
READ, WRITE = 40, 41

#: Commands the connector may WRITE: the steering angle only. Never the torque, the FFB
#: strength or the soft-limit strength (CLAUDE.md).
WRITABLE = {"limit": (1,), "max-angle": (23,)}
#: Commands it may read, for the log.
READABLE = {**WRITABLE, "soft-limit-stiffness": (31,), "soft-limit-strength": (27,)}


def checksum(head: bytes) -> int:
    return (MAGIC + sum(head)) & 0xFF


def _command(name: str, table: dict) -> tuple[int, ...]:
    try:
        return table[name]
    except KeyError:
        raise ValueError(f"{name!r} is not a command the connector may send this way") from None


def encode(group: int, device: int, command: tuple[int, ...], value: int) -> bytes:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"value {value} does not fit two bytes")
    head = bytes([START, len(command) + 2, group, device, *command]) + value.to_bytes(2, "big")
    return head + bytes([checksum(head)])


def read_request(name: str) -> bytes:
    return encode(READ, BASE, _command(name, READABLE), 1)


def write_request(name: str, value: int) -> bytes:
    return encode(WRITE, BASE, _command(name, WRITABLE), value)


def _swap(device: int) -> int:
    return ((device & 0x0F) << 4) | (device >> 4)


def find_reply(buf: bytes, group: int, name: str) -> tuple[int | None, int]:
    """The value in the base's reply to ``name`` sent in ``group``, if ``buf`` holds one with a
    good checksum, and how many bytes of ``buf`` are consumed (up to the end of that reply, or
    all but a possible partial frame). Other frames are skipped."""
    command = READABLE[name]
    prefix = bytes([START, len(command) + 2, group | 0x80, _swap(BASE), *command])
    size = len(prefix) + 3
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
            return int.from_bytes(frame[-3:-1], "big"), i + size
        at = i + 1


def angle_value(range_deg: float) -> int:
    """The base's value for a steering angle given lock to lock: degrees either side."""
    return int(round(range_deg / 2.0))


def range_deg(value: int) -> float:
    """The steering angle lock to lock for the base's value."""
    return 2.0 * value
