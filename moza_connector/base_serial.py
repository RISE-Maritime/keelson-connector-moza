"""The base's serial port: read and write its steering angle (``serial_proto``).

The only module that imports pyserial. A port is used only when its USB vendor id is
Moza's, so nothing is ever written to an unidentified serial device. Every write is read
back. ``FakeBaseSerial`` stands in for the tests.
"""

from __future__ import annotations

import json
import logging
import time

from . import serial_proto as proto

log = logging.getLogger(__name__)

MOZA_VID = 0x346E
REPLY_TIMEOUT_S = 0.6


class BaseSerialError(RuntimeError):
    """The base did not answer, or answered something else."""


def find_base_serial() -> str | None:
    """The serial port of a Moza base on this machine, or None."""
    from serial.tools import list_ports  # the edge: only here

    for port in list_ports.comports():
        if port.vid == MOZA_VID:
            return port.device
    return None


def is_moza(path: str) -> bool:
    """Whether ``path`` is a Moza device's serial port."""
    from serial.tools import list_ports

    return any(p.device == path and p.vid == MOZA_VID for p in list_ports.comports())


class BaseSerial:
    """A Moza base's serial port, opened on ``path``."""

    def __init__(self, path: str):
        import serial  # the edge: only here

        if not is_moza(path):
            raise BaseSerialError(f"{path} is not a Moza device's serial port")
        self.path = path
        self._port = serial.Serial(path, 115200, timeout=0.05, write_timeout=0.5)
        self._port.reset_input_buffer()

    def _ask(self, frame: bytes, group: int, name: str) -> int:
        self._port.write(frame)
        buf = b""
        end = time.monotonic() + REPLY_TIMEOUT_S
        while time.monotonic() < end:
            buf += self._port.read(256)
            value, used = proto.find_reply(buf, group, name)
            if value is not None:
                return value
            buf = buf[used:]
        raise BaseSerialError(f"no reply from the base to {name}")

    def read(self, name: str) -> int:
        return self._ask(proto.read_request(name), proto.READ, name)

    def write(self, name: str, value: int) -> int:
        """Write, then read it back. Raises BaseSerialError when the base keeps another value."""
        self._ask(proto.write_request(name, value), proto.WRITE, name)
        back = self.read(name)
        if back != value:
            raise BaseSerialError(f"the base kept {name} = {back}, not {value}")
        return back

    def close(self) -> None:
        try:
            self._port.close()
        except Exception as e:  # already gone
            log.debug("Closing %s: %s", self.path, e)


class FakeBaseSerial:
    """A base in memory, for the tests: records every write; ``gone`` makes every call fail.
    With ``path`` its angle lives in that JSON file, so a second process (a restart after a
    crash) finds the base as the first left it."""

    def __init__(self, angle: int = 450, path: str | None = None):
        self.path = "fake"
        self._file = path
        self.values = {"limit": angle, "max-angle": angle}
        if path:
            try:
                with open(path) as f:
                    self.values = {k: int(v) for k, v in json.load(f).items()}
            except FileNotFoundError:
                self._save()
        self.writes: list[tuple[str, int]] = []
        self.gone = False

    def _save(self) -> None:
        if self._file:
            with open(self._file, "w") as f:
                json.dump(self.values, f)

    def read(self, name: str) -> int:
        if self.gone:
            raise BaseSerialError("the fake base is gone")
        return self.values[name]

    def write(self, name: str, value: int) -> int:
        if self.gone:
            raise BaseSerialError("the fake base is gone")
        proto.write_request(name, value)  # the same allow-list as the real port
        self.values[name] = value
        self.writes.append((name, value))
        self._save()
        return value

    def close(self) -> None:
        pass
