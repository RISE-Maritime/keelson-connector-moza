"""The device profile: which evdev axis and key goes to which keelson subject."""

from __future__ import annotations

import re
from dataclasses import dataclass, field
from importlib import resources
from pathlib import Path

import keelson
import yaml

from .codes import ABS

AXIS_SCHEMA = "keelson.TimestampedFloat"
BUTTON_SUBJECT = "button_state_change"
BUTTON_SCHEMA = "keelson.TimestampedInt"


@dataclass(frozen=True)
class AxisMap:
    code: int
    code_name: str
    subject: str
    input: str
    bipolar: bool
    invert: bool = False
    steering: bool = False


@dataclass(frozen=True)
class Profile:
    name: str
    vendor: int
    name_regex: re.Pattern
    axes: tuple[AxisMap, ...]
    button_names: dict[int, str] = field(default_factory=dict)

    @property
    def steering(self) -> AxisMap:
        return next(a for a in self.axes if a.steering)

    def button_name(self, index: int) -> str:
        return self.button_names.get(index, f"btn_{index}")


def builtin_path(name: str) -> Path:
    return Path(str(resources.files("moza_connector") / "profiles" / f"{name}.yaml"))


def load(ref: str) -> Profile:
    """A profile by built-in name (``r21-ultra``) or by path to a YAML file."""
    path = Path(ref)
    if not path.suffix:
        path = builtin_path(ref)
    if not path.is_file():
        raise FileNotFoundError(f"no profile {ref!r} (looked for {path})")
    return parse(yaml.safe_load(path.read_text()), source=str(path))


def parse(doc: dict, *, source: str = "<profile>") -> Profile:
    def fail(msg: str):
        raise ValueError(f"{source}: {msg}")

    if not isinstance(doc, dict):
        fail("not a mapping")
    match = doc.get("match") or {}
    try:
        vendor = int(match["vendor"])
        name_regex = re.compile(match.get("name_regex", ".*"), re.IGNORECASE)
    except (KeyError, TypeError, ValueError, re.error) as e:
        fail(f"match needs an integer vendor and a valid name_regex ({e})")

    axes: list[AxisMap] = []
    seen_codes, seen_sources = set(), set()
    for i, a in enumerate(doc.get("axes") or []):
        code_name = a.get("code")
        if code_name not in ABS:
            fail(f"axes[{i}]: unknown code {code_name!r} (known: {', '.join(sorted(ABS))})")
        subject, inp = a.get("subject"), a.get("input")
        if not subject or not inp:
            fail(f"axes[{i}]: subject and input are required")
        if not keelson.is_subject_well_known(subject):
            fail(f"axes[{i}]: {subject!r} is not a keelson subject")
        if keelson.get_subject_schema(subject) != AXIS_SCHEMA:
            fail(f"axes[{i}]: {subject!r} is {keelson.get_subject_schema(subject)}, an axis needs {AXIS_SCHEMA}")
        rng = a.get("range")
        if rng not in ("bipolar", "unipolar"):
            fail(f"axes[{i}]: range must be bipolar or unipolar, got {rng!r}")
        if code_name in seen_codes:
            fail(f"axes[{i}]: {code_name} mapped twice")
        if (subject, inp) in seen_sources:
            fail(f"axes[{i}]: {subject}/{inp} published twice")
        seen_codes.add(code_name)
        seen_sources.add((subject, inp))
        axes.append(
            AxisMap(
                code=ABS[code_name],
                code_name=code_name,
                subject=subject,
                input=str(inp),
                bipolar=rng == "bipolar",
                invert=bool(a.get("invert", False)),
                steering=bool(a.get("steering", False)),
            )
        )

    steering = [a for a in axes if a.steering]
    if len(steering) != 1:
        fail(f"exactly one axis must be steering: true, found {len(steering)}")
    if not steering[0].bipolar:
        fail("the steering axis must be bipolar")

    names = (doc.get("buttons") or {}).get("names") or {}
    try:
        button_names = {int(k): str(v) for k, v in names.items()}
    except (TypeError, ValueError):
        fail("buttons.names keys must be integers")
    if len(set(button_names.values())) != len(button_names):
        fail("buttons.names has a name twice")

    return Profile(
        name=str(doc.get("name", source)),
        vendor=vendor,
        name_regex=name_regex,
        axes=tuple(axes),
        button_names=button_names,
    )
