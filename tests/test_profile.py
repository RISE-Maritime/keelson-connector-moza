import copy

import pytest
import yaml

from moza_connector import profile

BUILTIN = yaml.safe_load(profile.builtin_path("r21-ultra").read_text())


def doc(**changes):
    d = copy.deepcopy(BUILTIN)
    d.update(changes)
    return d


def test_builtin_loads():
    p = profile.load("r21-ultra")
    assert p.vendor == 0x346E
    assert p.steering.code_name == "ABS_X" and p.steering.subject == "wheel_position_pct"
    assert p.name_regex.search("Gudsen MOZA R21 Ultra Base")
    assert not p.name_regex.search("Logitech G29")


def test_load_by_path(tmp_path):
    f = tmp_path / "mine.yaml"
    f.write_text(yaml.safe_dump(doc(name="mine")))
    assert profile.load(str(f)).name == "mine"


def test_missing():
    with pytest.raises(FileNotFoundError):
        profile.load("no-such-base")


def test_button_names():
    p = profile.parse(doc(buttons={"names": {0: "ptt"}}))
    assert p.button_name(0) == "ptt"
    assert p.button_name(7) == "btn_7"


@pytest.mark.parametrize(
    "axes, message",
    [
        ([{"code": "ABS_NOPE", "subject": "wheel_position_pct", "input": "w", "range": "bipolar", "steering": True}],
         "unknown code"),
        ([{"code": "ABS_X", "subject": "not_a_subject", "input": "w", "range": "bipolar", "steering": True}],
         "not a keelson subject"),
        ([{"code": "ABS_X", "subject": "button_state_change", "input": "w", "range": "bipolar", "steering": True}],
         "an axis needs"),
        ([{"code": "ABS_X", "subject": "wheel_position_pct", "input": "w", "range": "sideways", "steering": True}],
         "range must be"),
        ([{"code": "ABS_X", "subject": "wheel_position_pct", "input": "w", "range": "bipolar"}],
         "exactly one axis must be steering"),
        ([{"code": "ABS_X", "subject": "wheel_position_pct", "input": "w", "range": "unipolar", "steering": True}],
         "must be bipolar"),
        ([{"code": "ABS_X", "subject": "wheel_position_pct", "input": "w", "range": "bipolar", "steering": True},
          {"code": "ABS_X", "subject": "joystick_rt_pct", "input": "t", "range": "unipolar"}],
         "mapped twice"),
    ],
)
def test_rejects(axes, message):
    with pytest.raises(ValueError, match=message):
        profile.parse(doc(axes=axes))


def test_rejects_duplicate_button_names():
    with pytest.raises(ValueError, match="name twice"):
        profile.parse(doc(buttons={"names": {0: "ptt", 1: "ptt"}}))
