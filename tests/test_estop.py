import threading

from moza_connector import serial_proto as proto
from moza_connector.base_serial import FakeEstop
from moza_connector.estop import GONE, PRESSED, RELEASED, UNKNOWN, EstopWatch


def test_the_frames_seen_on_the_bench():
    assert proto.estop_frame(True) == bytes.fromhex("7e02462100 01f5".replace(" ", ""))
    assert proto.estop_frame(False) == bytes.fromhex("7e024621000 0f4".replace(" ", ""))


def test_states_in_order_and_a_split_frame_kept():
    pressed, released = proto.estop_frame(True), proto.estop_frame(False)
    states, rest = proto.estop_states(pressed * 3 + released + released[:4])
    assert states == [True, True, True, False]
    assert rest == released[:4]
    states, rest = proto.estop_states(rest + released[4:])
    assert states == [False] and rest == b""


def test_a_bad_checksum_or_value_and_noise_are_skipped():
    bad = bytearray(proto.estop_frame(True))
    bad[-1] ^= 0xFF
    odd = bytearray(proto.estop_frame(True))
    odd[5] = 2
    odd[6] = proto.checksum(bytes(odd[:6]))
    states, _ = proto.estop_states(b"\x01\x02" + bytes(bad) + bytes(odd) + proto.estop_frame(False))
    assert states == [False]


def _run(port, actions):
    watch = EstopWatch(port)
    stop = threading.Event()
    seen = [watch.state]
    t = threading.Thread(target=watch.run, args=(stop,), daemon=True)
    t.start()
    for act in actions:
        act()
        for _ in range(200):
            if watch.state != seen[-1]:
                break
            threading.Event().wait(0.005)
        seen.append(watch.state)
    stop.set()
    t.join(1)
    return seen


def test_unknown_until_it_changes_then_follows_it():
    port = FakeEstop()
    assert _run(port, [port.press, port.release]) == [UNKNOWN, PRESSED, RELEASED]


def test_a_lost_estop_is_gone_never_a_guess():
    port = FakeEstop()

    def unplug():
        port.gone = True

    assert _run(port, [port.press, unplug]) == [UNKNOWN, PRESSED, GONE]
