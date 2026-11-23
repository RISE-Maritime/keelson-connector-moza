"""The Linux input event codes this connector uses, from <linux/input-event-codes.h>.

They are spelled out here, not taken from ``evdev.ecodes``, so the pure parts of the
connector (the model, the profile, the fake device) import on macOS, where evdev cannot
be built. ``device.EvdevDevice`` checks every value against ``evdev.ecodes`` on open,
so a mismatch fails loudly instead of reading the wrong axis.
"""

EV_SYN = 0x00
EV_KEY = 0x01
EV_ABS = 0x03
EV_FF = 0x15

ABS = {
    "ABS_X": 0x00,
    "ABS_Y": 0x01,
    "ABS_Z": 0x02,
    "ABS_RX": 0x03,
    "ABS_RY": 0x04,
    "ABS_RZ": 0x05,
    "ABS_THROTTLE": 0x06,
    "ABS_RUDDER": 0x07,
    "ABS_WHEEL": 0x08,
    "ABS_GAS": 0x09,
    "ABS_BRAKE": 0x0A,
    "ABS_HAT0X": 0x10,
    "ABS_HAT0Y": 0x11,
}
ABS_NAMES = {v: k for k, v in ABS.items()}

FF_CONSTANT = 0x52
FF_PERIODIC = 0x51
FF_SQUARE = 0x58
FF_SPRING = 0x53
FF_DAMPER = 0x55
FF_GAIN = 0x60
FF_AUTOCENTER = 0x61

EV_NAMES = {"EV_SYN": EV_SYN, "EV_KEY": EV_KEY, "EV_ABS": EV_ABS, "EV_FF": EV_FF}
