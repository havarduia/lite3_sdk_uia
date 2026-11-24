"""udp - the short commands the walking controller (jy_exe) takes over UDP.

A command is 12 bytes, three little-endian numbers: code (uint32), value
(int32, signed: stick axes go negative) and 0. Where each code came from is in
SOURCES; how they were found is in docs/robot.md.
"""
import socket
import struct

MOTION_IP = '192.168.1.120'          # the motion computer; 192.168.2.1 on the robot's own Wi-Fi
MOTION_ADDR = (MOTION_IP, 43893)     # jy_exe: every motion command


class Lite3Error(RuntimeError):
    """Refused for a reason the caller can act on."""


STAND_TOGGLE = 0x21010202   # stand <-> lie, a TOGGLE
MODE_AUTO = 0x21010C03      # needed before any velocity
MODE_MANUAL = 0x21010C02    # the app sends this when a stick moves
ZERO = 0x31010C05           # reset joints to zero / init pose
HEARTBEAT = 0x21040001      # the app's keepalive, >= 2 Hz
# Posture ("twist body") mode: the sticks set the body attitude,
# -32767..32767 at 10 Hz. NEGATIVE pitch is NOSE UP, full scale ~14 deg.
POSTURE_ENTER, POSTURE_EXIT = 0x21010D05, 0x21010D06
STICK_PITCH = 0x21010130
# STICK_ROLL = 0x21010131

# Built-in tricks: name -> (code, posture it starts from), named as in the app.
# backflip (0x21010502) is left out on purpose.
ACTIONS = {
    'hello':      (0x21010506, 'lie'),
    'moonwalk':   (0x2101030C, 'stand'),
    'twist':      (0x21010204, 'stand'),
    'twist_jump': (0x2101020D, 'stand'),
    'long_jump':  (0x2101050B, 'lie'),     # manual: needs 2 m clear ahead
    'turn_over':  (0x21010205, 'lie'),     # rolls onto its back
}
# The app's cancel, recorded 2026-11-24: sent once per press, value 1 one time and
# 0 the next; both stopped the trick. Sending 1 twice in a row did not (see cancel_trick). Only these tricks offer it in the app.
# The app sends each trick once too.
TRICK_CANCEL = 0x21010C0B
CANCELLABLE = ('twist', 'moonwalk')

RECORDED = 'recorded from the app'
THIRD_PARTY = 'third-party'                # https://github.com/infodriver/lite3-sdk
VENDOR = 'vendor SDK'                      # Lite3_MotionSDK
SOURCES = {
    STAND_TOGGLE: RECORDED, HEARTBEAT: RECORDED, MODE_AUTO: RECORDED,
    POSTURE_ENTER: RECORDED, POSTURE_EXIT: RECORDED, STICK_PITCH: RECORDED,
    MODE_MANUAL: THIRD_PARTY, ZERO: VENDOR, TRICK_CANCEL: RECORDED,
}
SOURCES.update({code: THIRD_PARTY for code, _ in ACTIONS.values()})
SOURCES.update({ACTIONS[n][0]: RECORDED for n in ('moonwalk', 'hello', 'twist', 'twist_jump')})

# Known but unused, kept so nobody has to dig them up again:
#   0x21010300 / 0x21010307 / 0x21010406  gait OR body height - sources
#       disagree, capture from the app before use
#   0x21010303  fast gait; 0x21010402 body up; 0x21010201 stance   COMMUNITY
#   0x0111 / 0x0113 / 0x0114  low-level joint control - BYPASSES the stock
#       controller, loses control after 5 ms without a 0x0111 frame  OFFICIAL
#   telemetry to UDP 43897: 0x0901 robot state, 0x0902 joints, 0x0905
#       app sticks. The ROS 2 bridge owns that port and republishes it.


def frame(code, value=0):
    """The 12 bytes of one short command."""
    return struct.pack('<IiI', code, int(value), 0)


def send(code, value=0):
    """One short command to jy_exe."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.sendto(frame(code, value), MOTION_ADDR)
    finally:
        s.close()
