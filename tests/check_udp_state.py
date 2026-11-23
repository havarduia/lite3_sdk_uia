#!/usr/bin/env python3
"""check_udp_state.py - the message format, the command table and the state parser.

    python3 tests/check_udp_state.py      # no robot, no ROS needed
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import lite3                       # noqa: E402
from lite3 import state, udp       # noqa: E402


def main():
    # a short command is three little-endian numbers: code, signed value, 0
    assert udp.frame(udp.HEARTBEAT) == struct.pack('<IiI', 0x21040001, 0, 0)
    assert len(udp.frame(udp.STICK_PITCH, -32767)) == 12
    assert struct.unpack('<IiI', udp.frame(udp.STICK_PITCH, -32767))[1] == -32767
    # every command the SDK sends says where it came from
    used = {udp.STAND_TOGGLE, udp.HEARTBEAT, udp.MODE_AUTO, udp.MODE_MANUAL, udp.ZERO,
            udp.POSTURE_ENTER, udp.POSTURE_EXIT, udp.STICK_PITCH}
    assert set(udp.SOURCES) >= used | {c for c, _ in udp.ACTIONS.values()}
    assert udp.SOURCES[udp.STAND_TOGGLE] == 'recorded from the app'
    assert udp.SOURCES[udp.ACTIONS['dance'][0]] == 'third-party, confirmed by recording'
    # the patched bridge sends 10 fields, battery at index 7; the stock bridge sends 7
    assert state.parse([6, 0, 0, 0, 0, 0, 0, 80]) == dict(zip(state.STATE_FIELDS[:8], [6, 0, 0, 0, 0, 0, 0, 80]))
    assert 'battery' not in state.parse([6, 0, 0, 0, 0, 0, 0])
    assert issubclass(lite3.Lite3Error, RuntimeError) and 'dance' in lite3.ACTIONS
    print('check_udp_state ok')


if __name__ == '__main__':
    main()
