#!/usr/bin/env python3
"""check_udp_state.py - the message format, the command table and the state parser.

    python3 tests/check_udp_state.py      # no robot, no ROS needed
"""
import os
import struct
import subprocess
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
    assert udp.SOURCES[udp.ACTIONS['moonwalk'][0]] == udp.RECORDED and udp.SOURCES[udp.TRICK_CANCEL] == udp.RECORDED
    assert set(udp.CANCELLABLE) <= set(udp.ACTIONS)
    # the patched bridge sends 10 fields, battery at index 7; the stock bridge sends 7
    assert state.parse([6, 0, 0, 0, 0, 0, 0, 80]) == dict(zip(state.STATE_FIELDS[:8], [6, 0, 0, 0, 0, 0, 0, 80]))
    assert 'battery' not in state.parse([6, 0, 0, 0, 0, 0, 0])
    assert issubclass(lite3.Lite3Error, RuntimeError) and 'moonwalk' in lite3.ACTIONS
    # the parts that need no ROS load without it: rclpy comes in only with Lite3
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    probe = ('import sys; sys.path.insert(0, %r); import lite3, lite3.udp, lite3.state, lite3.speaker, '
             'lite3.camera, lite3.depth; from lite3 import Lite3Error, ACTIONS; '
             'assert "rclpy" not in sys.modules, "rclpy loaded"' % root)
    r = subprocess.run([sys.executable, '-c', probe], stderr=subprocess.PIPE, universal_newlines=True)
    assert r.returncode == 0, r.stderr.strip().splitlines()[-1]
    print('check_udp_state ok')


if __name__ == '__main__':
    main()
