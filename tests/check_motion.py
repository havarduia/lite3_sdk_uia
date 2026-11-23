#!/usr/bin/env python3
"""check_motion.py - what the SDK promises, checked with NO robot.

    python3 tests/check_motion.py     # needs rclpy to import, nothing else

A fake node and a fake clock stand in for the robot (lite3.testing), and no
packet or ROS message leaves this process, so it is safe to run anywhere -
next to a live robot too.
"""
import os
import subprocess
import sys
import threading
import time
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy                                         # noqa: E402
from lite3 import Lite3, Lite3Error, depth, motion as M, state, udp      # noqa: E402
from lite3.testing import PACKETS, World, run        # noqa: E402


def boom():
    raise ZeroDivisionError


class Arming(World):
    """A robot fresh from boot: the first stand toggle only arms him, the second stands him up."""
    def on_packet(self, code, value):
        if code == udp.STAND_TOGGLE and PACKETS.count((udp.STAND_TOGGLE, 0)) == 2:
            self.state = dict(self.state, basic=state.STANDING)


def check_posture():
    r, _, _ = run(lambda b: b.stand(), Lite3, Arming, state={'basic': 1})     # stock bridge: no battery key
    assert r is True and PACKETS.count((udp.STAND_TOGGLE, 0)) == 2, (r, PACKETS)
    r, _, _ = run(lambda b: b.stand(), Lite3, state={'basic': 1, 'battery': 10})
    assert isinstance(r, Lite3Error) and 'battery' in str(r) and not PACKETS, (r, PACKETS)
    r, _, _ = run(lambda b: b.stand(), Lite3)                                  # already standing: nothing sent
    assert r is True and not PACKETS, PACKETS
    r, _, _ = run(lambda b: b.status(), Lite3, state={'basic': 6})
    assert 'battery=-' in r and 'standing' in r and 'nav2' not in r, r

    # no odometry: refused before anything moves, and the error names the topic it listened to
    r, _, sent = run(lambda b: b.walk(1.0, stop_distance=None), Lite3, odom=None)
    assert isinstance(r, Lite3Error) and '/leg_odom2' in str(r) and 'lio_relay' not in str(r) and not sent, r
    # estop(): cancels whatever runs (motion calls watch _stop_at) and leaves him halted
    r, halted, _ = run(lambda b: (time.sleep(1.0), b.estop(), b._stop_at)[2], Lite3)
    assert r == 1.0 and halted, r

    # upright(): armed BEFORE waiting for ready, and sat down on the way out even after a crash
    calls = []

    class Bot(Lite3):
        pass
    for step in ('heartbeat_start', 'stand', 'sit'):
        setattr(Bot, step, lambda self, step=step: calls.append(step))
    Bot.wait_ready = lambda self: calls.append('wait_ready') or True

    def crash(b):
        with b.upright():
            boom()
    r, _, _ = run(crash, Bot)
    assert isinstance(r, ZeroDivisionError) and calls == ['heartbeat_start', 'wait_ready', 'stand', 'sit'], calls


def check_close_order():
    """close(): the spin thread is joined, then destroy_clients(), then the node is destroyed.
    Any other order has crashed rclpy on the robot (segfault, InvalidHandle)."""
    order = []

    class Closing(Lite3):
        def destroy_clients(self):
            order.append('clients')

    bot = Closing.__new__(Closing)
    nop = types.SimpleNamespace(shutdown=lambda timeout_sec=None: None, remove_node=lambda n: None)
    bot._closed, bot._prev_sigint = False, None
    bot._hb_stop, bot._hb_thread, bot._stop_spin = threading.Event(), None, threading.Event()
    bot._thread = types.SimpleNamespace(join=lambda timeout=None: order.append('join'))
    bot._exec = bot._cloud_exec = nop
    bot._node = types.SimpleNamespace(
        pub=types.SimpleNamespace(publish=lambda t: None),
        cloud_node=types.SimpleNamespace(destroy_node=lambda: None),
        destroy_node=lambda: order.append('node'))
    real = rclpy.shutdown
    rclpy.shutdown = lambda: None
    try:
        bot.close()
    finally:
        rclpy.shutdown = real
    assert order == ['join', 'clients', 'node'], order


def check_motion():
    # every way out of a motion call leaves him stopped, and says why
    r, halted, _ = run(lambda b: b.walk(0.5, stop_distance=None), Lite3)
    assert r['reason'] == 'target reached' and 0.5 <= r['moved'] < 0.55 and halted, r
    r, halted, _ = run(lambda b: b.turn_deg(270, guard=False), Lite3)   # past 180: counted, not wrapped
    assert r['reason'] == 'target reached' and 260 < r['turned_deg'] < 275 and halted, r
    # the SDK knows no map: turn_blocked says nothing, so a guarded turn turns
    r, halted, sent = run(lambda b: b.turn_deg(90), Lite3)
    assert r['reason'] == 'target reached' and halted and sent, r
    assert Lite3.turn_blocked(None, 1.0) is None
    r, halted, sent = run(lambda b: b.steer(lambda: (9.0, -9.0), limit=1.0), Lite3)
    assert r['reason'] == 'time limit' and halted, r
    assert sent[0] == (M.MAX_SPEED, 0.0, -M.MAX_YAW_RATE), sent[0]
    r, halted, _ = run(lambda b: b.steer(lambda: (0.1, 0.0), limit=999), Lite3)    # HARD_TIMEOUT caps it
    assert r['reason'] == 'time limit' and r['moved'] < 0.1 * (M.HARD_TIMEOUT + 1) and halted, r
    r, halted, _ = run(lambda b: b.steer(lambda: 'seen enough'), Lite3)
    assert r['reason'] == 'seen enough' and halted, r
    r, halted, _ = run(lambda b: b.steer(boom), Lite3)                  # a crash still ends stopped
    assert isinstance(r, ZeroDivisionError) and halted, r
    r, halted, _ = run(lambda b: b.steer(lambda: (0.2, 0.0)), Lite3, at=(2.0, lambda w: setattr(w, 'stick_time', w.t)))
    assert r['reason'].startswith('handheld took over') and halted, r
    r, _, sent = run(lambda b: b.walk(0.5, stop_distance=None), Lite3, state={'basic': 1, 'battery': 80})
    assert isinstance(r, Lite3Error) and not sent, r           # lying: refused, nothing sent
    r, _, sent = run(lambda b: b.steer(lambda: (float('nan'), float('nan')), limit=1.0), Lite3)
    assert set(sent) == {(0.0, 0.0, 0.0)}, sent[0]               # a NaN is a stop, not full speed
    for bad in (lambda b: b.strafe(0.5, speed=0), lambda b: b.strafe(0.5, speed=9), lambda b: b.turn(1.0, rate=0)):
        r, _, sent = run(bad, Lite3)
        assert isinstance(r, Lite3Error) and not sent, r       # refused, not a ZeroDivisionError
    r, halted, _ = run(lambda b: b.walk(5.0, stop_distance=None), Lite3, tilt=(0.0, 0.0),
                       at=(1.0, lambda w: setattr(w, 'tilt', (35.0, 0.0))))
    assert r['reason'].startswith('body tilted') and halted, r          # tipping over stops him

    # odometry dies mid-walk: he stops, he does not walk on blind
    r, halted, _ = run(lambda b: b.walk(5.0, speed=0.5, stop_distance=None), Lite3,
                       at=(1.0, lambda w: setattr(w, 'odom_stopped', w.t)))
    assert r['reason'].startswith('odometry stopped') and halted and r['moved'] < 1.5, r
    r, _, sent = run(lambda b: b.walk(1.0, stop_distance=None), Lite3, odom_stopped=-10.0)
    assert isinstance(r, Lite3Error) and not sent, r           # and does not set off without it

    # a depth guard reads the cloud at 4 Hz however often the loop asks, and still stops him
    reads = []
    r, halted, _ = run(lambda b: setattr(b, 'clearance', lambda: reads.append(b._node.t) or 3.0 - b._node.odom[0])
                       or b.walk(5.0, speed=0.5, stop_distance=0.6), Lite3)
    assert r['reason'].startswith('obstacle at') and 2.2 < r['moved'] < 2.6 and halted, r
    assert all(b - a >= 0.25 - 1e-9 for a, b in zip(reads[1:], reads[2:])), reads    # reads[0] is the pre-flight
    # a camera that stopped sending: its last picture is not "clear", he does not set off on it
    r, _, sent = run(lambda b: b.walk(1.0), Lite3, cloud=object(), cloud_time=-5.0)
    assert isinstance(r, Lite3Error) and 'point cloud' in str(r) and 'robot.protocol' not in str(r) and not sent, r
    # cancel() from another thread ends any motion
    r, halted, _ = run(lambda b: b.walk(3.0, stop_distance=None), Lite3, at=(1.0, lambda w: w.bot.cancel()))
    assert r['reason'].startswith('cancelled') and halted and r['moved'] < 0.5, r


def check_camera_service():
    """The depth camera's systemd service, through the sudoers entry only."""
    calls, real = [], subprocess.run

    def fake(cmd, **kw):
        calls.append(cmd)
        return subprocess.CompletedProcess(cmd, 1 if cmd[0] == 'sudo' else 0, stderr='a password is required')
    subprocess.run = fake
    try:
        try:
            depth.camera(True)
            assert False, 'a failed sudo must raise'
        except Lite3Error as e:
            assert 'sudoers' in str(e), e
        assert calls[0] == ['sudo', '-n', '/usr/bin/systemctl', 'start', 'realsense_ros2.service'], calls
        del calls[:]
        assert depth.camera() is True and calls == [['systemctl', 'is-active', '--quiet', 'realsense_ros2.service']]
    finally:
        subprocess.run = real


def main():
    check_posture()
    check_close_order()
    check_motion()
    check_camera_service()
    print('check_motion ok')


if __name__ == '__main__':
    main()
