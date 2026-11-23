#!/usr/bin/env python3
"""check_motion.py - what the SDK promises, checked with NO robot.

    python3 tests/check_motion.py     # needs rclpy to import, nothing else

A fake node and a fake clock stand in for the robot (lite3.testing), and no
packet or ROS message leaves this process, so it is safe to run anywhere -
next to a live robot too.
"""
import os
import sys
import threading
import time
import types

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import rclpy                                         # noqa: E402
from lite3 import Lite3, Lite3Error, state, udp      # noqa: E402
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


def main():
    check_posture()
    check_close_order()
    print('check_motion ok')


if __name__ == '__main__':
    main()
