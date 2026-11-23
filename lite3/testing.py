"""testing - a fake robot, so motion code can be checked with no robot.

    from lite3 import Lite3
    from lite3.testing import PACKETS, run
    result, halted, twists = run(lambda bot: bot.walk(0.5, stop_distance=None), Lite3)

World stands in for the ROS node: a pose that follows the last Twist exactly,
on a clock that only sleep() moves. Nothing leaves the process.
"""
import math
import time

from . import state as S
from . import udp

PACKETS = []     # every (code, value) sent over UDP during the last run()


class World:
    """at=(seconds, fn(world)) is something that happens on the way."""
    state = {'basic': S.STANDING, 'battery': 80}
    cloud, tilt, stick_time, at = None, (0.0, 0.0), -1.0, None
    cloud_time = 0.0
    us_front = us_rear = None
    odom_time = property(lambda self: self.odom_stopped or self.t)     # fresh until it stops
    odom_stopped = None

    def __init__(self, **setup):
        self.t, self.odom, self.cmd, self.sent, self.pub = 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), [], self
        vars(self).update(setup)

    def on_packet(self, code, value):
        """Called for every UDP packet; override to make the robot react."""

    def publish(self, twist):
        self.cmd = (twist.linear.x, twist.linear.y, twist.angular.z)
        self.sent.append(self.cmd)

    def sleep(self, dt):
        if self.odom is not None:
            (x, y, yaw), (vx, vy, wz) = self.odom, self.cmd
            self.odom = (x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt,
                         y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt,
                         math.atan2(math.sin(yaw + wz * dt), math.cos(yaw + wz * dt)))
        self.t += dt
        if self.at and self.t >= self.at[0]:
            happen, self.at = self.at[1], None
            happen(self)


def run(move, bot_class, world_class=World, **setup):
    """move(bot) on a fresh fake robot -> (its result or exception, was he left
    halted, every Twist sent)."""
    w, bot = world_class(**setup), bot_class.__new__(bot_class)
    w.bot, bot._node = bot, w
    del PACKETS[:]

    def send(code, value=0):
        PACKETS.append((code, value))
        w.on_packet(code, value)

    real = time.monotonic, time.sleep, udp.send     # the motion code times everything on the monotonic clock
    time.monotonic, time.sleep, udp.send = (lambda: w.t), w.sleep, send
    try:
        r = move(bot)
    except Exception as e:
        r = e
    finally:
        time.monotonic, time.sleep, udp.send = real
    return r, set(w.sent[-20:]) == {(0.0, 0.0, 0.0)}, w.sent
