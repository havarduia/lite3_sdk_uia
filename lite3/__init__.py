"""lite3 - Python SDK for the DEEP Robotics Lite3 Pro.

    from lite3 import Lite3
    with Lite3() as bot:
        with bot.upright():
            bot.walk(1.0)

See README.md and docs/api.md.
"""
from .udp import ACTIONS, Lite3Error     # noqa: F401


def __getattr__(name):
    # Lite3 loads rclpy, so only when asked for: udp, state, speaker and camera work without ROS.
    if name == 'Lite3':
        from .robot import Lite3
        return Lite3
    raise AttributeError("module 'lite3' has no attribute %r" % name)
