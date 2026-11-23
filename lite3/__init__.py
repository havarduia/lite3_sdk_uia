"""lite3 - Python SDK for the DEEP Robotics Lite3 Pro.

    from lite3 import Lite3
    with Lite3() as bot:
        with bot.upright():
            bot.walk(1.0)

See README.md and docs/api.md.
"""
from .udp import ACTIONS, Lite3Error     # noqa: F401
