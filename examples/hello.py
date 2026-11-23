#!/usr/bin/env python3
"""hello.py - stand up, walk one metre, lie down again.

    python3 examples/hello.py      # clear space ahead, the robot lying down

Ctrl-C stops him at once.
"""
from lite3 import Lite3

with Lite3() as bot:
    with bot.upright():            # heartbeat, stand; lies down again on the way out
        print(bot.status())
        print(bot.walk(1.0))
