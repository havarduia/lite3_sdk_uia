"""state - what the robot reports on /robot_state_debug, and what the values mean.

The stock bridge sends the first 7 fields; with patches/transfer-jetson2motion.patch
it also sends battery, error and charging.
"""
STATE_FIELDS = ('basic', 'gait', 'policy', 'motion', 'task', 'need_move',
                'zero_flag', 'battery', 'error', 'charging')
STANDING = 6
# Settled lying states: 1 after a commanded lie-down, 8 no interlock, 98 cold boot.
LYING = (1, 8, 98)
# Settled and commandable. 9 is transitional (8 -> 9 -> 1), so it is not here.
READY_STATES = (1, 6, 98)
# Seconds before "still lying" means the toggle was swallowed. Too short
# sends a second toggle at a robot that IS standing up, which lies him down.
ARM_GRACE = 2.0

BATTERY_REFUSE = 20
BATTERY_WARN = 30


def parse(data):
    """The message's numbers as a dict by field name; a shorter message has fewer keys."""
    return dict(zip(STATE_FIELDS, data))
