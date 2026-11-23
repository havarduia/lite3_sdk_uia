"""python3 -m lite3 <command> - the robot from the shell.

    python3 -m lite3 status
    python3 -m lite3 stand
    python3 -m lite3 walk 0.5
"""
import sys
import time

from .udp import ACTIONS

USAGE = ('commands: status stand sit auto walk <m> turn <deg> scan action <%s> '
         'estop [--disarm] frame <file.jpg> clips play <clip>' % '|'.join(ACTIONS))


def main(argv=None):
    args = sys.argv[1:] if argv is None else list(argv)
    if not args:
        print(__doc__)
        print(USAGE)
        return None
    cmd = args[0]
    # These need no ROS node.
    if cmd == 'frame':
        from .camera import grab
        print(grab(args[1]))
        return None
    if cmd in ('clips', 'play'):
        from .speaker import Speaker
        print('\n'.join(Speaker().clips()) if cmd == 'clips' else Speaker().play(args[1]))
        return None
    from .robot import Lite3
    with Lite3(auto_mode=(cmd not in ('status', 'scan'))) as bot:
        if cmd == 'status':
            time.sleep(0.5)     # let the sonar and IMU topics arrive
            print(bot.status())
        elif cmd == 'stand':
            bot.stand()
            print(bot.status())
        elif cmd == 'sit':
            bot.sit()
            print(bot.status())
        elif cmd == 'auto':
            bot.auto()
            print('auto mode set')
        elif cmd == 'walk':
            print(bot.walk(float(args[1])))
        elif cmd == 'turn':
            print(bot.turn_deg(float(args[1])))
        elif cmd == 'scan':
            print(bot.scan_text())
        elif cmd == 'estop':
            print('estop:', bot.estop(disarm='--disarm' in args))
            print(bot.status())
        elif cmd == 'action':
            bot.action(args[1])
            print(bot.status())
        else:
            print('unknown command: %s\n%s' % (cmd, USAGE))
    return None
