# lite3_sdk_uia: Python for the DEEP Robotics Lite3 Pro

Move the robot and read its state from a few lines of Python, on the robot's
own perception computer, with nothing but the stock robot.

```python
from lite3 import Lite3

with Lite3() as bot:
    with bot.upright():          # heartbeat, stand up; lies down again on the way out
        print(bot.battery, bot.ultrasound)
        bot.walk(1.0)            # metres, stops for obstacles the depth camera sees
        bot.turn_deg(90)         # positive is left
```

## Five-minute start

On the perception computer (`ssh ysc@192.168.1.103`), once the one-time
[setup](docs/setup.md) is done:

```bash
git clone https://github.com/havarduia/lite3_sdk_uia.git ~/lite3_sdk
pip3 install --user --no-build-isolation -e ~/lite3_sdk
source /opt/ros/foxy/setup.bash
python3 -m lite3 status          # one line: state, battery, pose, sonars
python3 ~/lite3_sdk/examples/hello.py   # stands, walks one metre, lies down
```

`--no-build-isolation` matters: without it pip tries to download build tools,
and the robot has no package mirror.

## Safety

- **Ctrl-C is the stop button.** While your program holds the heartbeat
  (`upright()` or `heartbeat_start()`) the robot obeys it with no app
  connected. Ctrl-C runs `estop()`: halt, then the program ends.
- If the program crashes or is killed while walking, the robot stops by
  itself about 1.5 s after the last velocity command.
- Every motion call stops on its own for: the distance or angle reached, an
  obstacle, the app's sticks moving, odometry stopping, the body tilting past
  30°, and a 30 s ceiling. The result's `reason` says which.
- Only one program may drive the robot at a time.

## More

| | |
|---|---|
| [docs/setup.md](docs/setup.md) | one-time setup of a stock robot |
| [docs/api.md](docs/api.md) | every call, what it checks, what it raises |
| [docs/robot.md](docs/robot.md) | how the robot behaves, the command codes and where they came from, the hardware |
| `python3 -m lite3` | the command line |
| `tests/` | checks that need no robot: `python3 tests/check_motion.py` |

Split from [lite3_stack_uia](https://github.com/havarduia/lite3_stack_uia) (then `lite3_api`, commit `45be46a`), University of Agder, 2026.
