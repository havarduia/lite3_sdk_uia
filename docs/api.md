# The `Lite3` API

```python
from lite3 import Lite3, Lite3Error, ACTIONS
```

`Lite3` is one object with a flat API, put together from small files (mixins)
that share one ROS 2 node:

| File | Adds |
|---|---|
| `robot.py` | the node, the background spin thread, state, `close()`, `estop()`, `status()` |
| `posture.py` | heartbeat, `stand`, `sit`, `upright`, `auto`, `tilt`, `action` |
| `motion.py` | `walk`, `strafe`, `turn`, `turn_deg`, `steer`, `halt` |
| `depth.py` | `scan`, `clearance`, `side_clear`, `wait_cloud`, the camera service |
| `speaker.py`, `camera.py` | sound and front-camera frames |
| `udp.py`, `state.py` | the command codes and what the robot's state numbers mean |

Every refusal raises `Lite3Error` (a `RuntimeError`) with the reason and,
where there is one, what to do about it.

## Starting and stopping

```python
Lite3(auto_mode=True, timeout=10.0, estop_on_sigint=True, odom_topic='/leg_odom2', heartbeat=True)
```

1. Uses CycloneDDS (set before `rclpy` loads; with the default the node finds nothing).
2. Starts a node that listens to the robot's topics, and a thread that reads
   them 25 times a second. Callbacks only keep the newest message, so state is
   plain attributes: no callbacks or spin loops in your code.
3. Ctrl-C runs `estop()` (unless `estop_on_sigint=False`).
4. Waits `timeout` seconds for `/robot_state_debug`, else raises.
5. Starts the heartbeat (unless `heartbeat=False`): the robot then obeys this
   program with no app connected, and Ctrl-C is your stop. Touching an app stick
   still stops him (manual mode). Why always: with only the app's heartbeat, an
   app that goes off mid-walk leaves him walking about 3.5 s and then lying down
   mid-stride, which tipped him 23° and nearly over (2026-11-24, docs/robot.md).
6. Sets auto mode (needed before any velocity).

Use it as a context manager. `close()` halts, drops the heartbeat, and closes
ROS in an order that matters (spin thread, then `destroy_clients()`, then the
node); each step fixed a crash that happened.

| Call | Does |
|---|---|
| `close()` | as above; safe to call twice |
| `estop(disarm=False)` | ignore Ctrl-C meanwhile, `cancel()`, halt. `disarm=True` also drops the heartbeat (untested while standing) |
| `cancel()` | ends any running motion call, from any thread |
| `node` | the ROS node, already spinning: add your own subscriptions |

## State

| Attribute | From | Meaning |
|---|---|---|
| `state` | `/robot_state_debug` | dict: `basic, gait, policy, motion, task, need_move, zero_flag`, and with the bridge patch `battery, error, charging` |
| `basic` | same | the state number everything depends on ([robot.md](robot.md)) |
| `battery` | same | percent, `None` without the patch |
| `standing` | same | `basic == 6` |
| `pose` | `odom_topic` | `(x, y, yaw)` in the odometry frame, or `None` |
| `attitude` | `/imu/data` | `(roll, pitch)` in degrees |
| `ultrasound` | sonar topics | `(front, rear)` in metres; front needs the patch |
| `status()` | all of these | one line |
| `wait_pose(timeout=5)` | | the pose, or raises if odometry is not arriving |
| `pose_settled(window=1.5)` | | True when the pose has not moved for `window` s (standing up jumps it) |

## Posture

| Call | Does |
|---|---|
| `heartbeat_start()` / `heartbeat_stop()` | the 2 Hz keepalive the app normally sends; `Lite3()` starts it. With it the robot obeys **with no app connected**: Ctrl-C is then your stop |
| `wait_ready(timeout=10)` | True once the state is settled and can be commanded |
| `stand()` | idempotent. Refuses below 20 % battery and in moving states. Resends the toggle once if arming swallowed it |
| `sit()` | halt, lie down. Idempotent |
| `upright(heartbeat=True)` | context manager: heartbeat (if not already on), wait, stand; lies down on every way out |
| `auto()` | leave posture mode, set auto mode. The motion calls do it for you |
| `tilt(pitch_deg)` | context manager: body pitched nose up (negative: down) while standing still; about 14° at most |
| `action(name)` | a built-in trick from `ACTIONS`, named as in the app; does not stand or sit for you. Sent once, returns at once |
| `gait(name)` | the app's gait buttons: `slow`, `medium`, `fast` (Flat) or `general`, `grip`, `h_step` (Rug). Waits until the robot reports it, else raises. The app's Flat is `slow`, its Rug is `general` |
| `gait_name` | the mode he reports now, or `None` |
| `crawl_toggle()` | the app's NRM/Crawl button. Blind: flips the mode, and nothing reports which is on. Slow mode only, refused otherwise |
| `cancel_trick()` | stops a running `moonwalk` or `twist` (`udp.CANCELLABLE`); the others cannot be stopped, in the app either |

## Motion

All motion goes through one loop. Before moving: battery, standing, auto
mode, odometry. Every 50 ms: the app's sticks moved, `cancel()`, odometry
stale (older than 0.5 s), body tilted past 30°. On every way out, also a crash:
20 zero commands. A 30 s ceiling on every call. The result is a dict with
`reason`, `start`, `end`, and `moved` or `turned`. Stopping short is normal,
not an error: read `reason`.

| Call | Does |
|---|---|
| `walk(distance, speed=0.15, stop_distance=0.6)` | metres, negative is backward. Refuses to start, or stops, for anything the depth camera sees within `stop_distance` (`None`: no camera). Backward is never guarded |
| `strafe(distance, speed=0.15)` | sideways, positive is left. Not guarded |
| `turn(radians, rate=0.4, guard=True)` | in place, positive is left, any size. Stops early for his coast and reports where he settled. With `guard`, asks `turn_blocked()` first and stops for something on the side he turns to |
| `turn_deg(degrees, ...)` | the same in degrees; adds `turned_deg` |
| `turn_blocked(radians)` | a reason not to start a turn, or `None`. Always `None` here; override it if you have a map |
| `steer(control, limit=30)` | `control()` every 50 ms returns `(vx, wz)` or a string to stop. Clamped to 0.6 m/s and 1.6 rad/s |
| `halt()` | 20 zero commands over 0.4 s |

Speeds over 0.6 m/s, or zero, are refused. A NaN velocity is sent as zero.
`force=True` on a motion call skips the battery check.

## Depth camera

The RealSense D435i, pitched 20° down. Points are turned into the body frame,
the floor and anything above his back are dropped, and the rest is binned by
direction, ±45° ahead.

| Call | Does |
|---|---|
| `scan(fov_deg=45, bin_deg=5)` | `[(bearing_deg, range_m or None), ...]`, positive is left |
| `clearance(half_width=0.225)` | the nearest thing in his path, metres, or `inf` |
| `side_clear()` | `(left, right)` nearest, for turns |
| `scan_text()` | the scan as text bars |
| `wait_cloud(timeout=10)` / `cloud_fresh()` | the newest cloud, or raises |
| `lite3.depth.camera(on=None)` | start (True), stop (False) or look at the camera service |

## Sound and front camera

| Call | Does |
|---|---|
| `play(name)` | one of the robot's built-in clips, any case |
| `play_file(path)` | any audio file, converted with `ffmpeg` here |
| `speaker` | the `lite3.speaker.Speaker`: `clips()`, `play`, `play_file`, `stream` |
| `lite3.speaker.VOLUME` | 0.0 - 1.0 for files and streams |
| `lite3.camera.grab(path)` | one front-camera frame to a `.jpg` |
| `lite3.camera.mjpeg_command(...)` | an `ffmpeg` command line for the stream as JPEGs |

## Building on it

```python
from lite3 import Lite3
from lite3.robot import RobotNode

class MyNode(RobotNode):            # listen to more topics
    def __init__(self, odom_topic):
        super().__init__(odom_topic)
        ...

class MyRobot(Lite3):
    node_class = MyNode
    def turn_blocked(self, radians):  # e.g. look at a costmap
        ...
    def estop(self, disarm=False):    # stop your own things first
        ...
        return super().estop(disarm)
```

`destroy_clients()` is where a subclass closes ROS clients of its own.

## Testing without a robot

`lite3.testing` has a fake robot: a pose that follows the commanded velocity
on a clock that only moves when the code sleeps. Nothing leaves the process.

```python
from lite3 import Lite3
from lite3.testing import PACKETS, run
result, halted, twists = run(lambda bot: bot.walk(0.5, stop_distance=None), Lite3)
assert result['reason'] == 'target reached' and halted
```

## Command line

```
python3 -m lite3 status | stand | sit | auto | walk <m> | turn <deg> | scan |
                 action <name> (Enter or Ctrl-C cancels) | gait <name> | crawl | estop [--disarm] | frame <file.jpg> | clips | play <clip>
```
