# How the Lite3 Pro behaves

What was found on a real robot while building the SDK, so nobody has to find
it again.

## The hardware

```
                        ┌───────────────────────────────────────────┐
  The app ─────────────►│  MOTION computer  (RK3588, 192.168.1.120) │
  (sticks, heartbeat)   │  jy_exe      the walking controller.      │
                        │              UDP :43893 in, telemetry     │
                        │              out to :43897                │
                        │  front wide-angle camera → RTSP :8554     │
                        │  speaker (ES8388 codec), played by aplay  │
                        │  front + rear ultrasonic sensors          │
                        └──────────────────┬────────────────────────┘
                                   Ethernet │ UDP
                        ┌──────────────────▼────────────────────────┐
                        │  PERCEPTION computer (Xavier NX)          │
                        │  Ubuntu 20.04, ROS 2 Foxy, CycloneDDS     │
  RealSense D435i ─USB─►│  realsense_ros2.service  depth camera     │
  (pitched 20° down)    │  transfer_ros2.service   UDP <-> ROS 2    │
                        │  ► this SDK ◄                             │
                        └───────────────────────────────────────────┘
```

| Thing | Lives on | How the SDK reaches it |
|---|---|---|
| Walking, posture, tricks | motion (`jy_exe`) | short UDP commands (`udp.py`), velocity as `/cmd_vel` through the bridge |
| State, battery, leg odometry, IMU, sonars, the app's sticks | motion, republished on perception | ROS 2 topics from the bridge |
| Depth camera | perception, USB | `/camera/depth/color/points` |
| Front camera | motion | RTSP stream, `ffmpeg` |
| Speaker | motion | `ssh` + `aplay` |

### Bridge topics

| Topic | Contents |
|---|---|
| `/robot_state_debug` | `Int32MultiArray`: basic_state, gait, ... (7 numbers stock; 10 with the patch: + battery, error, charging) |
| `/leg_odom2` | odometry from the legs (×1.15 with the patch) |
| `/imu/data` | body IMU |
| `/us_publisher/ultrasound_distance` | rear sonar |
| `/us_publisher/ultrasound_front` | front sonar (patch only) |
| `/handle_state` | the app's stick positions |

## Two ways in

**Short UDP commands** for posture, mode, heartbeat and tricks: 12 bytes,
`<code:uint32, value:int32, 0>` little-endian, to `192.168.1.120:43893`.

**Velocity** as a ROS 2 `Twist` on `/cmd_vel`; the bridge turns it into UDP.

## The commands and where they came from

Every code the SDK sends, from `lite3.udp.SOURCES`:

| Code | Name | Meaning | Source |
|---|---|---|---|
| `0x21010202` | `STAND_TOGGLE` | stand ↔ lie. **A toggle**: sent to a standing robot it lies him down | recorded from the app |
| `0x21040001` | `HEARTBEAT` | the app's keepalive, needed at ≥ 2 Hz | recorded from the app |
| `0x21010C03` | `MODE_AUTO` | auto mode; without it velocity is silently ignored | recorded from the app |
| `0x21010D05` / `0D06` | `POSTURE_ENTER` / `EXIT` | posture ("twist body") mode | recorded from the app |
| `0x21010130` | `STICK_PITCH` | pitch stick, -32767..32767; in posture mode it tilts the body | recorded from the app |
| `0x21010C02` | `MODE_MANUAL` | manual mode (the app sends it when a stick moves) | third-party |
| `0x31010C05` | `ZERO` | joints to zero | vendor SDK |
| `0x2101030C` | `moonwalk` | trick, from standing; can be cancelled | recorded from the app |
| `0x21010506` | `hello` | trick, from lying; cannot be cancelled | recorded from the app |
| `0x21010204` | `twist` | trick, from standing; can be cancelled | recorded from the app |
| `0x2101020D` | `twist_jump` | trick, from standing; cannot be cancelled | recorded from the app |
| `0x21010C0B` | `TRICK_CANCEL` | stops moonwalk or twist; the app sends value 1 or 0, both work | recorded from the app |
| `0x2101050B` | `long_jump` | trick, from lying; needs 2 m clear | third-party |
| `0x21010205` | `turn_over` | rolls onto his back | third-party |

"Recorded from the app" means captured with `tcpdump` on the perception
computer while pressing the button in the app. "Third-party" is
<https://github.com/infodriver/lite3-sdk>; the vendor SDK is
[Lite3_MotionSDK](https://github.com/DeepRoboticsLab/Lite3_MotionSDK).
Backflip (`0x21010502`) is left out on purpose. `udp.py` also lists known
codes that are not used.

## The state number (`basic`)

| basic | Meaning | Can it be commanded? |
|---|---|---|
| 98 | just powered on | settled, **not armed** (the first command is eaten) |
| 8 | no heartbeat (interlock down) | settled, **not armed** |
| 9 | arming, 8 → 9 → 1 | no, moving |
| 1 | lying down, armed | yes |
| 17, 4, 5, 7 | standing up / lying down | no, moving |
| 6 | standing | yes; the only state that takes velocity |

## Things that surprise

- **Stand and lie are one toggle.** Sending it twice lies him down again, so
  `stand()` looks at the state first and only toggles from a settled lying state.
- **The first stand after power-on does nothing.** The robot uses it to arm
  (98 → 9 → 1). `stand()` waits 2 s, sees he is still lying, and sends it once more.
- **Nothing happens without a heartbeat.** `jy_exe` ignores every network
  command unless the app's keepalive keeps arriving. `Lite3()` sends it from
  Python (`heartbeat_start()`), so the robot then obeys with no app at all.
- **Losing the heartbeat lies him down, about 3.5 s later** (2026-11-24, app off,
  heartbeat stopped). Standing still: 6 → 7 → 8, pitch under 2°, gentle. Walking:
  he kept obeying velocity for those seconds, then lay down 0.3 s after stopping,
  pitched to 23° and started to fall (caught). Do not repeat that test. This is
  why `Lite3()` holds its own heartbeat instead of relying on the app's.
- **Auto mode is not reported.** Nothing says which mode he is in, so the
  motion calls set it every time.
- **Posture mode swallows velocity.** In it he reports standing but ignores
  `/cmd_vel`; `auto()` leaves it first.
- **Low battery: no stand, no reason.** Below 20 % he refuses to stand and
  says nothing. Only the bridge patch makes the battery visible.
- **He coasts.** He turns on for about 0.17 s after a zero command, so
  `turn()` stops that much early.
- **He stops by himself** about 1.5 s (and about 8 cm) after velocity stops
  arriving, for example if the program crashes.
- **He turns at about 0.7 of the commanded rate.**
- **The stock leg odometry is short by about 15 %.** The patch scales it by 1.15.
- **The rear sonar bottoms out at 0.28 m**, which also means "no echo" while
  lying down; about 4.5 m also means no echo.
