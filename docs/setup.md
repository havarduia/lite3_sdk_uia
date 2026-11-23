# Setting up a stock Lite3 Pro

Once per robot. Everything runs on the **perception computer** (Xavier NX,
Ubuntu 20.04, ROS 2 Foxy). Reach it with `ssh ysc@192.168.1.103` on the
robot's network, or `ssh ysc@192.168.2.x` on its own Wi-Fi.

The two computers:

| Computer | Address | What runs there |
|---|---|---|
| motion (RK3588) | 192.168.1.120, or 192.168.2.1 on the robot's Wi-Fi | `jy_exe`, the walking controller; the front camera; the speaker; the sonars |
| perception (Xavier NX) | 192.168.1.103 | ROS 2 Foxy, the bridge to the motion computer, the RealSense depth camera, **this SDK** |

## 1. ROS 2 and the bridge

The vendor's bridge, `transfer_ros2.service`, turns the motion computer's
UDP telemetry into ROS 2 topics and ROS 2 velocity into UDP. Check it runs and
publishes:

```bash
source /opt/ros/foxy/setup.bash
systemctl is-active transfer_ros2.service       # active
ros2 topic hz /robot_state_debug                # ~50-160 Hz
```

The vendor's `Lite_motion` program must **not** run at the same time: it
takes the same UDP port (43897) and the bridge then hears nothing.

## 2. The bridge patch (recommended)

The stock bridge works with the SDK, but leaves out the battery and the front
sonar. `patches/transfer-jetson2motion.patch` adds:

- battery, error and charging to `/robot_state_debug` (without it `bot.battery`
  is `None`, and a flat robot silently refuses to stand),
- the front sonar, `/us_publisher/ultrasound_front`,
- leg odometry scaled by 1.15 (measured: the stock value is short by that much),
- republishing at 50 Hz instead of ~160 Hz, which frees most of a CPU core,
- a fix for every velocity reaching the robot twice.

```bash
cd ~/lite_cog_ros2/transfer
cp src/transfer/src/Jetson2Motion.cpp src/transfer/src/Jetson2Motion.cpp.orig
patch -p1 < ~/lite3_sdk/patches/transfer-jetson2motion.patch
source /opt/ros/foxy/setup.bash
colcon build
sudo systemctl restart transfer_ros2.service
ros2 topic echo --once /robot_state_debug       # now 10 numbers; the 8th is the battery
```

## 3. Install the SDK

```bash
git clone <this repository> ~/lite3_sdk
pip3 install --user --no-build-isolation -e ~/lite3_sdk
```

If pip fails (no network), put the folder on the Python path instead, for
example in `~/.bashrc`:

```bash
export PYTHONPATH=~/lite3_sdk:$PYTHONPATH
```

## 4. The speaker: an SSH key to the motion computer

The speaker is on the motion computer; the SDK plays sound with `ssh` and
`aplay` there, so the perception computer needs a key it can log in with:

```bash
ssh-keygen -t ed25519            # only if ~/.ssh/id_ed25519 does not exist; no passphrase
ssh-copy-id ysc@192.168.1.120
ssh ysc@192.168.1.120 'id -nG | grep -w audio'   # ysc must be in the audio group
python3 -m lite3 clips           # the robot's built-in clips
python3 -m lite3 play OKstandup
```

Sound files (`bot.play_file()`) are converted here, so they need `ffmpeg` on
the perception computer.

## 5. The depth camera service

`walk()` uses the RealSense depth camera to stop for obstacles, so the camera
must be streaming. The SDK starts and stops its service with `sudo -n
systemctl`, allowed without a password by one sudoers entry:

```bash
sudo tee /etc/sudoers.d/lite3-camera > /dev/null <<'END'
ysc ALL=(root) NOPASSWD: /usr/bin/systemctl start realsense_ros2.service, \
                         /usr/bin/systemctl stop realsense_ros2.service
END
sudo chmod 0440 /etc/sudoers.d/lite3-camera && sudo visudo -c
python3 -c "from lite3.depth import camera; print(camera(True))"   # True
python3 -m lite3 scan            # a bar per direction, nearest obstacle
```

`walk(distance, stop_distance=None)` walks without the camera.

## 6. The front camera

The motion computer streams its wide-angle camera at
`rtsp://192.168.1.120:8554/test`. Grab one frame (needs `ffmpeg`):

```bash
python3 -m lite3 frame /tmp/front.jpg
```

## 7. Try it

Clear space ahead, robot lying down, app closed:

```bash
python3 -m lite3 status
python3 ~/lite3_sdk/examples/hello.py
```

## When something does not work

| Symptom | Cause |
|---|---|
| `no /robot_state_debug within 10s` | the bridge is down, or `Lite_motion` holds UDP 43897 (step 1) |
| `battery=-` in `status` | the bridge patch is not applied (step 2) |
| `timed out standing (basic_state=8)` | nothing holds the interlock: use `upright()` or `heartbeat_start()`, or connect the app |
| `no odometry on /leg_odom2` | the bridge is down, or another odometry topic is meant: `Lite3(odom_topic=...)` |
| `no point cloud` | the depth camera is off (step 5) |
| a sound plays nothing | `ysc` is not in the audio group on the motion computer (step 4) |
| standing, but velocity does nothing | posture mode is on; `bot.auto()` leaves it |
