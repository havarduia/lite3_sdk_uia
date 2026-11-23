"""robot - the Lite3 class: the ROS node, the spin thread, state, close().

State is live attributes (bot.battery, bot.pose, bot.standing). Motion calls
block and always leave a zero Twist behind. What the SDK enforces, and why, is
in docs/api.md.
"""
import math
import os
import signal
import threading
import time
from contextlib import contextmanager
os.environ.setdefault('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

import rclpy                                           # noqa: E402
from geometry_msgs.msg import Twist                    # noqa: E402
from nav_msgs.msg import Odometry                      # noqa: E402
from rclpy.executors import (ShutdownException,        # noqa: E402
                             SingleThreadedExecutor, TimeoutException)
from rclpy.node import Node                            # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy    # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2           # noqa: E402
from std_msgs.msg import Float64, Int32MultiArray      # noqa: E402

from .depth import Depth                               # noqa: E402
from .motion import Motion, dist                       # noqa: E402
from .posture import Posture                           # noqa: E402
from .state import STANDING, parse                     # noqa: E402
from .udp import Lite3Error                            # noqa: E402

# The robot's topics come 160 times a second each. Taking every message cost
# most of a core, so each queue keeps only the newest and is read this often.
SPIN_HZ = 25.0           # pose, tilt and state are at most 40 ms old; the motion loop runs at 20 Hz
SPIN_BURST = 12          # at most this many callbacks per read: one per subscription and a few to spare
# The point cloud is megabytes a frame and unpacking it is what costs: 30 a
# second took half a core. The depth checks look 4 times a second.
CLOUD_HZ = 10.0
# The vendor bridge's leg odometry. Older than ODOM_STALE_S: stop.
DEFAULT_ODOM_TOPIC = '/leg_odom2'
ODOM_STALE_S = 0.5
# An app stick past this (axes are -1..1) means a person has taken over.
HANDHELD_DEADBAND = 0.1


def take_ready(executor, most=SPIN_BURST):
    """Run the callbacks that are ready now, at most `most`, and return. Not
    spin_once(timeout_sec=0) in a loop: each of those that finds nothing
    still pays for a whole wait, and the waits are what cost."""
    for _ in range(most):
        try:
            handler, _, _ = executor.wait_for_ready_callbacks(timeout_sec=0)
        except (TimeoutException, ShutdownException):
            return
        handler()


def _set_sigint(handler):
    """Install a SIGINT handler and return the one it replaced - or None off
    the main thread, where signals cannot be set."""
    try:
        return signal.signal(signal.SIGINT, handler)
    except ValueError:
        return None


@contextmanager
def ignoring_sigint():
    """Ctrl-C does nothing inside: a second one must not cut an emergency stop short."""
    prev = _set_sigint(signal.SIG_IGN)
    try:
        yield
    finally:
        if prev is not None:
            _set_sigint(prev)


class RobotNode(Node):
    """The topics of the stock robot. Subclass it (Lite3.node_class) to listen to more."""

    def __init__(self, odom_topic=DEFAULT_ODOM_TOPIC):
        super().__init__('lite3_api')
        best = QoSProfile(depth=1)
        best.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(Twist, 'cmd_vel', best)
        self.state = None
        self.odom = None
        self.odom_time = 0.0        # time.monotonic() of the newest odometry
        self.cloud, self.cloud_time = None, 0.0     # the newest, and its time.monotonic()
        self.us_front = self.us_rear = None
        self.tilt = None            # (roll, pitch) in degrees
        self.stick_time = 0.0       # last time an app stick was pushed
        self.create_subscription(Int32MultiArray, '/robot_state_debug',
                                 self._state_cb, 1)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 1)
        # On a node of its own so it can be read at its own, slower, rate.
        self.cloud_node = Node('lite3_api_cloud')
        self.cloud_node.create_subscription(PointCloud2, '/camera/depth/color/points',
                                            self._cloud_cb, best)
        # The rear one bottoms out at 0.28 m, which lying down also means "no echo":
        # a reading, not a guard.
        self.create_subscription(Float64, '/us_publisher/ultrasound_distance',
                                 lambda m: setattr(self, 'us_rear', m.data), 1)
        self.create_subscription(Float64, '/us_publisher/ultrasound_front',
                                 lambda m: setattr(self, 'us_front', m.data), 1)
        self.create_subscription(Imu, '/imu/data', self._imu_cb, 1)
        self.create_subscription(Twist, '/handle_state', self._handle_cb, best)

    def _state_cb(self, m):
        self.state = parse(m.data)

    def _cloud_cb(self, m):
        self.cloud, self.cloud_time = m, time.monotonic()

    def _odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.odom = (p.x, p.y, yaw)
        self.odom_time = time.monotonic()

    def _imu_cb(self, m):
        q = m.orientation
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        self.tilt = (math.degrees(roll), math.degrees(pitch))

    def _handle_cb(self, m):
        if max(abs(m.linear.x), abs(m.linear.y),
               abs(m.angular.z)) > HANDHELD_DEADBAND:
            self.stick_time = time.monotonic()


class Lite3(Posture, Motion, Depth):
    """Live handle on the robot. Use as a context manager."""
    node_class = RobotNode
    odom_topic = DEFAULT_ODOM_TOPIC
    _stop_at = 0.0           # time.monotonic() of the last cancel(); motion calls started before it stop

    def __init__(self, auto_mode=True, timeout=10.0, estop_on_sigint=True,
                 odom_topic=DEFAULT_ODOM_TOPIC):
        rclpy.init(args=None)
        self.odom_topic = odom_topic
        self._node = self.node_class(odom_topic)
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._cloud_exec = SingleThreadedExecutor()
        self._cloud_exec.add_node(self._node.cloud_node)
        self._stop_spin = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        self._closed = False
        self._hb_stop = threading.Event()
        self._prev_sigint = _set_sigint(self._on_sigint) if estop_on_sigint else None
        if not self._wait(lambda: self._node.state is not None, timeout):
            self.close()
            raise Lite3Error(
                'no /robot_state_debug within %.0fs. Is transfer_ros2.service '
                'running, and is Lite_motion stopped? (They fight over UDP '
                '43897.)' % timeout)
        if auto_mode:
            self.auto()

    @property
    def node(self):
        """The ROS node, already spinning: add your own subscriptions to it."""
        return self._node

    # --- emergency stop ---
    def _on_sigint(self, signum, frame):
        print('\n*** Ctrl-C - EMERGENCY STOP ***')
        self.estop()
        raise KeyboardInterrupt

    def cancel(self):
        """End any motion call, from any thread: it halts and returns 'cancelled'."""
        self._stop_at = time.monotonic()

    def estop(self, disarm=False):
        """Stop as hard as software can: ignore Ctrl-C meanwhile, cancel, halt.

        disarm=True also drops the heartbeat. UNTESTED while standing, so opt-in.
        """
        with ignoring_sigint():
            self.cancel()
            self.halt()
            if disarm:
                self.heartbeat_stop()
        return True

    # --- plumbing ----------------------------------------------------------
    def _spin(self):
        cloud_due = 0.0
        while not self._stop_spin.is_set():
            tick = time.monotonic()
            take_ready(self._exec)
            if tick >= cloud_due:
                cloud_due = tick + 1.0 / CLOUD_HZ
                take_ready(self._cloud_exec, 1)
            self._stop_spin.wait(max(0.0, 1.0 / SPIN_HZ - (time.monotonic() - tick)))

    @staticmethod
    def _wait(pred, timeout, period=0.05):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(period)
        return pred()

    def destroy_clients(self):
        """Hook for subclasses with ROS clients of their own (Nav2 actions): close()
        calls it after the spin thread stops and before the node goes."""

    def close(self):
        if self._closed:
            return
        self._closed = True
        if self._prev_sigint is not None:
            _set_sigint(self._prev_sigint)
        # Halt BEFORE dropping the keepalive: what a standing robot does when it
        # disarms has never been observed.
        try:
            self.halt()
        except Exception:
            pass
        self.heartbeat_stop()
        # Order is load-bearing: the spin thread first (else segfault), then any
        # client while the node lives (else InvalidHandle), then the rest.
        self._stop_spin.set()
        self._thread.join(timeout=5.0)
        try:
            self.destroy_clients()
        except Exception:
            pass
        try:
            self._exec.shutdown(timeout_sec=2.0)
            self._exec.remove_node(self._node)
            self._cloud_exec.shutdown(timeout_sec=2.0)
            self._node.cloud_node.destroy_node()
            self._node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    # --- state -------------------------------------------------------------
    @property
    def state(self):
        """dict of state.STATE_FIELDS: basic, gait, policy, motion, task, need_move,
        zero_flag, and battery, error and charging with the bridge patch."""
        return dict(self._node.state or {})

    @property
    def basic(self):
        """basic_state, the one that gates everything, or None."""
        return self.state.get('basic')

    @property
    def battery(self):
        """Percent, or None on a bridge without the patch."""
        return self.state.get('battery')

    @property
    def standing(self):
        return self.basic == STANDING

    @property
    def ultrasound(self):
        """(front, rear) ultrasonic range in metres, None where not heard."""
        return (self._node.us_front, self._node.us_rear)

    @property
    def attitude(self):
        """(roll, pitch) of the body in degrees from the robot's IMU, or None."""
        return self._node.tilt

    @property
    def pose(self):
        """(x, y, yaw) in the odometry frame, or None."""
        return self._node.odom

    def wait_pose(self, timeout=5.0):
        if not self._wait(self._odom_fresh, timeout):
            raise Lite3Error('no odometry on %s within %.0fs - refusing to move blind'
                             % (self.odom_topic, timeout))
        return self._node.odom

    def _odom_fresh(self):
        n = self._node
        return n.odom is not None and time.monotonic() - n.odom_time < ODOM_STALE_S

    def pose_settled(self, window=1.5, tol=0.02):
        """True if odometry has not moved for `window` seconds. Standing up jumps
        the pose by over a metre, so anything building a world model waits for this.
        """
        self.wait_pose()
        a = self.pose
        time.sleep(window)
        b = self.pose
        return dist(a, b) <= tol

    # --- summary -----------------------------------------------------------
    def status(self):
        s = self.state
        p = self.pose
        f, r = self.ultrasound
        t = self.attitude
        b = s.get('battery')
        return ('basic=%s gait=%s battery=%s ' % (s.get('basic'), s.get('gait'),
                                                  '-' if b is None else '%d%%' % b)
                + ('error=%s ' % s['error'] if s.get('error') else '')
                + ('charging ' if s.get('charging') else '')
                + ('standing' if self.standing else 'not standing')
                + (' pose=(%.2f, %.2f, %.0f deg)' % (p[0], p[1], math.degrees(p[2]))
                   if p else ' pose=unknown')
                + ' sonar front=%s rear=%s' % tuple(
                    '-' if v is None else '%.2f' % v for v in (f, r))
                + (' tilt=(%.0f, %.0f) deg' % t if t else ''))
