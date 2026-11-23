"""motion - the one motion loop behind walk, strafe, turn and steer. Mixed into Lite3.

Every call blocks, checks the robot on the way in, runs the safety checks every
50 ms cycle, and leaves a zero Twist behind on every way out.
"""
import math
import time

from geometry_msgs.msg import Twist

from .depth import sampled
from .udp import Lite3Error

MAX_SPEED = 0.6          # m/s
MAX_YAW_RATE = 1.6       # rad/s COMMANDED: he turns at about 0.7 of what is asked
HARD_TIMEOUT = 30.0      # s, ceiling on any single motion call

# Nearest thing on the side a turn in place allows: the body corner sweeps ~0.36 m.
TURN_SWEEP = 0.43

# He turns on for ~0.17 s after the zero command, so turn() stops that much
# early and reports the heading once he settles.
TURN_COAST_S = 0.17
TURN_SETTLE_S = 0.5

# Roll or pitch past this aborts any motion call. Raise it before trying a slope.
TILT_LIMIT_DEG = 30.0


def clamp(v, limit):
    """v held within +-limit. Anything not finite becomes 0: max/min pass a
    NaN through as +limit, which is full speed, not a stop."""
    return max(-limit, min(limit, v)) if math.isfinite(v) else 0.0


def dist(a, b):
    """Metres between two poses."""
    return math.hypot(b[0] - a[0], b[1] - a[1])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Motion:
    def _drive(self, vx=0.0, vy=0.0, wz=0.0):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
        self._node.pub.publish(t)

    def halt(self):
        """Publish a zero Twist, repeatedly. Best-effort QoS can drop one."""
        for _ in range(20):
            self._drive()
            time.sleep(0.02)

    def _safety(self, since):
        """A reason to stop every motion call, whatever it is doing."""
        if self._node.stick_time > since:
            return 'handheld took over - stopped'
        if self._stop_at > since:
            return 'cancelled - stopped'
        if not self._odom_fresh():
            return 'odometry stopped arriving - stopped'
        t = self._node.tilt
        if t and max(abs(t[0]), abs(t[1])) > TILT_LIMIT_DEG:
            return 'body tilted %.0f deg roll / %.0f deg pitch - stopped' % t
        return None

    def _loop(self, cycle, limit, force):
        """The one motion loop behind walk, strafe, turn and steer. cycle(start)
        drives for one 50 ms cycle and returns a reason to stop, or None. Guards on
        the way in, _safety() every cycle, a halt on every way out.
        """
        self._require_battery(force)
        self._require_standing()
        self.auto()
        self.wait_pose()
        start = self.pose
        began = time.monotonic()
        deadline = began + min(limit, HARD_TIMEOUT)
        reason = 'time limit'
        try:
            while time.monotonic() < deadline:
                stop = self._safety(began)
                if stop is None:
                    stop = cycle(start)
                if stop is not None:
                    reason = stop
                    break
        finally:
            self.halt()
            time.sleep(0.5)
        return {'reason': reason, 'start': start, 'end': self.pose}

    def _run(self, vx, vy, wz, done, limit, force, abort=None):
        """Drive at a fixed velocity until done(start, pose), the time limit,
        or abort() returns a reason."""
        def cycle(start):
            self._drive(vx, vy, wz)
            time.sleep(0.05)
            return (abort() if abort else None) or (
                'target reached' if done(start, self.pose) else None)

        return self._loop(cycle, limit, force)

    def steer(self, control, limit=HARD_TIMEOUT, force=False):
        """Drive with velocities recomputed every 50 ms: control() returns (vx, wz)
        to keep going or a string reason to stop. Same guards, clamps, time ceiling
        and final halt as walk() and turn().
        """
        def cycle(start):
            out = control()
            if isinstance(out, str):
                return out
            vx, wz = out
            self._drive(clamp(vx, MAX_SPEED), 0.0, clamp(wz, MAX_YAW_RATE))
            time.sleep(0.05)

        r = self._loop(cycle, limit, force)
        r['moved'] = dist(r['start'], r['end'])
        return r

    @staticmethod
    def _speed(speed):
        """A walking speed, checked: 0 would never arrive (and divides by
        zero in the time limit), and over MAX_SPEED is not a request to clamp
        quietly."""
        speed = abs(speed)
        if not 0 < speed <= MAX_SPEED:
            raise Lite3Error('speed must be in (0, %s] m/s' % MAX_SPEED)
        return speed

    def walk(self, distance, speed=0.15, stop_distance=0.6, force=False):
        """Walk `distance` metres (negative = backward). Blocking. Stops early for
        anything the depth camera sees within stop_distance (None = walk blind);
        the result's 'reason' says why it stopped. Backing up is always blind.
        """
        speed = self._speed(speed)
        vx = math.copysign(speed, distance)
        target = abs(distance)
        abort = None
        if stop_distance is not None:
            # Posture first: lying down, the camera sees floor and reports an obstacle.
            self._require_standing()
            if distance < 0:
                print('NOTE: the depth camera only looks forward - backing up '
                      'is unguarded')
            else:
                # Fail before moving rather than stopping one cycle in.
                here = self.clearance()
                if here <= stop_distance:
                    raise Lite3Error(
                        'obstacle already at %.2f m, inside the %.2f m stop '
                        'distance. Refusing to start.' % (here, stop_distance))
                clear = sampled(self.clearance, here)

                def blocked():
                    try:
                        c = clear()
                    except Lite3Error:
                        return 'lost the depth stream - stopped rather than ' \
                               'walking blind'
                    return 'obstacle at %.2f m' % c if c <= stop_distance else None

                abort = blocked

        r = self._run(vx, 0.0, 0.0, lambda s, p: dist(s, p) >= target,
                      target / speed + 10.0, force, abort)
        r['moved'] = dist(r['start'], r['end'])
        return r

    def strafe(self, distance, speed=0.15, force=False):
        """Crab sideways. Positive is left."""
        speed = self._speed(speed)
        vy = math.copysign(speed, distance)
        target = abs(distance)
        return self._run(0.0, vy, 0.0, lambda s, p: dist(s, p) >= target,
                         target / speed + 10.0, force)

    def turn(self, radians, rate=0.4, force=False, guard=True):
        """Turn in place by `radians` relative to the current heading. Positive is
        left; any magnitude, beyond a full revolution too. With guard, refuses
        (turned 0) a turn that would swing his body onto something in Nav2's
        costmap, and stops if the depth camera sees something on the side
        he turns to.
        """
        rate = min(abs(rate), MAX_YAW_RATE)
        if not rate > 0:
            raise Lite3Error('rate must be above 0 rad/s')
        wz = math.copysign(rate, radians)
        target = abs(radians)
        stop_at = target - min(rate * TURN_COAST_S, target / 2)
        acc = {'prev': None, 'total': 0.0}

        def done(s, p):
            if p is None:
                return False
            if acc['prev'] is None:
                acc['prev'] = s[2]
            # Wrap each INCREMENT, never the running total: the total saturates at pi
            # and a 180 degree turn never finishes.
            acc['total'] += wrap(p[2] - acc['prev'])
            acc['prev'] = p[2]
            return abs(acc['total']) >= stop_at

        abort = None
        if guard:
            why = self.turn_blocked(radians)
            if why:
                p = self.pose
                return {'reason': why, 'start': p, 'end': p, 'turned': 0.0}
            if not self.cloud_fresh():
                print('NOTE: no depth stream - turning without the side check')
            else:
                side = 0 if radians > 0 else 1          # left, right
                sides = sampled(self.side_clear)

                def blocked():
                    try:
                        near = sides()[side]
                    except Lite3Error:
                        return 'lost the depth stream - stopped turning'
                    if near < TURN_SWEEP:
                        return 'obstacle %.2f m to the %s - stopped turning' % (
                            near, ('left', 'right')[side])
                    return None

                abort = blocked

        r = self._run(0.0, 0.0, wz, done, target / rate + 10.0, force, abort)
        # Let the coast finish, then count it, so turned is where he settled.
        time.sleep(TURN_SETTLE_S)
        p = self._node.odom
        if p is not None and acc['prev'] is not None:
            done(None, p)
        r['turned'] = acc['total']          # unwrapped, so 180+ reads correctly
        return r

    def turn_deg(self, degrees, **kw):
        r = self.turn(math.radians(degrees), **kw)
        r['turned_deg'] = math.degrees(r['turned'])
        return r
