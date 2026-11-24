"""posture - heartbeat, stand, sit, auto mode, body tilt and tricks. Mixed into Lite3."""
import threading
import time
from contextlib import contextmanager

from . import udp
from .motion import clamp
from .state import ARM_GRACE, BATTERY_REFUSE, BATTERY_WARN, LYING, READY_STATES, STANDING
from .udp import ACTIONS, GAITS, Lite3Error

MAX_TILT_DEG = 14.0      # STICK_PITCH full scale, see udp.py
HEARTBEAT_HZ = 2.0


class Posture:
    _hb_thread = None

    def heartbeat_start(self):
        """Emit the controller keepalive at 2 Hz until stopped.
        SAFETY: this replaces the app's role in the interlock - the robot
        will accept commands with no app or controller connected."""
        if self.heartbeat_running:
            return
        self._hb_stop.clear()
        self._hb_thread = threading.Thread(target=self._hb_loop, daemon=True)
        self._hb_thread.start()

    def _hb_loop(self):
        while not self._hb_stop.is_set():
            try:
                udp.send(udp.HEARTBEAT)
            except OSError:
                pass
            self._hb_stop.wait(1.0 / HEARTBEAT_HZ)

    def heartbeat_stop(self):
        self._hb_stop.set()
        if self._hb_thread is not None:
            self._hb_thread.join(timeout=2.0)
            self._hb_thread = None

    @property
    def heartbeat_running(self):
        return self._hb_thread is not None and self._hb_thread.is_alive()

    @contextmanager
    def tilt(self, pitch_deg, settle=1.5):
        """Context manager: hold the body pitched nose up by pitch_deg (negative =
        nose down) while standing in place, and level it again on every way out.
        Don't walk inside it. settle: seconds to wait for the tilt before the block.
        """
        self._require_standing()
        frac = clamp(pitch_deg / MAX_TILT_DEG, 1.0)
        value = int(-32767 * frac)              # negative stick = nose up
        stop = threading.Event()

        def hold():
            while not stop.is_set():
                udp.send(udp.STICK_PITCH, value)
                stop.wait(0.1)

        udp.send(udp.POSTURE_ENTER)
        time.sleep(0.5)
        th = threading.Thread(target=hold, daemon=True)
        th.start()
        try:
            time.sleep(settle)                  # let the body reach the tilt
            yield
        finally:
            stop.set()
            th.join()
            for _ in range(10):                 # back to level before leaving
                udp.send(udp.STICK_PITCH, 0)
                time.sleep(0.1)
            udp.send(udp.POSTURE_EXIT)
            time.sleep(1.0)

    @contextmanager
    def upright(self, heartbeat=True):
        """Stand for the with-block and sit on the way out, whatever happens.
        heartbeat=True holds the interlock from here (no app needed) until
        close(), so he sits FIRST and is disarmed SECOND.
        """
        if heartbeat:
            # Before waiting: with no app he sits at 8, not ready, until a keepalive arrives.
            self.heartbeat_start()
            if not self.wait_ready():
                raise Lite3Error('interlock never came up - is transfer_ros2 '
                                 'running?')
        try:
            self.stand()
            yield self
        finally:
            self.sit()

    def wait_ready(self, timeout=10.0):
        """Block until basic_state is settled and commandable (READY_STATES). True
        if it is. Settled is not armed: stand() handles the toggle that arming
        eats, so no sleep is needed after this.
        """
        return self._wait(lambda: self.basic in READY_STATES, timeout)

    def auto(self):
        """Required before any velocity; nothing reports the mode, so just set it.
        Leaves posture mode first: in it he reports standing but ignores velocity.
        """
        udp.send(udp.POSTURE_EXIT)
        time.sleep(0.3)
        udp.send(udp.MODE_AUTO)
        time.sleep(0.5)

    def stand(self, timeout=20.0):
        """Stand up. Idempotent, though the underlying command is a toggle. From an
        unarmed state the first toggle only arms him, so it is sent once more.
        """
        if self.standing:
            return True
        basic = self.basic
        if basic not in LYING:
            raise Lite3Error(
                'basic_state=%s is transitional or unknown; expected one of %s '
                '(lying). Not toggling - it could lie the robot down mid-move.'
                % (basic, LYING))
        self._require_battery()
        for retry in (False, True):
            if self._toggle_stand(timeout):
                time.sleep(1.0)      # let the pose settle before anyone reads it
                return True
            basic = self.basic
            if retry or basic not in LYING:
                break
            # Settled back in a lying state: the toggle armed him instead of
            # standing him. One retry, now that jy_exe is listening.
            print('NOTE: first stand toggle was consumed arming the robot '
                  '(basic_state=%s) - sending it again' % basic)
        raise Lite3Error(
            'timed out standing (basic_state=%s). Is the app connected, or '
            'heartbeat_start() running? With neither, jy_exe ignores every '
            'network command and basic_state sits at 8.' % self.basic)

    def _toggle_stand(self, timeout):
        """One stand toggle. True if standing; False as soon as he has settled back
        lying (the toggle was swallowed), without waiting out the timeout.
        """
        udp.send(udp.STAND_TOGGLE)
        sent = time.monotonic()
        end = sent + timeout
        while time.monotonic() < end:
            if self.standing:
                return True
            if time.monotonic() - sent > ARM_GRACE and self.basic in LYING:
                return False
            time.sleep(0.05)
        return self.standing

    def sit(self, timeout=20.0):
        """Lie down. Idempotent."""
        if not self.standing:
            return True
        self.halt()
        udp.send(udp.STAND_TOGGLE)
        return self._wait(lambda: self.basic in LYING, timeout)

    def action(self, name, force=False):
        """Play a built-in trick from ACTIONS. Does NOT stand or sit for
        you. Sent once, as the app does; returns when sent, not when the trick is
        over. cancel_trick() stops one in udp.CANCELLABLE.
        """
        code, posture = ACTIONS[name]
        self._require_battery(force)
        if posture == 'stand':
            self._require_standing()
            self.halt()
        elif self.basic != 1:
            # 8/98 are unarmed: jy_exe would eat the first send arming itself
            # and the next one would land mid-transition.
            raise Lite3Error(
                '%s starts lying and armed (basic_state 1), not %s. '
                'stand() then sit() gets there.' % (name, self.basic))
        udp.send(code)

    def gait(self, name, timeout=3.0):
        """Gait, as the app's buttons: slow, medium, fast (Flat) or general, grip, h_step (Rug).
        Checked against the gait the robot then reports; raises if it did not change."""
        code, reported = GAITS[name]
        udp.send(code)
        if not self._wait(lambda: self.state.get('gait') == reported, timeout):
            raise Lite3Error('gait %s not taken (reports gait=%s) - is he standing?'
                             % (name, self.state.get('gait')))

    @property
    def gait_name(self):
        """A name from GAITS, or None for a number not seen yet."""
        g = self.state.get('gait')
        return next((n for n, (_, r) in GAITS.items() if r == g), None)

    def crawl_toggle(self):
        """The app's NRM/Crawl button. BLIND: it flips the mode, and nothing reports
        which one he is in, so the program must keep count itself. Only works in
        slow (the app locks crawl to slow; in medium and fast the code did nothing), so refused otherwise."""
        if self.gait_name != 'slow':
            raise Lite3Error('crawl only works in slow mode (gait is %s): gait(\'slow\') first'
                             % (self.gait_name or self.state.get('gait')))
        udp.send(udp.CRAWL_TOGGLE)

    def cancel_trick(self):
        """Stop a running trick, as the app's cancel does. Only the tricks in
        udp.CANCELLABLE stop; the app offers no cancel for the others."""
        # It seems to act on a CHANGE of value (the app alternates 1, 0, 1; the CLI sending 1
        # twice did nothing the second time, 2026-11-24): 0 then 1 is a change either way.
        udp.send(udp.TRICK_CANCEL, 0)
        udp.send(udp.TRICK_CANCEL, 1)

    def _require_battery(self, force=False):
        b = self.battery
        if b is None or force:
            return
        if b < BATTERY_REFUSE:
            raise Lite3Error(
                'battery %d%% is below %d%%. The robot will refuse to stand and '
                'will not say why. Pass force=True to override.'
                % (b, BATTERY_REFUSE))
        if b < BATTERY_WARN:
            print('WARNING: battery %d%% - charge soon' % b)

    def _require_standing(self):
        if not self.standing:
            raise Lite3Error(
                'basic_state=%s, not standing (%d). Velocity is silently '
                'ignored unless the robot is standing - call stand() first.'
                % (self.basic, STANDING))
