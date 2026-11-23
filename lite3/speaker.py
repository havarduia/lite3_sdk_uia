"""speaker - sound out of the robot's own speaker.

The speaker is on the MOTION computer, so playing a sound is ssh + aplay there.
Needs an SSH key from this computer to ysc on the motion computer (docs/setup.md).

    from lite3.speaker import Speaker
    Speaker().play('OKstandup')        # one of the robot's built-in clips
    Speaker().play_file('alarm.wav')   # any local audio, converted with ffmpeg here
"""
import os
import shutil
import subprocess

from .udp import MOTION_IP, Lite3Error

HOST = 'ysc@' + MOTION_IP          # motion computer, where the speaker is
CLIP_DIR = '/home/ysc/lite3_voice'  # DEEP's own voice clips live here
RATE, CHANNELS = 48000, 2           # the codec's known-good playback format
# The amp swallows the start of a stream while it unmutes (a 1.1 s clip was
# inaudible). Raise it if a first syllable is ever clipped.
LEAD_IN_MS = 800
# Straight to the ES8388. ysc's PulseAudio default is a null sink, so a
# plain `aplay` "succeeds" silently. Needs ysc in the audio group on the motion computer.
DEVICE = 'plughw:0,0'
# Master volume, 0.0 - 1.0, for every stream. Read when a sound starts, so a
# change applies from the next sound on. Built-in clips play at their own level.
VOLUME = 1.0


class SpeakerError(Lite3Error):
    pass


def ssh_command(host):
    """ssh with no interactive prompt: rely on the installed key, and fail fast
    if it ever stops working rather than hang on a password ask."""
    return ['ssh', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8', host]


def aplay_command():
    """aplay on the robot, playing the raw samples to_speaker() makes."""
    return ['aplay', '-q', '-D', DEVICE, '-f', 'S16_LE', '-r', str(RATE),
            '-c', str(CHANNELS), '-']


def to_speaker(*filters):
    """ffmpeg output arguments: these filters (None is skipped), then the amp
    lead-in, as raw samples on stdout."""
    lead = 'adelay=%d|%d' % (LEAD_IN_MS, LEAD_IN_MS)
    return ['-af', ','.join(f for f in filters + (lead,) if f),
            '-f', 's16le', '-ar', str(RATE), '-ac', str(CHANNELS), '-']


class Speaker:
    """Sound out of the robot. One SSH hop per sound; nothing stays running."""

    def __init__(self, host=HOST, clip_dir=CLIP_DIR):
        self.host = host
        self.clip_dir = clip_dir
        self._ssh = ssh_command(host)

    # --- the built-in clips ------------------------------------------------
    def clips(self):
        """Names (without .wav) of the clips already on the robot."""
        out = subprocess.run(self._ssh + ['ls', '-1', self.clip_dir],
                             capture_output=True, text=True)
        if out.returncode != 0:
            raise SpeakerError('could not list %s on %s: %s'
                             % (self.clip_dir, self.host, out.stderr.strip()))
        return sorted(f[:-4] for f in out.stdout.split() if f.endswith('.wav'))

    def play(self, name, wait=True):
        """Play one of the robot's own clips by name, e.g. play('okstop').
        Case-insensitive, .wav optional. Plays entirely on the robot.
        """
        clips = self.clips()
        key = name[:-4] if name.lower().endswith('.wav') else name
        match = next((c for c in clips if c.lower() == key.lower()), None)
        if match is None:
            raise SpeakerError('no clip %r. Available: %s'
                             % (name, ', '.join(clips)))
        path = '%s/%s.wav' % (self.clip_dir, match)
        # -q so aplay does not print the format banner on every call.
        cmd = self._ssh + ['aplay', '-q', '-D', DEVICE, path]
        if not wait:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)
            return None
        out = subprocess.run(cmd, capture_output=True, text=True)
        if out.returncode != 0:
            raise SpeakerError('play %s failed: %s' % (match, out.stderr.strip()))
        return True

    # --- arbitrary local audio ---------------------------------------------
    def play_file(self, path, wait=True):
        """Play any local audio file out of the robot: ffmpeg here converts it and
        streams the raw samples over ssh into aplay.
        """
        return self.stream(path, wait, 'play_file %s' % path)

    def stream(self, src, wait, label, pre=None):
        """Run (pre | ffmpeg | ssh aplay -), ffmpeg converting to the codec's
        format. src is a file path, or '-' for what the command `pre` writes."""
        if not shutil.which('ffmpeg'):
            raise SpeakerError('ffmpeg not found on this machine - needed to '
                             'convert audio to the robot codec format')
        ff = (['ffmpeg', '-loglevel', 'error', '-i', src]
              + to_speaker('volume=%.2f' % VOLUME))
        sink = self._ssh + aplay_command()
        p_pre = subprocess.Popen(pre, stdout=subprocess.PIPE) if pre else None
        p_ff = subprocess.Popen(
            ff, stdin=(p_pre.stdout if p_pre else None),
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if p_pre:
            p_pre.stdout.close()      # let ffmpeg own the read end
        p_ap = subprocess.Popen(sink, stdin=p_ff.stdout,
                               stderr=subprocess.PIPE)
        p_ff.stdout.close()
        if not wait:
            return None
        ap_err = p_ap.communicate()[1]
        ff_err = p_ff.communicate()[1]
        # Check the producer first: when yt-dlp dies, ffmpeg
        # only sees empty input and its error would hide the real cause.
        if p_pre and p_pre.wait() != 0:
            raise SpeakerError('%s: %s failed (exit %d), see its message above'
                             % (label, os.path.basename(pre[0]), p_pre.returncode))
        if p_ap.returncode != 0:
            raise SpeakerError('%s: aplay on robot failed: %s'
                             % (label, ap_err.decode(errors='replace').strip()))
        if p_ff.returncode not in (0, None):
            raise SpeakerError('%s: ffmpeg failed: %s'
                             % (label, ff_err.decode(errors='replace').strip()))
        return True
