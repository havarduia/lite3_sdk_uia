"""camera - the front wide-angle camera, streamed by the motion computer over RTSP.

    from lite3 import camera
    camera.grab('/tmp/front.jpg')          # one frame, needs ffmpeg here
"""
import subprocess

from .udp import MOTION_IP, Lite3Error

CAMERA_URL = 'rtsp://%s:8554/test' % MOTION_IP


def mjpeg_command(*args, url=CAMERA_URL):
    """ffmpeg command line: the front camera as JPEGs on stdout. args are the
    output options (-vf, -q:v, ...)."""
    return ['ffmpeg', '-loglevel', 'error', '-rtsp_transport', 'tcp',
            '-i', url, *args, '-f', 'mjpeg', '-']


def grab_command(path, url=CAMERA_URL):
    """ffmpeg command line: one frame from the front camera into `path`."""
    return ['ffmpeg', '-loglevel', 'error', '-rtsp_transport', 'tcp',
            '-i', url, '-frames:v', '1', '-y', path]


def grab(path, url=CAMERA_URL, timeout=10.0):
    """Save one frame to `path` (a .jpg) and return the path."""
    try:
        r = subprocess.run(grab_command(path, url), stderr=subprocess.PIPE,
                           universal_newlines=True, timeout=timeout)
    except FileNotFoundError:
        raise Lite3Error('ffmpeg not found on this computer - needed for camera frames')
    except subprocess.TimeoutExpired:
        raise Lite3Error('no frame from %s within %.0fs - is the stream running on '
                         'the motion computer? (docs/setup.md)' % (url, timeout))
    if r.returncode:
        raise Lite3Error('no frame from %s: %s' % (url, r.stderr.strip()))
    return path
