#!/usr/bin/env python3
"""check_parts.py - speaker, front camera and command line, with no robot.

    python3 tests/check_parts.py      # needs rclpy to import, nothing else
"""
import contextlib
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from lite3 import Lite3, Lite3Error, camera, cli, speaker  # noqa: E402


def main():
    # the speaker: raw samples over ssh into aplay on the motion computer
    ap = speaker.aplay_command()
    assert ap[-1] == '-' and speaker.DEVICE in ap, ap
    assert speaker.ssh_command('ysc@x')[-1] == 'ysc@x' and 'BatchMode=yes' in speaker.ssh_command('ysc@x')
    assert speaker.to_speaker(None, 'volume=0.50')[1].startswith('volume=0.50,adelay=')
    s = speaker.Speaker()
    s.clips = lambda: ['OKstop']
    try:
        s.play('nope')
        assert False, 'an unknown clip must raise'
    except speaker.SpeakerError as e:
        assert 'OKstop' in str(e), e
    assert issubclass(speaker.SpeakerError, Lite3Error)          # one except catches every SDK error

    # bot.play() and bot.play_file() are the speaker's
    played = []
    bot = Lite3.__new__(Lite3)
    bot.__dict__['speaker'] = type('S', (), {'play': lambda self, n, wait=True: played.append(('clip', n, wait)),
                                             'play_file': lambda self, p, wait=True: played.append(('file', p, wait))})()
    bot.play('okstop')
    bot.play_file('/tmp/a.wav', wait=False)
    assert played == [('clip', 'okstop', True), ('file', '/tmp/a.wav', False)], played
    assert isinstance(Lite3.__new__(Lite3).speaker, speaker.Speaker)

    # the front camera: one frame from its RTSP stream
    c = camera.grab_command('/tmp/x.jpg')
    assert camera.CAMERA_URL in c and c[c.index('-frames:v') + 1] == '1' and c[-1] == '/tmp/x.jpg', c
    m = camera.mjpeg_command('-q:v', '5')
    assert m[-3:] == ['-f', 'mjpeg', '-'] and '-q:v' in m, m

    # the command line with no command: usage, no robot touched
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        assert cli.main([]) is None
    assert 'walk <m>' in out.getvalue() and 'frame <file.jpg>' in out.getvalue(), out.getvalue()
    print('check_parts ok')


if __name__ == '__main__':
    main()
