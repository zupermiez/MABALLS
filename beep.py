"""Non-blocking audio cues for catch.py's live decisions.

Fire-and-forget: each call spawns `aplay`/`paplay` on a pre-generated WAV
(via Popen, never waited on) so the hot decision loop is never blocked on
audio playback. Tones are synthesized once at import time with stdlib
`wave` + `array` - no extra dependencies, no network, no big machinery.
"""

import array
import atexit
import math
import os
import shutil
import subprocess
import tempfile
import wave

_PLAYER = shutil.which("paplay") or shutil.which("aplay")
_SAMPLE_RATE = 44100

# Each event is a list of segments rendered back to back. "note"/"sweep"
# segments carry their own harmonics (for timbre) and exponential decay
# (for a percussive, recognizable attack) rather than one flat sine, so
# events are distinguishable by ear, not just by pitch.
_EVENTS = {
    # predict: single clean mid beep - "I see it"
    "predict": [
        {"type": "note", "freq": 880, "dur": 0.09, "wave": "sine",
         "harmonics": [(1, 1.0)], "decay": 12},
    ],
    # reaim: two quick high blips - "adjusting"
    "reaim": [
        {"type": "note", "freq": 1400, "dur": 0.05, "wave": "sine",
         "harmonics": [(1, 1.0)], "decay": 20},
        {"type": "gap", "dur": 0.03},
        {"type": "note", "freq": 1400, "dur": 0.05, "wave": "sine",
         "harmonics": [(1, 1.0)], "decay": 20},
    ],
    # catch: "cha-ching" - bright ascending bell/coin chime
    "catch": [
        {"type": "note", "freq": 1046.50, "dur": 0.12, "wave": "sine",
         "harmonics": [(1, 0.6), (2, 0.25), (3, 0.15)], "decay": 14},
        {"type": "gap", "dur": 0.015},
        {"type": "note", "freq": 1567.98, "dur": 0.22, "wave": "sine",
         "harmonics": [(1, 0.6), (2, 0.25), (3, 0.15)], "decay": 7},
    ],
    # miss: low descending buzzer - unmistakable "FAIL"
    "miss": [
        {"type": "sweep", "f0": 220, "f1": 130, "dur": 0.16, "wave": "square",
         "harmonics": [(1, 0.5), (3, 0.2), (5, 0.1)], "decay": 3},
        {"type": "gap", "dur": 0.02},
        {"type": "sweep", "f0": 165, "f1": 90, "dur": 0.32, "wave": "square",
         "harmonics": [(1, 0.5), (3, 0.2), (5, 0.1)], "decay": 4},
    ],
}

_paths: dict = {}


def _wave_value(phase: float, shape: str) -> float:
    s = math.sin(phase)
    if shape == "square":
        return 1.0 if s >= 0 else -1.0
    return s


def _render_segment(samples: array.array, seg: dict) -> None:
    if seg["type"] == "gap":
        samples.extend([0] * int(_SAMPLE_RATE * seg["dur"]))
        return

    n = int(_SAMPLE_RATE * seg["dur"])
    attack = max(1, int(n * 0.03))
    decay = seg.get("decay", 8.0)
    shape = seg.get("wave", "sine")
    harmonics = seg.get("harmonics", [(1, 1.0)])
    is_sweep = seg["type"] == "sweep"
    f0 = seg.get("f0")
    f1 = seg.get("f1")
    freq = seg.get("freq")

    phases = [0.0] * len(harmonics)
    for i in range(n):
        if is_sweep:
            frac = i / n
            f = f0 + (f1 - f0) * frac
        else:
            f = freq
        env = (i / attack) if i < attack else math.exp(-decay * (i - attack) / n)
        val = 0.0
        for h, (mult, amp) in enumerate(harmonics):
            phases[h] += 2 * math.pi * (f * mult) / _SAMPLE_RATE
            val += amp * _wave_value(phases[h], shape)
        val *= env
        samples.append(int(max(-1.0, min(1.0, val)) * 32767))


def _make_wav(segments) -> str:
    fd, path = tempfile.mkstemp(suffix=".wav", prefix="beep_")
    os.close(fd)
    with wave.open(path, "w") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(_SAMPLE_RATE)
        samples = array.array("h")
        for seg in segments:
            _render_segment(samples, seg)
        f.writeframes(samples.tobytes())
    return path


def _cleanup():
    for p in _paths.values():
        try:
            os.remove(p)
        except OSError:
            pass


if _PLAYER:
    for _name, _segments in _EVENTS.items():
        _paths[_name] = _make_wav(_segments)
    atexit.register(_cleanup)


def play(kind: str) -> None:
    """Fire a beep for `kind` in {predict, reaim, catch, miss}. No-op if no player found."""
    path = _paths.get(kind)
    if path is None:
        return
    subprocess.Popen([_PLAYER, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


if __name__ == "__main__":
    import time as _time
    if not _PLAYER:
        print("no aplay/paplay found on PATH - beeps would be no-ops")
    else:
        print(f"using {_PLAYER}, playing each tone once")
        for k in _EVENTS:
            print(k)
            play(k)
            _time.sleep(1.0)
