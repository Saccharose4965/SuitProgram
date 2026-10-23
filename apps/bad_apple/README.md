Pending review

# Bad Apple screen

The screen selects a video file, coordinates its audio start, maps buttons to
playback commands and presents decoder or storage failures. Frame decoding is
owned by the [decoder service](../../services/bad_apple_decoder/README.md).

## Media discovery and startup

On entry the app mounts SD if needed, then chooses the first existing video
from fixed `badapple/` and SD-root candidates. Formats are tried in compressed
RLE, plain RLE and raw order. It schedules video one second later and starts the
first matching WAV immediately; that fixed offset is the current Bluetooth
audio-latency compensation. There is no measurement or clock shared between
audio and video.

A/B select the previous/next existing video candidate. C toggles decoder pause.
D restarts the current source and audio with the same one-second delay. Cycling
source starts video immediately and leaves the current audio untouched, so it
is primarily a format/debug control rather than synchronized track selection.
Closing stops both video and the global audio player.

## Display

Valid 128 by 64 frames copy directly into the OLED framebuffer. Paused and
ended states add a small overlay. Before a frame exists, the app reports audio
sync delay, SD failure, missing source, selected file or decoder error. The
video therefore deliberately occupies the complete panel rather than the
normal content rectangle.

App and decoder run synchronously in shell callbacks. A slow SD read or decode
can delay the shell tick; there is no playback worker or frame lock needed with
the current single-caller design.
