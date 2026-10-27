Pending review

# Bad Apple screen

The screen selects a video file, coordinates its audio start, maps buttons to
playback commands and presents decoder or storage failures. Frame decoding is
owned by the [decoder service](../../services/bad_apple_decoder/README.md).

## Media discovery and startup

On entry the app mounts SD if needed, chooses the first existing video from
fixed `badapple/` and SD-root candidates, then starts the matching WAV. Formats
are tried in compressed RLE, plain RLE and raw order. Video decoding begins
when Audio publishes its first valid audible-playhead sample. A five-second
timeout reports audio startup failure.

A/B select the previous/next existing video candidate and restart both audio
and video at zero. C pauses or resumes Audio Player and the video follows that
state. D restarts the current pair. Closing stops both streams.

Each shell tick reads Audio's shared audible timeline. The decoder advances to
the absolute frame belonging to that playhead, so shell timing, SD stalls and
Bluetooth latency do not accumulate into permanent drift. A stalled audio
timeline freezes the displayed frame until progress resumes.

## Display

Valid 128 by 64 frames copy directly into the OLED framebuffer. Paused and
ended states add a small overlay. Before a frame exists, the app reports audio
startup, SD failure, missing audio or video, selected file or decoder error. The
video therefore deliberately occupies the complete panel rather than the
normal content rectangle.

App and decoder run synchronously in shell callbacks. A slow SD read or decode
can delay the shell tick; there is no playback worker or frame lock needed with
the current single-caller design.
