Pending review

# Bad Apple frame decoder

This service converts a file into successive 128 by 64 one-bit frames. It owns
the open file, decoding buffers, current frame and status. It
contains the vendored heatshrink decoder and its license.

## Input formats

Raw input is exactly 1024 bytes per frame. RLE input treats 0x55 as a zero-byte
run marker and 0xaa as a 0xff run marker. A zero count emits the marker itself;
counts below 128 are stored in one byte and larger counts combine the low seven
bits with the next byte shifted by seven. Heatshrink+RLE first incrementally
decompresses 2048-byte file blocks into a 4096-byte buffer, then applies the
same RLE state machine. Auto detection uses `.hs`, `.rle`, or raw-style
extensions and defaults unknown extensions to heatshrink+RLE.

EOF is valid only between complete frames and complete RLE tokens. Partial
frames or tokens become decode errors. Looping closes and reopens the same path
and resets all stream state before decoding the first frame again.

## Playback clock

Start opens the path, clamps requested cadence to 1..60 fps (invalid low values
become 30), and decodes one frame immediately. The caller supplies an absolute
media playhead in milliseconds. The target frame is derived from that playhead,
and sequential formats decode up to four missing frames per call. Repeated
calls continue bounded catch-up until current, keeping shell work finite while
converging after an SD or UI stall. Pause keeps the file and displayed frame.

The service exposes a copied frame and a copied status snapshot. It performs no
locking because the current app is its sole caller on the shell path. It is not
safe for a worker and renderer to call concurrently without adding ownership.

Stopping closes the file and invalidates the frame. The configured path and
source type remain in state so explicit restart can reopen them. File errors do
not distinguish malformed compression from general decoder failure beyond the
stored ESP error.
