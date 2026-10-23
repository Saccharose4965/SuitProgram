Pending review

# Call service

This service implements direct, half-duplex voice between suits. It owns the
voice packet queue, microphone/speaker direction changes, the real-time worker
and the resource transition into and out of a call. The Call app supplies only
the push-to-talk control and status display.

## Session and audio ownership

Starting requires ESP-NOW as Link's active path because voice packets are
broadcast directly to nearby suits. The service marks Call active in audio
policy, stops FFT capture, recording and file playback, and disconnects A2DP.
Call always uses the integrated speaker: Bluetooth latency is too large for a
conversation and the existing A2DP path is output-only.

The shared I2S bus is configured for 16 kHz. Listen mode enables TX; talk mode
enables RX. A direction change disables both channels first, which flushes DMA
state before the peripheral is reused in the opposite direction. It also clears
queued packets and the selected peer so audio captured under the previous mode
cannot leak into the new one.

The worker keeps FFT excluded until I2S has been disabled on exit. `stop()`
requests termination and waits up to two seconds for that cleanup; the worker
retains ownership if hardware takes longer.

## Voice packets

One packet is 196 bytes: `CALL`, a 32-bit sequence, a 16-bit payload byte count,
two reserved bytes and 92 mono PCM16 samples. Ninety-two samples represent
5.75 ms at 16 kHz. The sender reads stereo-width I2S words, uses the microphone
slot, scales its 32-bit sample down to signed PCM16 and broadcasts without Link
acknowledgements.

The receive callback accepts packets only while actively listening. It checks
type, pointers, exact length, magic and sample byte count, then copies into a
16-packet queue without blocking the Link callback. The worker chooses the
first active sender for a 500 ms window and rejects another MAC or a repeated/
older sequence during that window. This prevents interleaved voices from being
mixed accidentally. A silent interval permits a different suit to take over.

Received PCM is multiplied by the speaker volume, saturated, expanded into the
two MSB-aligned I2S output slots and written with bounded 50 ms waits. The worker
yields periodically under continuous radio traffic so lower-priority audio and
idle tasks still run.

## Status and limitations

The app receives a value snapshot containing active/talk state,
latest error and accepted TX/RX packet counters. An I2S or Link-path failure
ends the worker and leaves its error visible.

The packet layout currently uses native integer byte order, has no version,
authentication, target or concealment for missing sequences. The shared Link
protocol review should address those together rather than changing Call alone.
Full duplex would also require echo cancellation and a different I2S/resource
model.

`tests/test_call_packets.py` compiles the production receive gate and checks
inactive, wrong-type, null, short, malformed and talk-mode rejection. Hardware
acceptance should switch talk/listen repeatedly between two suits, leave the app
mid-packet, re-enter, and confirm FFT resumes only after worker cleanup.
