# Messages and Call (2026-10-15)

## Message

Open Comm > Message on both updated suits once to start their direct receivers.
They remain available in the background until reboot. Both must be on the same
Wi-Fi radio channel for ESP-NOW; no router is required, but joining Wi-Fi can
change the radio channel. Automatic cross-channel discovery is not implemented.

- A starts/stops a 16 kHz mono recording on SD.
- B plays/stops the selected recording. Existing 16 kHz PCM16 mono files now
  resample to the A2DP peer's 44.1/48 kHz in the feeder, not in its BT callback.
  Same-rate music playback is unchanged. Other rate mismatches remain rejected.
- C randomly selects a valid WAV from either outbox or inbox. Corrupt/incomplete
  recordings are skipped, not deleted. Hold C still deletes latest valid outbox.
- D tries a direct ESP-NOW receiver first. First accepting receiver wins (not
  multicast delivery). START/DATA/END have application acknowledgements, sequence
  offsets, bounded retries, and an FNV whole-file corruption check. An ACK means
  the chunk was written or the complete WAV was closed and renamed, not merely
  that the radio callback received a packet. Duplicate chunks/final ACK retries
  do not duplicate data. Partial files expire after 10 s without activity.
- Only absence of a direct receiver falls back to PC UDP, avoiding ambiguous
  double delivery after a mid-transfer direct failure. PC is currently configured
  at 192.168.0.205:9001. Start ../MessageServer/message_server.py on the PC.

Received files: `/sdcard/messages/inbox/`; recorded files: `outbox/`. Transfer
limit 16 MiB. No automatic received-message playback. Radios/file transfer are
not authenticated: use trusted suits/network. Disk flush/close + rename confirms
delivery; it is not a guarantee against subsequent SD/power failure.

UDP legacy framing bug fixed (filename length at byte 14, name at byte 15).
Receiver rejects path traversal, sequence gaps, oversized/incomplete writes,
checks the source endpoint and times out. Sender now checks send failures and
waits for a source-matched completion ACK; no ACK is a timeout, not success.
Telemetry reconnect no longer requires a fresh IP event when already connected.

## Call

The old menu item had no registered shell app. The old standalone component is
retained but is not invoked by the new app (it reinitialized hardware and never
returned). New Call is integrated into the shell and is half-duplex:

1. Open Comm > Call on both suits, on the same ESP-NOW channel.
2. Both start listening on their integrated speakers.
3. Press A on one suit to talk; press A again to listen. Take turns.
4. Leave the app to stop its worker and return I2S to normal use.

Call disconnects Bluetooth and suspends FFT/recording/playback while it owns
I2S. It does not route calls through Bluetooth or the PC. PCM16 mono at 16 kHz,
92 samples/packet, bounded 16-packet RX queue; stale/duplicate packets rejected.
Only one remote talker accepted while active. No acoustic echo cancellation;
half-duplex avoids simultaneous local mic/speaker operation. Radio packet loss
can still produce gaps. End-to-end voice quality needs hardware testing.

## Verification

Host tests: main/tests/test_message_direct.py (production receiver state machine,
duplicate data/final ACK and bad checksum); main/tests/test_fft_audio_gate.py;
components/bt_audio/tests/test_voice_resample.py (frame counts, interpolation,
extreme PCM values and boundary cases); ../MessageServer/test_message_server.py.
Firmware builds; actual BT playback, two-suit delivery and live calling still
need physical testing. This does not resolve the separate SBC queue-pressure
investigation during long LED shows.
