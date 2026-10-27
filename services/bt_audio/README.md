Pending review

# Bluetooth A2DP audio

This service owns the ESP32 Classic Bluetooth A2DP-source stack, sink discovery
and connection state, media commands, volume, live generators and file-backed
PCM delivery. The Bluetooth screen is in
[apps/bluetooth](../../apps/bluetooth/README.md); WAV session selection is in
[audio_player](../audio_player/README.md).

## Stack and connection state

Startup initializes NVS, releases BLE controller memory, enables Classic BT,
registers GAP/A2DP/AVRCP callbacks and enters discovery/connection state. GAP
results retain up to eight audio devices and request missing remote names one
at a time. A selected peer is remembered while an existing link disconnects;
connection attempts are retried up to three times through a timer.

The A2DP callback is the source of connection, negotiated sample-rate and media
state. A connected link disables sniff in its link-policy mask because sleep
transitions previously interrupted sustained audio. Disconnect clears peer and
stream state, then services a pending selection or restarts discovery.

Only one media command may be outstanding. Start, pause and stop reject races
while an acknowledgement is pending. Stop closes file/generator delivery before
requesting A2DP suspend. Stack state and the user's requested streaming state
are tracked separately.

## PCM delivery

The A2DP data callback requests PCM at the peer cadence. A live generator can
produce mono PCM16 directly. File playback uses a PSRAM-backed stream, feeder
task and resampling path to convert PCM16 mono/stereo WAV data to the negotiated
rate. The callback consumes already prepared data and never performs SD reads.
Queued frame counts provide progress and stall observation to Audio Player.

Volume is a Q15-style software gain with percent helpers. The file streamer
also submits frame progress to the shared audio clock used by show timing.

## Event boundary

Bluetooth callbacks publish disconnect and AVRCP play/pause requests to a
fixed, statically allocated queue. The shell drains it in normal task context.
Bluetooth therefore has no build dependency on Audio Player or Music, and its
callbacks do not perform higher-level playback work. Audio Player still owns
session teardown and independently observes connection state while streaming.

The queue does not alter PCM delivery, media-command serialization, discovery,
reconnection or task placement. Volume commands remain local to this service.

The service relies on private Bluedroid headers for its no-sniff policy. That is
an ESP-IDF-version-sensitive dependency. The retained host test exercises voice
resampling separately from the stack.
