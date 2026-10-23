Pending review

# Microphone recording

This service owns recording sessions, WAV files, automatic filenames and the
amplitude history used by the recording screen. Its existing component/API name
is retained during migration. Audio acquisition uses the shared audio component.

## Session lifetime

Initialization remembers the requested sample rate and current RX handle.
The configuration's pin and I2S fields are legacy fields; this implementation
uses the shared audio bus configuration.

Starting mounts SD, opens the destination and writes an empty WAV header.
It sets the audio rate, enables RX, refreshes the RX handle and starts one
recording task on core 0, priority 5, with a 4096-byte stack. A running session
is reused; a previous task still finalizing blocks a new start.

The task reads stereo 32-bit frames in batches of up to 256 and takes the left
slot. Division by 16384 and saturation produce signed 16-bit mono samples.
It appends PCM and periodically flushes the file. Empty/read-error batches retry;
an all-zero batch warns once per recording. A short SD write ends the session.

Stopping clears the run flag, disables audio and waits up to one second.
The task rewrites the WAV header with the actual byte count, closes the file,
releases buffers and disables audio. The recording-status query remains true
while that task is finalizing, even after the run flag is cleared.

## Display data

The same saved PCM feeds a peak envelope: one column per 50 ms, up to 96 columns.
Each peak is scaled into a byte. Readers receive chronological columns through
the synchronized publication path. A final partial window is retained on stop,
and the history remains visible until the next recording starts.
The screen draws this data without opening another audio reader.

## Automatic names

recording_counter.c owns the private .rec_counter file on SD. It keeps the
existing four-byte format and increments before choosing rec_XXXXX.wav.
Missing or short input starts from zero; overflow wraps to zero. If persistence
fails, the caller falls back to the low 16 bits of the task tick count.
Explicit caller paths bypass this counter.

## Existing limits

The counter is written in place and is not power-loss atomic. Its flush/sync
results are unchecked. Automatic names and tick fallback do not check for
collisions, and opening a destination with wb+ can overwrite an existing file.
These behaviours are preserved for a separate persistence fix.

WAV-header writes/final flushes have limited error reporting. Task-creation
failure closes the file but does not undo RX enablement. Stop returns success
after its bounded wait even if finalization continues. Resource arbitration
must prevent competing bus users; it is described in
[audio policy](../audio_policy/README.md).
