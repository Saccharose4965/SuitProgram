Pending review

# Microphone recording

This service owns recording sessions, WAV files, automatic filenames and the
amplitude history used by the recording screen. Its existing component/API name
is retained during migration. Audio acquisition uses the shared audio component.

## Session lifetime

Initialization remembers the requested sample rate and current RX handle.
The configuration's pin and I2S fields are legacy fields; this implementation
uses the shared audio bus configuration.

Starting mounts SD and exclusively creates a `.part` file beside the requested
destination, so an existing WAV or unfinished recording is never overwritten.
It writes and syncs an empty WAV header before acquiring audio.
It sets the audio rate, enables RX, refreshes the RX handle and starts one
recording task on core 0, priority 5, with a 4096-byte stack. A running session
is reused; a previous task still finalizing blocks a new start.

The task reads stereo 32-bit frames in batches of up to 256 and takes the left
slot. Division by 16384 and saturation produce signed 16-bit mono samples.
It appends PCM and periodically flushes the file. Empty/read-error batches retry;
an all-zero batch warns once per recording. A short SD write ends the session.

Stopping clears the run flag, disables audio and waits up to one second. The
task rewrites and syncs the WAV header with the actual byte count, closes the
file, then renames `.part` to the final WAV name. Failed finalization leaves the
partial file for diagnosis/recovery. Buffer-allocation failure removes an empty
partial file. Every startup failure after RX acquisition disables the shared
audio bus, including worker-creation and missing-handle failures. The
recording-status query remains true while the task is finalizing.

## Display data

The same saved PCM feeds a peak envelope: one column per 50 ms, up to 96 columns.
Each peak is scaled into a byte. Readers receive chronological columns through
the synchronized publication path. A final partial window is retained on stop,
and the history remains visible until the next recording starts.
The screen draws this data without opening another audio reader.

## Automatic names

recording_counter.c owns the private four-byte `.rec_counter` file on SD. It
loads the highest valid primary, backup or temporary value, then persists the
next value through temporary and backup names. Corrupt copies restart at zero;
exclusive file creation skips every existing WAV or partial name while advancing
the recovered counter. Counter exhaustion returns an error. Explicit caller
paths bypass the counter but use the same exclusive creation rule.

## Existing limits

Stop returns success after its bounded wait even if finalization continues.
Periodic payload flush errors are detected by the following write or final
header sync rather than surfaced immediately. Resource arbitration must prevent
competing bus users; it is described in [audio policy](../audio_policy/README.md).
Hardware acceptance should cover an SD-full short write, forced worker-start
failure and power removal during counter and WAV finalization.
