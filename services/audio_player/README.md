Pending review

# Audio player

This service turns a selected WAV path into one asynchronous playback session.
It chooses Bluetooth or the local I2S speaker, owns the playback task, exposes
pause/stop state, and prevents a second track from starting before the previous
task has released ownership.

## Starting and output selection

Normal playback starts immediately and avoids local fallback when a connected
Bluetooth stream rejects the file. Show playback permits speaker fallback and
delays delivery by one second so follower suits can load their show package.
Both modes reset the shared audio progress counter before creating a priority-2
worker on core 1.

The worker opens and parses PCM16 mono/stereo WAV before choosing output. If a
Bluetooth peer is streamable, it waits up to 800 ms for an existing media
command, then hands the open file to the A2DP source. A pending command or
startup failure follows the caller's fallback policy. Local playback streams
through the shared audio service.

If open or initial parsing fails, the worker calls the older local file helper,
which reopens the path and is not abortable mid-play. This retry generally
reports the same failure and remains a compatibility path to simplify later.

## Bluetooth monitoring

During Bluetooth playback the worker polls status every 100 ms. Falling queued
frames count as progress. A nonempty queue unchanged for 1.2 seconds logs one
health snapshot; playback is not automatically restarted because overlapping
media commands previously destabilized A2DP. When the queue drains, the worker
requests one stream stop so the stack becomes idle. Disconnect and explicit
abort use the same worker-owned teardown path.

Pause changes the service flag and sends A2DP pause/resume only in Bluetooth
mode. Local streaming reads the flag through the audio service's current weak
hook. Stop only raises the abort flag; callers that need replacement playback
wait up to two seconds for the task handle to clear.

## Task and memory handling

Task creation first requests a 4096-byte internal stack, then PSRAM when IDF
allows it, followed by equivalent 3072-byte attempts. Failure leaves no active
session. The path is copied into task arguments, and only one global path/mode/
pause state exists.

Optional standalone heap tracing begins only for Bluetooth playback. It freezes
on a stall or internal heap below 16 KiB and reports grouped outstanding
internal allocations after playback, when logging is less likely to disturb
audio. These reports are diagnostics and are explicitly not leak proof.

## Current ownership limits

Control fields and the task handle are shared through volatile flags rather
than a lock. `audio_player_set_current` can be replaced by worker completion,
which clears the path. Local and Bluetooth output implementations remain in
other modules, and Bluetooth currently calls back into this service on
disconnect. Those cycles will be handled while separating the audio driver and
Bluetooth service; changing them independently would risk the stable stream.

The retained host tests exercise stall/drain/disconnect decisions and bounded
heap-diagnostic lifecycle without hardware.
