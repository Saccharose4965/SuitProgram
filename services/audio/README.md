Pending review

# Audio service

This service produces local-speaker audio. It parses PCM WAV data, converts it
to the I2S sample layout, applies speaker volume, generates tones or streamed
samples, reports playback progress and coordinates local-output ownership. The
physical peripheral is described by the
[I2S audio driver](../../drivers/audio_i2s/README.md).

## Local output ownership

Every public blocking playback entry increments a small ownership count and
decrements it on return. Resource policy uses `audio_output_active()` to avoid
releasing or reclocking I2S while a startup tone, WAV or generator is writing.
Generated audio requires exclusive ownership because it is used by the live
synth and has no higher-level Audio Player session to serialize it.

The count describes local speaker writers. Bluetooth playback reports progress
through this service but is owned and serialized by Audio Player and the A2DP
service.

## WAV handling and streaming

The file parser walks RIFF chunks until it has a PCM format and the first data
chunk. It accepts 16-bit mono or stereo input and records the payload offset so
the same parsed file can be handed to local or Bluetooth output. Unknown chunks
and RIFF padding are skipped.

Local file playback uses one implementation for direct calls and Audio Player.
A 32 KiB PSRAM ring absorbs SD-card read timing, with a direct-read path when
that allocation is unavailable. Each source frame is reduced to mono, scaled
by the Q15 volume value, saturated and duplicated into two MSB-aligned 32-bit
I2S slots. Writes are limited to 2048 bytes so one blocked DMA request does not
hold the scheduler for a large chunk. Timeouts retry the same samples, preserving
track duration. Pause and abort are read through Audio Player hooks during both
file reads and writes.

Memory WAV playback retains its bounded parser because no `FILE` is available.
It currently accepts mono PCM16. Embedded playback finds an optional linked WAV
asset and delegates to that path.

## Generated audio and endings

Tone generation builds a sine wave in fixed-size blocks. The generic generator
asks its caller for mono PCM16 blocks, then performs the same volume scaling and
stereo packing used by WAV playback. It is the common output boundary for the
arm-controlled synth.

Every completed local stream writes a short zero tail before muting and
disabling TX. The tail lets queued samples drain and reduces the edge that
would otherwise produce an audible pop.

## Playback timeline

The progress snapshot contains submitted frames, their sample rate, the most
recent update time and whether the frames went to Bluetooth. Updates and reads
use a short critical section. Progress is reset by Audio Player when a new
track is accepted.

The shared timeline converts that snapshot into estimated audible position.
It subtracts the configured Bluetooth delay or the local I2S DMA queue depth,
then advances from the last submission timestamp across that buffered interval.
It marks a timeline stalled when submissions stop beyond the queue delay. Show
Runtime and Bad Apple consume this clock, and the Show delay control updates
the shared Bluetooth value. The default is 100 ms.

Host-side WAV parser tests remain to be added. Hardware acceptance should play
mono and stereo files from SD, exercise pause/abort, compare local volume and
tone output, and switch repeatedly between local output, Bluetooth and FFT.
