Pending review

# Show system

The show system plays pre-authored LED choreography against a specific audio
track on up to three suits. Its desktop authoring tool and firmware runtime share
the same semantic suit layout and package format.

## End-to-end flow

```text
ShowEditor + audio
       ↓ export
SD package: show.sws + track.wav
       ↓
package validation → role program → absolute song clock → LED renderer
                                      ↕
                              peer synchronization
```

The desktop editor lives in the sibling `ShowEditor/` project. It authors
effects on an absolute timeline, previews A/B/C costume roles, and exports a
directory copied to `/sdcard/shows/<name>/`. Absolute time keeps choreography
tied to the actual recording, including tempo changes and sections without a
beat. BPM, beat and bar information are editing aids.

The exported program addresses named sections and common suit coordinates.
Physical LED counts are resolved on the costume, allowing the three suits to
share choreography while using different strip lengths.

## Firmware responsibilities

- [Show Player](../apps/show_player/README.md) browses packages and presents
  role, peer, playback and delay controls.
- [Show Package](../services/show_package/README.md) validates the binary format
  and loads the selected role program.
- [Show Runtime](../services/show_runtime/README.md) owns the session, audio
  clock, A/B/C roles, leader election, follower synchronization and LED output.
- [Show Renderer](../services/show_renderer/README.md) evaluates effects for one
  absolute timeline position.
- [Suit Geometry](../services/suit_geometry/README.md) maps semantic sections and
  positions onto the local costume.
- [Audio Player](../services/audio_player/README.md) reports submitted audio
  progress used by the show clock.

These module documents own the detailed state machines and tuning values.

## Multi-suit model

Suits discover each other while the Show app or a show session is active. Each
suit receives one of the stable roles A, B or C. The suit that starts playback
becomes the transport leader; role assignment and playback leadership are
separate so another suit can take over audio without reshuffling costumes.

The leader broadcasts the current session and time anchor. Followers estimate
clock offset and render from their corrected local clock, which also permits a
suit to join a song already in progress. Every suit renders locally; frames are
never streamed over the radio.

## Playback model

The leader anchors animation to audio frames that have actually entered the
output path. Output latency is subtracted before publishing song time. A
follower uses the synchronized anchor without playing a second copy of the
track. Rendering samples the current absolute time and discards missed frames,
so expensive animation cannot build an ever-growing delay behind the music.

Show Runtime owns LEDs for the session, then restores the lighting state that
was active beforehand. This provides one writer and keeps show behavior separate
from normal manual and FFT-driven animations.

## Package contract

Each show directory contains:

```text
/sdcard/shows/<name>/
  show.sws
  track.wav
```

All participating suits need a compatible package and firmware protocol. A
package is validated before playback; malformed role data or an unsupported
format is rejected rather than partially executed. Exact binary fields and
limits belong to the Show Package documentation and source.
