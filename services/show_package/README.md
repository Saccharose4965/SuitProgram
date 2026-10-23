Pending review

# Show package service

This service is the trust boundary between files exported by ShowEditor and the
firmware. It defines the binary format, validates a complete `show.bin`, and
loads one costume role into memory. Rendering, playback timing, radio sync and
screen controls begin only after this service accepts the file.

## Package location and identity

A show slug resolves to `/sdcard/shows/<slug>/show.bin`. Slugs may contain
letters, digits, `-`, `_` and `.`, are shorter than 32 bytes, cannot start with
`.` and cannot contain `..`. This makes a slug safe to place inside the fixed
runtime packets and prevents it from escaping the shows directory.

The header carries a 64-bit show identity, duration, role count, format version,
timing, initial formation and offsets to each table. The current reader accepts
major version 1 through minor version 11. A higher major or minor version is
rejected because its records may have meanings or lengths this firmware cannot
interpret. Earlier minor versions remain readable through explicit size and
suffix rules.

The CRC covers the exact declared file length with the header's CRC field
treated as zero. Validation also compares the declared and physical file sizes.
Consequently a truncated file, an appended partial export and a corrupted table
all fail before allocations are exposed to the rest of the system.

## Compiled timeline

The file contains one role record for each available A/B/C costume. A role
selects contiguous ranges from two global tables:

- buckets divide song time into fixed intervals and point at clips that can be
  active in that interval;
- clips contain time bounds, layer, target, blend mode, effect, fades, seed and
  an offset into the parameter blob.

This is work performed by the PC exporter so the ESP32 can jump directly from
the current song time to a short clip range. Bucket starts must match their
index and bucket duration exactly, and every referenced clip range must remain
inside its role. Roles must be unique and limited to A, B and C.

Targets are stable semantic section or group IDs from `suit_geometry`. The
package therefore describes an effect on a named part of a suit instead of
baking the current LED indices into the song. The renderer resolves those names
against the installed layout.

## Effect parameters and extensions

Fill has a compact color/intensity base and optional spatial bounds. Spatial
effects share axis, width, softness, frequency, phase, repetition, duty-cycle,
intensity and second-color fields. Radial Ray adds its coordinate space, plane
and origin. This common base is what lets Sweep, Sparkle, Fanout, Traveling Orb
and related effects share animation and color machinery.

Later format minors append data to the base block in a fixed order: parameter
automation, optional baked amplitude curves, spatial color mapping, then a
color sequence/grid or Perlin-color definition. The validator walks this order
and checks every count, enum, finite float, range and computed byte length. It
also rejects repeated automation keys and combinations unsupported by a given
effect/version. A clip is accepted only when its declared parameter length is
fully explained by that sequence.

Formation cues are stored in the extended header. They must be strictly ordered,
inside the show duration and use a supported line or circle formation. Tempo is
stored in milli-BPM with a signed beat offset so all costumes evaluate
tempo-controlled parameters from the same exported reference.

## Loading and memory ownership

`show_package_load_info*()` performs full validation and returns metadata
without retaining timeline data. `show_package_load_program*()` validates the
same file, finds the selected role and copies only that role's buckets and
clips, plus the shared parameter blob and formation cues. Large allocations
prefer PSRAM and fall back to ordinary 8-bit memory.

The returned program owns all four allocations. `show_package_program_free()`
releases them and clears the structure, including after a partial load failure.
`show_package_program_param()` translates a file-relative clip offset into the
loaded blob and repeats its bounds check before returning a pointer. Consumers
keep the program alive for the entire renderer lifetime.

`show_effect_presets.c` is a small catalogue for tools or UI code that need
human names and sensible initial parameters. It does not alter a loaded show.

## Failure behavior and checks

Malformed structure returns size/response errors, unsupported table features
return `ESP_ERR_NOT_SUPPORTED`, CRC mismatch is distinct, and missing files or
roles return not-found. The service never attempts to repair a package on the
suit; a corrected package must be exported and copied to the SD card.

The runtime tests load validated programs through a stub boundary. A dedicated
package corruption/fuzz test is still worth adding before suit-side authoring
writes this format directly.
