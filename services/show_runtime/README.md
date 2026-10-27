Pending review

# Show runtime service

This service owns a live show session. It coordinates A/B/C roles, chooses the
playback leader, starts audio, translates audio progress into song time,
synchronizes followers and submits renderer frames through exclusive LED
ownership. The Show Player app only requests these operations and displays the
resulting status.

## Presence and role assignment

A suit advertises a compact Hello while its Show menu is open, while it owns
show output, or while a shared service requests A/B/C role presence. Live Sync
uses the latter to obtain suit identity without starting Show transport.
Opening the role network starts a short discovery interval before choosing a
role. Peers expire after two seconds without presence.

The lowest station MAC among present suits coordinates one three-entry role
table. Existing A/B/C claims are placed before newcomers, so joining does not
move a running costume. Free roles are assigned in order. A role request carries
a monotonically increasing request number; when the requested role is occupied,
the coordinator swaps the two entries. Repeated Hellos and periodic role-table
snapshots make requests converge despite dropped packets. Table revisions and
boot IDs reject stale assignments from an earlier coordinator instance.

Role presence participates only in identity coordination. Show ping, transport,
leadership and LED ownership still require the Show menu or active output.
Role coordination is separate from playback leadership. The
coordinator resolves identity conflicts; whichever suit starts a track becomes
the audio and transport authority.

Initialization allocates only the receive queue and role state. The large Show
renderer is created lazily when a package is first loaded, so background A/B/C
presence does not reserve its geometry buffers on suits using only Live Sync.

## Starting and changing leadership

Starting validates that the local role is settled, loads that role, claims LED
output, increments the retained leadership term and creates a new session. The
audio file is `/sdcard/shows/<slug>/track.wav`. Rendering remains paused until
Audio Player reports actual submitted PCM, so task creation latency cannot make
the lights begin early.

A follower may start a new track and become leader. Higher leadership terms
supersede lower ones. Simultaneous starts at the same next term are resolved by
the lower MAC on every suit. The winning packet causes an old local leader to
stop its audio and follow, without changing costume roles. Stop packets must
match leader, term, session and generation, preventing delayed releases from
ending a newer song.

Changing role during playback preserves the session and audio owner. Only the
new role program is loaded, with a one-second retry limit after load failure.
The next frame samples current song time, so this unavoidable SD operation does
not create a queued animation delay.

## Audio clock

The leader reads the shared Audio audible timeline. Bluetooth uses the
configured output delay and local output uses its bounded queued-DMA estimate.
Show additionally clamps position monotonic so an update race cannot move the
animation backwards.

Paused audio or a progress stall freezes the playhead. A start that produces no
PCM for five seconds fails the session. Completion releases the show after the
audio task becomes inactive and its last progress is old. The default Bluetooth
delay is 100 ms and the app adjusts it in 25 ms steps.

## Follower clock synchronization

The leader broadcasts transport state every 250 ms and pings live followers
once per second. The ping exchange estimates round-trip time and follower-minus-
leader clock offset. A corrected unicast transport then gives that follower a
local target anchor. The follower renders from this absolute anchor, which also
lets a suit load and join a song already in progress.

Coarse broadcasts establish/recover a session but do not overwrite a corrected
anchor for the same leader, session and generation. If transport disappears for
two seconds, sync status becomes false while the already established local clock
continues. Pause/resume and session changes increment a generation so stale
transport cannot undo the new state.

The Link receive callback only validates the outer protocol and queues a bounded
copy. Role, transport and package work runs later in the shell task, outside the
radio callback.

## LED ownership and rendering budget

Acquiring a show records and disables standalone animation, beat and
audio-brightness modes, then claims the LED output owner. Release restores the
recorded modes. This makes the show frame the sole writer without permanently
changing the user's previous LED configuration.

Rendering targets 60 fps and always uses current absolute song time. The runtime
measures calculation plus LED submission. An expensive frame raises the next
render interval immediately; inexpensive frames reduce that budget gradually.
Skipped deadlines are discarded, so animation never queues behind audio or
radio service. Identical paused playhead/brightness frames are reused. Package
fps hints remain metadata and do not cap old 30 fps shows.

The runtime logs achieved/target FPS, average and worst render cost, calculation
and submission maxima, and internal heap. A diagnostic no-LED mode exists for
isolating physical output costs without changing renderer timing.

## Public status and verification

The status snapshot exposes role settlement, peers, role masks, leader role,
conflicts, show identity, playhead, sync, ownership and the latest short error.
It is calculated from service state and current peer expiry; apps do not inspect
wire structures.

`tests/test_runtime.py` covers audio anchoring, stalls, pause, output ownership,
late frames, adaptive rendering and bounded UI drawing. `tests/test_roles.py`
simulates three independent suits through discovery, packet loss, swaps,
concurrent requests, leadership takeover, late joining and coordinator rejoin.
Physical verification must still cover real ESP-NOW loss, Bluetooth delay and
three-suit visual alignment.
