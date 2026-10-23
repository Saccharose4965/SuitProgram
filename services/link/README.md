Pending review

# Suit link service

## Purpose and paths

Link provides one framed communications API to suit features. ESP-NOW is the
primary suit-to-suit path. An optional station Wi-Fi/UDP path reaches the PC
telemetry endpoint and acts as the framed fallback when ESP-NOW initialization
fails. Both transports may be initialized together while the reported active
path remains ESP-NOW.

Initialization owns the common Wi-Fi station base required by ESP-NOW and
Telemetry. It initializes NVS, the network stack, event loop, default station
interface and Wi-Fi driver once. ESP-NOW accepts either a configured peer or
the broadcast address. Telemetry joins the configured access point and owns
its UDP sending session.

An initialization is successful only when at least one configured transport is
ready. A missing Wi-Fi configuration no longer counts as a successful fallback
when ESP-NOW fails.

## Frame format

A link frame begins with four bytes: one message type, one flag byte and a
native-endian 16-bit sequence. The remaining payload is at most 200 bytes.
Message types separate information, Messages, games, master/show control and
Call audio. A single callback registered by the shell dispatches these domains
to their services.

ESP-NOW receive metadata supplies the six-byte sender MAC. UDP constructs a
six-byte source identity from the IPv4 address and port. That UDP identity is a
transport token and has no MAC-address meaning.

The frame protocol is compact and private to this firmware. It currently has
no version, byte-order declaration, authentication, integrity field or replay
protection. Protocol changes require simultaneous review of Show, master
control, Paint, Pong, Messages and Call.

## Sending and acknowledgements

Unacknowledged ESP-NOW frames are submitted directly. Acknowledged frames are
serialized by a mutex, retried up to three times and wait 60 ms per attempt.
The receiver immediately returns an ACK containing the sequence. Direct sends
match both sequence and peer; broadcast sends accept the first matching ACK.
Sequence allocation is protected by a critical section so concurrent
unacknowledged senders cannot reuse an increment accidentally.

If ESP-NOW submission fails, generic sends can fall back to Telemetry UDP.
Explicit peer addressing and Link ACKs only have full meaning on ESP-NOW.

## Receive and background work

ESP-NOW invokes parsing from its receive callback. Registered feature handlers
therefore need short, non-blocking receive paths. When a receive callback is
installed and Wi-Fi is ready, one pinned UDP task binds the configured port and
dispatches datagrams. Task-creation failure is returned to the registrant.

The optional information task snapshots battery, LED mode, connection and
clock state at a fixed period and broadcasts a nine-byte version-one payload.
Its period is fixed when the task starts and it lives until reboot.

## Ownership limits

Link has one raw callback and one framed callback. Registration replaces the
previous pair; the shell is the intended framed fan-out owner. Callback fields,
transport readiness and initialization are process-global and currently assume
boot-time setup followed by steady use. Runtime reinitialization, callback
replacement during traffic and shutdown are not coordinated.

Telemetry still has separate socket/ACK concurrency limits described in its
own service documentation. Those must be resolved together with any Link
protocol redesign.

## Verification

The firmware build verifies all users against the service API. Hardware review
should boot with ESP-NOW only, both paths, invalid Wi-Fi credentials and a
forced ESP-NOW failure; confirm the reported path in every case. Two suits
should exchange acknowledged direct frames, broadcast frames and simultaneous
unacknowledged feature traffic while the information broadcast runs.
