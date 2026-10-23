Pending review

# Telemetry service

Telemetry provides the older Wi-Fi/UDP transport used by Link, Messages, Call
and Comms. It owns one outgoing datagram socket, the destination addresses, the
sender Wi-Fi MAC and an optional file-receiver task. Wi-Fi base initialization
belongs to Link; `telemetry_start` assumes the default station netif and event
loop already exist.

## Network session

Starting validates the configured SSID, PC address and port, then creates its
event group and subscribes to station-IP events once. If the station is already
connected to that SSID with an address, startup reuses it. Otherwise it applies
the credentials, starts Wi-Fi, connects and waits up to ten seconds for an IP.
It then replaces the outgoing socket, enables broadcast, calculates the subnet
broadcast address and sends `hello` to both configured PC and broadcast targets.

Text lines and formatted MPU samples go to both destinations. Success means at
least one complete datagram was accepted by the local socket.

## Transfer format

A transfer begins with a native-endian header containing `FILE`, total byte
count, sender MAC, filename length and filename. Data packets contain `FILE`, a
zero-based sequence, payload size and up to 1024 payload bytes. This format is
local to the project: it has no version, byte-order declaration, checksum or
authentication.

File sending reads sequentially, yields after every four chunks, reports
progress, then waits up to three seconds for a matching peer address/port ACK.
Buffer sending uses the same packets with an empty filename, does not pace its
chunks, checks for an ACK only once without waiting, and returns success after
socket submission. The two APIs therefore provide different delivery strength.

The receiver task binds one UDP port and accepts named files up to 16 MiB. It
rejects separators, `..`, embedded NULs, existing final files, wrong sender,
out-of-order chunks and sizes beyond the declared total. Data is written to a
`.part` path, flushed, closed and atomically renamed before the weak arrival
hook and ACK. A transfer times out after three seconds without a datagram or
thirty seconds total. Failed partial files are removed. The receiver runs until
reboot; there is currently no stop API.

## Ownership limits

The outgoing socket and destination state are process globals without a mutex.
Concurrent starts/sends can replace or share the socket. ACK reads also share
that socket, so simultaneous senders can consume one another's packets. The
receiver processes one file at a time and ignores unrelated transfers while
inside that transfer. These constraints need a protocol/session redesign with
all callers before concurrency is expanded.

Initialization now releases its event group when event-handler registration
fails. Embedded suit MAC constants that were never read have been removed;
runtime identity continues to come from the Wi-Fi station MAC.
