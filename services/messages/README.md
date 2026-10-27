Pending review

# Message service

The Message service delivers completed WAV files to another suit and tracks new
inbox arrivals for the shell. Recording, browsing and playback controls belong
to the Message app.

## Storage and delivery

Recordings are written under `/sdcard/messages/outbox/`; received files are
completed under `/sdcard/messages/inbox/`. A transfer writes a temporary file,
checks ordering and size, then closes and renames it before reporting arrival.
That rename is the point at which the UI may treat the message as available.

Delivery first looks for an accepting suit over the direct ESP-NOW protocol.
The sender uses bounded retries, ordered chunks and a whole-file checksum. A
direct transfer either completes with an acknowledgement or fails as that
route; it does not also send an uncertain duplicate. When no direct receiver is
available, the PC UDP message server is the fallback.

Both transports are intended for trusted suits and networks. They provide
transfer checks, not identity, secrecy or protection against hostile packets.

## Arrival indicator

Successful inbox completion increments an in-memory arrival sequence. The menu
compares that sequence with the last acknowledged snapshot and caps its displayed
count at 99. Opening and successfully scanning Inbox acknowledges only the
snapshot seen before the scan, so a message arriving during that work remains
unseen.

This counter records arrivals during the current boot. The SD files persist;
the unread badge does not. It is deliberately driven by completed delivery
instead of repeatedly scanning the card while drawing menus.

Host tests cover direct-transfer ordering, checksums and arrival snapshot races.
Hardware review still needs two-suit delivery, PC fallback and interrupted
transfers.
