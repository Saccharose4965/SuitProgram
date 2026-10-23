Pending review

# TCP file receiver

Despite its historical name, this service receives arbitrary file bytes over
TCP and stores them on SD; it does not acquire microphone samples or play
audio. It owns a listening socket, one connection-handling task and, when
memory permits, a second SD-writer task.

## Connection and naming

The server listens on port 5000 by default with a backlog of two and handles
one client at a time. The optional first line `SUITRX1 filename` supplies a
name. The line may arrive over several reads and is limited to 256 bytes; an
incomplete or oversized header becomes ordinary payload. Path prefixes are
discarded, unsupported characters become underscores, and names without an
extension receive `.wav`. Existing files are preserved by adding `(1)`, `(2)`
and so on. Successful output is also recorded in `/sdcard/rx_rec_last.txt`.

Each transfer writes `<final>.part`. Successful completion flushes and syncs
the file before renaming it. A failed transfer deliberately retains its partial
file for inspection, except a failed final rename removes the temporary file.
The protocol has no declared byte count, checksum, authentication or success
response; connection close marks the end of input.

## Buffering and scheduling

The network task uses a 4096-byte read buffer and limits ingress to roughly
100 KiB/s. It normally feeds a 2 MiB PSRAM byte ring while a priority-4 writer
writes SD in 4 KiB slices. Low/high watermarks pause and resume network reads.
If ring or writer allocation fails, the network task writes directly with the
same 4 KiB slicing. Both tasks use core 1 on a dual-core build so core 0 remains
available for Wi-Fi/Bluetooth work.

After the client closes, the receiver waits for the writer to drain. Progress
extends the wait up to sixty seconds; prolonged stalls trigger forced task
deletion followed by best-effort flush, sync and close. Recovery is accepted
only when the ring was drained, byte counts agree and finalization succeeded.

## Lifetime

Start creates one server task and treats an existing task as success. Stop sets
a shared stop flag, closes the listening socket to unblock `accept`, and waits
up to two seconds. It can still wait for a client read timeout or writer
cleanup. The start/stop flag is now established before task creation so an
immediate stop cannot be overwritten by the task's entry code.

The service assumes Wi-Fi and SD are prepared by callers. It exposes no current
transfer progress or last path through its API; the UI can only show whether
the server task exists.
