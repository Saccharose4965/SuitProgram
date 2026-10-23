Pending review

# LED strip driver

This driver converts two physical RGB byte streams into the WS2815 waveform on
the suit's two data pins. It owns both RMT channels, their encoders, transmission
serialization and bounded timeout recovery.

## Wire encoding

RMT runs at 10 MHz, giving 0.1 microsecond symbols. A zero uses roughly 0.3
microseconds high and 1.2 low; a one reverses those durations. Bytes are sent
most-significant bit first. Each frame ends with a 600 microsecond low symbol,
formed from two 300 microsecond halves, so the strips latch their new colors.

Each output receives four 64-symbol RMT memory blocks. The larger hardware
buffer reduces refill interrupts without allocating DMA-capable frame memory.
The service supplies complete immutable byte arrays; the driver does not know
logical LED sections, costume geometry or animation meaning.

## Two-output transaction

One static mutex covers initialization and a complete write. A write first
waits for any earlier work on both channels, submits each nonempty output, then
waits for both to finish. Waiting after a partial submission is important: if
the second channel rejects a frame, the first channel may still be reading its
buffer.

The caller may reuse its physical buffers as soon as `led_strip_write()`
returns. This synchronous contract is the boundary that lets the LED service
accept a newer logical frame while the current physical frame is on the wire.

## Timeout recovery

A transmission timeout disables and re-enables both channels once, then
retries the same buffers. The error is returned after that bounded attempt.
There is no unbounded recovery loop inside the driver, so a damaged or
disconnected strip cannot permanently occupy its calling task.

Initialization is idempotent. If one output initializes and the second fails,
a later call reuses the complete first channel and retries the missing one.
GPIO drive strength is raised after the channel is enabled.

Hardware acceptance should verify both outputs together at the maximum wired
length, timeout recovery, RGB order and the test pattern through the service.
