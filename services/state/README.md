Pending review

# Shared status

This service holds the latest values used by status displays: LED mode and name,
FFT running/BPM, Live Sync source/link status, three battery percentages,
connection state and clock. Producers publish values; readers receive a copy
of the whole structure so a screen can use a consistent snapshot.

A mutex protects updates and copies. Initialization should happen during boot
before concurrent users start; the lazy initialization fallback is not itself
synchronized. Calls can wait for the mutex and belong in task context.
No task, persistence or notifications are created here.

Related fields are updated together: FFT running/BPM, sync source/link and clock
validity/time. Battery values clamp to 100 and invalid slot indices are ignored.
LED names are copied with a length limit; an empty name preserves the old label.
Valid times wrap hours and minutes into range. Invalidating the clock retains
its last numbers but clears validity.

Initial values describe idle LEDs, stopped FFT, disconnected networking and
unknown time. Battery slots start at zero. Calling initialization again retains
the accumulated status.
