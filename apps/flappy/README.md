Pending review

# Flappy screen

The app owns the splash, framebuffer, button handoff, drawing and worker
lifecycle. The [game service](../../services/flappy_game/README.md) owns rules.

Any shell press sets a pending flag. The worker consumes one pending press to
leave the splash and later uses presses to flap while alive. Multiple presses
before consumption collapse into one. Pending input is cleared on entry.

The worker has priority 5, a 4096-byte stack, and runs on core 1 on dual-core
builds or core 0 otherwise. Each gameplay iteration advances the service,
draws pipes/bird/ground/scores and waits about 33 ms. Processing time adds to
this delay, preserving the former fixed-step behaviour.

The splash contains a bird preview and decorative bars. During play, the bird
uses the service's collision mask; death flips it and blinks it in six-frame
intervals. Current/best scores use the existing tiny digit font. Layout uses
the full panel. The shell copies the framebuffer without synchronization.

## Existing lifecycle limits

The splash polls presses every 10 ms but does not check stop requests. Leaving
there can strand the worker. Exit waits only 500 ms and then clears its handle,
so a later entry could start a second worker sharing the same state. That
requires an explicit lifecycle fix before calling this app robust.

After a completed run the task exits and the final game frame stays visible;
the former placeholder menu drawing was immediately overwritten by that frame
and has been removed. There is no in-app restart prompt or automatic navigation.
Task creation failure is not reported to the screen.
