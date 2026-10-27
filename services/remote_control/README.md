Pending review

# Remote Control service

Remote Control establishes one temporary controller/target lease between two
suits. The controller sends discrete button presses; the target shell executes
them in its normal app context. Local target input is gated by the shell for the
lease, while either suit can end it with the same extra-long A+B gesture.

The protocol uses a random session, monotonic per-direction sequences,
target-addressed frames, heartbeats and a three-second expiry. Start, input and
end packets use Link acknowledgement and retry because losing one changes the
meaning of the session. Repeated view state and heartbeats are best-effort.
Link callbacks only validate packets, update bounded state and enqueue shell
commands. They do not switch apps or call app handlers.

Standard UI is returned as compact semantic snapshots: layout, app ID, title,
selection, nearby labels and legend. The controller renders these through the
shared widgets without starting the corresponding local app. Custom screens
produce an app-name placeholder. Pixel framebuffers are never transmitted.

INFO broadcasts provide the bounded recent-peer list. Starting a session sends
both shells to their main-menu state: the target runs the real menu while the
controller remains inside the Remote Control host app.
