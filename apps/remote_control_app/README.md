Pending review

# Remote Control app

The entry screen lists recently heard suits. Selecting one begins a target
lease and changes the controller into a virtual shell rooted at the target's
main menu. The controller remains in this host app; mirrored app initializers,
workers and hardware calls never run locally.

Standard list and grid snapshots are redrawn with the shared UI widgets and
animate locally between acknowledged selections. Custom apps show their app ID
and an explicit placeholder. A stronger four-corner mark identifies the
controller view; the target receives a smaller corner mark.

During a session A/B/C/D are discrete remote presses. Local target input is
ignored. Holding A+B for three seconds on either suit sends an end request and
returns both shells to their main menu. Heartbeat expiry has the same local
cleanup behavior.
