Pending review

# Show Player app

This app is the shell-facing Show screen. It discovers packages on the SD card,
turns four-button input into runtime commands and draws a compact lobby,
transport and show list. Session, rendering and networking state remain in the
Show runtime service, so leaving the screen does not stop an active show.

## Discovery and local state

On entry the app tells audio policy that Show is visible, opens runtime presence,
mounts the SD card and scans `/shows`. A list entry is accepted only when it is
a directory with a regular `show.bin`; hidden and overlong names are skipped.
Up to 32 slugs are stored and sorted. Navigation reads this in-memory list, so
scrolling performs no package validation or SD access.

The first three rows are Back, Role and Bluetooth delay. Remaining rows are show
slugs. Selection is clamped after every scan. Long-pressing D remounts and
rescans, providing an explicit refresh after changing the card.

## Controls

A and B move up and down. D activates the selected row. Back returns to the
menu; Role requests the next A/B/C role; Bluetooth delay adds 25 ms. On the
delay row C subtracts 25 ms. On other rows C requests the next role, or stops a
running show when this suit is its leader.

Selecting a show waits until role discovery has settled. Selecting the currently
running show on its leader toggles pause. Selecting another show stops the old
session, loads the selected role and starts this suit as leader. A follower
cannot stop the leader's show from the local screen; it reports that the master
controls transport.

## Display

The top row shows this suit's role and whether it is leader, follower or ready.
A/B/C each use `*` for the playback leader, `+` for a present follower and `-`
for absent. The second row prioritizes mount/discovery/runtime errors and then
shows transport state and elapsed seconds.

Only rows that fit inside the supplied shell content rectangle are drawn. The
list window follows the selection from below and labels are clipped to the 3x5
font width. The runtime host test exercises every list length, selection and
screen rectangle to keep drawing inside those bounds.

On exit the app closes lobby presence and releases its FFT policy request.
Show playback continues because the runtime keeps advertising while it owns LED
output. Reopening the app reconstructs only the SD list and adopts the runtime's
current role and transport state.
