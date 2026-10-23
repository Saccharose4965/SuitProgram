Pending review

# File receiver screen

This screen controls the TCP file receiver and reports its state, SD mount,
port and most recent start/stop result. C toggles the service; the other three
buttons have no action.

Starting first mounts SD. Because reception allocates a large PSRAM ring and
writes sustained SD traffic, the app stops the FFT visualizer and disconnects
Bluetooth when music is idle. Active music keeps Bluetooth and FFT state
untouched so opening the utility does not interrupt playback. This policy is
currently local to the screen rather than the shared audio-policy service.

The receiver continues after leaving the screen. Shell shutdown and Service
Restart also call the receiver service directly, so app deinitialization does
not stop it. Drawing only reads service and mount state; it performs no network
or file work.
