Pending review

# Bluetooth audio screen

This app presents discovered Classic Bluetooth audio sinks and maps shell
buttons to the A2DP service. Opening the screen tells audio policy to suspend
FFT ownership, then starts discovery only when the service is not already
connected, connecting, streaming or discovering. Closing releases that FFT
blocker without disconnecting Bluetooth. Pending FFT client requests remain in
Audio Policy and can resume once the Bluetooth service is idle.

A/B move through up to eight devices, C starts a fresh scan, and D connects the
selection. Selecting the current peer disconnects it. Selecting another device
first requests disconnect, then asks the service to connect the new address;
the service owns deferred connection attempts while the old link closes.

The display shows service state, connected name and four visible devices. A
connected peer missing from the latest discovery list is inserted temporarily
so it remains selectable. Unnamed devices display the last three address bytes.
The viewport follows selection. Drawing only reads Bluetooth state; connection
lifecycle and playback control continue even when this screen is closed.
