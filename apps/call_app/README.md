Pending review

# Call app

This app presents the direct-suit Call service. Entry starts a session; exit
requests a clean stop. Button A toggles between listening and talking. The
screen displays the current direction, successfully sent and accepted packet
counts, and the service's latest ESP error name.

The app does not process Link frames, touch I2S, select a peer or coordinate
FFT/Bluetooth. Those choices belong to the Call service, so radio delivery and
audio cleanup continue to follow one lifecycle even when the shell redraw rate
changes.
