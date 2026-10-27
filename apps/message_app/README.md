Pending review

# Message app

The Message app is the visible recording, inbox and outbox workflow. It asks the
Message service to deliver files and observe arrivals, while Microphone and Audio
Player own capture and playback.

## Pages and controls

The first page offers New, Outbox, Inbox and Back. New records one 16 kHz mono
WAV, previews it and sends it. Outbox and Inbox scan their directory only when
opened, cache at most 96 valid WAV names, then navigate that cache without SD
work during drawing. Inbox is acknowledged only after a complete, untruncated
scan.

Starting a recording stops FFT capture and existing playback, then disconnects
Bluetooth if needed so the microphone can own I2S. A new filename is selected
after valid WAVs and unfinished `.part` files, preventing a failed recording
from being overwritten. Leaving the app stops an active recording. The receiver
remains a background service, and an already-started send is allowed to finish.

## Display state

The app shows the selected file, recording duration, transfer progress and the
latest error. During recording it draws the peak envelope already accumulated by
the Microphone service. The cached envelope freezes after stop and requires no
second audio reader or draw-time file access.

The send task keeps its source path immutable until completion. Direct suit
delivery is attempted first; absence of a receiver selects the PC fallback.
Input is ignored while that transfer is active so navigation cannot replace the
file underneath it.

The UI host test covers filename allocation, controls, list bounds and drawing
at every viewport size. Hardware review still needs recording, preview, direct
delivery, PC fallback and audio ownership recovery.
