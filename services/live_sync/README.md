Pending review

# Live Sync service

Live Sync makes one suit reproduce another suit's live LED output. The LED
Source menu chooses Follow A, B or C. Every suit using a local FFT or manual
source publishes automatically, so there is no separate master mode.

## Identity and authority

Live Sync uses Show Runtime's settled A/B/C role as suit identity. It enables
role presence without opening the Show player or accepting Show playback. A
follower answers only the role it selected and accepts state only from that
sender and session. Sequence numbers reject delayed state packets. Remote
Control remains the way to change the source suit's settings; the source suit
is the only configuration authority.
The service status exposes both local identity and selected-source health for
the Source screen and HUD.

The Link callback copies protocol packets into a fixed queue. Clock estimation,
renderer changes and responses run later in the shell task, keeping radio
callback work bounded and avoiding renderer access from the Wi-Fi task.

## Shared state

State includes the active Beat or Ambient renderer, animation, brightness,
speed, both colours, colour cycle/style, highlight, backgrounds, audio
brightness/range, BPM, phase and animation time. The follower applies a whole
renderer configuration at once and advances animation locally between the
four-per-second updates.

Entering Follow saves the local renderer configuration. Returning to FFT or
Manual restores it, so received choices never become persistent settings.

## Clock synchronization and loss

Timestamped ping/pong exchanges estimate follower-minus-source clock offset and
one-way delay. Beat anchors are sent 120–450 ms into the follower's future.
Small timing errors are corrected gradually; an error larger than one period
reanchors the next beat. Ambient animation time is sent with a follower-local
sample timestamp and then advances locally.

The link is considered lost after 2.5 seconds. The HUD and Source menu expose
that state, while the last valid clock continues locally instead of freezing.
A new session from the selected role replaces the old one.

## Verification

The firmware build verifies component integration. Hardware acceptance needs
two or three suits: verify A/B/C selection, all renderer settings, FFT/manual
timing, brief and long packet loss, source restart, and restoration after
leaving Follow.
