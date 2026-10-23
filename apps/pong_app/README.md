Pending review

# Pong app

The app owns shell navigation, AI/Human selection, button mapping and OLED
drawing. Opening initializes a fresh service session. A leaves from the mode
menu and resets an active match back to that menu. B/C select a mode or move
the local paddle; D starts the selected mode and toggles pause in AI play.

Rendering consumes one immutable service snapshot. Menu and connection text,
court lines, paddles, corrected ball position, score, local-side marker and
pause indicator are presentation data only. Radio packets and simulation do
not depend on the framebuffer.

Human mode is unavailable when the service could not allocate its receive
queue. Hardware review should cover menu return, waiting/connecting screens,
both player sides, score placement and AI pause.
