Pending review

# Pong app

The app owns AI/Human selection, button mapping and OLED drawing. Opening
initializes a fresh service session. In the mode menu, A/B move through the two
choices and D confirms. During a match, holding A/B moves the local paddle
continuously up/down. D pauses AI play and has no match action in Human mode. C
is unused. The shell's
global exit gesture remains available; Pong has no app-specific back action.

The mode picker uses the shared menu blocks. The button legend follows the current screen. It shows selection controls in
the menu, paddle controls during play and no controls while a Human match is
waiting or connecting.

Rendering consumes one immutable service snapshot. Menu and connection text,
court lines, paddles, corrected ball position, score, local-side marker and
pause indicator are presentation data only. Radio packets and simulation do
not depend on the framebuffer.

Human mode is unavailable when the service could not allocate its receive
queue. Hardware review should cover selection, waiting/connecting screens,
both player sides, score placement, AI difficulty and AI pause.
