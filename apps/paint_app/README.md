Pending review

# Paint app

## Controls and lifecycle

The app owns menu selection, button interpretation and screen text. Role and
arm choices use the shared three-visible-item menu blocks and scrolling. Selecting
Painter or Canvas sends a command to the [Paint service](../../services/paint/README.md).
The displayed role follows the returned status, so a failed Canvas entry leaves
the user in the menu with an explanation.

Painter offers calibration, colour selection and brush activation. Canvas offers
clear. Back leaves the active role before returning to the menu; exiting the app
also requests that cleanup. This ordering releases the active session's resources
before the screen changes.

App entry and exit also set the existing FFT gate. This keeps the suspension
aligned with the Paint screen's lifetime, including its menu.

## Display

Each draw reads a status snapshot containing mode, arm, calibration, aim, colour,
brush activity, peer freshness and error. Error codes become user-facing strings
here. Menu selection remains local because browsing entries only affects the UI.

Commands, status reads and drawing run on the shell task. The current layout uses
fixed pixel offsets and has limited clipping. Layout improvements are tracked in
[the work queue](../../TODO.md).

The app/service regression test exercises button navigation and session commands
with fake hardware. Physical interaction remains a hardware acceptance check.
