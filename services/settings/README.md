Pending review

# Settings service

The Settings service owns persistent user preferences and applies them to their
runtime owners. Its model contains speaker and Bluetooth gain/mute, the global
direction-icon preference, the selected LED renderer and animation, ambient
speed and backgrounds, and the complete shared LED colour profile.

At boot it starts from safe defaults, mounts the SD card when available, parses
`suit_settings.txt`, validates each known value and applies the resulting audio
and UI state. LED values are retained until suit geometry has initialized; the
shell then applies the complete colour and animation snapshots and starts the
saved renderer. Unknown or malformed lines leave their existing default intact.

Updates replace the in-memory model immediately, apply the affected runtime
state, and optionally rewrite the settings file. Animation and colour setters
publish complete snapshots to both LED renderers, keeping their common controls
coherent when the active renderer changes. A failed save leaves the live choice
active for the current boot. The returned model is shared read-only; callers
change it through the service setters.
