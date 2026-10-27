Pending review

# Music app

The Music app browses PCM WAV files at the SD-card root and controls the shared
Audio Player. Opening the app mounts the card and builds a bounded catalogue of
up to 100 filenames in PSRAM. Drawing and scrolling use that cached catalogue
and perform no filesystem work.

A/B move through the list. C starts the selected file or toggles pause for the
active player. D restarts the selected file, and holding D stops playback. A
pending replacement waits for Audio Player to release its previous worker before
starting, preventing two playback tasks from owning the route simultaneously.

The app owns selection and filenames only. WAV parsing, Bluetooth/local route
selection, progress and teardown belong to Audio Player. It has no Bluetooth
dependency and observes playback through the Audio Player API.
