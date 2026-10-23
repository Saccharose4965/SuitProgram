Pending review

# Game high scores

This service stores one unsigned high score per game on SD. Games supply a key
and candidate score; the service owns filenames, loading and the decision to
write. It has no UI or background task. Operations block the calling task during
mounting and file access.

## Identity and loading

Keys contain 1–40 alphanumeric, underscore or hyphen characters. This keeps
game names inside the scores directory. The service mounts SD and creates that
directory if needed, then reads scores/<key>.txt as a decimal integer.

A missing file returns zero successfully. Other open failures return an error.
An unreadable numeric value currently also returns zero successfully. Parsing
accepts a numeric prefix without checking trailing content or overflow; stricter
validation remains pending.

## Updating

The current score is loaded before comparing. Equal or lower candidates cause
no write. A higher candidate is written to a temporary file, flushed, synced
when a file descriptor is available, and closed. Any detected failure removes
the temporary file and returns an error.

The final step renames the temporary file onto the destination. This keeps
the write separate from the old file, but replacement depends on filesystem
rename semantics. FAT replacement of an existing file needs verification and
may require the backup/install approach used elsewhere. There is no backup
recovery here, so this must not be described as guaranteed atomic persistence.

On success the caller receives the resulting high score. There is no mutex
around read/compare/write and the temporary filename is shared per key.
Callers must serialize updates to the same game.

The module already has a coherent service boundary: score policy and persistence
stay together, with card access supplied by the SD driver. Migration preserves
the API, file format and current error behaviour.
