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

A missing score returns zero successfully. A stored value must contain one
unsigned 32-bit decimal integer with optional surrounding whitespace. Empty
files, signs, overflow, embedded zero bytes and trailing content are rejected.

Loading inspects the primary, backup and temporary copies. It returns the
highest valid value, matching the monotonic high-score rule, and repairs the
primary copy when recovery won. If files exist but none are valid, loading
returns `ESP_ERR_INVALID_RESPONSE`; corrupt evidence is not silently replaced
with zero.

## Updating

The current score is loaded before comparing. Equal or lower candidates cause
no write. A higher candidate is written, flushed and synced to a temporary
file. A valid primary is then moved to the backup name before the temporary file
is installed as primary, avoiding replacement-rename assumptions on FAT. An
invalid primary is removed while an existing valid backup is retained. Failed
installation attempts restore the previous primary where possible; remaining
backup or temporary files are handled by the next load.

On success the caller receives the resulting high score. There is no mutex
around read/compare/write and the temporary filename is shared per key.
Callers must serialize updates to the same game.

The decimal primary format and public API remain unchanged. Host regressions
cover missing files, monotonic updates, strict parsing and recovery from backup
or an interrupted temporary write. Hardware acceptance should remove power at
different points of a record update on the actual FAT card and confirm the next
boot retains one of the valid monotonic values.
