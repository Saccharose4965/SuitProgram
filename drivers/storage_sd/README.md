Pending review

# SD access

The driver mounts FAT at /sdcard on the shared SPI bus. Formatting on mount
failure is disabled so a connection problem cannot erase the card. Mounting
tries 10 MHz, then 5 MHz, 2 MHz and 400 kHz. The lower initial speed was chosen
after sustained reads produced CRC errors at 20 MHz.

Before mounting, board setup deselects the peripherals and sets SD-related
pull-ups and drive strength. FAT allows eight open files. A successful mount is
cached; unmount releases it. A statically allocated lifecycle mutex serializes
the complete mount retry sequence and unmount, so a second caller waits for the
active lifecycle operation and then observes the resulting mounted state.
Callers must still finish file activity before requesting unmount.

Path construction joins the mount point and caller-provided relative path,
reporting insufficient buffer space. It does not mount the card or sanitize
path traversal. Directory listing is a diagnostic operation on an already
available filesystem.

Recording-number persistence belongs to the
[recording service](../../services/microphone/README.md).
