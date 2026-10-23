Pending review

# OLED driver

The driver initializes the panel and sends monochrome pixel buffers over SPI.
It converts the application's row-major, most-significant-bit-first pixels into
the panel's vertical page format. Panel dimensions are 128 by 64.

Full-frame output refreshes all pages by default. The optional partial-update
mode compares packed pages with the previous output and transfers changed spans.
Full refresh trades additional SPI traffic for a predictable complete redraw.

The 64-by-64 transfer helpers support centred and horizontally offset buffers.
Offset output clears the surrounding page span so a moving image leaves no trail.
These functions pack and transmit existing pixels.

A mutex serializes individual SPI transfers. It does not make a whole multi-call
frame sequence atomic; callers must coordinate concurrent panel writers.

The emulated-panel test checks full and partial refresh paths. Drawing operations
are described in the [UI module](../../apps/ui_shell/README.md).
