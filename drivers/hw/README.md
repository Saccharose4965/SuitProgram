Pending review

# Board support

hw.h centralizes wiring. hw.c brings up shared GPIO, SPI and ADC resources.
Peripheral drivers use these resources with their own protocols.

## Shared SPI

Startup drives chip selects high, initializes OLED control pins and creates
SPI2 with DMA and a 4096-byte transfer limit. An already-initialized bus is
accepted. Device registration remembers up to eight handles by chip select,
allowing later callers to reuse a handle.

Reuse keeps the original device configuration even if a caller requests a
different speed or mode. Once tracking is full, new devices still register but
cannot be found through this table. Registration has no additional mutex and
should be coordinated during initialization. Driving every chip select high is
an initialization operation, requiring other transfers to be idle.

## Shared ADC

The ADC path releases RTC pad holds, configures the three wired ADC1 channels
for 12-bit conversion and attempts line-fitting calibration. Initialization and
reads share a mutex with a 100 ms acquisition timeout.

Millivolt readings use calibration when available. If calibration is unavailable
or conversion fails, they use a linear 0–3300 mV approximation. Channel setup
errors are logged without aborting, so initialization success alone does not
prove every channel works.

## Ownership boundaries

This module retains the physical I2S pin definitions, while the
[audio I2S driver](../audio_i2s/README.md) exclusively owns channel creation,
sample-rate changes and TX/RX direction. The [input driver](../input/README.md)
owns button-voltage decoding and debounce. Board support exposes the shared ADC
reads those drivers need and contains no competing I2S or button state.
