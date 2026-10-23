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

## Remaining ownership cleanup

Legacy fixed-rate I2S setup helpers still exist alongside the audio component's
bus setup. Their independent handles and partial-initialization behaviour need
an audited consolidation before further use.

A legacy voltage-based button decoder also remains here. The active input
driver owns its decoding and debounce; callers and diagnostic uses must be
checked before removing the older API.
