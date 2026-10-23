Pending review

# I2S audio driver

This driver owns the ESP32 I2S peripheral shared by the microphone and local
speaker. It creates the paired receive/transmit channels, applies their common
sample clock, controls the optional amplifier-enable pin and exposes the raw
channel handles needed by streaming services.

## One bus with two directions

The hardware shares bit clock and word-select between input and output. The
driver therefore allocates RX and TX as a full-duplex pair and keeps one saved
pin configuration. TX is always created because it supplies clocks to the
microphone; RX is discarded after initialization when no input pin is present.
Both directions use stereo 32-bit Philips slots. Microphone sample alignment
and speaker sample conversion belong to their consuming services.

Eight 256-frame DMA descriptors balance buffering with internal-memory cost.
The amplifier starts muted. Enabling capture also mutes it, enables TX to
generate the shared clocks, writes a short silent block and then enables RX.
Enabling playback arms the paired RX channel when needed because the ESP32
full-duplex driver can otherwise leave TX unable to drain. A silent DMA prime
turns that hardware failure into an immediate error before a playback loop
starts. Failed enable or prime attempts mute and disable TX before returning,
so a caller never inherits a half-started speaker path.

## Clock changes and observations

Rates in the 44.1 kHz family select APLL; 8/16/32/48 kHz-family rates use the
default clock. Reconfiguration disables any running direction before applying
the new shared clock. `audio_i2s_get_rate()` returns zero while this transition
is active, allowing capture code to discard a window spanning two clock
domains. A monotonically increasing clock revision gives the same protection
when a caller retained a channel handle across a change.

The transition flag is cleared on both success and failure. The previous
implementation returned directly from a failed reconfiguration and could
leave the reported rate at zero until reboot.

The RX overflow callback increments a diagnostic counter in interrupt context.
Consumers compare revisions and overflow counts before and after a read to
decide whether its samples form a valid uninterrupted window.

## Release and restoration

`audio_i2s_release()` disables both directions, mutes the amplifier and deletes
the DMA-backed channels while retaining the pin configuration and requested
rate. A later rate or enable request reconstructs the bus and restores that
rate. The audio service checks output ownership before requesting release;
capture/playback policy remains above this hardware layer.

`audio_i2s_disable_tx()` is the local-playback ending operation: it mutes and
stops the speaker direction while leaving RX state alone. `disable_all()` is
used when Call or microphone ownership deliberately changes the entire bus.

Hardware verification should cover repeated FFT/playback/recording transitions,
44.1 and 48 kHz files, Call direction changes and amplifier pop suppression.
