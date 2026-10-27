Pending review

# MPU acquisition

The driver receives an existing SPI device handle. It resets and wakes the
sensor, disables I2C, selects 200 Hz internal sampling, filtering near 42–44 Hz,
±4 g acceleration and ±500 degrees/second rotation ranges. These settings set
the conversion factors used when decoding samples.

Initialization checks identity up to four times, accepting the three IDs already
supported by this hardware. SPI errors return immediately; an unexpected identity
is logged and reported as an invalid response.

Each reading fetches acceleration, temperature and gyro in one burst. The
driver converts signed values into g, degrees/second and Celsius, and timestamps
the completed reading. Calibration and orientation estimation belong to the
[orientation service](../../services/orientation/README.md).

A mutex serializes this driver's SPI transactions. Optional periodic acquisition
uses delay-until scheduling and publishes the latest complete sample under a
critical section. Failed reads leave the previous sample and timestamp intact,
allowing consumers to detect stale data.

The caller selects sampling rate, task priority, core and stack allocation.
Starting an existing sampler succeeds without changing its settings. There is
no stop API. The stack-size parameter is explicitly measured in bytes; zero
selects the 4096-byte default used by both current callers.
