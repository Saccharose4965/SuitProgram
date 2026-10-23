> Archived reference. Describes an earlier revision; current explanations live beside the code.

# MPU acquisition and orientation fusion

Full-file review of `components/orientation/orientation.c`,
`components/orientation/include/orientation.h`, `components/mpu6500/mpu6500.c`,
`components/mpu6500/include/mpu6500.h` and `components/mpu6500/CMakeLists.txt`.
The active singleton that joins acquisition to fusion is documented in
[orientation and painting](orientation-and-paint.md); consumers are described
in [pose displays](pose-displays.md) and [arm synth](arm-synth.md).
This is a description of the implementation, not a sensor datasheet audit.

## Pipeline and frames

SPI register read → latest physical-unit sample → orientation service →
Madgwick accelerometer/gyro update → absolute quaternion snapshot → app-local
calibration and pointing. The legacy getter instead returns a shared zeroed
quaternion. Neither path measures position or reconstructs a shoulder joint.

The driver preserves register axis order. The filter's `map_accel` and
`map_gyro` are identity mappings too: there is no screen/arm mounting rotation
here. App-level mapping must account for the mounted MPU (+Y towards fingers,
and the screen's different axes). Do not move an app mounting correction into
this layer without auditing every consumer.

## `components/mpu6500/include/mpu6500.h`: objects and ownership

`mpu6500_sample_t` holds acceleration in g, angular velocity in degrees/second,
temperature in Celsius and a signed 64-bit microsecond timestamp.
`mpu6500_config_t` supplies an already-created SPI device and optional borrowed
log-tag string. The driver does not create the SPI bus/device or copy the tag.

`mpu6500_t` is caller-owned and contains that device, tag, unexpected-ID flag,
requested polling frequency, sampler task handle, a snapshot spinlock and the
latest sample. Initialize its `mux` with `portMUX_INITIALIZER_UNLOCKED` before
use; `mpu6500_init` does not initialize that field. Keep the object, device and
tag alive for the sampler's lifetime. There is no stop/deinit API. Reinitializing
a live object is unsafe: init clears its task handle without stopping its task.

The C/C++ interface offers initialization, a synchronous read, starting a
sampler, and copying the latest sample. These are task-context APIs, not ISR
APIs. Caller serialization is required for initialization/start; the snapshot
lock does not make lifecycle changes safe. A null `latest` argument returns
an all-zero sample; timestamp zero is the consumer's unavailable convention.

## `components/mpu6500/mpu6500.c`: bus access and startup

### SPI helpers

A lazily created static mutex serializes transactions across all instances of
this driver. Creation is guarded by a spinlock; acquisition waits indefinitely.
`mpu_spi_transmit_locked` checks object/device/transaction, acquires the mutex,
calls synchronous `spi_device_transmit`, and releases it even on SPI failure.
This mutex does not serialize unrelated SD/OLED code or reserve the shared
bus for an entire initialization sequence. Underlying bus arbitration is a
separate responsibility.

`reg_write` sends two inline bytes: register with bit 7 clear, then value.
`reg_read` sends register with bit 7 set plus a dummy byte and returns the
second received byte. `burst_read` uses 64-byte stack TX/RX arrays and clocks
one address byte plus data, discarding the first received byte. Zero-length
reads succeed immediately; requests above 63 data bytes are rejected. Private
helpers assume valid output buffers supplied by their callers.

Init rejects missing object/config/device with `ESP_ERR_INVALID_ARG`, installs
borrowed handles, clears latest/task/rate and the log flag, then performs:

| Register | Value | Implemented intent |
| --- | --- | --- |
| PWR_MGMT_1 (0x6B) | 0x80 | Reset, then delay 100 ms. |
| PWR_MGMT_1 | 0x01 | Wake/PLL selection, then delay 50 ms. |
| USER_CTRL (0x6A) | 0x10 | Disable I2C, then delay 10 ms. |
| SMPLRT_DIV (0x19) | 0x04 | Source comment targets 200 Hz from 1 kHz. |
| CONFIG (0x1A) | 0x03 | Source comment describes approximately 44/42 Hz filtering. |
| GYRO_CFG (0x1B) | 0x08 | Select range matching 65.5 counts/(degree/second). |
| ACC_CFG (0x1C) | 0x08 | Select range matching 8192 counts/g. |

Delays are converted to scheduler ticks. Any write/read error returns directly;
there is no rollback or device removal inside this driver. Configuration values
are not read back for verification. The driver does not write a separate
accelerometer-filter register, so the CONFIG comment must not be treated as
verification of every compatible part's accelerometer bandwidth.

WHO_AM_I (0x75) is tried up to four times. IDs 0x70, 0x68 or 0x71 are accepted;
this acceptance alone does not prove identical behaviour across devices. Each
unexpected value incurs a 10 ms delay. On exhaustion, it attempts a power-register
read for a warning and returns `ESP_ERR_INVALID_RESPONSE`. The warning flag is
reset on each init; repeated initialization attempts can each log a warning.

### Samples and timing

`mpu6500_read_once` reads 14 bytes from 0x3B: acceleration XYZ, temperature,
then gyro XYZ. `be16` interprets big-endian signed 16-bit values. Conversion is
accel/8192, gyro/65.5, and temperature/333.87 + 21. These constants are applied
to every accepted ID; there is no per-part scale selection or calibration.
The timestamp comes from `esp_timer_get_time()` **after** the SPI read, not a
hardware data-ready timestamp. SPI contention therefore affects timestamp
spacing. On read failure the caller's output is not updated.

The sampler repeatedly reads immediately, publishes successful samples under
`imu->mux`, then waits with `vTaskDelayUntil`. Failed reads silently retain the
previous sample/time. There are no error counters, retries beyond the next
iteration, restart logic, interrupt trigger or FIFO. Polling can repeat a
hardware sample or skip samples; the new host timestamp is not proof that
the sensor produced a new measurement.

Start returns success unchanged if a task already exists. Otherwise defaults
are 100 Hz for nonpositive frequency, priority 2 for zero priority and core 1
for negative core. Default stack argument is `4096 / sizeof(StackType_t)`.
Despite the parameter name `stack_words`, it is forwarded unchanged to
ESP-IDF's `xTaskCreatePinnedToCore`, whose stack-size argument is in **bytes**.
Do not infer a portable 4096-byte allocation without checking the target's
`StackType_t`; the name and expression obscure the contract. Task-creation
failure returns generic `ESP_FAIL`. The requested rate is recorded even on
failure; changing it while running does not change the already computed delay.

The period is `pdMS_TO_TICKS(1000.0f/hz)`, clamped to one tick. Actual frequency
depends on tick conversion, work duration and scheduling. Changing sampler
rate does not reprogram SMPLRT_DIV. `mpu6500_latest` copies the whole sample
under the same spinlock, with no freshness test. No heap buffer is allocated
explicitly by this file; task creation and SPI internals can allocate memory.

## `components/mpu6500/CMakeLists.txt`

Builds `mpu6500.c`, exports `include`, and declares private dependencies on
`esp_driver_spi`, `freertos` and `esp_timer`. The exported header itself includes
SPI and FreeRTOS headers, so private dependency visibility is a build-coupling
detail to retain in future component-isolation audits. No optional build path
or per-board configuration is implemented here.

## `components/orientation/include/orientation.h`: filter contract

`imu_quat_t` is four floats in w,x,y,z order. Caller-owned `imu_orientation_t`
stores initialization/readiness/sample flags, previous timestamp, absolute
body-to-world quaternion `q_BW`, shared recenter reference `q_zero` and gain beta.
The filter is multi-instance, allocation-free and has no tasks or I/O. Use one
serialized owner per instance; there are no internal locks or atomic snapshots.

Initialize before updating. Update accepts acceleration in g, gyro in degrees/
second and monotonically increasing microseconds. `ready` checks both ready
and have_sample, not age, accuracy or convergence. Null init/update/recenter
are no-ops, null ready returns false and null get returns identity. The getter
does not require ready state. Callers must supply finite physical input: this
implementation does not generally validate NaN/infinity or detect saturation.

## `components/orientation/orientation.c`: fusion and zeroing

### Initialization and timestamp policy

Init resets flags/time and both quaternions to identity. Positive beta is used
as supplied; otherwise beta defaults to 0.05. Zero therefore cannot disable
accelerometer feedback through this initializer. Positive infinity is not
rejected. The rationale for the exact default is not established here.

The first update initializes the time origin and returns without integration.
Subsequent updates calculate unsigned timestamp difference in seconds. A zero
interval or interval above 50 ms updates the time origin and returns without
changing orientation/readiness. A backwards timestamp normally underflows to
a huge interval and follows the same rejection path. There is no catch-up
integration or reset after gaps. With an accepted interval, the filter marks
have_sample and integrates; the first such update also sets ready and copies
the new quaternion to q_zero. Thus initial readiness is **not** a settling test.

The initial quaternion is identity, not estimated directly from initial gravity.
An initially tilted sensor needs correction over subsequent samples. A shared
relative getter can evolve after its automatic initial zero as gravity correction
converges. Recenter copies current q_BW only when ready; it does not restart the
filter, update timestamps or measure gyro bias.

### Gyro and accelerometer contributions

Before conversion to radians/second, fixed biases are subtracted: X=-1.9 dps,
Y=+0.5 dps, Z=0. Consequently the X input has 1.9 added and Y has 0.5 subtracted.
These are global constants for every filter instance, not per-suit calibration
or online estimates. Their measurement provenance is not recorded in this file.
Raw zero gyro input is therefore not zero corrected angular velocity.

`madgwick_step` first computes quaternion derivative `0.5*q*omega` from gyro.
If squared acceleration norm exceeds 1e-12, acceleration is normalized and
the four-component gravity-objective gradient is evaluated. If its squared
norm exceeds 1e-24, that gradient is normalized and beta times it is subtracted
from the gyro derivative. A zero gradient simply leaves gyro integration.
The result is integrated with a forward Euler step and renormalized.

The helper called `inv_sqrtf_fast` is exactly `1/sqrtf(x)`, not an approximate
bit-hack or hardware-specific algorithm. Public quaternion helpers implement
Hamilton multiplication, conjugation and normalization; the latter substitutes
identity below norm 1e-9. The final normalization inside Madgwick does not have
that small-norm guard. Invalid arithmetic can poison state without clearing
ready. Unused general vector helpers and quaternion-vector rotation remain in
this source; they are not extra stages of the active fusion path.

Normalizing acceleration removes its amplitude from the feedback. There is no
gate checking closeness to 1 g, motion confidence, jerk or impact: strong linear
acceleration can be interpreted as tilted gravity. The low beta limits correction
rate but is not an explicit outlier rejection strategy. There is no magnetometer
input; gravity cannot provide an absolute horizontal heading, so yaw drift and
the need for user calibration remain expected limitations. No velocity/position
integration, smoothing of output beyond fusion, or app-specific mounting exists.

`imu_orientation_get` returns normalized `conjugate(q_zero)*q_BW`. It does not
modify q_BW. Snapshot users of the service receive q_BW instead and maintain
their own calibration. This separation explains why recentering 3D Render does
not inherently reset Paint's private reference.

## Cross-layer freshness caveat

The orientation service publishes the incoming sample timestamp whenever the
filter is already ready, even if update rejected that sample's interval (>50 ms).
After a scheduling gap, an unchanged quaternion can therefore be paired with a
fresh timestamp. Likewise, repeatedly reading unchanged sensor registers creates
new host timestamps. The 250 ms app checks detect stopped publication, not all
forms of stale orientation. This is a documented limitation, not a fix made in
this review.

## Verification and remaining work

The existing Paint/synth tests check downstream coordinate conversion and app
behaviour using artificial quaternions. They do not exercise this fusion source
or actual SPI acquisition. No dedicated filter/driver regression test was found
in the inspected firmware test locations. Passing those app tests is not proof
of sensor accuracy, bias suitability, fusion convergence or sampler timing.

Useful future tests: deterministic gyro integration with the fixed biases
accounted for; gravity convergence from tilted startup; first/duplicate/gapped/
backwards timestamps; independent instances and recenter; register-byte decoding;
SPI failure retention; and task lifecycle/default arguments. Hardware checks
need stationary bias measurements per suit, actual part identity, acceleration
under arm motion, read timing under shared-bus load, and sampler stack headroom.
These are verification candidates, not authorization to retune working behaviour.
