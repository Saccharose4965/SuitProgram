Pending review

# Firmware architecture

SuitProgram is divided by responsibility: apps present controls and status,
services implement reusable behavior, drivers operate hardware, and `main/`
assembles the system.

## Layers

### Apps

Each directory under `apps/` owns one visible activity or the application shell.
An app translates button events into service requests and draws service status.
Its lifecycle decides whether the underlying feature continues in the background.
For example, the FFT screen observes a detector that can remain active, while
Light Paint owns a session that ends when its app closes.

### Services

Each directory under `services/` owns a feature, shared model, or resource rule.
A service may have a task, receive periodic ticks, or provide synchronous
functions. Services expose commands and coherent status snapshots so apps do not
need to manipulate their internal state.

Services may depend on other services. Those relationships are kept explicit in
their component declarations and APIs. Shared concepts have one owner: suit
coordinates live in Suit Geometry, orientation and calibration live in
Orientation, and audio conflicts live in Audio Policy.

### Drivers

Each directory under `drivers/` owns a hardware boundary such as the OLED, IMU,
SD card, LED strip, buttons, or I2S peripheral. Drivers convert ESP-IDF and board
details into small project APIs. Scheduling and feature choices stay in services
and apps.

### Main

`main/` is the ESP-IDF entry boundary. It creates the application-shell task on
CPU1 and contains project Kconfig/build declarations. The shell owns feature
assembly, registry and service-event routing.

## Dependency direction

The usual flow is:

```text
button / radio event
        ↓
       app  ───────→ service command
                         ↓
                    shared services
                         ↓
                       driver
                         ↓
                      hardware
```

Status travels back through snapshots and bounded callbacks. Radio callbacks do
only validation and queueing when their work can block. Rendering, SD access and
session decisions run later in their owning task or the shell loop.

Apps can use the display and input interfaces directly. A display service would
only add indirection because drawing is already app behavior. Drivers never call
apps or choose user-facing policy.

## Runtime ownership

The ESP32 has several resources that cannot be freely shared: I2S capture and
playback, internal DMA-capable RAM, Bluetooth, LED output and radio callbacks.
The service that owns a feature also owns its release path. Audio Policy records
which activity blocks FFT and resumes pending analysis only after the resource is
available. Show Runtime claims LED output for the complete session and restores
the previous lighting modes when it releases it.

Task/core placement, queue sizes, allocation capabilities and update rates are
part of a service's behavior. Folder cleanup must preserve those properties
unless a measured change is being made. Detailed decisions belong in the owning
module's README.

## Shared coordinate space

Suit Geometry defines the semantic LED sections and the common 2D body space.
Lighting, shows and painting use that model so an authored position means the
same thing everywhere and remains independent of a costume's LED count.
Orientation owns the IMU-to-body conversion and stored calibration used by arm
features. Their exact conventions are documented by those two services.

## Documentation ownership

Every app, service and driver keeps its design explanation in its own
`README.md`. Cross-system documents in this folder explain only relationships
that span several owners. The generated [file checklist](documentation/file-checklist.md)
tracks source-level inspection separately from design approval.
