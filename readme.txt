SuitProgram (ESP32 Tron suit)
=============================

Target + peripherals
--------------------
- ESP32-WROVER-E (16 MB) bare module; flash via USB-UART @115200 baud. Known MACs: 24:d7:eb:6b:26:40 and 24:d7:eb:6b:26:c8.
- SPI2: SSD1309 128x64 OLED, microSD (SDSPI), up to 3x MPU6500.
- I2S: INMP441 mic (RX) + MAX98357A mono-left amp (TX).
- ADC1: Hall A GPIO34, Hall B GPIO35 (used as pack divider), ladder buttons on GPIO39; pressing A+B ≃ 1.1 V, B+C ≃ 2.2 V
- LEDs: WS2815 strips on GPIO32 and 33 via level shifter, default length 2x700 (code currently drives PIN_LED_STRIP_A / GPIO32).
- UART0: GPS NEO-6M on GPIO1/3 (console must be disabled).
- Wi-Fi STA + ESP-NOW; GPS + UDP telemetry; OLED UI at 128x64 1 bpp.
- SD mount: `/sdcard` with hidden `.rec_counter`.
- Parts checklist: ESP32 module, IMUs, microSD, SSD1309 OLED, NEO-6M, INMP441, MAX98357A, WS2815 + 74HCT level shifter, ACS712 + divider, 3S pack + BMS, 5 V/3.3 V regs, hall sensor, ladder buttons, speaker, connectors/harness/straps, LED rings if used.

Hardware map (hw.h)
-------------------
- SPI2: MOSI 23, MISO 19, CLK 18, CS OLED 5, CS SD 21, CS IMU1/2/3 14/26/27, DC OLED 13, RST OLED NC.
- I2S: BCLK 4, LRCK 22, DOUT 25 (amp), DIN 36 (mic).
- UART0 GPS: TX 1, RX 3.
- LEDs: WS2815 on GPIO 32/33.
- ADC1: Hall A GPIO34 CH6, Hall B GPIO35 CH7, buttons ladder GPIO39 CH3. Button targets: ~0.825 / 1.65 / 2.475 / 3.3 V; top-combo target ~1.1 V with ~350 mV tolerance.
- SD mount: `/sdcard` with hidden `.rec_counter`.

Runtime shell + controls
------------------------
- Boots straight into the menu (title screen is blank); any press on the title jumps to the menu, and a ~1.1 V top-combo long-press always escapes to the menu.
- Ladder input defaults: 1.2 s long-press, combo ~1.1 V with 350 mV tolerance, combo verify 80 ms.
- HUD shows LED mode label, connection icon (`none/link/on`), HH:MM from GPS (stale -> `--:--`), and three battery bars.
- LED modes, power monitor, and GPS services are gated by `CONFIG_APP_ENABLE_*` (default off while debugging resets); HUD seeds placeholder battery/time until services update.
- Shell app implementations live in `main/app_*.c` and are registered by `main/app_shell.c`; rendering/logic components (e.g., FFT) stay under `components/`.

Apps and key bindings
---------------------
- Menu: A up, B down, D select; long top-combo -> menu. Pages: Settings (Status, Bluetooth, Volume, Keyboard), Sim (GPS placeholder, FFT, Fluid, 3D Render), Games (Flappy, 2048, Tetris, Pong, Snake), Comm (Message/Call placeholders), root shortcuts to LEDs + Music.
- Status: shows LED label, three battery bars, link path/state, and clock; long A -> menu.
- Volume: B louder, C quieter, D mute/unmute (syncs BT and local audio gains); long top-combo -> menu.
- LEDs: B/C = prev/next mode, D = apply; D long toggles beat sync; B/C long raise/lower brightness; A long -> menu. Modes: idle/off, breathe, rainbow, beatwave (uses FFT beat queue when sync on), planes (placeholder layout).
- Music: mounts `/sdcard`, lists up to 100 WAV (PCM16 mono); A/B move selection, C toggles play/pause on current, D (re)starts selection, D long = stop, A long -> menu. Streams to BT sink if connected else local amp. BT pause/disconnect captures a resume offset so the same track can continue later.
- Bluetooth: A up, B down, C = rescan, D = connect/disconnect (disconnect if current peer), D long = force disconnect; list includes current peer even if not discovered.
- FFT viewer: B/C cycle views (BPM text, spectrum, spectrogram, novelty peaks, flux, tempo raw/comb, phase comb); A long -> menu.
- Fluid: accel-driven 2D fluid; A/B/C/D move emitter, B+C dumps ink, D long resets, long A -> menu.
- 3D Render: IMU-driven cube field; D recenters zero; long A -> menu.
- Keyboard: on-screen QWERTY with accents; A left, B down, C right, D press/release; B+C (fast-fall) behaves as down and long-press triggers backspace; shows text at top.
- Pong: A pause, B paddle up, C paddle down, D toggles host/follower (host serves); long A -> menu; syncs paddle/ball/score over link frames (ACK when on ESP-NOW).
- Snake: A left, D right, B up, C down; any press after death restarts; long A -> menu.
- Flappy / 2048 / Tetris: legacy external apps launched in their own tasks; exit via their built-in long-press/escape (shell skips draw while they run).
- GPS/message/call menu items are stubs today (no registered shell apps yet).

Background services
-------------------
- Audio bus: shared I2S0 (amp + mic) initialized at boot for FFT/music/mic/BT; volume path shared with BT percent and BT disconnect stops music playback.
- LED modes task (`led_modes_start`) runs at boot; uses `led_layout.c` placeholder helix (~4 turns over ~1.2 m, 0.35 m radius). Update with measured coordinates for planes/beat patterns. Global brightness (0-255) and sync toggle stored in the service.
- Power monitor (`power_monitor_start`) samples Hall B divider on ADC1_CH7, converts via `POWER_*` menuconfig (divider ratio + per-cell empty/full) and pushes a smoothed % into all three HUD batteries.
- GPS service reads NMEA on UART0, caches fix/time, and `gps_time_task` updates `system_state` once per second; `gps_display_time` helper draws HH:MM for other UIs.
- Link service chooses ESP-NOW first (optional peer MAC via menuconfig), falls back to Wi-Fi/UDP with SSID/pass/IP/port from menuconfig. Frames carry typed messages with optional ACK on ESP-NOW; 1 s info broadcast sends LED mode, 3x battery %, connection flag, and time (fed into `system_state`). `link_path_name()` is shown on the HUD/status app.
- Audio RX server (`audio_rx_start`) spins up after link+SD mount; listens on TCP (default 5000) and writes each connection to `/sdcard/rx_rec_XXXXX.wav`.
- Input service decodes the ladder (buttons + top combo + fast-fall), enforces debounce/long-press, and punches novelty holes in FFT on transitions to reduce click noise.

Storage / audio / sensing
-------------------------
- Storage: SDSPI mount retries 20/10/5/2/0.4 MHz; helper to list directories; boot counter stored at `/sdcard/.rec_counter`.
- Audio core: shared I2S0 for amp + mic; `audio_set_rate` retimes TX/RX; playback supports PCM16 mono WAV from file/memory, tones, and an embedded sample. Volume is Q1.15 gain. TX mute pin optional via config.
- Audio capture: PSRAM-backed mono ring buffer at 48 kHz (default ~0.5 s) with `audio_capture_read_latest` helper and stats export.
- Audio player + BT: `audio_player` streams WAVs to A2DP sinks via `bt_audio` when available, else plays locally; BT device scan/list UI exposed in shell. Tracks BT playback progress for resume on pause/disconnect and exposes a small in-RAM queue API (play now/next/last).
- Microphone: recorder writes 16-bit mono WAV to `/sdcard/rec_XXXXX.wav` at 44.1 kHz (slot auto-picked by energy), handles zero-checks, and exposes last path for telemetry send.
- FFT: continuous mic sampler + beat/BPM detector, novelty history, multiple OLED views; exports beat events for LED beatwave mode and lets callers zero novelty (`fft_punch_novelty_hole`).
- Power: pack voltage sensing only; current sensing TODO. Divider ratio and cell thresholds configurable (menuconfig) or hardcoded fallback (6.0x, 3.30-4.15 V).

Networking / comms modules
--------------------------
- Telemetry: Wi-Fi STA UDP client; sends lines, MPU samples, files, or raw buffers; broadcast/unicast to PC IP/port; file receiver can save to `/sdcard`.
- Link: framing/ACK layer used by the shell and Pong (see above); ESP-NOW preferred with optional peer MAC, UDP fallback via Wi-Fi.
- audio_rx: TCP listener (default 5000) that streams incoming audio directly to `/sdcard/rx_rec_XXXXX.wav` (one file per connection) once link + SD are up.
- Comms (standalone): duplex UDP call helper that streams mic->peer and plays peer audio on amp; button A sends last mic recording over telemetry; requires SSID/pass/IP/ports in `comms_cfg_t`.
- Call (standalone): simpler TX/RX roles for 16 kHz mono audio over UDP with OLED status.
- mpu6500 + stick: lightweight SPI driver and a 3-IMU stick-figure demo (exit with button A).
- Logo: 64x64 animated boot logo helper; Fluid: accel-driven 64x32 OLED fluid sim (shell-wrapped).

Power / thermal notes
---------------------
- WS2815 length default 1400; budget ~15 W idle / ~24 W pulse / ~40 W full-white for 5 m of 144 LED/m strips. Four hours ~160 Wh -> size batteries or cap brightness/duty/segments.
- Level-shift LED data (74HCT series) and keep short traces; share ground between 12 V rail and logic rails.

Open TODO / wishlist
--------------------
- separate the volume settings between volume for the imbeded speaker and the bluetooth volume
- games sfx
- Menu/UI: replace placeholder icons, measure real ladder voltages and tighten combo thresholds; optionally show GPS time status flag and link quality.
- Shell plumbing: wire GPS/message/call menu items to real apps or hide them; surface audio_rx status/controls.
- LED/FFT: replace `led_layout.c` with measured coordinates; add richer patterns (planes tied to IMUs, waves from chest ring, randomizer presets), improve FFT noise handling/tempo fallback.
- Link/comms: define structured bundles (leaderboard/GPS/messages/audio headers), add per-type ACK/retry, and integrate link frames into games/remote LED paint tools.
- Power: measure divider ratio and per-cell thresholds, add current sensing/export + low-battery guardrails.
- Games/content: polish Pong host election/ACK UI, add scores/levels to Snake, port more titles (tron/doom/pacman/etc.), add calculator and lightweight 3D renderer/virtual-plane LED effect.
- Extras: costume/LED/audio preset randomizer, Tournai map on OLED, IMU fusion for limb pose to drive LEDs/stick, task priority/core pinning guidance for FreeRTOS.
- fix audio recording and calls
- fix fft postprocessing, for now the selected BPM is flickering, we need to have a more robust BPM identification.
- create audio file that when displayed on the spectrogram, spell out words, as hidden messages

Task core pinning / priorities (FreeRTOS)
-----------------------------------------
Core 0 (Wi-Fi/BT friendly, lower contention)
- audio_rx writer task (`audio_rx_wr`, prio 3)
- Audio player/music tasks (`audio_play_task`, `music_play_task`) are unpinned; scheduler often keeps BT/A2DP on core0
- Telemetry RX, call/comms tasks, LED modes, OLED/UI, GPS tasks: unpinned (scheduler may place on core0)

Core 1 (heavy DSP/render)
- `audio_rx_task` (prio 1)
- FFT visualizer `fft_task` (prio 5)
- Audio capture `audio_capture_task` (prio configMAX_PRIORITIES-2)
- IMU sampler default (`mpu6500_start_sampler`, prio 2) and IMU orientation task (`imu_orientation_task`, prio 2)

Notes
- Unpinned tasks (LED modes, OLED/UI, music player, GPS, call/comms, telemetry) can float; pin heavy DSP to core1 and keep Wi-Fi/BT/A2DP free on core0 when tuning.
- Audio RX filenames auto-increment via `rx_rec_index.txt`; last path stored in `rx_rec_last.txt`.
