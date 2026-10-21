---
paths:
  - "firmware/**"
---

# Firmware implementation rules (ESP32-S3-N16R8)

## Build
- PlatformIO env `esp32s3`: `platform = espressif32@6.9.0` (Arduino-ESP32 2.0.17, ESP-IDF 4.4.7), `board = esp32-s3-devkitc-1`.
- Memory: `board_build.arduino.memory_type = qio_opi` (quad flash, octal PSRAM). Wrong type = boot loop.
- Compiler is `-std=gnu++11`: no `std::clamp`, no C++14 aggregate init with default member initializers, no `if constexpr`.
- `ARDUINO_USB_CDC_ON_BOOT=0`: `Serial` = UART0 = CH343 = right-hand USB-C port "COM". Flashing and the host link (laptop now, Pi later) use COM. The left port "USB" (native, GPIO19/20) is reserved as a regulated power input; never use it for comms.
- Build/flash only via `make build` / `make flash` (both delete `firmware/.pio` first).

## Architecture invariants
- `controlTask()` (100 Hz default, core 1, high prio) is the ONLY writer of PWM outputs.
- `loop()` is comms only: parse lines, print replies/telemetry. Never block it, never write PWM from it.
- Shared state (`g_cfg`, `g_cmd`, `g_out`, `g_armed`, `g_rawSteerUs`) is read/written only inside `portENTER_CRITICAL(&g_mux)`. Copy out, then work on the copy. No Serial calls inside critical sections.
- PWM uses the ESP-IDF LEDC driver (`driver/ledc.h`), not Arduino `ledc*` (its API differs between core 2.x and 3.x). Servo = LEDC timer 0 / channel 0, ESC = timer 1 / channel 1, 14-bit, low-speed mode.
- `setup()` must initialise both PWM outputs at 50 Hz / 1500 us BEFORE anything else (the ESC learns neutral at power-on).
- The control task is registered with the task watchdog; a hang must reset the chip (pins go idle, ESC cuts after 0.1 s).

## Safety invariants (do not weaken without an explicit request)
- Throttle requires `ARM`; boot state is disarmed.
- No `D` command for `timeoutMs` -> disarm + neutral + `EVT FAILSAFE`.
- Steering clamp: calibrated `left`/`right` in drive mode; 500-2500 us absolute (DS3225 datasheet) in RAW mode.
- ESC clamp: 900-2100 us absolute; `esc_min < esc_neutral < esc_max` enforced on every SET.
- Frame rates: servo 50-330 Hz (datasheet), ESC 50-400 Hz (above 50 Hz only after confirming the ESC accepts it).
- `SAVE` refuses unless center lies between left and right.

## Pins
- Single source of truth: `firmware/include/pins.h`; keep the README table identical.
- Left header P1 only. Never use GPIO3/46 (strapping), 35-37 (PSRAM), 19/20 (native USB), 43/44 (COM UART), 0/45 (strapping).
- Reserved (declared, not configured): BATT_LOW 6, IMU_INT 7, ENC_A/B 15/16, I2C SDA/SCL 8/9, DIST_TRIG/ECHO 10/11. Spare: 12, 13, 14, 17, 18.
- ESP32 supplies no power to peripherals; sensors use a separate rail with common ground and <=3.3 V signals.

## Protocol (ASCII lines, 115200 8N1)
- Commands: `D h s`, `ARM`, `DISARM`/`STOP`, `PING [seq]`, `TIME ms`, `TEL hz`, `RAW us|OFF`, `CAL LEFT|CENTER|RIGHT us`, `SET key val`, `GET`, `SAVE`, `DEFAULTS`, `SENS name`, `INFO`.
- Reply prefixes: `OK`, `ERR`, `CFG`, `PONG`, `INFO`, `TEL`, `EVT`, `READY`, `SENS`. Host code matches on these prefixes.
- `D` is never acknowledged (streamed at `send_hz`).
- Adding or renaming a command or `SET` key requires matching changes in `tools/rccar.py` (`FIRMWARE_KEYS`), `tuning.toml`, and `docs/`.
- `tuning.toml` layout: `[device]` hand-tuned knobs, `[device.calibration]` measured values (written by `make calibrate`, later auto-calibration warm-up), `[warmup.device]` routine settings. Never add unconfirmed parameters.

## Config persistence
- `Config` struct is stored as one NVS blob (`Preferences`, namespace `rccar`, key `cfg`).
- Any change to the `Config` layout MUST bump `CFG_MAGIC` (old blobs are then ignored and defaults load).

## Adding a sensor (future)
- Add its pins to `pins.h` (already reserved) and fill its empty section in `tuning.toml` once parameters are confirmed against `docs/datasheets/`.
- Read sensors in their own task or in `controlTask()` if needed for safety (e.g. emergency brake from distance); report via `SENS <name> k=v ...` and telemetry fields.
- Emergency braking must live in firmware so it works without the host.
