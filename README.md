# microcontroller: real-time actuation & comms layer for an autonomous RC car

A 1:10 Tamiya TT-02 is being built into a self-driving research platform on a two-layer compute stack: a Raspberry Pi 5 for perception, sensor fusion, semantic SLAM and racing-line optimization, and an ESP32-S3 for hard-real-time control. This repo is the electrical and embedded foundation those models stand on: PWM actuation, failsafes, the host-to-MCU protocol, runtime tuning and data logging.

The ML models and simulators live in separate repos and are deployed to the Pi separately. This repo stays focused on infrastructure and communications.

## Compute stack

| Layer | Hardware | Role | Status |
| --- | --- | --- | --- |
| High level | Raspberry Pi 5 | autonomy stack (ROS 2), sends heading + speed | bench stand-in: laptop + `tools/` |
| Real time | ESP32-S3-N16R8 | hardware PWM to servo/ESC, safety, telemetry | this repo, `firmware/` |

## Current inventory

Source: [`docs/datasheets/components_list.xlsx`](docs/datasheets/components_list.xlsx)

| Component | Item | Documentation |
| --- | --- | --- |
| microcontroller | ESP32 S3 N16R8 | [user guide](https://github.com/microrobotics/ESP32-S3-N16R8/blob/main/ESP32-S3-N16R8_User_Guide.pdf) |
| chassis | Tamiya TT-02 | — |
| battery | 2S 7.4V 80C 5200mAh LiPo Battery Pack | — |
| computer | Raspberry Pi 5 | needs 5V, 5A |
| voltage regulator for rpi | Pololu D36V50F5 (5V 5.5A, 6-50V in) | [pololu.com/product/4091](https://www.pololu.com/product/4091), [archived](docs/datasheets/Pololu%20-%205V%2C%205.5A%20Step-Down%20Voltage%20Regulator%20D36V50F5.htm) |
| lidar | Garmin LIDAR-Lite v4 LED, part number 010-02022-00 | [instructions](https://static.garmin.com/pumac/LIDAR-Lite%20LED%20v4%20Instructions_EN-US.pdf) |
| motor controller | THW-1060-BRUSHED | [thw-1060-brushed-esc.md](docs/datasheets/thw-1060-brushed-esc.md) |
| steering servo | DS3225MG | [ds3225_servo.md](docs/datasheets/ds3225_servo.md) |

## Quick start

Follow [docs/setup.md](docs/setup.md). In short: `make setup` → `make detect` → `make flash` → `make test` → `make calibrate` → `make tune`. Run `make help` for every target.

## USB-C ports (ESP32-S3-N16R8)

| Port | Position | Connects to | Use |
| --- | --- | --- | --- |
| **COM** | right | CH343 USB-UART → UART0 (GPIO43/44) | **flashing** + host link (laptop now, Raspberry Pi later) |
| **USB** | left | native USB (GPIO19/20) | planned regulated power input. Not used for data |

Flash and talk to the board **only through COM**.

## Pin map (left header P1 only)

Source of truth: [`firmware/include/pins.h`](firmware/include/pins.h). Keep this table identical to it.

```
                ESP32-S3-N16R8   (top view, antenna up)
          P1 (LEFT header)
        ┌─────┬──────────────────────────────┐
  3V3   │  1  │ unused (sensors: own rail)   │
  3V3   │  2  │ unused                       │
  RST   │  3  │ ✗ do not use                 │
  GPIO4 │  4  │ ● STEER_PWM   → servo signal │
  GPIO5 │  5  │ ● THROTTLE_PWM→ ESC signal   │
  GPIO6 │  6  │ ○ BATT_LOW                   │
  GPIO7 │  7  │ ○ IMU_INT                    │
  GPIO15│  8  │ ○ ENC_A                      │
  GPIO16│  9  │ ○ ENC_B                      │
  GPIO17│ 10  │ · spare (UART1 TX)           │
  GPIO18│ 11  │ · spare (UART1 RX)           │
  GPIO8 │ 12  │ ○ I2C SDA                    │
  GPIO3 │ 13  │ ✗ strapping                  │
  GPIO46│ 14  │ ✗ strapping                  │
  GPIO9 │ 15  │ ○ I2C SCL                    │
  GPIO10│ 16  │ ○ DIST_TRIG                  │
  GPIO11│ 17  │ ○ DIST_ECHO (≤3.3V!)         │
  GPIO12│ 18  │ · spare                      │
  GPIO13│ 19  │ · spare                      │
  GPIO14│ 20  │ · spare                      │
  5V    │ 21  │ input only - leave unwired   │
  G     │ 22  │ ■ COMMON GROUND (ESC/servo)  │
        └─────┴──────────────────────────────┘
          [USB] left        [COM] right ← flash / host
   ● active   ○ reserved (no code yet)   · spare   ✗ do not use   ■ ground
```

| P1 # | GPIO | Function | Status |
| --- | --- | --- | --- |
| 4 | 4 | **STEER_PWM** → DS3225 signal | **active** |
| 5 | 5 | **THROTTLE_PWM** → ESC signal | **active** |
| 6 | 6 | *BATT_LOW* (hi/low input) | *reserved* |
| 7 | 7 | *IMU_INT* | *reserved* |
| 8 / 9 | 15 / 16 | *ENC_A / ENC_B* (shaft encoder) | *reserved* |
| 12 / 15 | 8 / 9 | *I2C SDA / SCL* (IMU, LIDAR) | *reserved* |
| 16 / 17 | 10 / 11 | *DIST_TRIG / DIST_ECHO* (distance sensor) | *reserved* |
| 10, 11, 18-20 | 17, 18, 12-14 | spare | spare |
| 3, 13, 14 | RST, 3, 46 | ~~reset / strapping pins~~ | ~~do not use~~ |
| 22 | G | common ground | power |

Wire colours: signal = yellow/white, power = red, ground = black. Put a tape flag on the two active signal wires.

## Power rules

1. **The ESP32 powers nothing.** It only outputs actuator signals and talks to the host. Sensors run from a separate regulated rail that shares ground with the ESP32 and keeps all signals at ≤ 3.3 V.
2. Servo power comes from the ESC's built-in 6 V / 3 A BEC (red wire of the throttle lead), connected by Y-harness. **Never connect it to the ESP32**, and don't run servo current through the breadboard.
3. The ESP32, ESC and servo grounds must all be connected (P1-22).

## Power-up sequence

1. ESC **off**. Wheels off the ground.
2. USB cable into **COM**. The ESP32 boots and sends neutral (1500 µs) on both channels.
3. ESC **on**. It beeps and learns neutral.
4. Power down in reverse: ESC off first. **Always switch the ESC off before `make flash`.**

## Repo layout

```
firmware/        ESP32 firmware (PlatformIO): src/main.cpp, include/pins.h
tools/           rccar.py (CLI + link/log/tuning API), esp32_session.py (interactive scratchpad)
tuning.toml      all runtime tuning; pushed to the ESP32 on connect
logs/            session history (*.jsonl, gitignored)
docs/            setup.md, design proposal, datasheets/
Makefile         every bring-up command (make help)
```

Design rationale (architecture, safety, protocol, latency): [docs/esp32-firmware-proposal.md](docs/esp32-firmware-proposal.md).
