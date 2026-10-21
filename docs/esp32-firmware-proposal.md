# Proposal: ESP32-S3 actuator firmware + laptop test pipeline

Status: **proposal, nothing implemented yet**. This PR only adds this document.
Once it is approved, a follow-up PR will add the files in [Appendix A](#appendix-a-draft-files),
which were drafted and checked while writing this proposal (see [Verification](#11-verification-done-for-this-proposal)).

## 1. Goal and scope

Build a simple bench and tuning pipeline for the RC car (Tamiya TT-02 chassis):

```
laptop (Python CLI)  --USB-->  ESP32-S3  --PWM-->  DS3225 steering servo
   heading, speed                          --PWM-->  THW-1060 brushed ESC -> motor
```

- The laptop sends live **heading** (degrees) and **speed** (-1..1) commands over USB.
- The ESP32 generates the PWM signals on its own, in hardware. It also enforces the safety
  rules (arming, command timeout, speed limit, steering end stops), so a laptop that
  crashes or a cable that comes loose cannot leave the car running.
- A root `Makefile` wraps every bring-up step (check, detect, sync time, clean+flash,
  test, calibrate, teleop).
- Pins for the future battery-low input, IMU, distance sensor and shaft encoder are
  **reserved and documented only**. No code drives them.
- Later the laptop is replaced by a Raspberry Pi 5 running ROS 2. The firmware is split
  so that only its comms layer changes then (see [section 10](#10-future-ros-2-migration)).

Out of scope: closed-loop speed or heading control, Wi-Fi/BLE, and drivers for the
reserved sensors.

## 2. Hardware facts this design relies on

| Item | Fact | Source |
| --- | --- | --- |
| MCU | ESP32-S3 (QFN56) rev v0.2, 240 MHz dual core, 16 MB quad flash, 8 MB **octal** PSRAM | `esptool` output in the request |
| Board | "ESP32-S3-N16R8" (YD-ESP32-S3 clone of Espressif's DevKitC-1), ESP32-S3-WROOM-1 module | Micro Robotics user guide (linked in the inventory sheet) |
| USB | Two USB-C ports: **COM** = CH343P USB-to-UART bridge (UART0, GPIO43/44); **USB** = native USB on GPIO19/20 | user guide |
| 5V pin | Works as an **input** by default. It only becomes an output if the `IN-OUT` pads are bridged | user guide |
| Unusable GPIO | GPIO35/36/37 are wired to the octal PSRAM inside the module | user guide |
| Servo | DS3225MG: 500-2500 us pulses, 1500 us = center, 50-330 Hz frame, 3 us deadband, 180 deg unit, 3.3 V logic OK, stall 1.9 A @ 5 V / 2.3 A @ 6.8 V | `ds3225_servo.md` |
| ESC | THW-1060-BRUSHED: standard RC throttle input, auto throttle-range calibration, cuts power after **0.1 s** without a signal, built-in BEC **6 V / 3 A**, 2S LiPo cutoff at 6.0 V | `thw-1060-brushed-esc.md` |
| Battery | 2S 7.4 V LiPo (8.4 V when full) | inventory sheet |

## 3. Background primer (embedded concepts used here)

You know Python and data work, so this section covers the hardware ideas the design
depends on.

**PWM servo signals ("RC PWM").** Hobby servos and ESCs do not read a voltage level.
They read the *width* of a short 3.3-5 V pulse that repeats 50 times a second (one
frame every 20 ms). The pulse width carries the value:

```
      1500 us                       20 ms frame
     ┌──────┐                      ┌──────┐
 ────┘      └──────────────────────┘      └─────   1500 us = servo centered / ESC neutral
     ┌───┐                         ┌───┐
 ────┘   └─────────────────────────┘   └────────   1000 us = full one way
     ┌─────────┐                   ┌─────────┐
 ────┘         └───────────────────┘         └──   2000 us = full the other way
```

- **Servo** (DS3225): the pulse width sets the output *angle*. The servo's own control
  loop holds that angle. Here 500 us = 0°, 1500 us = 90° (center), 2500 us = 180°.
- **ESC** (electronic speed controller): it uses the same signal format, but the pulse
  width sets *motor power*: 1500 us = neutral, wider = forward, narrower = brake/reverse.
  The ESC switches the battery current (up to 60 A) through the motor. The ESP32 only
  sends the pulse.

**BEC and common ground.** A *BEC* (battery eliminator circuit) is a regulator inside the
ESC. It turns battery voltage into 6 V for the servo, on the ESC's red wire. A signal
voltage only means something relative to a ground, so the ESP32 GND, the ESC ground and
the servo ground **must be connected together** (a "common ground"). Power can come from
different supplies, but grounds are shared.

**Logic level.** ESP32 pins output 3.3 V and are **not 5 V tolerant**. The servo datasheet
accepts 3.3 V signals. Almost every RC ESC does too (see [open questions](#12-open-questions--verify-on-hardware)).
Anything that *sends* 5 V into the ESP32 (ultrasonic echo, some encoders) needs a divider
or level shifter.

**Strapping pins.** At reset, the chip reads a few pins to decide how to boot (normal or
firmware download, flash voltage, and so on). Hardware connected to those pins can stop
the board from booting, so we avoid them.

**LEDC (hardware PWM).** The ESP32 has a peripheral called LEDC that generates PWM
waveforms in hardware. You set a duty value once and it keeps producing pulses with no CPU
involvement. This is how the ESP32 "operates independently for PWM". There is a catch: if
the firmware hangs, LEDC keeps sending the *last* value. [Section 7](#7-safety-design) covers how we handle that.

**FreeRTOS tasks.** The ESP32 Arduino core runs on FreeRTOS, a small real-time OS.
Tasks are like Python threads, but the scheduler is deterministic and priority-based.
We use two tasks: the comms loop (`loop()`) and a high-priority 100 Hz control task.

**PlatformIO.** A Python-based build system for microcontrollers (`pip install platformio`).
`platformio.ini` declares the chip, board and framework. `pio run -t upload` downloads the
compiler toolchain, compiles, and flashes the board over USB through `esptool`. Build
output and cache go in `firmware/.pio/`.

**NVS (non-volatile storage).** NVS is a small key-value store in the ESP32's flash.
Think of it as an on-chip `shelve`. Calibration values are saved there, so they survive
reflashing and power cycles. A full chip erase deletes them.

## 4. Architecture

### 4.1 Data flow

```
 Laptop                                ESP32-S3
┌──────────────────────┐   USB-C     ┌─────────────────────────────────────────────────┐
│ tools/rccar.py       │  "COM" port │ CH343 ── UART0 ── loop()  (core 1, low prio)     │
│  teleop / test /     │────────────▶│                   line parser, telemetry         │
│  calibrate / drive   │  115200 8N1 │                       │ writes g_cmd, g_cfg      │
│                      │◀────────────│                       ▼  (spinlock)              │
│  sends "D hdg spd"   │  TEL lines  │                controlTask() (100 Hz, high prio) │
│  at 20 Hz            │             │   failsafe → slew → reverse logic → mapping     │
└──────────────────────┘             │                       │                          │
                                     │      LEDC timer0 ─────┴───── LEDC timer1         │
                                     │      GPIO4 50 Hz             GPIO5 50 Hz         │
                                     └────────┬────────────────────────┬────────────────┘
                                              ▼                        ▼
                                       DS3225 servo             THW-1060 ESC → motor
```

**Why the COM port and not the native USB port.** The CH343 bridge keeps the serial port
open when the ESP32 resets, and it handles the auto-reset used for flashing. The vendor
guide also says to use it. With the native port the USB device disappears on every reset
or crash, which is bad for a tuning loop. The firmware builds with
`ARDUINO_USB_CDC_ON_BOOT=0`, so `Serial` is UART0, which is the COM port.

### 4.2 Firmware layers (`firmware/src/main.cpp`)

| Layer | Runs in | Responsibility | Changes for ROS 2? |
| --- | --- | --- | --- |
| Comms | `loop()` | parse ASCII lines, print telemetry | **replaced** by micro-ROS subscribers/publishers |
| Shared state | globals behind a `portMUX` spinlock | `Command`, `Config`, `Output`, armed flag | no |
| Control | `controlTask()` 100 Hz, pinned to core 1 | failsafe, slew limit, ESC reverse sequence, heading→steering, pulse mapping | no |
| PWM | LEDC hardware | emit pulses | no |
| Persistence | NVS (`Preferences`) | calibration and tuning values | no |

The control task is the **only** code that writes PWM outputs. The comms layer only
writes the requested command, so a slow or blocked serial port cannot disturb pulse timing.

### 4.3 Power architecture (bench setup)

```
 Laptop USB 5V ──▶ ESP32 "COM" port            (logic only, ~100 mA)

 2S LiPo ──▶ ESC ──▶ motor
              │
              └─ throttle lead:  signal ──▶ GPIO5
                                 red 6V ──▶ servo red     (servo power, Y-harness)
                                 black  ──▶ servo black ──▶ ESP32 GND (P1-22)
```

Rules:
1. **Never connect the ESC's red 6 V wire to the ESP32** (not to `5V`, not to `3V3`).
   With USB plugged in, two supplies would fight each other through the board's power
   path.
2. **Do not run servo power through the breadboard.** Breadboard contacts are rated about
   1 A, and the servo stalls at about 2 A. Connect servo red/black straight to the ESC lead
   with a Y-harness. Bring only the **signal** wires and **one ground** wire to the
   breadboard.
3. The ESC BEC (6 V / 3 A) covers the servo (2.3 A stall at 6.8 V, about 2.1 A at 6 V). This
   is the normal RC-car setup and meets the datasheet's "dedicated BEC" advice. Do not
   power the servo from the Pi 5 rail later either.
4. 5 V sensors added later (LIDAR-Lite v4) need their own 5 V supply, such as the Pololu
   D36V50F5 rail planned for the Pi, sharing ground. The board's `5V` pin is an *input* by
   default and cannot supply them.

## 5. Pin plan (left header P1 only, verified)

The board's left header (P1, the side with `5V` and `G`) is, top to bottom with the USB-C
ports at the bottom:
`3V3 3V3 RST 4 5 6 7 15 16 17 18 8 3 46 9 10 11 12 13 14 5V G`.
This matches the P1 table in the vendor guide and Espressif's DevKitC-1 J1 header.

| P1 # | Label | GPIO capabilities (vendor table) | Assignment | Status |
| --- | --- | --- | --- | --- |
| 1 | 3V3 | 3.3 V out (LDO, 1 A total) | 3.3 V for IMU / 3.3 V sensors | power |
| 2 | 3V3 | 3.3 V out | — | power |
| 3 | RST | EN (reset) | **do not use** | — |
| 4 | 4 | ADC1_CH3, TOUCH4 | **STEER_PWM** → DS3225 signal | **active** |
| 5 | 5 | ADC1_CH4, TOUCH5 | **THROTTLE_PWM** → ESC signal | **active** |
| 6 | 6 | ADC1_CH5 | BATT_LOW (digital in; ADC1 so it can become an analog reading later) | reserved |
| 7 | 7 | ADC1_CH6 | IMU_INT (data-ready interrupt) | reserved |
| 8 | 15 | ADC2_CH4, XTAL_32K_P | ENC_A (shaft encoder, PCNT peripheral) | reserved |
| 9 | 16 | ADC2_CH5, XTAL_32K_N | ENC_B (shaft encoder, PCNT peripheral) | reserved |
| 10 | 17 | U1TXD | spare: UART1 TX (fallback wired link to the Pi) | spare |
| 11 | 18 | U1RXD | spare: UART1 RX | spare |
| 12 | 8 | ADC1_CH7 | I2C_SDA (shared: IMU + LIDAR-Lite v4) | reserved |
| 13 | 3 | **strapping** (JTAG source) | **do not use** | — |
| 14 | 46 | **strapping** (boot mode / ROM log) | **do not use** | — |
| 15 | 9 | ADC1_CH8 | I2C_SCL (shared: IMU + LIDAR-Lite v4) | reserved |
| 16 | 10 | ADC1_CH9 | DIST_TRIG (ultrasonic TRIG, or LIDAR GPIO/mode) | reserved |
| 17 | 11 | ADC2_CH0 | DIST_ECHO (ultrasonic ECHO **through a 5→3.3 V divider**, or LIDAR data-ready) | reserved |
| 18 | 12 | ADC2_CH1 | free | spare |
| 19 | 13 | ADC2_CH2 | free | spare |
| 20 | 14 | ADC2_CH3 | free | spare |
| 21 | 5V | 5 V **input** (unless IN-OUT pads bridged) | not used (board powered by USB) | power |
| 22 | G | ground | **common ground** (ESC/servo ground) | power |

Why these choices:
- **PWM on GPIO4/5.** These are the first general-purpose pins on the header. They have
  no strapping or boot function, so the outputs stay quiet during reset.
- **I2C on GPIO8/9.** These are the Arduino-ESP32 defaults for the S3 (`Wire` with no
  arguments), so future example code works unchanged. The IMU and LIDAR-Lite v4 (I2C
  address 0x62) share the bus.
- **Analog-capable pins (ADC1) for the battery and IMU.** On ESP32 chips, ADC2 cannot be
  used while Wi-Fi is on, so analog inputs go on ADC1. The encoder pins only need
  digital input.
- **Encoder on GPIO15/16.** The S3's PCNT (pulse counter) peripheral can decode
  quadrature on any GPIO with no CPU load. GPIO15/16 can drive a 32 kHz crystal, but this
  board has none fitted, so they are plain GPIO.
- **GPIO3 and GPIO46 are excluded.** Both are strapping pins.
- The right header (P2) is unused for now. It holds GPIO0/45 (strapping), GPIO19/20
  (native USB), GPIO35-37 (PSRAM), GPIO43/44 (COM port UART), and GPIO48 (RGB LED pad).

Notes for the reserved devices (wiring guidance only, no code in this PR):
- **Battery hi/low.** A 2S pack reaches 8.4 V, so **never** wire it directly to a pin.
  A bare resistor divider does not give a clean hi/low either: ESP32 inputs read
  undefined between about 0.8 V and 2.5 V. Use a comparator or voltage-detector module
  set to about 6.8 V (3.4 V per cell, above the ESC's 6.0 V cutoff), with an open-drain
  output pulled up to 3.3 V. GPIO6 is ADC-capable, so we can switch to a divider plus
  analog reading later.
- **IMU (6-axis).** Most breakouts (MPU-6050, ICM-42688, BMI270, LSM6DS3) run on 3.3 V
  I2C. Power it from P1-1, use SDA/SCL as above and INT on GPIO7.
- **LIDAR-Lite v4 LED.** Garmin's manual could not be downloaded from this environment.
  From memory, it runs on 4.75-5.25 V with 3.3 V I2C logic at 0x62. **Verify against the
  manual before wiring.** It needs an external 5 V supply (see power rule 4).
- **Ultrasonic (HC-SR04 type).** Runs on 5 V and its ECHO output is 5 V, so add a 1 kΩ/2 kΩ
  divider into GPIO11.
- **Shaft encoder.** If the outputs are open-collector, pull up to **3.3 V**, not 5 V.

### 5.1 README marking plan

The implementation PR adds a root `README.md` with:
1. The P1 table above, cut to `P1 # | GPIO | Function | Status`, using **bold** for
   active pins, *italics* for reserved pins and ~~strikethrough~~ for do-not-use pins.
2. This ASCII map of the header:

```
                ESP32-S3-N16R8   (top view, antenna up)
          P1 (LEFT header)
        ┌─────┬──────────────────────────────┐
  3V3   │  1  │ 3.3V out → IMU VCC (future)  │
  3V3   │  2  │                              │
  RST   │  3  │ ✗ do not use                 │
  GPIO4 │  4  │ ● STEER_PWM   → servo signal │
  GPIO5 │  5  │ ● THROTTLE_PWM→ ESC signal   │
  GPIO6 │  6  │ ○ BATT_LOW (future)          │
  GPIO7 │  7  │ ○ IMU_INT  (future)          │
  GPIO15│  8  │ ○ ENC_A    (future)          │
  GPIO16│  9  │ ○ ENC_B    (future)          │
  GPIO17│ 10  │ · spare (UART1 TX)           │
  GPIO18│ 11  │ · spare (UART1 RX)           │
  GPIO8 │ 12  │ ○ I2C SDA  (future)          │
  GPIO3 │ 13  │ ✗ strapping                  │
  GPIO46│ 14  │ ✗ strapping                  │
  GPIO9 │ 15  │ ○ I2C SCL  (future)          │
  GPIO10│ 16  │ ○ DIST_TRIG (future)         │
  GPIO11│ 17  │ ○ DIST_ECHO (future, ≤3.3V!) │
  GPIO12│ 18  │ · spare                      │
  GPIO13│ 19  │ · spare                      │
  GPIO14│ 20  │ · spare                      │
  5V    │ 21  │ input only - leave unwired   │
  G     │ 22  │ ■ COMMON GROUND (ESC/servo)  │
        └─────┴──────────────────────────────┘
              [COM]  [USB]   ← use COM
   ● active   ○ reserved   · spare   ✗ do not use   ■ ground
```

3. The power rules from 4.3 and the power-up sequence from 7.
4. `firmware/include/pins.h` as the single source of truth. The README table and
   `pins.h` must change together.
5. A breadboard wire-color convention: signal = yellow/white, power = red, ground =
   black. Put a tape flag on the two active signal wires.

## 6. Signals and mappings

### 6.1 Steering servo (GPIO4)

- 50 Hz frame on its own LEDC timer, 14-bit resolution: 20000 us / 16384 = **1.22 us per
  step**. That is finer than the servo's 3 us deadband, so steering moves smoothly.
- Absolute clamp: 500-2500 us, from the datasheet. This applies even in raw calibration mode.
- Drive-mode clamp: between the **calibrated** `left` and `right` pulses. These are the
  firmware end stops the servo notes ask for, so the linkage is never driven into a stall.
- Frame rate can later go up to 330 Hz (datasheet) for lower latency. That is a one-line
  change (`SERVO_HZ`), because the servo has its own timer.

### 6.2 ESC (GPIO5)

- 50 Hz, 1000 / **1500** / 2000 us (reverse-full / neutral / forward-full).
- The ESC auto-calibrates its throttle range and learns *neutral at power-on*. The
  firmware therefore outputs **1500 us from the first line of `setup()`**, before Serial
  even starts.
- `limit` (default **0.30**) scales the command: `speed = 1.0` means 30 % throttle while
  we are on the bench.
- `deadband` (default 0 us) skips the ESC's dead zone around neutral, so small speed
  commands still move the car. It is tuned on the bench.
- `accel` (default 2.0 / s) limits how fast the throttle command can change. This
  protects gears and stops wheelies when a key is pressed.
- **Reverse double-tap.** Most brushed RTR ESCs in Forward/Brake/Reverse mode treat the
  first reverse pulse as *brake*, and only reverse after the signal returns to neutral.
  The control task runs this sequence automatically when the command crosses from ≥0 to
  <0: brake pulse at 50 % for 150 ms, then neutral for 150 ms, then reverse. Set
  `SET revmode 0` if this ESC's jumpers select direct reverse (the jumper pages were
  not in the extract, see [open questions](#12-open-questions--verify-on-hardware)).

### 6.3 Heading → steering angle

Convention follows **ROS REP-103**: angles are counter-clockwise positive, so
**+heading = turn left**. That makes the ROS 2 migration a direct mapping.

```
steer_deg = clamp(gain × heading_deg, −maxdeg, +maxdeg)          # open loop for now

            ⎧ center + (steer_deg / maxdeg) × (left  − center)   if steer_deg ≥ 0
pulse_us =  ⎨
            ⎩ center + (|steer_deg| / maxdeg) × (right − center) if steer_deg < 0
```

- `heading_deg` is the desired direction of travel relative to the car's nose. With no
  IMU yet, the car cannot measure its heading, so this is a proportional map from heading
  error to wheel angle. With `gain = 1` (default), heading is the requested road-wheel
  angle, up to the lock angle.
- The map has two linear halves, so an asymmetric linkage (more throw one way) still
  gives 0° = straight and ±maxdeg = each lock.
- `left` can be greater or less than `center`, so servo mounting direction does not
  matter. The DS3225 turns CCW as pulse width increases, but whether that steers left
  depends on the linkage. Calibration records it.
- `maxdeg` is the measured road-wheel angle at full lock. Default 25° is a guess for the
  TT-02, so measure it during calibration.
- Later, with the IMU, this becomes a closed heading loop:
  `steer = Kp·(target_yaw − yaw) + …`. The `D` command keeps the same meaning.

## 7. Safety design

| Hazard | Mitigation |
| --- | --- |
| Motor spins at boot | Throttle starts **disarmed** at 1500 us. `ARM` is required before throttle responds. Steering works while disarmed, so steering can be tuned with the motor off. |
| Laptop crash / cable pulled / script stopped | Command timeout (default **300 ms**, `SET timeout`) **disarms** and returns throttle to neutral. Prints `EVT FAILSAFE`. Re-arming takes an explicit `ARM`. |
| Firmware hangs (LEDC would keep the last pulse) | The control task is registered with the ESP-IDF task watchdog. A hang resets the chip, the pins go idle (no pulses), and the ESC's own **0.1 s signal-loss protection** cuts the motor. On reboot, outputs come back at neutral. |
| Servo stalled against linkage | Drive mode is clamped to the calibrated end stops. Raw mode is clamped to the datasheet's 500-2500 us. Calibration help says to stop before the servo buzzes. |
| Too fast on the bench | `limit` 0.30 default. `make test-throttle` asks you to type `yes` after confirming the wheels are off the ground. |
| Bad calibration saved | `SAVE` refuses unless `center` lies between `left` and `right`. |

**Power-up sequence** (goes in the README):
1. ESC switch **OFF**. Put the car on a stand with the wheels off the ground.
2. Plug in USB (COM port). The ESP32 boots and outputs neutral on both channels.
3. Switch the ESC **ON**. It beeps and learns 1500 us as neutral.
4. `make test`, then `make calibrate`, then `make test-throttle`, then `make teleop`.

Power-down is the reverse: ESC off first, then unplug USB. **Always switch the ESC off
before `make flash`.** During flashing the pins stop pulsing. The ESC would cut out
anyway, but don't rely on that.

## 8. Serial protocol

ASCII, one command per line (`\n`), 115200 baud 8N1, case-insensitive. Lines are easy
to type in `make monitor` and easy to parse from Python.

| Command | Reply | Meaning |
| --- | --- | --- |
| `D <heading_deg> <speed>` | *(none)* | Drive command. heading −180..180 (+ = left), speed −1..1. Stream at ≥ 10 Hz (CLI uses 20 Hz). |
| `ARM` / `DISARM` / `STOP` | `OK ARM` / `OK DISARM` | Enable / disable throttle (`STOP` = `DISARM`). |
| `PING [seq]` | `PONG <seq> <esp_ms>` | Liveness + round-trip time. |
| `TIME <unix_ms>` | `OK TIME <unix_ms>` | Set the ESP32 clock (`settimeofday`). |
| `TEL <hz>` | `OK TEL <hz>` | Telemetry rate 0-50 Hz (0 = off). |
| — | `TEL t=… unix=… hdg=… spd=… steer_deg=… steer_us=… thr_us=… armed=…` | Telemetry line. |
| `RAW <us>` / `RAW OFF` | `OK RAW <us>` | Calibration: drive the servo to an exact pulse. |
| `CAL LEFT\|CENTER\|RIGHT <us>` | `CFG …` | Set a steering end point (RAM). |
| `SET <key> <value>` | `CFG …` | keys: `maxdeg gain limit deadband accel timeout revmode`. |
| `GET` | `CFG …` | Print all config. |
| `SAVE` | `OK SAVE` / `ERR …` | Persist config to NVS. |
| `DEFAULTS` | `CFG …` | Restore defaults (RAM, `SAVE` to persist). |
| `INFO` | `INFO fw=… chip=… …` | Firmware version, chip, pins. |
| — | `READY fw=…` | Printed at boot. |
| — | `EVT FAILSAFE …` | Command timeout fired. |

`D` has no reply on purpose. At 20 Hz, acks would double the traffic, and telemetry
already reports what the actuators are doing. Bandwidth is about 20 bytes × 20 Hz = 400
B/s, against 11.5 kB/s available at 115200 baud.

**Time sync** (`make sync-time`). The ESP32 has no battery-backed clock and no network, so
the laptop sets it. The CLI sends 10 `PING`s, takes the fastest round trip, and sends
`TIME <laptop_now + rtt/2>`. This is Cristian's algorithm, accurate to about 1 ms over
USB. Telemetry then carries `unix=` timestamps that line up with laptop-side logs, which
helps when you analyse tuning runs in pandas. The time is lost on reset, so re-run after
each flash. Under ROS 2, micro-ROS `rmw_uros_sync_session()` replaces this.

## 9. Repository layout and tooling

```
Makefile                     # bring-up commands (below)
README.md                    # pin map, wiring, power-up sequence
.gitignore                   # + firmware/.pio/, __pycache__/
docs/
  esp32-firmware-proposal.md # this file
firmware/
  platformio.ini             # board/flash/PSRAM config (currently empty in repo)
  include/pins.h             # single source of truth for pins
  src/main.cpp               # firmware
tools/
  requirements.txt           # platformio, pyserial, esptool
  rccar.py                   # laptop CLI (stdlib + pyserial)
```

### 9.1 PlatformIO config

- `platform = espressif32@6.9.0` (pinned): Arduino-ESP32 **2.0.17**, ESP-IDF 4.4.7.
  Pinning keeps builds reproducible.
- `board = esp32-s3-devkitc-1` plus `board_build.arduino.memory_type = qio_opi`:
  **Q**uad flash with **O**ctal PSRAM, which is what "N16R8" means. Picking the wrong
  memory type is the most common reason N16R8 boards boot-loop.
- `board_upload.flash_size = 16MB`, `partitions = default_16MB.csv`.
- The firmware calls the ESP-IDF LEDC driver (`driver/ledc.h`) directly, not Arduino's
  `ledcAttach`/`ledcSetup`, whose API changed between core 2.x and 3.x. Moving to
  Arduino core 3.x (the community `pioarduino` platform) later should need only small
  edits.

### 9.2 Makefile targets

| Target | Does |
| --- | --- |
| `make setup` | create `.venv` (already gitignored), install `tools/requirements.txt` |
| `make check` | check Python ≥ 3.9, `pyserial`, `esptool`, `pio`, and (Linux) membership of the `dialout`/`uucp` group needed to open serial ports |
| `make detect` | list USB serial ports and mark the CH343 (`1a86:55d3`). If the cable is in the native port (`303a:*`), say so. Run `esptool chip_id` (should print `ESP32-S3 (QFN56) (revision v0.2)`) and query firmware `INFO` |
| `make sync-time` | set the ESP32 clock (section 8) |
| `make clean` | `rm -rf firmware/.pio` |
| `make build` | `clean` → `pio run` |
| `make flash` | `clean` → `pio run -t upload` over the COM port |
| `make monitor` | `pio device monitor` (raw protocol console, with exception decoder) |
| `make test` | steering sweep (0 → +max → 0 → −max → 0) + failsafe test (arm, stop sending, expect `EVT FAILSAFE`). **Motor never commanded.** |
| `make test-throttle` | `test` + forward / neutral / reverse at 0.5 × `limit`, after typing `yes` |
| `make calibrate` | interactive servo calibration (below) |
| `make teleop` | curses keyboard driving: ←/→ heading ±5°, ↑/↓ speed ±0.1, space = speed 0, `a` = arm/disarm, `q` = quit |

`PORT` is auto-detected (CH343 VID:PID). Override with `make flash PORT=/dev/ttyACM0`.
Linux shows the CH343 as `/dev/ttyACM*`, macOS as `/dev/cu.usbmodem*`, Windows as `COMx`
(needs the WCH driver). The Makefile targets Linux/macOS. On Windows use Git Bash with
`VENV_BIN=.venv/Scripts`. WSL needs `usbipd` to pass USB through.

`flash` always cleans first, as requested. The cost is a full rebuild each time (roughly
1-2 min). The gain is that no stale object files or config survive a change to
`platformio.ini`. NVS calibration is **not** erased by flashing.

### 9.3 Calibration procedure (`make calibrate`)

1. Throttle is disarmed. The servo goes to `RAW 1500`.
2. Nudge with `+10`, `-2`, `=1520` until the front wheels point straight, then type `center`.
3. Nudge toward full left until just **before** the linkage binds (the servo starts
   buzzing or drawing current), back off about 20 us, then type `left`. Do the same for
   `right`.
4. Measure the wheel angle at lock with a protractor or phone app, then type `maxdeg 27`.
5. `save`, then `q`. Finally `make test` to see the sweep using the new limits.

## 10. Future: ROS 2 migration

- Add the `micro_ros_platformio` library. The ESP32 becomes a micro-ROS client over the
  same USB serial link, and `micro-ros-agent` runs on the Pi 5.
- Replace `loop()`'s line parser with subscribers that write the same `g_cmd`:
  `ackermann_msgs/AckermannDrive` (`steering_angle`, `speed`) maps 1:1 because of the
  REP-103 sign convention. Alternatively subscribe to `geometry_msgs/Twist` `/cmd_vel`
  and convert yaw rate to steering with the bicycle model `δ = atan(L·ω / v)`.
- Keep the failsafe: no message within `timeout` → disarm, as now.
- Publish from the reserved devices: `sensor_msgs/Imu`, `sensor_msgs/Range` (LIDAR /
  ultrasonic; emergency braking should stay **in firmware** so it works without the Pi),
  `sensor_msgs/BatteryState`, and encoder ticks → `nav_msgs/Odometry`.
- `controlTask()`, the PWM layer, NVS calibration and `pins.h` stay unchanged. The Python
  CLI stays useful for calibration and bench tests.

## 11. Verification done for this proposal

- **Pinout**: checked against the vendor's P1/P2 tables (Micro Robotics user guide, the
  link in the inventory sheet) and the ESP32-S3 strapping and PSRAM pin rules.
- **Firmware**: `main.cpp` + `pins.h` (Appendix A) were compiled to an object file with
  Espressif's `xtensa-esp32s3-elf-gcc` 8.4.0 (esp-2021r2-patch5) against the Arduino-ESP32
  2.0.17 core and ESP32-S3 SDK headers. Those are the versions `espressif32@6.9.0` uses,
  with the `qio_opi` memory config. Result: no errors or warnings with `-Wall -Wextra`.
  A full `pio run` (link + `firmware.bin`) **could not** run here, because this cloud
  environment blocks the PlatformIO package registry. The first step of the
  implementation PR is `make build` on a laptop.
- **CLI**: `rccar.py` subcommands `ping`, `sync-time`, `test`, `calibrate`, `drive`,
  `check-env`, `find-port` ran against a protocol mock on a pseudo-terminal. The mock
  failsafe fired as expected.
- **Makefile**: `make help` and `make -n` dry runs of `flash`/`clean` checked.
- **Not verified**: anything on real hardware.

## 12. Open questions / verify on hardware

1. **ESC running mode and jumpers.** The extract leaves out the jumper and beep pages.
   Confirm: battery jumper = **LiPo** (2S cutoff at 6.0 V); running mode =
   Fwd/Brake/Rev, which needs `revmode 1`, or Fwd/Brake, where reverse never engages.
2. **ESC accepts 3.3 V signal.** It almost certainly does. If throttle is erratic, add a
   74AHCT125 buffer powered from 5 V.
3. **ESC throttle endpoints.** The design assumes the standard 1000/1500/2000 us. The ESC
   auto-calibrates neutral. Check whether full throttle needs 2000 us exactly, or whether
   the ESC learns endpoints too.
4. **Servo wire colours.** These are not in the datasheet. Usually orange or yellow =
   signal, red = +, brown or black = −. Confirm with a multimeter.
5. **LIDAR-Lite v4 electrical specs** (section 5 notes). Check against Garmin's manual.
6. **TT-02 lock angle** for `maxdeg`.
7. **Which USB-C is COM.** It is labelled on the silkscreen. If `make detect` reports the
   native USB port (`303a:*`), move the cable.

## Appendix A: draft files

These are the exact files that were checked in section 11. They will be added by the
implementation PR. Nothing under `firmware/`, `tools/` or the repo root is changed by
*this* PR.

### `firmware/platformio.ini`

```ini
; ESP32-S3-N16R8 (YD-ESP32-S3 / Micro Robotics clone of ESP32-S3-DevKitC-1)
; 16 MB quad flash + 8 MB octal PSRAM
[env:esp32s3]
platform = espressif32@6.9.0
board = esp32-s3-devkitc-1
framework = arduino

board_build.arduino.memory_type = qio_opi
board_build.flash_mode = qio
board_build.psram_type = opi
board_upload.flash_size = 16MB
board_upload.maximum_size = 16777216
board_build.partitions = default_16MB.csv

build_flags =
    -DBOARD_HAS_PSRAM
    -DARDUINO_USB_CDC_ON_BOOT=0   ; Serial = UART0 = "COM" port (CH343)

upload_speed = 921600
monitor_speed = 115200
monitor_filters = esp32_exception_decoder, time
```

### `firmware/include/pins.h`

```cpp
#pragma once
// Pin map for ESP32-S3-N16R8 -- LEFT header (P1) only, see README.md.
// P1 order (USB ports at the bottom, chip facing you):
//   3V3 3V3 RST 4 5 6 7 15 16 17 18 8 3 46 9 10 11 12 13 14 5V G

// ---- Active (driven by this firmware) ----
constexpr int PIN_STEER_PWM    = 4;   // P1-4  -> DS3225 servo signal
constexpr int PIN_THROTTLE_PWM = 5;   // P1-5  -> THW-1060 ESC throttle signal

// ---- Reserved for future devices (NOT configured by this firmware) ----
constexpr int PIN_BATT_LOW     = 6;   // P1-6  battery hi/low digital in (ADC1_CH5 capable)
constexpr int PIN_IMU_INT      = 7;   // P1-7  6-axis IMU interrupt / data-ready
constexpr int PIN_ENC_A        = 15;  // P1-8  shaft encoder channel A (PCNT)
constexpr int PIN_ENC_B        = 16;  // P1-9  shaft encoder channel B (PCNT)
constexpr int PIN_SPARE_UART_TX = 17; // P1-10 spare (UART1 TX, alt. Pi link)
constexpr int PIN_SPARE_UART_RX = 18; // P1-11 spare (UART1 RX, alt. Pi link)
constexpr int PIN_I2C_SDA      = 8;   // P1-12 I2C SDA (IMU + LIDAR-Lite v4)
constexpr int PIN_I2C_SCL      = 9;   // P1-15 I2C SCL (IMU + LIDAR-Lite v4)
constexpr int PIN_DIST_TRIG    = 10;  // P1-16 ultrasonic TRIG / LIDAR GPIO
constexpr int PIN_DIST_ECHO    = 11;  // P1-17 ultrasonic ECHO (via divider) / LIDAR ready
// Free: 12 (P1-18), 13 (P1-19), 14 (P1-20)

// ---- Do not use ----
// GPIO3  (P1-13): strapping pin (JTAG signal source)
// GPIO46 (P1-14): strapping pin (boot mode / ROM log)
// GPIO35-37: wired to octal PSRAM inside the module (right header)
// GPIO19/20: native USB D-/D+ (right header); GPIO43/44: UART0 -> CH343 "COM" port
```

### `firmware/src/main.cpp`

```cpp
// RC car actuator bridge: USB serial commands -> hardware PWM for
// DS3225 steering servo + THW-1060 brushed ESC.
//
// Architecture
//   loop()        : USB serial parser + telemetry (comms only)
//   controlTask() : 100 Hz, owns the actuators: failsafe, slew, reverse logic
//   LEDC hardware : generates the 50 Hz pulse trains with zero CPU load
// The comms layer is the only part that changes when this becomes a
// micro-ROS node; controlTask() and the PWM layer stay as they are.

#include <Arduino.h>
#include <Preferences.h>
#include <driver/ledc.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <sys/time.h>

#include "pins.h"

static const char *FW_VERSION = "0.1.0";

// ---------------------------------------------------------------------------
// Fixed limits from the datasheets (docs/*.md). Never exceeded, even in RAW.
// ---------------------------------------------------------------------------
// DS3225: 500..2500 us, 1500 us neutral, 50..330 Hz frame, 3 us deadband
constexpr uint32_t SERVO_HZ         = 50;
constexpr uint16_t SERVO_ABS_MIN_US = 500;
constexpr uint16_t SERVO_ABS_MAX_US = 2500;
// THW-1060: standard RC throttle, 1500 us neutral (auto-calibrates at power-on)
constexpr uint32_t ESC_HZ           = 50;
constexpr uint16_t ESC_MIN_US       = 1000;
constexpr uint16_t ESC_NEUTRAL_US   = 1500;
constexpr uint16_t ESC_MAX_US       = 2000;

constexpr uint32_t PWM_RES_BITS = 14;   // 20000 us / 16384 = 1.22 us per tick
constexpr uint32_t CONTROL_HZ   = 100;
constexpr uint32_t BRAKE_MS     = 150;  // double-tap reverse: brake pulse
constexpr uint32_t GAP_MS       = 150;  // double-tap reverse: neutral gap
constexpr float    BRAKE_LEVEL  = 0.5f; // brake pulse strength (fraction)

constexpr ledc_mode_t      LEDC_MODE      = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t     STEER_TIMER    = LEDC_TIMER_0;
constexpr ledc_timer_t     THROTTLE_TIMER = LEDC_TIMER_1;
constexpr ledc_channel_t   STEER_CH       = LEDC_CHANNEL_0;
constexpr ledc_channel_t   THROTTLE_CH    = LEDC_CHANNEL_1;

// ---------------------------------------------------------------------------
// Tunable config, persisted in NVS flash
// ---------------------------------------------------------------------------
constexpr uint32_t CFG_MAGIC = 0xC0FFEE01;  // bump when Config layout changes

struct Config {
  uint32_t magic        = CFG_MAGIC;
  uint16_t steerCenterUs = 1500;
  uint16_t steerLeftUs   = 1800;   // pulse at full LEFT lock (may be < center)
  uint16_t steerRightUs  = 1200;   // pulse at full RIGHT lock
  float    maxSteerDeg   = 25.0f;  // road-wheel angle at full lock
  float    headingGain   = 1.0f;   // steering deg per heading deg
  float    speedLimit    = 0.30f;  // speed 1.0 -> this fraction of full throttle
  uint16_t deadbandUs    = 0;      // ESC dead zone skipped around neutral
  float    accelPerSec   = 2.0f;   // max change of speed command per second
  uint16_t timeoutMs     = 300;    // no command for this long -> failsafe
  uint8_t  doubleTapRev  = 1;      // ESC needs brake-neutral-reverse sequence
};

struct Command {
  float    headingDeg = 0.0f;  // + = left (ROS REP-103), relative to car's nose
  float    speed      = 0.0f;  // -1..1 normalized
  uint64_t stampMs    = 0;
};

struct Output {
  uint16_t steerUs    = 1500;
  uint16_t throttleUs = ESC_NEUTRAL_US;
  float    steerDeg   = 0.0f;
};

static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static Config  g_cfg;
static Command g_cmd;
static Output  g_out;
static bool    g_armed       = false;
static int     g_rawSteerUs  = -1;     // >0: calibration override
static bool    g_failsafeEvt = false;  // set by control task, reported by loop
static uint32_t g_telemetryHz = 0;
static bool    g_timeSynced  = false;

static Preferences g_prefs;

static uint64_t nowMs() { return esp_timer_get_time() / 1000ULL; }

template <typename T>
static T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

// ---------------------------------------------------------------------------
// PWM layer (LEDC). Once a duty is set the hardware keeps emitting pulses.
// ---------------------------------------------------------------------------
static uint32_t usToDuty(uint32_t us, uint32_t hz) {
  const uint32_t periodUs = 1000000UL / hz;
  return (uint32_t)(((uint64_t)us << PWM_RES_BITS) / periodUs);
}

static void pwmInit(int pin, ledc_timer_t timer, ledc_channel_t ch,
                    uint32_t hz, uint16_t initialUs) {
  ledc_timer_config_t t = {};
  t.speed_mode      = LEDC_MODE;
  t.duty_resolution = (ledc_timer_bit_t)PWM_RES_BITS;
  t.timer_num       = timer;
  t.freq_hz         = hz;
  t.clk_cfg         = LEDC_AUTO_CLK;
  ESP_ERROR_CHECK(ledc_timer_config(&t));

  ledc_channel_config_t c = {};
  c.gpio_num   = pin;
  c.speed_mode = LEDC_MODE;
  c.channel    = ch;
  c.timer_sel  = timer;
  c.duty       = usToDuty(initialUs, hz);
  c.hpoint     = 0;
  ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static void pwmWriteUs(ledc_channel_t ch, uint32_t hz, uint16_t us) {
  ledc_set_duty(LEDC_MODE, ch, usToDuty(us, hz));
  ledc_update_duty(LEDC_MODE, ch);
}

// ---------------------------------------------------------------------------
// Mapping
// ---------------------------------------------------------------------------
// heading (deg) -> road-wheel steering angle (deg), open loop for now.
// With an IMU this becomes a closed heading loop; the interface stays.
static float headingToSteerDeg(float headingDeg, const Config &c) {
  return clampv(headingDeg * c.headingGain, -c.maxSteerDeg, c.maxSteerDeg);
}

// steering angle -> pulse, piecewise linear so an asymmetric linkage works.
static uint16_t steerDegToUs(float deg, const Config &c) {
  float frac = deg / c.maxSteerDeg;  // -1 (right) .. +1 (left)
  float us = frac >= 0 ? c.steerCenterUs + frac * (c.steerLeftUs - c.steerCenterUs)
                       : c.steerCenterUs + -frac * (c.steerRightUs - c.steerCenterUs);
  uint16_t lo = min(c.steerLeftUs, c.steerRightUs);
  uint16_t hi = max(c.steerLeftUs, c.steerRightUs);
  return clampv<uint16_t>((uint16_t)lroundf(us), lo, hi);
}

// throttle fraction (-1..1, already speed-limited) -> ESC pulse
static uint16_t throttleToUs(float u, const Config &c) {
  if (fabsf(u) < 0.001f) return ESC_NEUTRAL_US;
  float span = (u > 0 ? ESC_MAX_US - ESC_NEUTRAL_US : ESC_NEUTRAL_US - ESC_MIN_US) - c.deadbandUs;
  float us = ESC_NEUTRAL_US + (u > 0 ? 1 : -1) * (c.deadbandUs + fabsf(u) * span);
  return clampv<uint16_t>((uint16_t)lroundf(us), ESC_MIN_US, ESC_MAX_US);
}

// ---------------------------------------------------------------------------
// Control task: the only writer of the PWM outputs
// ---------------------------------------------------------------------------
enum class RevState { Forward, Brake, Gap, Reverse };

static void controlTask(void *) {
  esp_task_wdt_add(nullptr);  // hang -> chip reset -> no pulses -> ESC cuts in 0.1 s
  TickType_t last = xTaskGetTickCount();
  float speedOut = 0.0f;
  RevState rs = RevState::Forward;
  uint64_t rsSince = 0;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / CONTROL_HZ));
    esp_task_wdt_reset();

    Config cfg; Command cmd; bool armed; int raw;
    portENTER_CRITICAL(&g_mux);
    cfg = g_cfg; cmd = g_cmd; armed = g_armed; raw = g_rawSteerUs;
    portEXIT_CRITICAL(&g_mux);

    const uint64_t now = nowMs();
    const bool stale = (now - cmd.stampMs) > cfg.timeoutMs;
    if (armed && stale) {
      armed = false;
      portENTER_CRITICAL(&g_mux);
      g_armed = false; g_failsafeEvt = true;
      portEXIT_CRITICAL(&g_mux);
    }

    // Speed: slew-limited toward target; disarm snaps to neutral.
    const float target = armed ? clampv(cmd.speed, -1.0f, 1.0f) : 0.0f;
    const float step = cfg.accelPerSec / CONTROL_HZ;
    speedOut = armed ? speedOut + clampv(target - speedOut, -step, step) : 0.0f;
    float u = speedOut * cfg.speedLimit;

    // Many brushed ESCs (Fwd/Brake/Rev mode) treat the first reverse pulse as
    // BRAKE and only reverse after a return to neutral. Do that sequence here.
    if (cfg.doubleTapRev) {
      switch (rs) {
        case RevState::Forward:
          if (u < 0) { rs = RevState::Brake; rsSince = now; u = -BRAKE_LEVEL; }
          break;
        case RevState::Brake:
          if (u >= 0) rs = RevState::Forward;
          else if (now - rsSince >= BRAKE_MS) { rs = RevState::Gap; rsSince = now; u = 0; }
          else u = -BRAKE_LEVEL;
          break;
        case RevState::Gap:
          if (u >= 0) rs = RevState::Forward;
          else if (now - rsSince >= GAP_MS) rs = RevState::Reverse;
          else u = 0;
          break;
        case RevState::Reverse:
          if (u > 0) rs = RevState::Forward;
          break;
      }
    }

    // Steering follows commands even when disarmed (safe steering tuning).
    const float steerDeg = headingToSteerDeg(cmd.headingDeg, cfg);
    const uint16_t steerUs = raw > 0
        ? clampv<uint16_t>((uint16_t)raw, SERVO_ABS_MIN_US, SERVO_ABS_MAX_US)
        : steerDegToUs(steerDeg, cfg);
    const uint16_t thrUs = throttleToUs(u, cfg);

    pwmWriteUs(STEER_CH, SERVO_HZ, steerUs);
    pwmWriteUs(THROTTLE_CH, ESC_HZ, thrUs);

    portENTER_CRITICAL(&g_mux);
    g_out.steerUs = steerUs; g_out.throttleUs = thrUs; g_out.steerDeg = steerDeg;
    portEXIT_CRITICAL(&g_mux);
  }
}

// ---------------------------------------------------------------------------
// Persistence
// ---------------------------------------------------------------------------
static void loadConfig() {
  Config c;
  g_prefs.begin("rccar", true);
  size_t n = g_prefs.getBytes("cfg", &c, sizeof(c));
  g_prefs.end();
  if (n == sizeof(c) && c.magic == CFG_MAGIC) g_cfg = c;
}

static bool saveConfig() {
  Config c;
  portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
  g_prefs.begin("rccar", false);
  size_t n = g_prefs.putBytes("cfg", &c, sizeof(c));
  g_prefs.end();
  return n == sizeof(c);
}

// ---------------------------------------------------------------------------
// Serial protocol (one ASCII command per line, see docs)
// ---------------------------------------------------------------------------
static void printConfig() {
  Config c;
  portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
  Serial.printf("CFG center=%u left=%u right=%u maxdeg=%.1f gain=%.2f limit=%.2f "
                "deadband=%u accel=%.2f timeout=%u revmode=%u\n",
                c.steerCenterUs, c.steerLeftUs, c.steerRightUs, c.maxSteerDeg,
                c.headingGain, c.speedLimit, c.deadbandUs, c.accelPerSec,
                c.timeoutMs, c.doubleTapRev);
}

static void printTelemetry() {
  Command cmd; Output o; bool armed;
  portENTER_CRITICAL(&g_mux); cmd = g_cmd; o = g_out; armed = g_armed; portEXIT_CRITICAL(&g_mux);
  struct timeval tv; gettimeofday(&tv, nullptr);
  uint64_t unixMs = g_timeSynced ? (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000 : 0;
  Serial.printf("TEL t=%llu unix=%llu hdg=%.1f spd=%.2f steer_deg=%.1f steer_us=%u thr_us=%u armed=%d\n",
                nowMs(), unixMs, cmd.headingDeg, cmd.speed, o.steerDeg,
                o.steerUs, o.throttleUs, armed);
}

static bool setKey(Config &c, const char *key, float v) {
  if      (!strcasecmp(key, "maxdeg"))   c.maxSteerDeg = clampv(v, 1.0f, 60.0f);
  else if (!strcasecmp(key, "gain"))     c.headingGain = clampv(v, 0.0f, 10.0f);
  else if (!strcasecmp(key, "limit"))    c.speedLimit  = clampv(v, 0.0f, 1.0f);
  else if (!strcasecmp(key, "deadband")) c.deadbandUs  = (uint16_t)clampv(v, 0.0f, 200.0f);
  else if (!strcasecmp(key, "accel"))    c.accelPerSec = clampv(v, 0.1f, 100.0f);
  else if (!strcasecmp(key, "timeout"))  c.timeoutMs   = (uint16_t)clampv(v, 50.0f, 2000.0f);
  else if (!strcasecmp(key, "revmode"))  c.doubleTapRev = v != 0;
  else return false;
  return true;
}

static void handleLine(char *line) {
  char *tok = strtok(line, " \t\r");
  if (!tok) return;
  char *a1 = strtok(nullptr, " \t\r");
  char *a2 = strtok(nullptr, " \t\r");

  if (!strcasecmp(tok, "D")) {  // D <heading_deg> <speed>
    if (!a1 || !a2) { Serial.println("ERR usage: D <heading_deg> <speed>"); return; }
    portENTER_CRITICAL(&g_mux);
    g_cmd.headingDeg = clampv(strtof(a1, nullptr), -180.0f, 180.0f);
    g_cmd.speed      = clampv(strtof(a2, nullptr), -1.0f, 1.0f);
    g_cmd.stampMs    = nowMs();
    portEXIT_CRITICAL(&g_mux);
    // no reply: commands stream at 20-50 Hz, telemetry is the feedback
  } else if (!strcasecmp(tok, "ARM")) {
    portENTER_CRITICAL(&g_mux);
    g_cmd.speed = 0; g_cmd.stampMs = nowMs(); g_armed = true;
    portEXIT_CRITICAL(&g_mux);
    Serial.println("OK ARM");
  } else if (!strcasecmp(tok, "DISARM") || !strcasecmp(tok, "STOP")) {
    portENTER_CRITICAL(&g_mux); g_armed = false; g_cmd.speed = 0; portEXIT_CRITICAL(&g_mux);
    Serial.println("OK DISARM");
  } else if (!strcasecmp(tok, "PING")) {
    Serial.printf("PONG %s %llu\n", a1 ? a1 : "0", nowMs());
  } else if (!strcasecmp(tok, "TIME")) {  // TIME <unix_ms>
    if (!a1) { Serial.println("ERR usage: TIME <unix_ms>"); return; }
    uint64_t ms = strtoull(a1, nullptr, 10);
    struct timeval tv = { (time_t)(ms / 1000), (suseconds_t)((ms % 1000) * 1000) };
    settimeofday(&tv, nullptr);
    g_timeSynced = true;
    Serial.printf("OK TIME %llu\n", ms);
  } else if (!strcasecmp(tok, "RAW")) {  // RAW <steer_us> | RAW OFF
    if (!a1) { Serial.println("ERR usage: RAW <us>|OFF"); return; }
    int us = strcasecmp(a1, "OFF") ? clampv<int>(atoi(a1), SERVO_ABS_MIN_US, SERVO_ABS_MAX_US) : -1;
    portENTER_CRITICAL(&g_mux); g_rawSteerUs = us; portEXIT_CRITICAL(&g_mux);
    Serial.printf("OK RAW %d\n", us);
  } else if (!strcasecmp(tok, "CAL")) {  // CAL LEFT|CENTER|RIGHT <us>
    if (!a1 || !a2) { printConfig(); return; }
    uint16_t us = clampv<int>(atoi(a2), SERVO_ABS_MIN_US, SERVO_ABS_MAX_US);
    bool ok = true;
    portENTER_CRITICAL(&g_mux);
    if      (!strcasecmp(a1, "LEFT"))   g_cfg.steerLeftUs = us;
    else if (!strcasecmp(a1, "CENTER")) g_cfg.steerCenterUs = us;
    else if (!strcasecmp(a1, "RIGHT"))  g_cfg.steerRightUs = us;
    else ok = false;
    portEXIT_CRITICAL(&g_mux);
    if (ok) printConfig(); else Serial.println("ERR CAL LEFT|CENTER|RIGHT <us>");
  } else if (!strcasecmp(tok, "SET")) {  // SET <key> <value>
    if (!a1 || !a2) { Serial.println("ERR usage: SET <key> <value>"); return; }
    portENTER_CRITICAL(&g_mux);
    bool ok = setKey(g_cfg, a1, strtof(a2, nullptr));
    portEXIT_CRITICAL(&g_mux);
    if (ok) printConfig(); else Serial.printf("ERR unknown key %s\n", a1);
  } else if (!strcasecmp(tok, "GET")) {
    printConfig();
  } else if (!strcasecmp(tok, "SAVE")) {
    Config c;
    portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
    bool between = (c.steerCenterUs > min(c.steerLeftUs, c.steerRightUs)) &&
                   (c.steerCenterUs < max(c.steerLeftUs, c.steerRightUs));
    if (!between) { Serial.println("ERR center must lie between left and right"); return; }
    Serial.println(saveConfig() ? "OK SAVE" : "ERR SAVE failed");
  } else if (!strcasecmp(tok, "DEFAULTS")) {
    portENTER_CRITICAL(&g_mux); g_cfg = Config(); portEXIT_CRITICAL(&g_mux);
    printConfig();  // not persisted until SAVE
  } else if (!strcasecmp(tok, "TEL")) {  // TEL <hz>, 0 = off
    g_telemetryHz = a1 ? clampv<int>(atoi(a1), 0, 50) : 10;
    Serial.printf("OK TEL %u\n", g_telemetryHz);
  } else if (!strcasecmp(tok, "INFO")) {
    Serial.printf("INFO fw=%s chip=%s rev=%d psram=%u flash=%u steer_pin=%d throttle_pin=%d\n",
                  FW_VERSION, ESP.getChipModel(), ESP.getChipRevision(),
                  ESP.getPsramSize(), ESP.getFlashChipSize(),
                  PIN_STEER_PWM, PIN_THROTTLE_PWM);
  } else {
    Serial.printf("ERR unknown command %s\n", tok);
  }
}

// ---------------------------------------------------------------------------
void setup() {
  // Outputs first: neutral pulses from the very first moment, so the ESC
  // learns 1500 us as neutral when it is switched on.
  pwmInit(PIN_STEER_PWM, STEER_TIMER, STEER_CH, SERVO_HZ, 1500);
  pwmInit(PIN_THROTTLE_PWM, THROTTLE_TIMER, THROTTLE_CH, ESC_HZ, ESC_NEUTRAL_US);

  Serial.begin(115200);
  loadConfig();
  xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr,
                          configMAX_PRIORITIES - 2, nullptr, 1);
  Serial.printf("READY fw=%s\n", FW_VERSION);
}

void loop() {
  static char buf[96];
  static size_t len = 0;
  static uint64_t lastTel = 0;

  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n') { buf[len] = '\0'; handleLine(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = ch;
    else len = 0;  // overlong line: drop it
  }

  bool fs;
  portENTER_CRITICAL(&g_mux); fs = g_failsafeEvt; g_failsafeEvt = false; portEXIT_CRITICAL(&g_mux);
  if (fs) Serial.println("EVT FAILSAFE command timeout -> disarmed");

  if (g_telemetryHz && nowMs() - lastTel >= 1000 / g_telemetryHz) {
    lastTel = nowMs();
    printTelemetry();
  }
  delay(1);
}
```

### `tools/rccar.py`

```python
#!/usr/bin/env python3
"""Laptop-side CLI for the ESP32-S3 RC car actuator bridge.

Talks the line protocol in firmware/src/main.cpp over the board's "COM"
USB-C port (CH343 USB-UART bridge). Every subcommand is wrapped by a
Makefile target; run `python tools/rccar.py -h` for the full list.
"""
from __future__ import annotations

import argparse
import importlib.util
import shutil
import subprocess
import sys
import time
from pathlib import Path

BAUD = 115200
SEND_HZ = 20                      # drive command rate; firmware timeout is 300 ms
CH343_VID, CH343_PID = 0x1A86, 0x55D3
ESPRESSIF_VID = 0x303A            # native USB port (GPIO19/20), not used for comms


# ---------------------------------------------------------------------------
# Serial link
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, port: str, baud: int = BAUD):
        import serial

        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, baud, 0.05
        self.ser.dtr = False      # keep EN/IO0 released so opening doesn't reset
        self.ser.rts = False
        self.ser.open()
        self.handshake()

    def send(self, line: str) -> None:
        self.ser.write((line + "\n").encode())

    def readline(self) -> str | None:
        raw = self.ser.readline()
        return raw.decode(errors="replace").strip() if raw else None

    def expect(self, prefix: str, timeout: float = 1.0) -> str:
        """Read lines until one starts with prefix; print unexpected ones."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            line = self.readline()
            if line is None:
                continue
            if line.startswith(prefix):
                return line
            if line.startswith(("ERR", "EVT")):
                print(f"  esp32: {line}")
        raise TimeoutError(f"no '{prefix}' reply within {timeout}s")

    def request(self, line: str, prefix: str, timeout: float = 1.0) -> str:
        self.send(line)
        return self.expect(prefix, timeout)

    def handshake(self, attempts: int = 20) -> None:
        for _ in range(attempts):          # covers a reboot on port open (~1 s)
            self.send("PING 0")
            try:
                self.expect("PONG", 0.15)
                self.ser.reset_input_buffer()
                return
            except TimeoutError:
                pass
        raise SystemExit("ESP32 not answering PING - is the firmware flashed? (make flash)")

    def ping(self, seq: int) -> tuple[float, int]:
        t0 = time.monotonic()
        reply = self.request(f"PING {seq}", f"PONG {seq}")
        return (time.monotonic() - t0) * 1000.0, int(reply.split()[2])

    def drive_for(self, heading: float, speed: float, seconds: float) -> None:
        """Stream D commands so the firmware failsafe stays satisfied."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(f"D {heading:.1f} {speed:.3f}")
            time.sleep(1.0 / SEND_HZ)

    def close(self) -> None:
        try:
            self.send("STOP")
            self.send("RAW OFF")
            self.send("TEL 0")
        finally:
            self.ser.close()


def find_port() -> str | None:
    from serial.tools import list_ports

    ports = list(list_ports.comports())
    for p in ports:
        if (p.vid, p.pid) == (CH343_VID, CH343_PID):
            return p.device
    for p in ports:
        if p.vid == CH343_VID:             # other WCH bridges (CH340/CH9102)
            return p.device
    return None


def open_link(args) -> Link:
    port = args.port or find_port()
    if not port:
        raise SystemExit("no ESP32 COM port found - run `make detect`")
    return Link(port)


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------
def cmd_check_env(args) -> int:
    ok = True
    print(f"python   {sys.version.split()[0]}", "OK" if sys.version_info >= (3, 9) else "NEED >= 3.9")
    ok &= sys.version_info >= (3, 9)
    for mod, pkg in (("serial", "pyserial"), ("esptool", "esptool")):
        found = importlib.util.find_spec(mod) is not None
        print(f"{pkg:<8} {'OK' if found else 'MISSING (make setup)'}")
        ok &= found
    venv_pio = Path(sys.executable).parent / "pio"
    pio = str(venv_pio) if venv_pio.exists() else shutil.which("pio")
    print(f"pio      {pio or 'MISSING (make setup)'}")
    ok &= pio is not None
    if sys.platform.startswith("linux"):
        groups = subprocess.run(["id", "-Gn"], capture_output=True, text=True).stdout.split()
        member = bool({"dialout", "uucp"} & set(groups))
        print(f"serial permission  {'OK' if member else 'add yourself: sudo usermod -aG dialout $USER, then log out/in'}")
    return 0 if ok else 1


def cmd_find_port(args) -> int:
    port = find_port()
    if port:
        print(port)
    return 0 if port else 1


def cmd_detect(args) -> int:
    from serial.tools import list_ports

    print("USB serial ports:")
    for p in list_ports.comports():
        tag = ""
        if (p.vid, p.pid) == (CH343_VID, CH343_PID):
            tag = "  <- ESP32 'COM' port (CH343) - use this one"
        elif p.vid == ESPRESSIF_VID:
            tag = "  <- ESP32 native 'USB' port - move the cable to 'COM'"
        vidpid = f"{p.vid:04x}:{p.pid:04x}" if p.vid else "----:----"
        print(f"  {p.device:<22} {vidpid}  {p.description}{tag}")
    port = args.port or find_port()
    if not port:
        print("ESP32 not found. Check cable (must carry data) and that it is in the 'COM' port.")
        return 1
    print(f"\nesptool chip_id on {port} (resets the board):")
    rc = subprocess.run([sys.executable, "-m", "esptool", "--port", port, "chip_id"]).returncode
    if rc != 0:
        return rc
    time.sleep(1.5)                        # let the firmware boot after the reset
    try:
        link = Link(port)
        print("\nfirmware:", link.request("INFO", "INFO"))
        link.close()
    except SystemExit as e:
        print(f"\nfirmware: {e}")
    return 0


def cmd_ping(args) -> int:
    link = open_link(args)
    rtts = [link.ping(i)[0] for i in range(1, 11)]
    print(f"10 pings: min {min(rtts):.1f} ms  avg {sum(rtts) / len(rtts):.1f} ms  max {max(rtts):.1f} ms")
    link.close()
    return 0


def cmd_sync_time(args) -> int:
    """Cristian's algorithm: best of N round trips, assume symmetric latency."""
    link = open_link(args)
    best = min(link.ping(i)[0] for i in range(1, 11))
    unix_ms = int(time.time() * 1000 + best / 2)
    link.request(f"TIME {unix_ms}", "OK TIME")
    link.request("TEL 10", "OK TEL")
    tel = link.expect("TEL ", 1.0)
    esp_unix = int(dict(kv.split("=") for kv in tel.split()[1:])["unix"])
    print(f"synced: rtt {best:.1f} ms, esp32 - laptop = {esp_unix - time.time() * 1000:+.0f} ms")
    link.close()
    return 0


def cmd_test(args) -> int:
    link = open_link(args)
    print(link.request("INFO", "INFO"))
    cfg = link.request("GET", "CFG")
    print(cfg)
    maxdeg = float(cfg.split("maxdeg=")[1].split()[0])

    print("\n[1/3] steering sweep (throttle disarmed)")
    for hdg in (0, maxdeg, 0, -maxdeg, 0):
        print(f"  heading {hdg:+.0f} deg")
        link.drive_for(hdg, 0.0, 1.0)

    print("\n[2/3] failsafe: stop sending while armed")
    link.request("ARM", "OK ARM")
    link.drive_for(0, 0.0, 0.3)
    t0 = time.monotonic()
    link.expect("EVT FAILSAFE", 2.0)
    print(f"  failsafe fired after {(time.monotonic() - t0) * 1000:.0f} ms of silence  OK")

    if not args.throttle:
        print("\n[3/3] throttle skipped (use `make test-throttle` with the wheels OFF the ground)")
        link.close()
        return 0
    if input("\n[3/3] throttle test. Wheels off the ground? type 'yes': ").strip() != "yes":
        link.close()
        return 1
    link.request("ARM", "OK ARM")
    for label, spd, sec in (("forward", 0.5, 1.5), ("neutral", 0.0, 1.0),
                            ("reverse", -0.5, 1.5), ("neutral", 0.0, 0.5)):
        print(f"  {label} speed {spd:+.1f}")
        link.drive_for(0, spd, sec)
    link.close()
    print("done")
    return 0


CAL_HELP = """commands:
  +N / -N        nudge pulse by N us (e.g. +10, -2)
  =N             jump to N us
  center|left|right   store current pulse as that end point
  maxdeg D       road-wheel angle (deg) at full lock, measured
  gain G         heading -> steering gain
  show           print config      save   persist to flash
  q              quit (unsaved changes are kept until reboot)"""


def cmd_calibrate(args) -> int:
    link = open_link(args)
    link.send("STOP")
    us = 1500
    link.request(f"RAW {us}", "OK RAW")
    print(CAL_HELP)
    print("Start: servo at 1500 us. Find where the wheels point straight, then each lock.")
    print("Stop BEFORE the linkage binds (servo buzzing = stall current).")
    while True:
        cmd = input(f"[{us} us] > ").strip().lower()
        if cmd in ("q", "quit", "exit"):
            break
        try:
            if cmd[:1] in "+-" and cmd[1:].isdigit():
                us += int(cmd)
            elif cmd.startswith("="):
                us = int(cmd[1:])
            elif cmd in ("center", "left", "right"):
                print(link.request(f"CAL {cmd.upper()} {us}", "CFG"))
                continue
            elif cmd.split()[0] in ("maxdeg", "gain"):
                print(link.request(f"SET {cmd}", "CFG"))
                continue
            elif cmd == "show":
                print(link.request("GET", "CFG"))
                continue
            elif cmd == "save":
                link.send("SAVE")
                print(link.expect("OK SAVE", 2.0))
                continue
            else:
                print(CAL_HELP)
                continue
        except (ValueError, IndexError, TimeoutError) as e:
            print(f"  ? {e}")
            continue
        us = max(500, min(2500, us))
        link.request(f"RAW {us}", "OK RAW")
    link.close()
    return 0


def cmd_teleop(args) -> int:
    import curses

    link = open_link(args)
    link.request("TEL 10", "OK TEL")

    def ui(scr) -> None:
        scr.nodelay(True)
        heading, speed, armed, tel = 0.0, 0.0, False, ""
        while True:
            key = scr.getch()
            if key in (ord("q"), 27):
                break
            if key == curses.KEY_LEFT:
                heading += 5
            elif key == curses.KEY_RIGHT:
                heading -= 5
            elif key == curses.KEY_UP:
                speed = min(1.0, speed + 0.1)
            elif key == curses.KEY_DOWN:
                speed = max(-1.0, speed - 0.1)
            elif key == ord(" "):
                speed = 0.0
            elif key == ord("c"):
                heading = 0.0
            elif key == ord("a"):
                link.send("DISARM" if armed else "ARM")
                armed = not armed
            link.send(f"D {heading:.1f} {speed:.2f}")
            while (line := link.readline()) is not None:
                if line.startswith("TEL"):
                    tel = line
                elif line.startswith("EVT FAILSAFE"):
                    armed = False
            scr.erase()
            scr.addstr(0, 0, "arrows: heading/speed  space: speed 0  c: center  a: arm/disarm  q: quit")
            scr.addstr(2, 0, f"heading {heading:+6.1f} deg   speed {speed:+.2f}   {'ARMED' if armed else 'disarmed'}")
            scr.addstr(4, 0, tel[:curses.COLS - 1])
            scr.refresh()
            time.sleep(1.0 / SEND_HZ)

    try:
        curses.wrapper(ui)
    finally:
        link.close()
    return 0


def cmd_drive(args) -> int:
    """One-shot scripted command, e.g. for tuning runs or other scripts."""
    link = open_link(args)
    if args.speed:
        link.request("ARM", "OK ARM")
    link.drive_for(args.heading, args.speed, args.seconds)
    link.close()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: auto-detect CH343)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check-env").set_defaults(fn=cmd_check_env)
    sub.add_parser("find-port").set_defaults(fn=cmd_find_port)
    sub.add_parser("detect").set_defaults(fn=cmd_detect)
    sub.add_parser("ping").set_defaults(fn=cmd_ping)
    sub.add_parser("sync-time").set_defaults(fn=cmd_sync_time)
    t = sub.add_parser("test")
    t.add_argument("--throttle", action="store_true", help="also spin the motor")
    t.set_defaults(fn=cmd_test)
    sub.add_parser("calibrate").set_defaults(fn=cmd_calibrate)
    sub.add_parser("teleop").set_defaults(fn=cmd_teleop)
    d = sub.add_parser("drive")
    d.add_argument("heading", type=float, help="deg, + = left")
    d.add_argument("speed", type=float, help="-1..1")
    d.add_argument("--seconds", type=float, default=1.0)
    d.set_defaults(fn=cmd_drive)
    args = ap.parse_args()
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())
```

### `tools/requirements.txt`

```text
platformio>=6.1,<7
pyserial>=3.5
esptool>=4.8,<5
```

### `Makefile`

```make
# RC car ESP32-S3 bring-up. Run `make help`.
# Linux/macOS. On Windows use Git Bash and set VENV_BIN=.venv/Scripts.
PYTHON   ?= python3
VENV     := .venv
VENV_BIN ?= $(VENV)/bin
PY       := $(VENV_BIN)/python
PIO      := $(VENV_BIN)/pio
FW_DIR   := firmware
CLI      := $(PY) tools/rccar.py
ifeq ($(origin PORT),undefined)       # auto-detect once per make run
PORT     := $(shell $(CLI) find-port 2>/dev/null)
endif
PORT_ARG := $(if $(PORT),--port $(PORT),)

.DEFAULT_GOAL := help
.PHONY: help setup check detect sync-time clean build flash monitor \
        test test-throttle calibrate teleop

help:            ## list targets
	@grep -E '^[a-z-]+:.*## ' $(MAKEFILE_LIST) | awk -F':.*## ' '{printf "  make %-14s %s\n", $$1, $$2}'

$(PY):
	$(PYTHON) -m venv $(VENV)

setup: $(PY)     ## create .venv and install platformio, pyserial, esptool
	$(PY) -m pip install -q --upgrade pip
	$(PY) -m pip install -q -r tools/requirements.txt

check: $(PY)     ## verify python packages, pio, serial permissions
	$(CLI) check-env

detect: $(PY)    ## find the ESP32 on USB, read chip id, query firmware
	$(CLI) $(PORT_ARG) detect

sync-time: $(PY) ## set the ESP32 clock from this laptop
	$(CLI) $(PORT_ARG) sync-time

clean:           ## delete firmware/.pio build cache
	rm -rf $(FW_DIR)/.pio

build: clean     ## clean + compile firmware
	$(PIO) run -d $(FW_DIR)

flash: clean     ## clean + compile + upload firmware over the COM port
	$(PIO) run -d $(FW_DIR) -t upload $(if $(PORT),--upload-port $(PORT),)

monitor:         ## raw serial monitor (Ctrl+C to exit)
	$(PIO) device monitor -d $(FW_DIR) $(if $(PORT),--port $(PORT),)

test: $(PY)      ## steering sweep + failsafe check (motor stays off)
	$(CLI) $(PORT_ARG) test

test-throttle: $(PY) ## test + short motor run, WHEELS OFF THE GROUND
	$(CLI) $(PORT_ARG) test --throttle

calibrate: $(PY) ## interactive servo center/left/right calibration
	$(CLI) $(PORT_ARG) calibrate

teleop: $(PY)    ## keyboard driving: arrows = heading/speed
	$(CLI) $(PORT_ARG) teleop
```

### `.gitignore` (additions)

```text
firmware/.pio/
__pycache__/
```
