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
- **The ESP32 powers nothing.** Its only jobs are outputting actuator signals and talking
  to the host computer. Sensors will get power from a separate, tightly regulated sensor
  rail. Their *signal* wires may later land on the reserved pins, with a common ground
  and ≤ 3.3 V logic guaranteed by that rail's design.
- Tuning lives in one root file (`tuning.toml`), pushed to the ESP32 at runtime, so most
  adjustments need no firmware edit or reflash. Every session is logged to `logs/` for
  later analysis.
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

 2S LiPo ──▶ sensor power rail (separate, regulated; future)
              ├─▶ IMU / LIDAR / encoder / battery-monitor VCC
              └─ GND ──▶ common ground (P1-22)          signals ≤ 3.3 V ──▶ reserved pins
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
4. **Sensors are powered by a separate, tightly regulated sensor rail, never by the
   ESP32.** That includes 3.3 V parts: the `3V3` pins stay unused. This keeps sensor
   current spikes off the MCU's regulator and lets the rail be filtered and sequenced
   independently. (The board's `5V` pin could not supply them anyway: it is an *input* by
   default.) The rail shares ground with the ESP32. Any sensor output that reaches an
   ESP32 pin must be ≤ 3.3 V. This should be a design requirement for the rail
   (level-shift on the rail side).

## 5. Pin plan (left header P1 only, verified)

The board's left header (P1, the side with `5V` and `G`) is, top to bottom with the USB-C
ports at the bottom:
`3V3 3V3 RST 4 5 6 7 15 16 17 18 8 3 46 9 10 11 12 13 14 5V G`.
This matches the P1 table in the vendor guide and Espressif's DevKitC-1 J1 header.

| P1 # | Label | GPIO capabilities (vendor table) | Assignment | Status |
| --- | --- | --- | --- | --- |
| 1 | 3V3 | 3.3 V out (LDO, 1 A total) | not used (sensors use their own rail) | power |
| 2 | 3V3 | 3.3 V out | not used | power |
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

Notes for the reserved devices (wiring guidance only, no code in this PR). Each device's
**power** comes from the sensor rail (rule 4 above). These notes cover only what its
*signal* lines need before they reach an ESP32 pin.
- **Battery hi/low.** A 2S pack reaches 8.4 V, so **never** wire it directly to a pin.
  A bare resistor divider does not give a clean hi/low either: ESP32 inputs read
  undefined between about 0.8 V and 2.5 V. Use a comparator or voltage-detector module
  set to about 6.8 V (3.4 V per cell, above the ESC's 6.0 V cutoff), with an open-drain
  output pulled up to 3.3 V. GPIO6 is ADC-capable, so we can switch to a divider plus
  analog reading later.
- **IMU (6-axis).** Most breakouts (MPU-6050, ICM-42688, BMI270, LSM6DS3) use 3.3 V I2C.
  Power it from the sensor rail, use SDA/SCL as above and INT on GPIO7. I2C pull-ups go
  to 3.3 V.
- **LIDAR-Lite v4 LED.** Garmin's manual could not be downloaded from this environment.
  From memory, it runs on 4.75-5.25 V with 3.3 V I2C logic at 0x62. **Verify against the
  manual before wiring.** It is powered from the sensor rail's 5 V output.
- **Ultrasonic (HC-SR04 type).** Runs on 5 V (sensor rail) and its ECHO output is 5 V, so
  add a 1 kΩ/2 kΩ divider (or the rail's level shifter) into GPIO11.
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
  3V3   │  1  │ unused (sensors: own rail)   │
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
- The frame rate is **runtime-tunable** from 50 to 330 Hz (`servo_hz` in `tuning.toml`)
  because the servo has its own LEDC timer. The firmware calls `ledc_set_freq()` live, and
  pulse widths are recomputed for the new period. The board always boots at 50 Hz, then
  applies the saved rate. See the [latency budget](#94-tuning-file-tuningtoml).

### 6.2 ESC (GPIO5)

- 50 Hz, 1000 / **1500** / 2000 us (reverse-full / neutral / forward-full). All four
  values are tunable (`esc_hz`, `esc_min_us`, `esc_neutral_us`, `esc_max_us`). The
  firmware enforces a 900-2100 us hard window and `min < neutral < max`. Leave `esc_hz` at
  50 unless the ESC is confirmed to accept faster frames: many RC ESCs only accept
  50-60 Hz input.
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
| Bad tuning value | Every `SET` is range-clamped in firmware. ESC endpoint changes that would break `min < neutral < max` are rejected with `ERR`, and the CLI aborts and names the offending `tuning.toml` line. |

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
| `SET <key> <value>` | `CFG …` / `ERR …` | keys: `maxdeg gain limit deadband accel timeout revmode brake_ms gap_ms brake_level esc_min esc_neutral esc_max servo_hz esc_hz control_hz` (see the 9.4 mapping). |
| `GET` | `CFG …` | Print all config. |
| `SAVE` | `OK SAVE` / `ERR …` | Persist config to NVS. |
| `DEFAULTS` | `CFG …` | Restore defaults (RAM, `SAVE` to persist). |
| `SENS <name>` | `SENS <name> k=v …` / `ERR …` | **Reserved** for sensor reads (`BATT IMU DIST ENC`). Returns `ERR … not implemented` for now. |
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
CLAUDE.md                    # tiny machine-readable project map (9.7)
Makefile                     # bring-up commands (9.2)
README.md                    # pin map, wiring, power-up sequence
tuning.toml                  # all runtime tuning, source of truth (9.4)
.gitignore                   # + firmware/.pio/, __pycache__/, logs/*.jsonl
.claude/
  rules/firmware.md          # firmware rules, loaded only for firmware/** (9.7)
docs/
  esp32-firmware-proposal.md # this file
  hardware/                  # datasheet extracts (ds3225_servo.md, thw-1060-brushed-esc.md)
firmware/
  platformio.ini             # board/flash/PSRAM config (currently empty in repo)
  include/pins.h             # single source of truth for pins
  src/main.cpp               # firmware
logs/
  .gitkeep                   # session logs land here, *.jsonl gitignored (9.6)
test/
  esp32_session.py           # interactive tuning / quick-test scaffold (9.5)
tools/
  requirements.txt           # platformio, pyserial, esptool
  rccar.py                   # CLI + shared Link / SessionLog / tuning API
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
| `make check` | check Python ≥ 3.11 (for `tomllib`), that `tuning.toml` parses, `pyserial`, `esptool`, `pio`, and (Linux) membership of the `dialout`/`uucp` group needed to open serial ports |
| `make detect` | list USB serial ports and mark the CH343 (`1a86:55d3`). If the cable is in the native port (`303a:*`), say so. Run `esptool chip_id` (should print `ESP32-S3 (QFN56) (revision v0.2)`) and query firmware `INFO` |
| `make sync-time` | set the ESP32 clock (section 8) |
| `make tune` | push `tuning.toml` to the ESP32 **and save it to flash**, so the car boots tuned without a laptop (and later under ROS 2) |
| `make clean` | `rm -rf firmware/.pio` |
| `make build` | `clean` → `pio run` |
| `make flash` | `clean` → `pio run -t upload` over the COM port |
| `make monitor` | `pio device monitor` (raw protocol console, with exception decoder) |
| `make test` | steering sweep (0 → +max → 0 → −max → 0) + failsafe test (arm, stop sending, expect `EVT FAILSAFE`). **Motor never commanded.** |
| `make test-throttle` | `test` + forward / neutral / reverse at 0.5 × `limit`, after typing `yes` |
| `make calibrate` | interactive servo calibration (below) |
| `make teleop` | curses keyboard driving: ←/→ heading ±5°, ↑/↓ speed ±0.1, space = speed 0, `a` = arm/disarm, `q` = quit |
| `make session` | interactive scratch session (`test/esp32_session.py`, 9.5) |

Every target that connects to the ESP32 first pushes `tuning.toml` into its RAM, so
the file on disk is what the car runs.

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
5. `save` writes the values to the ESP32's flash **and back into `tuning.toml`**. Only
   the `center_us`/`left_us`/`right_us`/`max_steer_deg`/`heading_gain` lines change, and
   comments are kept. Commit the updated file. Then `q`, and `make test` to see the
   sweep using the new limits.

### 9.4 Tuning file (`tuning.toml`)

One root file holds every adjustable number. You can tune the car without reading C++.

**Why TOML.** It allows comments next to each value, Python reads it with the standard
library (`tomllib`, Python ≥ 3.11), and the sections map naturally onto subsystems.
Future sensor sections are already sketched in it as comments.

**How values reach the car.** Nothing is compiled in:

```
tuning.toml ──(Link.apply_tuning on every connect)──▶ SET/CAL commands ──▶ ESP32 RAM
            ──(make tune)──────────────────────────▶ … + SAVE          ──▶ ESP32 flash (NVS)
make calibrate "save" ──▶ ESP32 flash + rewrites the steering lines in tuning.toml
```

So `tuning.toml` is the **source of truth**. Flash only holds a copy, so the car can
boot tuned without a laptop. Keep the file in git and every tuning change gets history
and diffs. Each session log also embeds the tuning that was active, so you can compare
runs later.

| Section | Key | Default | Firmware `SET` | Notes |
| --- | --- | --- | --- | --- |
| link | `send_hz` | 20 | (host only) | heartbeat/command rate |
| link | `telemetry_hz` | 10 | `TEL` | 0-50 |
| control | `control_hz` | 100 | `control_hz` | 20-500 |
| safety | `cmd_timeout_ms` | 300 | `timeout` | 50-2000; keep ≥ 3 × (1000 / `send_hz`) |
| steering | `servo_hz` | 50 | `servo_hz` | 50-330 (DS3225 datasheet) |
| steering | `center_us` / `left_us` / `right_us` | 1500 / 1800 / 1200 | `CAL …` | written by `make calibrate` |
| steering | `max_steer_deg` / `heading_gain` | 25.0 / 1.0 | `maxdeg` / `gain` | |
| throttle | `esc_hz` | 50 | `esc_hz` | 50-400; leave at 50 unless ESC confirmed |
| throttle | `esc_min_us` / `esc_neutral_us` / `esc_max_us` | 1000 / 1500 / 2000 | `esc_min` / `esc_neutral` / `esc_max` | 900-2100 hard window |
| throttle | `speed_limit` / `deadband_us` / `accel_per_s` | 0.30 / 0 / 2.0 | `limit` / `deadband` / `accel` | |
| throttle | `reverse_double_tap`, `brake_ms`, `gap_ms`, `brake_level` | true, 150, 150, 0.5 | `revmode`, … | ESC reverse sequence |
| logging | `enabled`, `dir`, `log_heartbeats` | true, "logs", true | (host only) | |
| battery / imu / distance / encoder | *commented placeholders* | — | — | e.g. `ebrake_m`, `rate_hz`, `ticks_per_rev` |

**Latency budget.** This shows which knobs actually reduce delay. Worst case from a
key press to the servo starting to move:

| Stage | Default | Knob | Faster setting |
| --- | --- | --- | --- |
| waiting for next host send | ≤ 50 ms | `send_hz` 20 | 50 Hz → ≤ 20 ms |
| USB + 115200 baud UART (~20 bytes) | ~2-3 ms | — | — |
| waiting for next control tick | ≤ 10 ms | `control_hz` 100 | 200 Hz → ≤ 5 ms |
| waiting for next PWM frame | ≤ 20 ms | `servo_hz` 50 | 200 Hz → ≤ 5 ms |
| servo travel (0.15 s / 60° @ 5 V) | ~60-100 ms | supply voltage | 6.8 V → 0.13 s / 60° |

The biggest electronic win is `send_hz` (this matters most during teleop; a ROS 2
controller will publish at its own rate). After that come `servo_hz` and `control_hz`
together. A faster servo frame only helps if `control_hz ≥ servo_hz`. The physical
servo speed is the floor. A faster servo frame also raises the servo's holding current
slightly, so watch temperature during long tests.

### 9.5 Test session scaffold (`test/esp32_session.py`)

This is a thin, hackable layer for tuning and quick hardware checks. It reuses the
`Link`, `SessionLog` and tuning code from `tools/rccar.py` (imported by path), so there
is one implementation of the protocol. The file only holds small helpers and a loop:

| Helper | Does |
| --- | --- |
| `connect(port=None)` | open COM port, handshake, push `tuning.toml`, start log |
| `send(cmd)` / `ask(cmd, prefix)` / `responses(seconds)` | raw send / request-reply / collect replies |
| `set_param(key, value)` / `config()` | live-tune one value (RAM only) / read all as a dict |
| `steer_raw(us)` / `drive(heading, speed, seconds)` | exact servo pulse / arm + stream + stop |
| `telemetry(n)` / `latency(n)` | next n `TEL` records as dicts / USB round-trip stats |
| `note(text)` | put a marker in the session log ("swapped to 6.8 V BEC") |
| `read_battery()` `read_imu()` `read_distance()` `read_encoder()` | **stubs** that send `SENS <name>` and print `ERR … not implemented` until firmware support lands |

The loop takes either a raw firmware command (`GET`, `SET servo_hz 100`) or a Python
helper call prefixed with `:` (`:drive(10, 0.2, 2)`, `:telemetry(5)`). A typo prints an
error and keeps the session alive. Raw `ARM` with no heartbeats times out after
`cmd_timeout_ms`. That is deliberate, so use `:drive()` to move. When a sensor gets
firmware support, its `read_*` stub starts returning a dict with no other change.

### 9.6 Session logs (`logs/`)

Every connection, whether from a `make` target or the session scaffold, writes one file:
`logs/YYYYMMDD-HHMMSS_<label>.jsonl` (label = `test`, `teleop`, `calibrate`, `session`, …).
The format is **JSON Lines**: one JSON object per line, appended as it happens. The file
is line-buffered, so a crash loses at most the line being written. It loads straight
into pandas:

```python
import pandas as pd
df = pd.read_json("logs/20261021-153012_teleop.jsonl", lines=True)
tel = pd.json_normalize(df[df.kind == "TEL"]["data"])   # t, unix, hdg, spd, steer_us, thr_us, armed
cmds = df[(df.dir == "tx") & (df.kind == "D")]          # heartbeats / speed changes
```

| Field | Meaning |
| --- | --- |
| `t` | laptop wall clock (unix s); lines up with ESP32 `unix=` after `make sync-time` |
| `mono` | laptop monotonic clock (for precise intervals; unaffected by clock changes) |
| `dir` | `tx` (laptop → ESP32), `rx` (ESP32 → laptop), empty for local records |
| `kind` | first token: `D`, `SET`, `TEL`, `CFG`, `EVT`, `ERR`, … plus `session` and `NOTE` |
| `line` | the raw protocol line |
| `data` | parsed `key=value` pairs for `TEL`, `CFG`, `INFO`, `SENS` |

The first record (`kind = "session"`) captures the context: label, port, command line,
git commit, and the **full tuning snapshot**. Any run can be traced back to the exact
code and parameters. Today the logs mostly hold heartbeats, speed/heading changes,
config echoes and failsafe events. Future `SENS`/telemetry fields (IMU, LIDAR, encoder)
are parsed into `data` by the same code, with no logger changes. At 20 Hz heartbeats
plus 10 Hz telemetry, a log grows by roughly 4 MB per hour. Set
`log_heartbeats = false` to record only commands that change heading or speed. `*.jsonl`
is gitignored and `logs/.gitkeep` keeps the folder. Use `git add -f` to keep a notable
run.

### 9.7 Claude Code context files

- **`CLAUDE.md`** (root): a tiny, YAML-style project map (stack, goals, layout, commands,
  invariants) meant for machines, not people. It loads in every session, so it stays a
  few hundred tokens. People read the README.
- **`.claude/rules/firmware.md`**: firmware implementation details (build flags,
  architecture and safety invariants, pin rules, protocol, NVS layout, how to add a
  sensor). Its front-matter `paths: ["firmware/**"]` means Claude Code loads it only when
  working on files under `firmware/`. Host-side or ML work doesn't pay its context cost.
- Both repeat the cross-file invariants: pins ↔ README, protocol ↔ CLI ↔ docs, and
  tuning keys ↔ `FIRMWARE_KEYS` ↔ `setKey()`. Changes made with Claude then stay
  consistent.

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
- **Firmware**: `main.cpp` + `pins.h` (Appendix A, including the runtime-tunable
  frame rates, ESC endpoints and `SENS` stub) were compiled to an object file with
  Espressif's `xtensa-esp32s3-elf-gcc` 8.4.0 (esp-2021r2-patch5) against the Arduino-ESP32
  2.0.17 core and ESP32-S3 SDK headers. Those are the versions `espressif32@6.9.0` uses,
  with the `qio_opi` memory config. Result: no errors or warnings with `-Wall -Wextra`.
  A full `pio run` (link + `firmware.bin`) **could not** run here, because this cloud
  environment's network policy blocks the PlatformIO package registry
  (`api.registry.platformio.org`, re-checked when this revision was written: still
  rejected). The first step of the implementation PR is `make build` on a laptop.
- **CLI**: `rccar.py` subcommands `ping`, `sync-time`, `tune`, `test`, `calibrate`,
  `drive`, `check-env`, `find-port` ran against a protocol mock on a pseudo-terminal. The
  mock failsafe fired as expected. `tuning.toml` was applied on connect. Calibrate's
  `save` rewrote only the intended lines of `tuning.toml` and kept the comments.
- **Session scaffold**: raw commands, `:latency()`, `:telemetry()`, `:drive()`, the
  `:read_imu()` stub (`ERR … not implemented`) and a typo all behaved as described.
- **Logs**: JSONL files were produced for each label. They parse with `json`, start with
  a `session` record holding the tuning snapshot, and contain tx/rx/NOTE records.
- **Makefile**: `make help` (including `tune`, `session`) and `make -n` dry runs of
  `flash`/`clean` checked.
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

These are the exact files that were checked in section 11. Like everything else here,
they are drafts. They will be added by the
implementation PR. Nothing under `firmware/`, `tools/` or the repo root is changed by
*this* PR.

### `CLAUDE.md`

```yaml
project: autonomous_rc_car_platform
chassis: tamiya_tt02_1:10
goals: [sensor_fusion, object_detection, semantic_slam, racing_trajectory_optimization]
phase: actuator_bringup  # laptop -usb-> esp32 -pwm-> servo+esc
next: ros2_on_rpi5 + micro_ros_on_esp32
stack:
  mcu: {chip: esp32-s3-n16r8, build: platformio, platform: espressif32@6.9.0, framework: arduino-esp32-2.0.17, lang: c++gnu11}
  host: {lang: python>=3.11, deps: tools/requirements.txt, venv: .venv}
  future: [ros2, micro_ros, raspberry_pi_5]
hw: {steer: ds3225mg@gpio4, throttle: thw1060_esc@gpio5, battery: 2s_lipo}
layout:
  firmware/: esp32 firmware; rules in .claude/rules/firmware.md
  tools/rccar.py: host cli + Link/SessionLog/tuning api
  test/esp32_session.py: interactive tuning/sensor scaffold
  tuning.toml: runtime tuning source of truth, pushed on connect
  logs/: session jsonl, gitignored
  docs/: proposals, hardware notes
cmd: make help
invariants:
  - esp32 role: actuator pwm + host comms only; powers nothing
  - sensors on separate power rail, common ground, <=3.3v signals
  - pins: left header P1 only; pins.h == README table
  - protocol change => main.cpp + rccar.py + docs
  - tuning key change => tuning.toml + rccar.FIRMWARE_KEYS + main.cpp setKey
```

### `.claude/rules/firmware.md`

````markdown
---
paths:
  - "firmware/**"
---

# Firmware implementation rules (ESP32-S3-N16R8)

## Build
- PlatformIO env `esp32s3`: `platform = espressif32@6.9.0` (Arduino-ESP32 2.0.17, ESP-IDF 4.4.7), `board = esp32-s3-devkitc-1`.
- Memory: `board_build.arduino.memory_type = qio_opi` (quad flash, octal PSRAM). Wrong type = boot loop.
- Compiler is `-std=gnu++11`: no `std::clamp`, no C++14 aggregate init with default member initializers, no `if constexpr`.
- `ARDUINO_USB_CDC_ON_BOOT=0`: `Serial` = UART0 = CH343 "COM" port. Do not use the native USB port for comms.
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

## Config persistence
- `Config` struct is stored as one NVS blob (`Preferences`, namespace `rccar`, key `cfg`).
- Any change to the `Config` layout MUST bump `CFG_MAGIC` (old blobs are then ignored and defaults load).

## Adding a sensor (future)
- Add its pins to `pins.h` (already reserved), its tuning keys to `tuning.toml` (commented placeholders exist).
- Read sensors in their own task or in `controlTask()` if needed for safety (e.g. emergency brake from distance); report via `SENS <name> k=v ...` and telemetry fields.
- Emergency braking must live in firmware so it works without the host.
````

### `tuning.toml`

```toml
# tuning.toml - the one place to tune the car without touching firmware.
#
#   make tune       push this file to the ESP32 and save it to its flash
#   (every other make target also pushes it to ESP32 RAM on connect)
#   make calibrate  writes steering values back into this file on `save`
#
# Keys must stay unique across sections (calibrate edits lines by key name).
# Hard limits from the datasheets are enforced in firmware; values outside
# them are clamped or rejected with an error.

[link]                      # laptop <-> ESP32
send_hz = 20                # command/heartbeat rate. Worst-case command delay = 1000/send_hz ms
telemetry_hz = 10           # TEL lines from the ESP32, 0..50

[control]
control_hz = 100            # firmware control loop, 20..500. Keep >= servo_hz to use faster frames

[safety]
cmd_timeout_ms = 300        # no command for this long -> disarm. Must be > 1000/send_hz (x3 is safe)

[steering]                  # DS3225 servo on GPIO4
servo_hz = 50               # PWM frame rate, 50..330. 50 Hz = 20 ms frame; 200 Hz = 5 ms (less latency)
center_us = 1500            # pulse for straight wheels      (make calibrate)
left_us = 1800              # pulse at full LEFT lock        (make calibrate)
right_us = 1200             # pulse at full RIGHT lock       (make calibrate)
max_steer_deg = 25.0        # road-wheel angle at full lock  (measure)
heading_gain = 1.0          # steering deg per heading deg (+ = left)

[throttle]                  # THW-1060 ESC on GPIO5
esc_hz = 50                 # 50 is the RC standard. Only raise after confirming the ESC accepts it
esc_min_us = 1000           # full reverse/brake
esc_neutral_us = 1500       # must match what the ESC learned at power-on
esc_max_us = 2000           # full forward
speed_limit = 0.30          # speed 1.0 -> 30 % throttle (bench safety)
deadband_us = 0             # skip the ESC dead zone so small speeds move the car
accel_per_s = 2.0           # max change of speed command per second (slew limit)
reverse_double_tap = true   # ESC needs brake -> neutral -> reverse (Fwd/Br/Rev mode)
brake_ms = 150
gap_ms = 150
brake_level = 0.5

[logging]
enabled = true
dir = "logs"
log_heartbeats = true       # false = log D commands only when heading/speed change

# ---------------------------------------------------------------------------
# Future sensors - placeholders, not read by anything yet. Uncomment and wire
# into firmware when the device is added (see docs/esp32-firmware-proposal.md).
# ---------------------------------------------------------------------------
# [battery]
# low_active_level = 0      # GPIO6 level that means "battery low"
# debounce_ms = 500
#
# [imu]
# i2c_hz = 400000
# rate_hz = 200
# accel_range_g = 4
# gyro_range_dps = 500
#
# [distance]
# sensor = "lidar_lite_v4"  # or "ultrasonic"
# rate_hz = 50
# ebrake_m = 0.5            # firmware emergency stop distance
# ebrake_min_speed = 0.05   # ignore when nearly stopped
#
# [encoder]
# ticks_per_rev = 0
# wheel_diameter_m = 0.065
# gear_ratio = 0.0
```

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
constexpr uint16_t SERVO_ABS_MIN_US = 500;
constexpr uint16_t SERVO_ABS_MAX_US = 2500;
constexpr uint16_t SERVO_MAX_HZ     = 330;
// THW-1060: standard RC throttle. Hard window; tuned endpoints live in Config.
constexpr uint16_t ESC_ABS_MIN_US   = 900;
constexpr uint16_t ESC_ABS_MAX_US   = 2100;
constexpr uint16_t ESC_MAX_HZ       = 400;   // above 50 Hz: verify ESC first
constexpr uint16_t BOOT_NEUTRAL_US  = 1500;

constexpr uint32_t PWM_RES_BITS = 14;   // at 50 Hz: 20000 us / 16384 = 1.22 us per tick

constexpr ledc_mode_t      LEDC_MODE      = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t     STEER_TIMER    = LEDC_TIMER_0;
constexpr ledc_timer_t     THROTTLE_TIMER = LEDC_TIMER_1;
constexpr ledc_channel_t   STEER_CH       = LEDC_CHANNEL_0;
constexpr ledc_channel_t   THROTTLE_CH    = LEDC_CHANNEL_1;

// ---------------------------------------------------------------------------
// Tunable config, persisted in NVS flash
// ---------------------------------------------------------------------------
constexpr uint32_t CFG_MAGIC = 0xC0FFEE02;  // bump when Config layout changes

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
  uint16_t brakeMs       = 150;    // double-tap reverse: brake pulse length
  uint16_t gapMs         = 150;    // double-tap reverse: neutral gap length
  float    brakeLevel    = 0.5f;   // double-tap reverse: brake strength
  uint16_t escMinUs      = 1000;   // full reverse
  uint16_t escNeutralUs  = 1500;
  uint16_t escMaxUs      = 2000;   // full forward
  uint16_t servoHz       = 50;     // DS3225 accepts 50..330
  uint16_t escHz         = 50;     // RC standard; raise only if the ESC accepts it
  uint16_t controlHz     = 100;    // control loop rate
};

struct Command {
  float    headingDeg = 0.0f;  // + = left (ROS REP-103), relative to car's nose
  float    speed      = 0.0f;  // -1..1 normalized
  uint64_t stampMs    = 0;
};

struct Output {
  uint16_t steerUs    = 1500;
  uint16_t throttleUs = BOOT_NEUTRAL_US;
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
  if (fabsf(u) < 0.001f) return c.escNeutralUs;
  float span = (u > 0 ? c.escMaxUs - c.escNeutralUs : c.escNeutralUs - c.escMinUs) - c.deadbandUs;
  float us = c.escNeutralUs + (u > 0 ? 1 : -1) * (c.deadbandUs + fabsf(u) * span);
  return clampv<uint16_t>((uint16_t)lroundf(us), c.escMinUs, c.escMaxUs);
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
  uint16_t servoHz = 50, escHz = 50;  // what the LEDC timers currently run at
  uint16_t controlHz = 100;

  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(1000 / controlHz));
    esp_task_wdt_reset();

    Config cfg; Command cmd; bool armed; int raw;
    portENTER_CRITICAL(&g_mux);
    cfg = g_cfg; cmd = g_cmd; armed = g_armed; raw = g_rawSteerUs;
    portEXIT_CRITICAL(&g_mux);

    // Runtime frame-rate changes (tuning.toml -> SET servo_hz / esc_hz)
    if (cfg.servoHz != servoHz) { ledc_set_freq(LEDC_MODE, STEER_TIMER, cfg.servoHz); servoHz = cfg.servoHz; }
    if (cfg.escHz != escHz)     { ledc_set_freq(LEDC_MODE, THROTTLE_TIMER, cfg.escHz); escHz = cfg.escHz; }
    controlHz = cfg.controlHz;

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
    const float step = cfg.accelPerSec / controlHz;
    speedOut = armed ? speedOut + clampv(target - speedOut, -step, step) : 0.0f;
    float u = speedOut * cfg.speedLimit;

    // Many brushed ESCs (Fwd/Brake/Rev mode) treat the first reverse pulse as
    // BRAKE and only reverse after a return to neutral. Do that sequence here.
    if (cfg.doubleTapRev) {
      switch (rs) {
        case RevState::Forward:
          if (u < 0) { rs = RevState::Brake; rsSince = now; u = -cfg.brakeLevel; }
          break;
        case RevState::Brake:
          if (u >= 0) rs = RevState::Forward;
          else if (now - rsSince >= cfg.brakeMs) { rs = RevState::Gap; rsSince = now; u = 0; }
          else u = -cfg.brakeLevel;
          break;
        case RevState::Gap:
          if (u >= 0) rs = RevState::Forward;
          else if (now - rsSince >= cfg.gapMs) rs = RevState::Reverse;
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

    pwmWriteUs(STEER_CH, servoHz, steerUs);
    pwmWriteUs(THROTTLE_CH, escHz, thrUs);

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
                "deadband=%u accel=%.2f timeout=%u revmode=%u brake_ms=%u gap_ms=%u "
                "brake_level=%.2f esc_min=%u esc_neutral=%u esc_max=%u servo_hz=%u "
                "esc_hz=%u control_hz=%u\n",
                c.steerCenterUs, c.steerLeftUs, c.steerRightUs, c.maxSteerDeg,
                c.headingGain, c.speedLimit, c.deadbandUs, c.accelPerSec,
                c.timeoutMs, c.doubleTapRev, c.brakeMs, c.gapMs, c.brakeLevel,
                c.escMinUs, c.escNeutralUs, c.escMaxUs, c.servoHz, c.escHz, c.controlHz);
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
  else if (!strcasecmp(key, "brake_ms")) c.brakeMs     = (uint16_t)clampv(v, 0.0f, 1000.0f);
  else if (!strcasecmp(key, "gap_ms"))   c.gapMs       = (uint16_t)clampv(v, 0.0f, 1000.0f);
  else if (!strcasecmp(key, "brake_level")) c.brakeLevel = clampv(v, 0.0f, 1.0f);
  else if (!strcasecmp(key, "esc_min"))     c.escMinUs     = (uint16_t)clampv(v, (float)ESC_ABS_MIN_US, 1500.0f);
  else if (!strcasecmp(key, "esc_neutral")) c.escNeutralUs = (uint16_t)clampv(v, 1300.0f, 1700.0f);
  else if (!strcasecmp(key, "esc_max"))     c.escMaxUs     = (uint16_t)clampv(v, 1500.0f, (float)ESC_ABS_MAX_US);
  else if (!strcasecmp(key, "servo_hz"))    c.servoHz   = (uint16_t)clampv(v, 50.0f, (float)SERVO_MAX_HZ);
  else if (!strcasecmp(key, "esc_hz"))      c.escHz     = (uint16_t)clampv(v, 50.0f, (float)ESC_MAX_HZ);
  else if (!strcasecmp(key, "control_hz"))  c.controlHz = (uint16_t)clampv(v, 20.0f, 500.0f);
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
    Config c;
    portENTER_CRITICAL(&g_mux); c = g_cfg; portEXIT_CRITICAL(&g_mux);
    if (!setKey(c, a1, strtof(a2, nullptr))) { Serial.printf("ERR unknown key %s\n", a1); return; }
    if (!(c.escMinUs < c.escNeutralUs && c.escNeutralUs < c.escMaxUs)) {
      Serial.println("ERR need esc_min < esc_neutral < esc_max"); return;
    }
    portENTER_CRITICAL(&g_mux); g_cfg = c; portEXIT_CRITICAL(&g_mux);
    printConfig();
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
  } else if (!strcasecmp(tok, "SENS")) {  // reserved: future sensor reads
    Serial.printf("ERR SENS %s not implemented\n", a1 ? a1 : "?");
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
  // Always boot at 50 Hz / 1500 us; saved frame rates are applied by the
  // control task once it starts.
  pwmInit(PIN_STEER_PWM, STEER_TIMER, STEER_CH, 50, BOOT_NEUTRAL_US);
  pwmInit(PIN_THROTTLE_PWM, THROTTLE_TIMER, THROTTLE_CH, 50, BOOT_NEUTRAL_US);

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
USB-C port (CH343 USB-UART bridge). Tuning comes from tuning.toml at the
repo root; every session is logged to logs/*.jsonl. Every subcommand is
wrapped by a Makefile target; run `python tools/rccar.py -h` for the list.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import re
import shutil
import subprocess
import sys
import time
import tomllib
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TUNING_FILE = ROOT / "tuning.toml"
BAUD = 115200
CH343_VID, CH343_PID = 0x1A86, 0x55D3
ESPRESSIF_VID = 0x303A            # native USB port (GPIO19/20), not used for comms

# tuning.toml (section, key) -> firmware command prefix. Order matters for ESC
# endpoints: the firmware rejects any SET that breaks esc_min < neutral < max.
FIRMWARE_KEYS = [
    (("steering", "center_us"), "CAL CENTER"),
    (("steering", "left_us"), "CAL LEFT"),
    (("steering", "right_us"), "CAL RIGHT"),
    (("steering", "max_steer_deg"), "SET maxdeg"),
    (("steering", "heading_gain"), "SET gain"),
    (("steering", "servo_hz"), "SET servo_hz"),
    (("throttle", "esc_min_us"), "SET esc_min"),
    (("throttle", "esc_max_us"), "SET esc_max"),
    (("throttle", "esc_neutral_us"), "SET esc_neutral"),
    (("throttle", "esc_hz"), "SET esc_hz"),
    (("throttle", "speed_limit"), "SET limit"),
    (("throttle", "deadband_us"), "SET deadband"),
    (("throttle", "accel_per_s"), "SET accel"),
    (("throttle", "reverse_double_tap"), "SET revmode"),
    (("throttle", "brake_ms"), "SET brake_ms"),
    (("throttle", "gap_ms"), "SET gap_ms"),
    (("throttle", "brake_level"), "SET brake_level"),
    (("safety", "cmd_timeout_ms"), "SET timeout"),
    (("control", "control_hz"), "SET control_hz"),
]


def load_tuning(path: Path = TUNING_FILE) -> dict:
    with open(path, "rb") as f:
        return tomllib.load(f)


def update_tuning_file(values: dict[str, float | int], path: Path = TUNING_FILE) -> None:
    """Rewrite `key = value` lines in place, keeping comments and layout.

    Keys in tuning.toml are unique across sections, so a per-key regex is safe.
    """
    text = path.read_text()
    for key, val in values.items():
        text, n = re.subn(rf"(?m)^(\s*{key}\s*=\s*)[^#\n]*?(\s*(#.*)?)$",
                          rf"\g<1>{val}\g<2>", text, count=1)
        if n != 1:
            raise KeyError(f"{key} not found in {path.name}")
    path.write_text(text)


def parse_kv(line: str) -> dict[str, float | str]:
    """'TEL t=1 hdg=2.0 ...' -> {'t': 1.0, 'hdg': 2.0, ...}"""
    out: dict[str, float | str] = {}
    for tok in line.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            try:
                out[k] = float(v)
            except ValueError:
                out[k] = v
    return out


# ---------------------------------------------------------------------------
# Session log: one JSON object per line, load with pandas.read_json(lines=True)
# ---------------------------------------------------------------------------
class SessionLog:
    def __init__(self, label: str, tuning: dict, port: str):
        cfg = tuning.get("logging", {})
        self.heartbeats = cfg.get("log_heartbeats", True)
        self._last_drive: str | None = None
        self.file = None
        if not cfg.get("enabled", True):
            return
        log_dir = ROOT / cfg.get("dir", "logs")
        log_dir.mkdir(exist_ok=True)
        self.path = log_dir / f"{datetime.now():%Y%m%d-%H%M%S}_{label}.jsonl"
        self.file = open(self.path, "w", buffering=1)  # line-buffered: survives crashes
        git = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True).stdout.strip()
        self.write("session", "", {"label": label, "port": port, "argv": sys.argv,
                                   "git": git, "tuning": tuning})

    def write(self, kind: str, line: str, data: dict | None = None, direction: str = "") -> None:
        if not self.file:
            return
        rec = {"t": time.time(), "mono": time.monotonic(), "dir": direction,
               "kind": kind, "line": line}
        if data:
            rec["data"] = data
        self.file.write(json.dumps(rec) + "\n")

    def tx(self, line: str) -> None:
        kind = line.split()[0].upper() if line.strip() else ""
        if kind == "D":
            if not self.heartbeats and line == self._last_drive:
                return                       # unchanged heartbeat
            self._last_drive = line
        self.write(kind, line, direction="tx")

    def rx(self, line: str) -> None:
        kind = line.split()[0] if line else ""
        data = parse_kv(line) if kind in ("TEL", "CFG", "INFO", "SENS") else None
        self.write(kind, line, data, direction="rx")

    def note(self, text: str) -> None:
        self.write("NOTE", text)

    def close(self) -> None:
        if self.file:
            self.file.close()
            print(f"log: {self.path.relative_to(ROOT)}")


# ---------------------------------------------------------------------------
# Serial link
# ---------------------------------------------------------------------------
class Link:
    def __init__(self, port: str, label: str = "session", apply: bool = True):
        import serial

        self.tuning = load_tuning()
        self.send_hz = self.tuning["link"]["send_hz"]
        self.log = SessionLog(label, self.tuning, port)
        self.ser = serial.Serial()
        self.ser.port, self.ser.baudrate, self.ser.timeout = port, BAUD, 0.05
        self.ser.dtr = False      # keep EN/IO0 released so opening doesn't reset
        self.ser.rts = False
        self.ser.open()
        self.handshake()
        if apply:
            self.apply_tuning()

    def send(self, line: str) -> None:
        self.log.tx(line)
        self.ser.write((line + "\n").encode())

    def readline(self) -> str | None:
        raw = self.ser.readline()
        if not raw:
            return None
        line = raw.decode(errors="replace").strip()
        self.log.rx(line)
        return line

    def drain(self) -> list[str]:
        """Read (and log) every complete line already received, without waiting."""
        lines = []
        while self.ser.in_waiting:
            line = self.readline()
            if line is not None:
                lines.append(line)
        return lines

    def expect(self, prefix: str | tuple[str, ...], timeout: float = 1.0) -> str:
        """Read lines until one starts with prefix; print unexpected errors."""
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

    def request(self, line: str, prefix: str | tuple[str, ...], timeout: float = 1.0) -> str:
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

    def apply_tuning(self) -> str:
        """Push tuning.toml into firmware RAM. Returns the resulting CFG line."""
        for (section, key), cmd in FIRMWARE_KEYS:
            val = self.tuning[section][key]
            val = int(val) if isinstance(val, bool) else val
            reply = self.request(f"{cmd} {val}", ("CFG", "ERR"))
            if reply.startswith("ERR"):
                raise SystemExit(f"tuning.toml [{section}] {key} = {val}: {reply}")
        self.request(f"TEL {self.tuning['link']['telemetry_hz']}", "OK TEL")
        return self.request("GET", "CFG")

    def ping(self, seq: int) -> tuple[float, int]:
        t0 = time.monotonic()
        reply = self.request(f"PING {seq}", f"PONG {seq}")
        return (time.monotonic() - t0) * 1000.0, int(reply.split()[2])

    def drive_for(self, heading: float, speed: float, seconds: float) -> None:
        """Stream D commands so the firmware failsafe stays satisfied."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.send(f"D {heading:.1f} {speed:.3f}")
            self.drain()                       # log telemetry, keep buffer empty
            time.sleep(1.0 / self.send_hz)

    def close(self) -> None:
        try:
            self.send("STOP")
            self.send("RAW OFF")
            self.send("TEL 0")
        finally:
            self.ser.close()
            self.log.close()


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


def open_link(args, label: str, apply: bool = True) -> Link:
    port = args.port or find_port()
    if not port:
        raise SystemExit("no ESP32 COM port found - run `make detect`")
    return Link(port, label, apply)


# ---------------------------------------------------------------------------
# Subcommands
# ---------------------------------------------------------------------------
def cmd_check_env(args) -> int:
    ok = sys.version_info >= (3, 11)       # tomllib
    print(f"python   {sys.version.split()[0]}", "OK" if ok else "NEED >= 3.11")
    for mod, pkg in (("serial", "pyserial"), ("esptool", "esptool")):
        found = importlib.util.find_spec(mod) is not None
        print(f"{pkg:<8} {'OK' if found else 'MISSING (make setup)'}")
        ok &= found
    venv_pio = Path(sys.executable).parent / "pio"
    pio = str(venv_pio) if venv_pio.exists() else shutil.which("pio")
    print(f"pio      {pio or 'MISSING (make setup)'}")
    ok &= pio is not None
    try:
        load_tuning()
        print("tuning.toml OK")
    except (OSError, tomllib.TOMLDecodeError) as e:
        print(f"tuning.toml BROKEN: {e}")
        ok = False
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
        link = Link(port, "detect", apply=False)
        print("\nfirmware:", link.request("INFO", "INFO"))
        link.close()
    except SystemExit as e:
        print(f"\nfirmware: {e}")
    return 0


def cmd_ping(args) -> int:
    link = open_link(args, "ping", apply=False)
    rtts = [link.ping(i)[0] for i in range(1, 11)]
    print(f"10 pings: min {min(rtts):.1f} ms  avg {sum(rtts) / len(rtts):.1f} ms  max {max(rtts):.1f} ms")
    link.close()
    return 0


def cmd_sync_time(args) -> int:
    """Cristian's algorithm: best of N round trips, assume symmetric latency."""
    link = open_link(args, "sync-time", apply=False)
    best = min(link.ping(i)[0] for i in range(1, 11))
    unix_ms = int(time.time() * 1000 + best / 2)
    link.request(f"TIME {unix_ms}", "OK TIME")
    link.request("TEL 10", "OK TEL")
    esp_unix = parse_kv(link.expect("TEL ", 1.0))["unix"]
    print(f"synced: rtt {best:.1f} ms, esp32 - laptop = {esp_unix - time.time() * 1000:+.0f} ms")
    link.close()
    return 0


def cmd_tune(args) -> int:
    """Push tuning.toml to the ESP32 and persist it, so it boots tuned."""
    link = open_link(args, "tune")
    print(link.request("GET", "CFG"))
    print(link.request("SAVE", ("OK SAVE", "ERR"), 2.0))
    link.close()
    return 0


def cmd_test(args) -> int:
    link = open_link(args, "test-throttle" if args.throttle else "test")
    print(link.request("INFO", "INFO"))
    cfg = link.request("GET", "CFG")
    print(cfg)
    maxdeg = parse_kv(cfg)["maxdeg"]

    print("\n[1/3] steering sweep (throttle disarmed)")
    link.log.note("steering sweep")
    for hdg in (0, maxdeg, 0, -maxdeg, 0):
        print(f"  heading {hdg:+.0f} deg")
        link.drive_for(hdg, 0.0, 1.0)

    print("\n[2/3] failsafe: stop sending while armed")
    link.log.note("failsafe test")
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
    link.log.note("throttle test")
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
  show           print config
  save           write values to tuning.toml AND the ESP32 flash
  q              quit (unsaved values are discarded on next connect)"""

CAL_TO_TOML = {"center": "center_us", "left": "left_us", "right": "right_us",
               "maxdeg": "max_steer_deg", "gain": "heading_gain"}


def cmd_calibrate(args) -> int:
    link = open_link(args, "calibrate")
    link.send("STOP")
    us = 1500
    changed: dict[str, float | int] = {}
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
                changed[CAL_TO_TOML[cmd]] = us
                continue
            elif cmd.split()[0] in ("maxdeg", "gain"):
                name, val = cmd.split()
                print(link.request(f"SET {name} {float(val)}", "CFG"))
                changed[CAL_TO_TOML[name]] = float(val)
                continue
            elif cmd == "show":
                print(link.request("GET", "CFG"))
                continue
            elif cmd == "save":
                print(link.request("SAVE", ("OK SAVE", "ERR"), 2.0))
                if changed:
                    update_tuning_file(changed)
                    print(f"  tuning.toml updated: {changed}")
                continue
            else:
                print(CAL_HELP)
                continue
        except (ValueError, IndexError, KeyError, TimeoutError) as e:
            print(f"  ? {e}")
            continue
        us = max(500, min(2500, us))
        link.request(f"RAW {us}", "OK RAW")
    link.close()
    return 0


def cmd_teleop(args) -> int:
    import curses

    link = open_link(args, "teleop")

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
            for line in link.drain():
                if line.startswith("TEL"):
                    tel = line
                elif line.startswith("EVT FAILSAFE"):
                    armed = False
            scr.erase()
            scr.addstr(0, 0, "arrows: heading/speed  space: speed 0  c: center  a: arm/disarm  q: quit")
            scr.addstr(2, 0, f"heading {heading:+6.1f} deg   speed {speed:+.2f}   {'ARMED' if armed else 'disarmed'}")
            scr.addstr(4, 0, tel[:curses.COLS - 1])
            scr.refresh()
            time.sleep(1.0 / link.send_hz)

    try:
        curses.wrapper(ui)
    finally:
        link.close()
    return 0


def cmd_drive(args) -> int:
    """One-shot scripted command, e.g. for tuning runs or other scripts."""
    link = open_link(args, "drive")
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
    sub.add_parser("tune").set_defaults(fn=cmd_tune)
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

### `test/esp32_session.py`

```python
#!/usr/bin/env python3
"""Interactive scratch session with the ESP32 for tuning and quick tests.

    make session            # or: .venv/bin/python test/esp32_session.py [--port X]

At the prompt:
    PING / GET / SET limit 0.4 / RAW 1600 ...   raw firmware commands (replies printed)
    :drive(10, 0.2, 2)                          call a helper below (Python, prefix ':')
    :set_param("servo_hz", 100)
    :read_imu()
    q                                           quit (disarms, logs to logs/)

Typed raw commands do not stream heartbeats, so `ARM` alone times out after
cmd_timeout_ms by design. Use :drive() to move the car.

Add your own helpers here; the transport, tuning and logging live in
tools/rccar.py so this file stays a thin, hackable layer.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from rccar import Link, find_port, parse_kv  # noqa: E402

link: Link | None = None


# --- basics -----------------------------------------------------------------
def connect(port: str | None = None) -> Link:
    """Open the COM port, handshake, push tuning.toml, start a log file."""
    global link
    port = port or find_port()
    if not port:
        raise SystemExit("no ESP32 COM port found - run `make detect`")
    link = Link(port, label="session")
    return link


def send(cmd: str) -> None:
    link.send(cmd)


def ask(cmd: str, prefix: str | tuple[str, ...] = ("OK", "ERR", "CFG", "PONG", "INFO", "SENS"),
        timeout: float = 1.0) -> str:
    """Send one command and return the first matching reply line."""
    return link.request(cmd, prefix, timeout)


def responses(seconds: float = 0.3) -> list[str]:
    """Collect everything the ESP32 says for a short window."""
    end, lines = time.monotonic() + seconds, []
    while time.monotonic() < end:
        lines += link.drain()
        time.sleep(0.01)
    return lines


def note(text: str) -> None:
    """Drop a marker into the session log (e.g. 'changed tyres')."""
    link.log.note(text)


# --- actuators ----------------------------------------------------------------
def set_param(key: str, value: float) -> dict:
    """Live-tune one firmware value (RAM only; edit tuning.toml to keep it)."""
    return parse_kv(ask(f"SET {key} {value}", ("CFG", "ERR")))


def config() -> dict:
    return parse_kv(ask("GET", "CFG"))


def steer_raw(us: int | None) -> str:
    """Hold the servo at an exact pulse; None returns to normal steering."""
    return ask(f"RAW {us if us else 'OFF'}", "OK RAW")


def drive(heading: float, speed: float, seconds: float = 1.0) -> None:
    """Arm (if moving), stream commands for `seconds`, then stop."""
    if speed:
        ask("ARM", "OK ARM")
    note(f"drive heading={heading} speed={speed} seconds={seconds}")
    link.drive_for(heading, speed, seconds)
    link.drive_for(heading, 0.0, 0.2)
    ask("DISARM", "OK DISARM")


def telemetry(n: int = 10, timeout: float = 3.0) -> list[dict]:
    """Return the next n TEL records as dicts."""
    out, end = [], time.monotonic() + timeout
    while len(out) < n and time.monotonic() < end:
        out += [parse_kv(l) for l in link.drain() if l.startswith("TEL")]
        time.sleep(0.01)
    return out[:n]


def latency(n: int = 20) -> dict:
    """USB round-trip stats in ms."""
    rtts = sorted(link.ping(i)[0] for i in range(1, n + 1))
    return {"min": rtts[0], "median": rtts[n // 2], "max": rtts[-1]}


# --- sensors (stubs until firmware implements SENS <name>) ---------------------
def read_sensor(name: str) -> dict | None:
    reply = ask(f"SENS {name}", ("SENS", "ERR"))
    if reply.startswith("ERR"):
        print(f"  {reply}")
        return None
    return parse_kv(reply)


def read_battery() -> dict | None:
    return read_sensor("BATT")


def read_imu() -> dict | None:
    return read_sensor("IMU")


def read_distance() -> dict | None:
    return read_sensor("DIST")


def read_encoder() -> dict | None:
    return read_sensor("ENC")


# --- loop -----------------------------------------------------------------------
HELPERS = {name: obj for name, obj in globals().items()
           if callable(obj) and not name.startswith("_") and name not in ("Link", "main")}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    args = ap.parse_args()
    connect(args.port)
    print(ask("INFO", "INFO"))
    print("raw command, :python_helper(...), or q to quit")
    try:
        while True:
            try:
                line = input("esp32> ").strip()
            except EOFError:
                break
            if line in ("q", "quit", "exit"):
                break
            if not line:
                continue
            if line.startswith(":"):
                try:
                    result = eval(line[1:], HELPERS)
                    if result is not None:
                        print(result)
                except Exception as e:  # keep the session alive on typos
                    print(f"  {type(e).__name__}: {e}")
                continue
            send(line)
            for reply in responses():
                if not reply.startswith("TEL"):
                    print(reply)
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
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
.PHONY: help setup check detect sync-time tune clean build flash monitor \
        test test-throttle calibrate teleop session

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

tune: $(PY)      ## push tuning.toml to the ESP32 and save it to flash
	$(CLI) $(PORT_ARG) tune

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

session: $(PY)   ## interactive tuning/test session (test/esp32_session.py)
	$(PY) test/esp32_session.py $(PORT_ARG)
```

### `logs/.gitkeep`

Empty file, so git tracks the folder.

### `.gitignore` (additions)

```text
firmware/.pio/
__pycache__/
logs/*.jsonl
```
