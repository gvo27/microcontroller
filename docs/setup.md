# Setup: zero to tuning

Assumptions are marked **Assume:**. If one is false, fix it before continuing.

## 1. Laptop

**Assume:** Linux or macOS, `git`, `make`, **Python ≥ 3.11** on `PATH` as `python3`, internet access to pypi.org and registry.platformio.org.

```sh
git clone https://github.com/gvo27/microcontroller && cd microcontroller
make setup      # creates .venv/ in the repo root, installs platformio, pyserial, esptool
make check      # every line must say OK
```

**Assume:** all Python runs from `.venv/` (gitignored). The Makefile uses it automatically, so there is no need to activate it.
Linux only: if `make check` flags serial permission, run `sudo usermod -aG dialout $USER`, then log out and back in.

## 2. Wiring (power off)

**Assume:** ESC switch **off**, battery connected to the ESC, car on a stand with the wheels off the ground.

1. Servo signal → P1-4 (GPIO4). ESC throttle-lead signal → P1-5 (GPIO5).
2. ESC black and servo black → P1-22 (G), the common ground.
3. ESC red (6 V) → servo red only, through a Y-harness. **Nothing red touches the ESP32.**

See the pin map in the [README](../README.md#pin-map-left-header-p1-only).

## 3. Connect

**Assume:** a USB-C cable that carries **data**, plugged into the board's **right** USB-C port, labelled **COM**. The left port, **USB**, is reserved for future regulated power. Never flash through it.

```sh
make detect     # expect: CH343 1a86:55d3 marked "use this one", then "ESP32-S3 (QFN56) (revision v0.2)"
```

If it says "move the cable to 'COM'", you are in the left port.

## 4. Flash

```sh
make flash      # deletes firmware/.pio, builds, uploads over COM
```

If the upload can't connect: hold **BOOT**, tap **RST**, release **BOOT**, then re-run.
**Assume:** the ESC is still **off** while flashing.

```sh
make ping       # firmware answers
make sync-time  # ESP32 clock = laptop clock (repeat after every reset)
```

## 5. Arm the ESC

Switch the ESC **on** while the ESP32 is running. It beeps and learns 1500 µs as neutral.

## 6. Test, calibrate, tune

```sh
make test            # steering sweep + failsafe check, motor stays off
make calibrate       # find center / left / right, measure maxdeg, `save`, `q`
git diff tuning.toml # calibrate wrote the [steering.calibration] values
make tune            # push tuning.toml to the ESP32 and save it to flash
make test-throttle   # short forward/reverse; type `yes` (wheels off the ground!)
```

## 7. Tuning loop

1. Edit `tuning.toml`.
2. `make tune`, or try values live with `make session` (e.g. `SET servo_hz 100`, then `:drive(10, 0.2, 2)`).
3. `make teleop` to drive: arrows = heading/speed, `a` = arm, space = stop, `q` = quit.
4. Look at the session history in `logs/*.jsonl` (`pandas.read_json(path, lines=True)`).
5. Commit `tuning.toml` when it feels right.

## Shutdown

ESC off → unplug USB.
