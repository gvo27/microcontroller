#!/usr/bin/env python3
"""Laptop-side CLI for the ESP32-S3 RC car actuator bridge.

Talks the line protocol in firmware/src/main.cpp over the board's "COM"
USB-C port (right-hand port, CH343 USB-UART bridge). Tuning comes from tuning.toml at the
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

# tuning.toml (table, key) -> firmware command prefix. Order matters for ESC
# endpoints: the firmware rejects any SET that breaks esc_min < neutral < max.
FIRMWARE_KEYS = [
    (("steering.calibration", "center_us"), "CAL CENTER"),
    (("steering.calibration", "left_us"), "CAL LEFT"),
    (("steering.calibration", "right_us"), "CAL RIGHT"),
    (("steering.calibration", "max_steer_deg"), "SET maxdeg"),
    (("steering", "heading_gain"), "SET gain"),
    (("steering", "servo_hz"), "SET servo_hz"),
    (("throttle.calibration", "esc_min_us"), "SET esc_min"),
    (("throttle.calibration", "esc_max_us"), "SET esc_max"),
    (("throttle.calibration", "esc_neutral_us"), "SET esc_neutral"),
    (("throttle.calibration", "deadband_us"), "SET deadband"),
    (("throttle", "esc_hz"), "SET esc_hz"),
    (("throttle", "speed_limit"), "SET limit"),
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


def tuning_value(tuning: dict, table: str, key: str):
    """tuning_value(t, "steering.calibration", "center_us")"""
    node = tuning
    for part in table.split("."):
        node = node[part]
    return node[key]


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
            val = tuning_value(self.tuning, section, key)
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
