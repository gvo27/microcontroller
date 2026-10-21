#!/usr/bin/env python3
"""Interactive scratch session with the ESP32 for tuning and quick tests.

    make session            # or: .venv/bin/python tools/esp32_session.py [--port X]

At the prompt:
    PING / GET / SET limit 0.4 / RAW 1600 ...   raw firmware commands (replies printed)
    :drive(10, 0.2, 2)                          call a helper below (Python, prefix ':')
    :set_param("servo_hz", 100)
    :read_imu()
    q                                           quit (disarms, logs to logs/)

Typed raw commands do not stream heartbeats, so `ARM` alone times out after
cmd_timeout_ms by design. Use :drive() to move the car.

Add your own helpers here; the transport, tuning and logging live in
rccar.py (same folder) so this file stays a thin, hackable layer.
"""
from __future__ import annotations

import argparse
import sys
import time

from rccar import Link, find_port, parse_kv  # same folder: tools/

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
