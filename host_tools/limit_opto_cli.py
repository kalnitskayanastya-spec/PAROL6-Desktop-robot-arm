#!/usr/bin/env python3
"""Host-side CLI for Octopus limit/opto diagnostic firmware.

The firmware is input-only: it reads Octopus Stop0..Stop5 inputs and reports
their raw/debounced states. This tool sends one text command over USB Serial
and prints the reply. For `stream on`, it listens for a short, configurable
window and then sends `stream off` before exiting.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import glob
import sys
import time
from pathlib import Path


DEFAULT_BAUD = 115200
DEFAULT_TIMEOUT = 2.0
DEFAULT_STREAM_SECONDS = 5.0
PORT_PATTERNS = (
    "/dev/cu.usbmodem*",
    "/dev/tty.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/tty.usbserial*",
)

NO_ARG_COMMANDS = {
    "help",
    "version",
    "status",
    "raw",
    "counts",
    "reset_counts",
    "safe",
}
MODE_COMMANDS = {"stream", "invert", "pullup", "active_low"}
MAX_DEBOUNCE_MS = 1000


def import_serial():
    """Import pyserial only when a real serial connection is needed."""
    try:
        import serial  # type: ignore
    except ImportError:
        print("ERROR: pyserial is not installed.", file=sys.stderr)
        print("Install with: python3 -m pip install pyserial", file=sys.stderr)
        return None
    return serial


def detect_ports() -> list[str]:
    """Return likely macOS USB serial ports for STM32 USB CDC boards."""
    ports: list[str] = []
    for pattern in PORT_PATTERNS:
        ports.extend(glob.glob(pattern))
    return sorted(dict.fromkeys(ports))


def usage_text() -> str:
    return (
        "Supported commands: help, version, status, raw, counts, reset_counts, safe, "
        "stream on|off, active_low on|off, pullup on|off, debounce [0..1000], "
        "invert on|off"
    )


def build_firmware_command(tokens: list[str]) -> str:
    """Validate CLI command tokens and return the exact firmware command."""
    if not tokens:
        raise ValueError(f"Missing command. {usage_text()}")

    command = tokens[0]
    args = tokens[1:]

    if command in NO_ARG_COMMANDS:
        if args:
            raise ValueError(f"{command} does not take arguments. {usage_text()}")
        return command

    if command == "debounce":
        if not args:
            return command
        if len(args) != 1:
            raise ValueError(f"debounce accepts zero or one value. {usage_text()}")
        try:
            debounce_ms = int(args[0], 10)
        except ValueError:
            raise ValueError("debounce value must be an integer in range 0..1000.") from None
        if debounce_ms < 0 or debounce_ms > MAX_DEBOUNCE_MS:
            raise ValueError("debounce value must be in range 0..1000.")
        return f"debounce {debounce_ms}"

    if command in MODE_COMMANDS:
        if len(args) != 1 or args[0] not in ("on", "off"):
            raise ValueError(f"{command} accepts only 'on' or 'off'. {usage_text()}")
        return f"{command} {args[0]}"

    raise ValueError(f"Unknown command: {command}. {usage_text()}")


def choose_port(manual_port: str | None) -> str:
    """Use --port if provided, otherwise auto-detect a likely port."""
    if manual_port:
        return manual_port

    ports = detect_ports()
    if not ports:
        raise RuntimeError(
            "No candidate serial ports found. Use --list-ports or pass --port /dev/cu.usbmodem..."
        )
    if len(ports) > 1:
        print(f"Multiple candidate ports found; using {ports[0]}", file=sys.stderr)
    return ports[0]


def read_response(ser, seconds: float, quiet_extend: float = 0.2) -> str:
    """Read serial text until the timeout expires."""
    deadline = time.monotonic() + seconds
    chunks: list[bytes] = []

    while time.monotonic() < deadline:
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            chunks.append(ser.read(waiting))
            if quiet_extend:
                deadline = max(deadline, time.monotonic() + quiet_extend)
        else:
            time.sleep(0.02)

    return b"".join(chunks).decode("utf-8", errors="replace").strip()


def open_serial(port: str, baud: int, timeout: float):
    """Open a serial port and convert pyserial errors into helpful messages."""
    serial = import_serial()
    if serial is None:
        raise RuntimeError("pyserial is required for serial communication.")

    try:
        return serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=timeout)
    except serial.SerialException as exc:
        raise RuntimeError(f"Serial error on {port}: {exc}") from None
    except OSError as exc:
        raise RuntimeError(f"Could not use serial port {port}: {exc}") from None


def send_line(ser, command: str) -> None:
    """Send one newline-terminated firmware command."""
    ser.write((command + "\n").encode("utf-8"))
    ser.flush()


def run_serial_command(
    port: str,
    baud: int,
    timeout: float,
    firmware_command: str,
    stream_seconds: float,
) -> str:
    """Send a command and return the firmware response."""
    try:
        with open_serial(port, baud, timeout) as ser:
            time.sleep(0.5)
            ser.reset_input_buffer()

            send_line(ser, firmware_command)

            if firmware_command == "stream on":
                response = read_response(ser, stream_seconds, quiet_extend=0.0)
                send_line(ser, "stream off")
                trailing = read_response(ser, min(timeout, 1.0), quiet_extend=0.1)
                if trailing:
                    response = f"{response}\n{trailing}" if response else trailing
                return response

            return read_response(ser, timeout)
    except OSError as exc:
        raise RuntimeError(f"Serial communication failed: {exc}") from None


def append_log(log_path: str, port: str, firmware_command: str, response: str) -> None:
    """Append a timestamped command/response record."""
    timestamp = _dt.datetime.now().isoformat(timespec="seconds")
    path = Path(log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as log_file:
        log_file.write(f"[{timestamp}] port={port} command={firmware_command!r}\n")
        log_file.write(response if response else "<no response>")
        log_file.write("\n\n")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Talk to Octopus limit/opto diagnostic firmware over USB Serial."
    )
    parser.add_argument("command", nargs="*", help="Firmware command, for example: stream on")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1234")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baudrate, default 115200")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Read timeout, default 2")
    parser.add_argument("--dry-run", action="store_true", help="Print what would be sent; do not open serial")
    parser.add_argument("--list-ports", action="store_true", help="List detected candidate ports and exit")
    parser.add_argument("--log", help="Append timestamped command/response log to this file")
    parser.add_argument(
        "--stream-seconds",
        type=float,
        default=DEFAULT_STREAM_SECONDS,
        help="How long to read after 'stream on', default 5",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = make_parser()
    args = parser.parse_args(argv)

    if args.list_ports:
        ports = detect_ports()
        if ports:
            print("Detected candidate ports:")
            for port in ports:
                print(port)
        else:
            print("No candidate ports detected.")
        return 0

    try:
        firmware_command = build_firmware_command(args.command)

        if args.stream_seconds <= 0:
            raise ValueError("--stream-seconds must be greater than 0")

        if args.dry_run:
            port_text = args.port if args.port else "<auto-detect>"
            print(f"DRY RUN: port={port_text} baud={args.baud} timeout={args.timeout}")
            print(f"DRY RUN: would send: {firmware_command}")
            if firmware_command == "stream on":
                print(f"DRY RUN: would read for {args.stream_seconds} seconds")
                print("DRY RUN: would send: stream off")
            return 0

        port = choose_port(args.port)
        response = run_serial_command(
            port=port,
            baud=args.baud,
            timeout=args.timeout,
            firmware_command=firmware_command,
            stream_seconds=args.stream_seconds,
        )

        if response:
            print(response)
        else:
            print("No response received. Check firmware, port, USB, and board power.")

        if args.log:
            append_log(args.log, port, firmware_command, response)

        return 0
    except (RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nERROR: interrupted by user.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
