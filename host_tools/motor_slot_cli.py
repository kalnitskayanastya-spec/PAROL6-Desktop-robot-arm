#!/usr/bin/env python3
"""Host-side CLI for the octopus_parol6_motor_slot_test_0000 firmware.

This tool sends one text command over USB Serial and prints the firmware reply.
It is intentionally small and explicit so it is easy to inspect before using it
near hardware. Commands that can energize or move a motor require confirmation
unless --yes is passed.
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
PORT_PATTERNS = (
    "/dev/cu.usbmodem*",
    "/dev/tty.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/tty.usbserial*",
)

NO_CONFIRM_COMMANDS = {
    "disable",
    "emergency-disable",
    "status",
    "all_status",
    "slots",
    "where",
    "safe",
    "help",
    "select",
}
DANGEROUS_COMMANDS = {"enable", "step", "jog", "dir"}
COMMANDS_WITHOUT_ARGS = {
    "help",
    "slots",
    "where",
    "safe",
    "status",
    "all_status",
    "enable",
    "disable",
    "emergency-disable",
}


def import_serial():
    """Import pyserial only when serial access is needed."""
    try:
        import serial  # type: ignore
    except ImportError:
        print("ERROR: pyserial is not installed.", file=sys.stderr)
        print("Install with: python3 -m pip install pyserial", file=sys.stderr)
        return None
    return serial


def detect_ports() -> list[str]:
    """Find likely macOS USB serial ports for STM32/USB serial boards."""
    ports: list[str] = []
    for pattern in PORT_PATTERNS:
        ports.extend(glob.glob(pattern))
    return sorted(dict.fromkeys(ports))


def parse_int(value: str, name: str) -> int:
    """Parse a base-10 integer and print a beginner-friendly error."""
    try:
        return int(value, 10)
    except ValueError:
        raise ValueError(f"{name} must be an integer: {value}") from None


def build_firmware_command(tokens: list[str]) -> str:
    """Validate CLI tokens and return the exact firmware command to send."""
    if not tokens:
        raise ValueError("Missing command. Try: python3 host_tools/motor_slot_cli.py help")

    command = tokens[0]
    args = tokens[1:]

    if command in COMMANDS_WITHOUT_ARGS:
        if args:
            raise ValueError(f"{command} does not take arguments.")
        return "disable" if command == "emergency-disable" else command

    if command == "select":
        if len(args) != 1:
            raise ValueError("Usage: select 0..5")
        slot = parse_int(args[0], "slot")
        if slot < 0 or slot > 5:
            raise ValueError("slot must be in range 0..5")
        return f"select {slot}"

    if command == "dir":
        if len(args) != 1:
            raise ValueError("Usage: dir 0|1")
        direction = parse_int(args[0], "dir")
        if direction not in (0, 1):
            raise ValueError("dir must be 0 or 1")
        return f"dir {direction}"

    if command in ("step", "jog"):
        if len(args) != 1:
            raise ValueError(f"Usage: {command} N, where abs(N) <= 200")
        pulses = parse_int(args[0], "N")
        if pulses == 0:
            raise ValueError("N must be non-zero")
        if abs(pulses) > 200:
            raise ValueError("N must satisfy abs(N) <= 200")
        return f"{command} {pulses}"

    raise ValueError(f"Unsupported command: {command}")


def command_requires_confirmation(firmware_command: str) -> bool:
    """Return True for commands that can energize or move hardware."""
    command = firmware_command.split()[0]
    return command in DANGEROUS_COMMANDS


def confirm_or_abort(firmware_command: str, assume_yes: bool) -> None:
    """Ask for confirmation before risky commands."""
    if assume_yes or not command_requires_confirmation(firmware_command):
        return

    print(f"About to send safety-sensitive command: {firmware_command}")
    answer = input("Type YES to continue: ")
    if answer != "YES":
        raise RuntimeError("Aborted by user. Nothing was sent.")


def choose_port(manual_port: str | None) -> str:
    """Use the manual port, or auto-detect the first likely USB serial port."""
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


def read_response(ser, timeout: float) -> str:
    """Read all available serial text until the quiet timeout expires."""
    deadline = time.monotonic() + timeout
    chunks: list[bytes] = []

    while time.monotonic() < deadline:
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            chunks.append(ser.read(waiting))
            deadline = time.monotonic() + 0.2
        else:
            time.sleep(0.02)

    return b"".join(chunks).decode("utf-8", errors="replace").strip()


def send_serial_command(port: str, baud: int, timeout: float, firmware_command: str) -> str:
    """Open serial, send one firmware command, and return the text response."""
    serial = import_serial()
    if serial is None:
        raise RuntimeError("pyserial is required for serial communication.")

    try:
        with serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=timeout) as ser:
            time.sleep(0.5)
            ser.reset_input_buffer()
            ser.write((firmware_command + "\n").encode("utf-8"))
            ser.flush()
            return read_response(ser, timeout)
    except serial.SerialException as exc:
        raise RuntimeError(f"Serial error on {port}: {exc}") from None
    except OSError as exc:
        raise RuntimeError(f"Could not use serial port {port}: {exc}") from None


def append_log(log_path: str, port: str, firmware_command: str, response: str) -> None:
    """Append a simple timestamped transaction log."""
    timestamp = _dt.datetime.now().isoformat(timespec="seconds")
    path = Path(log_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as log_file:
        log_file.write(f"[{timestamp}] port={port} command={firmware_command!r}\n")
        log_file.write(response if response else "<no response>")
        log_file.write("\n\n")


def make_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Send commands to octopus_parol6_motor_slot_test_0000 over USB Serial."
    )
    parser.add_argument("command", nargs="*", help="Firmware command, for example: select 3")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1234")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baudrate, default 115200")
    parser.add_argument(
        "--timeout",
        type=float,
        default=DEFAULT_TIMEOUT,
        help="Read timeout in seconds, default 2",
    )
    parser.add_argument("--yes", action="store_true", help="Skip confirmation for risky commands")
    parser.add_argument("--log", help="Append timestamped command/response log to this file")
    parser.add_argument("--list-ports", action="store_true", help="List detected candidate ports and exit")
    parser.add_argument("--dry-run", action="store_true", help="Print what would be sent; do not open serial")
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
        confirm_or_abort(firmware_command, args.yes)

        if args.dry_run:
            port_text = args.port if args.port else "<auto-detect>"
            print(f"DRY RUN: port={port_text} baud={args.baud} timeout={args.timeout}")
            print(f"DRY RUN: would send: {firmware_command}")
            return 0

        port = choose_port(args.port)
        response = send_serial_command(port, args.baud, args.timeout, firmware_command)

        if response:
            print(response)
        else:
            print("No response received. Check firmware, port, and power/USB connection.")

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
