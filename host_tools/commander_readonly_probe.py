#!/usr/bin/env python3
"""Safe read-only PAROL6 Commander protocol probe."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import glob
import json
from pathlib import Path
import time

from commander_protocol import (
    DEFAULT_SAFE_MAIN_BAUD,
    ProtocolError,
    SafetyError,
    bytes_to_hex,
    decode_packet,
    decode_response_compat,
    encode_commander_packet,
    explain_command,
    hex_to_bytes,
    safest_readonly_command_id,
    validate_safe_command,
)


PORT_PATTERNS = [
    "/dev/cu.usbmodem*",
    "/dev/tty.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/tty.usbserial*",
]


def list_ports() -> list[str]:
    ports: list[str] = []
    for pattern in PORT_PATTERNS:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def choose_port(explicit_port: str | None) -> str:
    if explicit_port:
        return explicit_port
    ports = list_ports()
    if not ports:
        raise SystemExit("No serial port found. Use --port or --list-ports.")
    return ports[0]


def append_log(path: str | Path | None, *, port: str | None, command: int | None, action: str, data: str) -> None:
    if not path:
        return
    timestamp = datetime.now(timezone.utc).isoformat()
    with Path(path).open("a", encoding="utf-8") as handle:
        handle.write(f"{timestamp}\tport={port or ''}\tcommand={command if command is not None else ''}\taction={action}\t{data}\n")


def open_serial(port: str, baud: int, timeout: float):
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise SystemExit("pyserial is required for real serial I/O. Install it or use --dry-run.") from exc
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)


def read_available(ser) -> bytes:
    waiting = getattr(ser, "in_waiting", 0)
    if waiting:
        return ser.read(waiting)
    return ser.read(4096)


def cmd_list_ports() -> int:
    ports = list_ports()
    if not ports:
        print("No matching USB serial ports found.")
        return 0
    for port in ports:
        print(port)
    return 0


def cmd_decode_hex(hex_string: str) -> int:
    data = hex_to_bytes(hex_string)
    print(bytes_to_hex(data))
    try:
        decoded = decode_packet(data)
    except ProtocolError as exc:
        print(f"Not a confirmed Commander packet: {exc}")
        return 0
    print(json.dumps(decoded, indent=2, sort_keys=True))
    return 0


def cmd_decode_response_hex(hex_string: str) -> int:
    data = hex_to_bytes(hex_string)
    print(f"response bytes: {bytes_to_hex(data)}")
    decoded = decode_response_compat(data)
    if decoded.get("format_status") != "confirmed_firmware_to_host":
        print("Response format partially confirmed; unknown fields are shown as raw bytes.")
    print(json.dumps(decoded, indent=2, sort_keys=True))
    return 0


def cmd_encode(command_id: int, *, allow_low: bool, dry_run: bool, log: str | None) -> int:
    packet = encode_commander_packet(command_id, allow_low=allow_low)
    prefix = "DRY-RUN " if dry_run else ""
    print(f"{prefix}encoded packet: {bytes_to_hex(packet)}")
    append_log(log, port=None, command=command_id, action="encode", data=bytes_to_hex(packet))
    return 0


def print_response(response: bytes) -> None:
    response_hex = bytes_to_hex(response)
    print(f"response: {response_hex}")
    if response:
        decoded = decode_response_compat(response)
        if decoded.get("format_status") != "confirmed_firmware_to_host":
            print("Response format partially confirmed; unknown fields are shown as raw bytes.")
        print(json.dumps(decoded, indent=2, sort_keys=True))


def send_command_packet(args: argparse.Namespace, command_id: int, *, action: str) -> int:
    try:
        validate_safe_command(command_id, allow_low=args.allow_low)
        packet = encode_commander_packet(command_id, allow_low=args.allow_low)
    except SafetyError:
        print("Refusing to send non-read-only command in safe probe.")
        return 2

    if args.dry_run:
        port = args.port or "not opened"
        print(f"DRY-RUN port={port} baud={args.baud} packet={bytes_to_hex(packet)}")
        append_log(args.log, port=port, command=command_id, action=f"dry-run-{action}", data=bytes_to_hex(packet))
        return 0

    port = choose_port(args.port)
    with open_serial(port, args.baud, args.response_timeout) as ser:
        ser.write(packet)
        ser.flush()
        response = ser.read(4 + 56)
    response_hex = bytes_to_hex(response)
    print(f"sent: {bytes_to_hex(packet)}")
    if args.expect_response:
        print_response(response)
    else:
        print(f"response: {response_hex}")
    append_log(args.log, port=port, command=command_id, action=action, data=f"sent={bytes_to_hex(packet)} response={response_hex}")
    return 0


def cmd_send(args: argparse.Namespace) -> int:
    return send_command_packet(args, int(args.send_readonly), action="send")


def cmd_send_status(args: argparse.Namespace) -> int:
    command_id = safest_readonly_command_id()
    info = validate_safe_command(command_id, allow_low=False)
    print(f"selected_readonly_status_command={command_id}")
    print(f"meaning={info.meaning}")
    return send_command_packet(args, command_id, action="send-status")


def cmd_listen(args: argparse.Namespace) -> int:
    port = choose_port(args.port)
    seconds = args.listen_seconds if args.listen_seconds is not None else args.response_timeout
    deadline = time.monotonic() + float(seconds)
    if args.dry_run:
        print(f"DRY-RUN would listen on {port} at {args.baud} baud for {seconds} seconds.")
        return 0

    with open_serial(port, args.baud, args.timeout) as ser:
        while time.monotonic() < deadline:
            data = read_available(ser)
            if data:
                data_hex = bytes_to_hex(data)
                print(data_hex)
                if args.listen_readonly:
                    decoded = decode_response_compat(data)
                    if decoded.get("format_status") != "confirmed_firmware_to_host":
                        print("Response format partially confirmed; unknown fields are shown as raw bytes.")
                    print(json.dumps(decoded, indent=2, sort_keys=True))
                append_log(args.log, port=port, command=None, action="listen", data=data_hex)
            else:
                time.sleep(0.05)
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Safe read-only PAROL6 Commander protocol probe.")
    parser.add_argument("--port", help="Serial port. If omitted, first matching USB modem/serial port is used.")
    parser.add_argument("--baud", type=int, default=DEFAULT_SAFE_MAIN_BAUD, help="Serial baud rate. Default: 115200.")
    parser.add_argument("--timeout", type=float, default=2.0, help="Serial timeout in seconds. Default: 2.")
    parser.add_argument("--response-timeout", type=float, default=2.0, help="Commander response timeout in seconds. Default: 2.")
    parser.add_argument("--list-ports", action="store_true", help="List matching USB serial ports.")
    parser.add_argument("--dry-run", action="store_true", help="Print actions without opening serial.")
    parser.add_argument("--log", help="Append probe log lines to this file.")
    parser.add_argument("--explain-command", type=int, help="Explain a Commander command ID.")
    parser.add_argument("--decode-hex", help="Decode bytes from a hex string.")
    parser.add_argument("--decode-response-hex", help="Decode firmware response bytes from a hex string.")
    parser.add_argument("--encode-readonly", type=int, help="Encode a read-only Commander packet.")
    parser.add_argument("--send-readonly", type=int, help="Send a read-only Commander packet.")
    parser.add_argument("--send-status", action="store_true", help="Send the safest confirmed READ_ONLY status command.")
    parser.add_argument("--expect-response", action="store_true", help="Decode and print a response after sending.")
    parser.add_argument("--listen-readonly", action="store_true", help="Listen and decode incoming Commander response bytes.")
    parser.add_argument("--listen-seconds", type=float, help="Listen for raw serial bytes for N seconds.")
    parser.add_argument("--allow-low", action="store_true", help="Allow LOW commands 102/103; HIGH/DANGEROUS are still refused.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    try:
        if args.list_ports:
            return cmd_list_ports()
        if args.explain_command is not None:
            print(explain_command(args.explain_command))
            return 0
        if args.decode_hex is not None:
            return cmd_decode_hex(args.decode_hex)
        if args.decode_response_hex is not None:
            return cmd_decode_response_hex(args.decode_response_hex)
        if args.encode_readonly is not None:
            return cmd_encode(args.encode_readonly, allow_low=args.allow_low, dry_run=args.dry_run, log=args.log)
        if args.send_readonly is not None:
            return cmd_send(args)
        if args.send_status:
            return cmd_send_status(args)
        if args.listen_seconds is not None or args.listen_readonly:
            return cmd_listen(args)
    except SafetyError:
        print("Refusing to send non-read-only command in safe probe.")
        return 2
    except ProtocolError as exc:
        print(f"Protocol error: {exc}")
        return 2

    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
