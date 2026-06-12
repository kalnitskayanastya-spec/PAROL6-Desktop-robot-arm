#!/usr/bin/env python3
"""Host-side calibration storage tool for the PAROL6 Octopus port."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import glob
import json
from pathlib import Path
import sys
import time
from typing import Any


BAUD_DEFAULT = 115200
PORT_PATTERNS = [
    "/dev/cu.usbmodem*",
    "/dev/tty.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/tty.usbserial*",
]
TEMPLATE_PATH = Path(__file__).with_name("octopus_calibration_template.json")
VALID_STATUSES = {
    "ORIGINAL_PAROL6_CONFIRMED",
    "OCTOPUS_PORT_CONFIRMED",
    "PLACEHOLDER_NOT_CONFIRMED",
    "MEASUREMENT_REQUIRED",
    "HARDWARE_VALIDATION_REQUIRED",
}
UNCONFIRMED_STATUSES = {
    "PLACEHOLDER_NOT_CONFIRMED",
    "MEASUREMENT_REQUIRED",
    "HARDWARE_VALIDATION_REQUIRED",
}
EXPECTED_MOTORS = {1: "MOTOR0", 2: "MOTOR1", 3: "MOTOR2", 4: "MOTOR3", 5: "MOTOR4", 6: "MOTOR5"}
EXPECTED_LIMITS = {1: "Stop0", 2: "Stop1", 3: "Stop2", 4: "Stop3", 5: "Stop4", 6: "Stop5"}
FIELDS = [
    "steps_per_degree",
    "direction_invert",
    "soft_min_deg",
    "soft_max_deg",
    "home_offset_deg",
    "homing_max_travel_steps",
    "max_speed",
    "max_acceleration",
    "tmc_current_ma",
    "safe_joint_step_limit",
]
CRITICAL_FIELDS = ["steps_per_degree", "soft_min_deg", "soft_max_deg", "home_offset_deg", "homing_max_travel_steps"]


def list_ports() -> list[str]:
    ports: list[str] = []
    for pattern in PORT_PATTERNS:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def choose_port(port: str | None) -> str:
    if port:
        return port
    ports = list_ports()
    if not ports:
        raise RuntimeError("No serial port found. Use --port or --list-ports.")
    return ports[0]


def import_serial():
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pyserial is required for serial communication. Use --dry-run-apply for offline checks.") from exc
    return serial


def load_json(path: str | Path) -> dict[str, Any]:
    with Path(path).open("r", encoding="utf-8") as handle:
        return json.load(handle)


def field_value(joint: dict[str, Any], field: str) -> Any:
    raw = joint.get(field)
    if isinstance(raw, dict):
        return raw.get("value")
    return raw


def field_status(joint: dict[str, Any], field: str) -> str:
    raw = joint.get(field)
    if isinstance(raw, dict):
        status = raw.get("status", joint.get("calibration_status", "PLACEHOLDER_NOT_CONFIRMED"))
    else:
        status = joint.get("calibration_status", "PLACEHOLDER_NOT_CONFIRMED")
    return str(status)


def is_number_or_none(value: Any) -> bool:
    return value is None or isinstance(value, (int, float))


def validate_config(data: dict[str, Any]) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []
    joints = data.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        errors.append("JSON must contain exactly six joints.")
        return errors, warnings

    seen: set[int] = set()
    for joint in joints:
        number = joint.get("joint_number")
        if not isinstance(number, int) or number < 1 or number > 6:
            errors.append(f"Invalid joint_number: {number!r}.")
            continue
        if number in seen:
            errors.append(f"Duplicate joint_number: {number}.")
        seen.add(number)

        motor = joint.get("motor_connector")
        limit = joint.get("limit_input")
        if motor != EXPECTED_MOTORS[number]:
            errors.append(f"Joint{number} motor mapping must be {EXPECTED_MOTORS[number]}, got {motor!r}.")
        if motor == "MOTOR2_2":
            errors.append("Joint4 must not be MOTOR2_2.")
        if limit != EXPECTED_LIMITS[number]:
            errors.append(f"Joint{number} limit mapping must be {EXPECTED_LIMITS[number]}, got {limit!r}.")

        for field in FIELDS:
            value = field_value(joint, field)
            status = field_status(joint, field)
            if status not in VALID_STATUSES:
                errors.append(f"Joint{number} {field} has invalid status {status!r}.")
            if status in UNCONFIRMED_STATUSES:
                warnings.append(f"Joint{number} {field} status={status}.")
            if field == "direction_invert":
                if value is not None and not isinstance(value, bool):
                    errors.append(f"Joint{number} direction_invert must be boolean or null.")
            elif not is_number_or_none(value):
                errors.append(f"Joint{number} {field} must be a number or null.")

        steps = field_value(joint, "steps_per_degree")
        if steps is not None and steps <= 0:
            errors.append(f"Joint{number} steps_per_degree must be > 0 if present.")
        min_deg = field_value(joint, "soft_min_deg")
        max_deg = field_value(joint, "soft_max_deg")
        if min_deg is not None and max_deg is not None and min_deg >= max_deg:
            errors.append(f"Joint{number} soft_min_deg must be less than soft_max_deg.")
        homing = field_value(joint, "homing_max_travel_steps")
        if homing is not None and homing <= 0:
            errors.append(f"Joint{number} homing_max_travel_steps must be > 0 if present.")

    if seen != set(range(1, 7)):
        errors.append("Joint numbers must be exactly 1..6.")
    return errors, warnings


def summary(data: dict[str, Any]) -> dict[str, Any]:
    errors, warnings = validate_config(data)
    critical_unconfirmed: list[str] = []
    for joint in data.get("joints", []):
        number = joint.get("joint_number", "?")
        for field in CRITICAL_FIELDS:
            value = field_value(joint, field)
            status = field_status(joint, field)
            if value is None or status in UNCONFIRMED_STATUSES:
                critical_unconfirmed.append(f"Joint{number} {field} status={status} value={value}")
    readiness = "NOT READY FOR REAL HOMING OR FULL MOTION" if critical_unconfirmed or errors else "READY_FOR_REVIEW"
    return {
        "errors": errors,
        "warnings": warnings,
        "critical_unconfirmed": critical_unconfirmed,
        "readiness": readiness,
    }


def commands_for_apply(data: dict[str, Any]) -> list[str]:
    commands: list[str] = []
    for joint in data.get("joints", []):
        number = joint["joint_number"]
        for field in FIELDS:
            value = field_value(joint, field)
            if value is None:
                continue
            if isinstance(value, bool):
                encoded = "on" if value else "off"
            else:
                encoded = str(value)
            commands.append(f"calib_storage_set {number} {field} {encoded}")
    return commands


def read_until_quiet(ser, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    chunks: list[bytes] = []
    while time.monotonic() < deadline:
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            chunks.append(ser.read(waiting))
            deadline = time.monotonic() + min(timeout, 0.25)
        else:
            time.sleep(0.03)
    return b"".join(chunks).decode("utf-8", errors="replace")


def send_commands(port: str, baud: int, timeout: float, commands: list[str]) -> dict[str, str]:
    serial = import_serial()
    responses: dict[str, str] = {}
    with serial.Serial(port=port, baudrate=baud, timeout=0.05, write_timeout=timeout) as ser:
        time.sleep(0.2)
        read_until_quiet(ser, min(timeout, 0.5))
        for command in commands:
            ser.write((command + "\n").encode("ascii"))
            ser.flush()
            responses[command] = read_until_quiet(ser, timeout)
    return responses


def write_json_report(path: str | None, report: dict[str, Any]) -> None:
    if path:
        Path(path).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def print_validation(errors: list[str], warnings: list[str]) -> int:
    if errors:
        print("VALIDATION: FAIL")
        for error in errors:
            print(f"ERROR: {error}")
    else:
        print("VALIDATION: PASS")
    for warning in warnings:
        print(f"WARNING: {warning}")
    return 1 if errors else 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manage host-side PAROL6 Octopus calibration JSON and RAM-only firmware apply.")
    parser.add_argument("--template", action="store_true", help="Print the host-side calibration JSON template.")
    parser.add_argument("--validate", metavar="JSON_PATH", help="Validate calibration JSON.")
    parser.add_argument("--summary", metavar="JSON_PATH", help="Print calibration readiness summary.")
    parser.add_argument("--export-defaults", action="store_true", help="Print compiled host-side default template JSON.")
    parser.add_argument("--dry-run-apply", metavar="JSON_PATH", help="Show RAM-only firmware commands without opening serial.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1234.")
    parser.add_argument("--baud", type=int, default=BAUD_DEFAULT, help="Serial baud rate. Default: 115200.")
    parser.add_argument("--timeout", type=float, default=2.0, help="Serial timeout. Default: 2.")
    parser.add_argument("--list-ports", action="store_true", help="List likely USB serial ports.")
    parser.add_argument("--apply-ram", metavar="JSON_PATH", help="Apply calibration JSON to firmware RAM-only config.")
    parser.add_argument("--read-ram", action="store_true", help="Read firmware RAM calibration export.")
    parser.add_argument("--json-report", help="Write JSON report to path.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    report: dict[str, Any] = {"timestamp": datetime.now(timezone.utc).isoformat(), "actions": []}

    try:
        if args.list_ports:
            ports = list_ports()
            print("\n".join(ports) if ports else "No matching USB serial ports found.")
            report["actions"].append({"list_ports": ports})
            write_json_report(args.json_report, report)
            return 0
        if args.template or args.export_defaults:
            text = TEMPLATE_PATH.read_text(encoding="utf-8")
            print(text, end="" if text.endswith("\n") else "\n")
            report["actions"].append({"template": str(TEMPLATE_PATH)})
            write_json_report(args.json_report, report)
            return 0
        if args.validate:
            data = load_json(args.validate)
            errors, warnings = validate_config(data)
            report["actions"].append({"validate": args.validate, "errors": errors, "warnings": warnings})
            write_json_report(args.json_report, report)
            return print_validation(errors, warnings)
        if args.summary:
            data = load_json(args.summary)
            result = summary(data)
            print(json.dumps(result, indent=2, sort_keys=True))
            if result["readiness"] == "NOT READY FOR REAL HOMING OR FULL MOTION":
                print("NOT READY FOR REAL HOMING OR FULL MOTION")
            report["actions"].append({"summary": args.summary, **result})
            write_json_report(args.json_report, report)
            return 0 if not result["errors"] else 1
        if args.dry_run_apply:
            data = load_json(args.dry_run_apply)
            result = summary(data)
            print(json.dumps(result, indent=2, sort_keys=True))
            print("DRY-RUN RAM APPLY COMMANDS:")
            commands = commands_for_apply(data)
            for command in commands:
                print(command)
            print("No serial port opened. No firmware flash/EEPROM writes will be performed.")
            report["actions"].append({"dry_run_apply": args.dry_run_apply, "commands": commands, **result})
            write_json_report(args.json_report, report)
            return 0 if not result["errors"] else 1
        if args.apply_ram:
            data = load_json(args.apply_ram)
            result = summary(data)
            if result["errors"]:
                print_validation(result["errors"], result["warnings"])
                report["actions"].append({"apply_ram": args.apply_ram, **result})
                write_json_report(args.json_report, report)
                return 1
            commands = commands_for_apply(data)
            port = choose_port(args.port)
            print(f"Applying RAM-only calibration overrides to {port} at {args.baud} baud.")
            responses = send_commands(port, args.baud, args.timeout, commands)
            for command, response in responses.items():
                print(f"> {command}")
                print(response.rstrip())
            report["actions"].append({"apply_ram": args.apply_ram, "port": port, "commands": commands, "responses": responses, **result})
            write_json_report(args.json_report, report)
            return 0
        if args.read_ram:
            port = choose_port(args.port)
            responses = send_commands(port, args.baud, args.timeout, ["calib_storage_export"])
            response = responses.get("calib_storage_export", "")
            print(response.rstrip())
            report["actions"].append({"read_ram": True, "port": port, "response": response})
            write_json_report(args.json_report, report)
            return 0
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        report["error"] = str(exc)
        write_json_report(args.json_report, report)
        return 2

    parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
