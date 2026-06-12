#!/usr/bin/env python3
"""Host-side safe bring-up runner for the PAROL6 Octopus port."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, asdict
from datetime import datetime, timezone
import glob
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

from commander_protocol import (
    DEFAULT_SAFE_MAIN_BAUD,
    SafetyError,
    bytes_to_hex,
    decode_response_compat,
    encode_commander_packet,
    explain_command,
    get_command_info,
    safest_readonly_command_id,
    validate_safe_command,
)


PORT_PATTERNS = [
    "/dev/cu.usbmodem*",
    "/dev/tty.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/tty.usbserial*",
]

STAGES = [
    "safe_main_status",
    "tmc_status",
    "limits",
    "commander_readonly",
    "joint_test_status",
    "homing_readiness",
]

SAFE_SHELL_COMMANDS = {
    "safe_main_status": ["version", "safe", "status"],
    "tmc_status": ["tmc_status"],
    "limits": ["limits", "limits_raw", "limits_config"],
    "joint_test_status": ["joint_test_status", "joint_safe"],
    "homing_readiness": ["homing_preflight_status", "homing_dryrun_status", "homing_exec_status", "real_homing_status"],
}


@dataclass
class StageResult:
    name: str
    result: str
    summary: str
    commands: list[str]
    raw_responses: dict[str, str]
    warnings: list[str]
    blockers: list[str]


def list_ports() -> list[str]:
    ports: list[str] = []
    for pattern in PORT_PATTERNS:
        ports.extend(glob.glob(pattern))
    return sorted(set(ports))


def choose_port(explicit_port: str | None) -> str | None:
    if explicit_port:
        return explicit_port
    ports = list_ports()
    return ports[0] if ports else None


def import_serial():
    try:
        import serial  # type: ignore
    except ImportError as exc:
        raise RuntimeError("pyserial is required for serial communication. Use --dry-run for offline checks.") from exc
    return serial


def read_until_quiet(ser, timeout: float) -> str:
    deadline = time.monotonic() + timeout
    chunks: list[bytes] = []
    while time.monotonic() < deadline:
        waiting = getattr(ser, "in_waiting", 0)
        if waiting:
            chunks.append(ser.read(waiting))
            deadline = time.monotonic() + min(0.25, timeout)
        else:
            time.sleep(0.03)
    return b"".join(chunks).decode("utf-8", errors="replace")


def run_shell_commands(args: argparse.Namespace, stage: str, commands: list[str]) -> StageResult:
    if args.dry_run:
        responses = {command: f"DRY-RUN would send safe diagnostic shell command: {command}" for command in commands}
        summary = "dry-run only; no serial port opened"
        result = "PASS"
        warnings: list[str] = []
        blockers: list[str] = []
        if stage == "tmc_status":
            result = "PARTIAL"
            warnings.append("hardware TMC status not checked in dry-run")
        elif stage == "limits":
            result = "PARTIAL"
            warnings.append("limit polarity and signals not validated in dry-run")
        elif stage == "joint_test_status":
            result = "NOT_AVAILABLE_IN_THIS_ENV"
            warnings.append("joint-test firmware availability cannot be confirmed in dry-run")
        elif stage == "homing_readiness":
            summary = "dry-run confirms only non-moving readiness commands would be queried"
        return StageResult(stage, result, summary, commands, responses, warnings, blockers)

    port = choose_port(args.port)
    if port is None:
        return StageResult(stage, "BLOCKED_BY_HARDWARE", "no serial port found", commands, {}, [], ["no serial port found"])

    serial = import_serial()
    responses: dict[str, str] = {}
    try:
        with serial.Serial(port=port, baudrate=args.baud, timeout=0.05, write_timeout=args.timeout) as ser:
            time.sleep(0.2)
            read_until_quiet(ser, min(0.5, args.timeout))
            for command in commands:
                ser.write((command + "\n").encode("ascii"))
                ser.flush()
                responses[command] = read_until_quiet(ser, args.timeout)
    except Exception as exc:
        return StageResult(stage, "BLOCKED_BY_HARDWARE", f"serial communication failed: {exc}", commands, responses, [], [str(exc)])

    joined = "\n".join(responses.values())
    warnings = [line.strip() for line in joined.splitlines() if "WARNING" in line or "Unknown command" in line]
    blockers: list[str] = []
    result = "PASS"
    summary = "safe diagnostic commands responded"

    if stage == "safe_main_status":
        if "Motion" not in joined and "MOTION" not in joined:
            warnings.append("motion blocked status not found in response")
            result = "PARTIAL"
        if "Homing" not in joined and "HOMING" not in joined:
            warnings.append("homing blocked status not found in response")
            result = "PARTIAL"
    elif stage == "tmc_status":
        if not joined.strip():
            result = "BLOCKED_BY_HARDWARE"
            blockers.append("no tmc_status response")
        elif warnings:
            result = "PARTIAL"
            summary = "tmc_status responded with warnings"
        elif "test_connection = 0" in joined or "TMC5160" in joined:
            summary = "tmc_status readable"
        else:
            result = "PARTIAL"
            warnings.append("TMC status response was readable but expected fields were not recognized")
    elif stage == "limits":
        if not joined.strip():
            result = "BLOCKED_BY_HARDWARE"
            blockers.append("no limits response")
        elif "Stop0" in joined and "Stop5" in joined:
            result = "PARTIAL"
            summary = "limits readable; polarity still requires physical validation"
            warnings.append("limit polarity may still be unvalidated")
        else:
            result = "PARTIAL"
            warnings.append("limit mapping not fully recognized")
    elif stage == "joint_test_status":
        if "Unknown command" in joined:
            result = "NOT_AVAILABLE_IN_THIS_ENV"
            summary = "joint-test shell is not available in this firmware environment"
        elif "SAFE JOINT TEST MODE" in joined or "joint_test_compiled=YES" in joined:
            summary = "joint-test status readable"
        else:
            result = "PARTIAL"
            warnings.append("joint-test response not fully recognized")
    elif stage == "homing_readiness":
        if "Unknown command" in joined:
            result = "PARTIAL"
            warnings.append("some homing readiness commands are not available")
        if "BLOCKED" in joined or "blocked" in joined:
            summary = "real homing blocked as expected"
        else:
            result = "PARTIAL"
            warnings.append("real homing blocked status not clearly visible")

    return StageResult(stage, result, summary, commands, responses, warnings, blockers)


def commander_readonly_stage(args: argparse.Namespace) -> StageResult:
    commands: list[str] = ["explain 255", "explain 100", "encode/send safest READ_ONLY status"]
    raw: dict[str, str] = {}
    warnings: list[str] = []
    blockers: list[str] = []

    raw["explain_255"] = explain_command(255)
    raw["explain_100"] = explain_command(100)
    dangerous = get_command_info(100)
    try:
        validate_safe_command(100, allow_low=args.allow_low_commander)
        warnings.append("unexpected: command 100 was not refused")
    except SafetyError:
        raw["dangerous_refusal"] = "Refusing to send non-read-only command in safe probe."

    command_id = safest_readonly_command_id()
    packet = encode_commander_packet(command_id)
    raw["selected_status_command"] = str(command_id)
    raw["encoded_packet_hex"] = bytes_to_hex(packet)

    if args.dry_run or args.port is None:
        summary = "command map loaded, READ_ONLY packet encoded, dangerous commands refused; dry-run/no port so nothing sent"
        return StageResult("commander_readonly", "PASS", summary, commands, raw, warnings, blockers)

    port = choose_port(args.port)
    if port is None:
        blockers.append("no serial port found")
        return StageResult("commander_readonly", "BLOCKED_BY_HARDWARE", "no serial port found", commands, raw, warnings, blockers)

    serial = import_serial()
    try:
        with serial.Serial(port=port, baudrate=args.baud, timeout=args.timeout, write_timeout=args.timeout) as ser:
            ser.write(packet)
            ser.flush()
            response = ser.read(4 + 56)
    except Exception as exc:
        blockers.append(str(exc))
        return StageResult("commander_readonly", "BLOCKED_BY_HARDWARE", f"serial communication failed: {exc}", commands, raw, warnings, blockers)

    raw["response_hex"] = bytes_to_hex(response)
    raw["decoded_response"] = json.dumps(decode_response_compat(response), sort_keys=True)
    result = "PASS" if response else "PARTIAL"
    summary = "READ_ONLY status command sent and response decoded" if response else "READ_ONLY status command sent but no response captured"
    return StageResult("commander_readonly", result, summary, commands, raw, warnings, blockers)


def run_stage(args: argparse.Namespace, stage: str) -> StageResult:
    if stage == "commander_readonly":
        if args.skip_commander:
            return StageResult(stage, "SKIPPED", "skipped by --skip-commander", [], {}, [], [])
        return commander_readonly_stage(args)
    if stage == "joint_test_status" and args.skip_joint_test:
        return StageResult(stage, "SKIPPED", "skipped by --skip-joint-test", [], {}, [], [])
    return run_shell_commands(args, stage, SAFE_SHELL_COMMANDS[stage])


def final_classification(results: list[StageResult]) -> tuple[list[str], list[str]]:
    blockers: list[str] = []
    classifications = [
        "READY_FOR_READONLY_SAFE_MAIN_CHECKS",
        "NOT_READY_FOR_REAL_HOMING",
        "NOT_READY_FOR_FULL_MOTION",
    ]
    by_name = {result.name: result for result in results}
    tmc = by_name.get("tmc_status")
    limits = by_name.get("limits")
    if tmc and tmc.result in ("PASS", "PARTIAL"):
        classifications.append("READY_FOR_TMC_DIAGNOSTIC")
    else:
        blockers.append("TMC diagnostic not confirmed")
    if limits and limits.result in ("PASS", "PARTIAL"):
        classifications.append("READY_FOR_LIMIT_POLARITY_VALIDATION")
    else:
        blockers.append("limit diagnostic not confirmed")
    for result in results:
        blockers.extend(result.blockers)
    return classifications, blockers


def write_reports(args: argparse.Namespace, report: dict[str, Any], text: str) -> None:
    if args.json_report:
        Path(args.json_report).write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.text_report:
        Path(args.text_report).write_text(text, encoding="utf-8")
    if args.log:
        with Path(args.log).open("a", encoding="utf-8") as handle:
            handle.write(text)
            handle.write("\n")


def render_report(args: argparse.Namespace, results: list[StageResult], classifications: list[str], blockers: list[str]) -> str:
    port = choose_port(args.port) if not args.dry_run else (args.port or "not opened")
    lines = [
        "PAROL6 OCTOPUS BRING-UP RUNNER",
        f"Port: {port or 'not found'}",
        f"Baud: {args.baud}",
        f"Mode: {'dry-run' if args.dry_run else 'serial'}",
        "",
        "Stage results:",
    ]
    for result in results:
        suffix = f" - {result.summary}" if result.summary else ""
        lines.append(f"* {result.name}: {result.result}{suffix}")
        for warning in result.warnings:
            lines.append(f"  warning: {warning}")
    lines.append("")
    lines.append("Final classification:")
    for item in classifications:
        lines.append(f"* {item}")
    if blockers:
        lines.append("")
        lines.append("Final blockers:")
        for blocker in blockers:
            lines.append(f"* {blocker}")
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run safe host-side PAROL6 Octopus bring-up checks.")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbmodem1234")
    parser.add_argument("--baud", type=int, default=DEFAULT_SAFE_MAIN_BAUD, help="Serial baud rate. Default: 115200.")
    parser.add_argument("--timeout", type=float, default=2.0, help="Serial response timeout. Default: 2.")
    parser.add_argument("--list-ports", action="store_true", help="List likely USB serial ports.")
    parser.add_argument("--dry-run", action="store_true", help="Print and report without opening serial.")
    parser.add_argument("--log", help="Append text report to a log file.")
    parser.add_argument("--stage", choices=STAGES, help="Run one stage.")
    parser.add_argument("--all", action="store_true", help="Run all stages.")
    parser.add_argument("--json-report", help="Write JSON report to path.")
    parser.add_argument("--text-report", help="Write text report to path.")
    parser.add_argument("--allow-low-commander", action="store_true", help="Allow LOW Commander commands in policy checks only.")
    parser.add_argument("--skip-commander", action="store_true", help="Skip Commander read-only stage.")
    parser.add_argument("--skip-joint-test", action="store_true", help="Skip joint-test shell stage.")
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    if args.list_ports:
        ports = list_ports()
        if not ports:
            print("No matching USB serial ports found.")
        else:
            print("\n".join(ports))
        return 0

    selected_stages = STAGES if args.all else ([args.stage] if args.stage else [])
    if not selected_stages:
        parser.print_help()
        return 0

    results = [run_stage(args, stage) for stage in selected_stages]
    classifications, blockers = final_classification(results)
    text = render_report(args, results, classifications, blockers)
    report = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "port": args.port or ("not opened" if args.dry_run else choose_port(args.port)),
        "mode": "dry-run" if args.dry_run else "serial",
        "baud": args.baud,
        "stages": [asdict(result) for result in results],
        "final_classification": classifications,
        "final_blockers": blockers,
    }
    write_reports(args, report, text)
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
