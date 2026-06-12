#!/usr/bin/env python3
"""PAROL6 Commander/GUI protocol helpers for safe host-side probing."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import json
from pathlib import Path
import re
from typing import Any


START_BYTES = bytes([0xFF, 0xFF, 0xFF])
END_BYTES = bytes([0x01, 0x02])
HOST_TO_FIRMWARE_LEN = 52
FIRMWARE_TO_HOST_LEN = 56
HOST_CRC_CONVENTION_BYTE = 228
DEFAULT_SAFE_MAIN_BAUD = 115200


class ProtocolError(ValueError):
    """Raised when packet bytes do not match the confirmed protocol shape."""


class SafetyError(ValueError):
    """Raised when a command is not allowed by the safe host probe policy."""


class CommandRisk(str, Enum):
    READ_ONLY = "READ_ONLY"
    LOW = "LOW"
    MEDIUM = "MEDIUM"
    HIGH = "HIGH"
    DANGEROUS = "DANGEROUS"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class CommandInfo:
    id: int
    risk: CommandRisk
    meaning: str
    status: str
    source: str
    source_file: str
    source_line_function: str
    allowed_in_safe_host_probe: bool
    notes: str


def command_map_path() -> Path:
    return Path(__file__).with_name("commander_protocol_map.json")


def load_protocol_map(path: str | Path | None = None) -> dict[str, Any]:
    map_path = Path(path) if path is not None else command_map_path()
    with map_path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_command_map(path: str | Path | None = None) -> dict[int, CommandInfo]:
    protocol_map = load_protocol_map(path)
    commands: dict[int, CommandInfo] = {}
    for raw in protocol_map["commands"]:
        commands[int(raw["id"])] = CommandInfo(
            id=int(raw["id"]),
            risk=CommandRisk(raw["risk"]),
            meaning=raw["meaning"],
            status=raw["status"],
            source=raw["source"],
            source_file=raw["source_file"],
            source_line_function=raw["source_line_function"],
            allowed_in_safe_host_probe=bool(raw["allowed_in_safe_host_probe"]),
            notes=raw["notes"],
        )
    return commands


def get_command_info(command_id: int) -> CommandInfo:
    commands = load_command_map()
    return commands.get(
        int(command_id),
        CommandInfo(
            id=int(command_id),
            risk=CommandRisk.UNKNOWN,
            meaning="Unknown Commander command.",
            status="not_confirmed",
            source="not_confirmed",
            source_file="",
            source_line_function="",
            allowed_in_safe_host_probe=False,
            notes="Unknown commands are blocked by safe main and refused by this host probe.",
        ),
    )


def explain_command(command_id: int) -> str:
    info = get_command_info(command_id)
    lines = [
        f"command={info.id}",
        f"risk={info.risk.value}",
        f"status={info.status}",
        f"meaning={info.meaning}",
        f"source={info.source}",
    ]
    if info.source_file:
        lines.append(f"source_file={info.source_file}")
    if info.source_line_function:
        lines.append(f"source_line_function={info.source_line_function}")
    lines.append(f"allowed_in_safe_host_probe={str(info.allowed_in_safe_host_probe).lower()}")
    lines.append(f"notes={info.notes}")
    return "\n".join(lines)


def validate_safe_command(command_id: int, *, allow_low: bool = False) -> CommandInfo:
    info = get_command_info(command_id)
    if info.risk == CommandRisk.READ_ONLY:
        return info
    if allow_low and info.risk == CommandRisk.LOW:
        return info
    raise SafetyError("Refusing to send non-read-only command in safe probe.")


def bytes_to_hex(data: bytes | bytearray) -> str:
    return " ".join(f"{byte:02X}" for byte in data)


def hex_to_bytes(value: str) -> bytes:
    cleaned = re.sub(r"[^0-9a-fA-F]", "", value)
    if len(cleaned) % 2 != 0:
        raise ProtocolError("Hex string must contain an even number of hex digits.")
    return bytes.fromhex(cleaned)


def _encode_signed(value: int, width: int) -> bytes:
    bits = width * 8
    minimum = -(1 << (bits - 1))
    maximum = (1 << (bits - 1)) - 1
    if value < minimum or value > maximum:
        raise ProtocolError(f"{value} does not fit in signed {bits}-bit field.")
    if value < 0:
        value = (1 << bits) + value
    return value.to_bytes(width, byteorder="big", signed=False)


def _decode_signed(data: bytes) -> int:
    value = int.from_bytes(data, byteorder="big", signed=False)
    sign_bit = 1 << (len(data) * 8 - 1)
    if value & sign_bit:
        value -= 1 << (len(data) * 8)
    return value


def _bits_msb_first(byte_value: int) -> list[int]:
    return [(byte_value >> shift) & 1 for shift in range(7, -1, -1)]


def _bits_to_byte_msb_first(bits: list[int] | tuple[int, ...]) -> int:
    if len(bits) != 8:
        raise ProtocolError("Bitfield must contain exactly 8 bits.")
    value = 0
    for bit in bits:
        if bit not in (0, 1, False, True):
            raise ProtocolError("Bitfield entries must be 0/1 or bool.")
        value = (value << 1) | int(bit)
    return value


def encode_commander_packet(
    command_id: int,
    *,
    positions: list[int] | tuple[int, ...] | None = None,
    speeds: list[int] | tuple[int, ...] | None = None,
    affected_joint: int = 0,
    inout_bits: list[int] | tuple[int, ...] | None = None,
    timeout: int = 0,
    gripper_position: int = 0,
    gripper_speed: int = 0,
    gripper_current: int = 0,
    gripper_command: int = 0,
    gripper_mode: int = 0,
    gripper_id: int = 0,
    crc_or_convention_byte: int = HOST_CRC_CONVENTION_BYTE,
    allow_low: bool = False,
) -> bytes:
    """Encode a host-to-firmware packet after safe-probe command validation."""
    validate_safe_command(command_id, allow_low=allow_low)
    positions = list(positions if positions is not None else [0] * 6)
    speeds = list(speeds if speeds is not None else [0] * 6)
    if len(positions) != 6 or len(speeds) != 6:
        raise ProtocolError("positions and speeds must each contain exactly 6 values.")
    if inout_bits is None:
        inout = 0
    else:
        inout = _bits_to_byte_msb_first(list(inout_bits))

    one_byte_fields = {
        "command_id": command_id,
        "affected_joint": affected_joint,
        "inout": inout,
        "timeout": timeout,
        "gripper_command": gripper_command,
        "gripper_mode": gripper_mode,
        "gripper_id": gripper_id,
        "crc_or_convention_byte": crc_or_convention_byte,
    }
    for name, value in one_byte_fields.items():
        if value < 0 or value > 255:
            raise ProtocolError(f"{name} must fit in one byte.")

    payload = bytearray()
    for value in positions:
        payload.extend(_encode_signed(int(value), 3))
    for value in speeds:
        payload.extend(_encode_signed(int(value), 3))
    payload.append(command_id)
    payload.append(affected_joint)
    payload.append(inout)
    payload.append(timeout)
    payload.extend(_encode_signed(int(gripper_position), 2))
    payload.extend(_encode_signed(int(gripper_speed), 2))
    payload.extend(_encode_signed(int(gripper_current), 2))
    payload.append(gripper_command)
    payload.append(gripper_mode)
    payload.append(gripper_id)
    payload.append(crc_or_convention_byte)
    payload.extend(END_BYTES)

    if len(payload) != HOST_TO_FIRMWARE_LEN:
        raise ProtocolError(f"Internal encoder error: payload length {len(payload)} != {HOST_TO_FIRMWARE_LEN}.")
    return START_BYTES + bytes([HOST_TO_FIRMWARE_LEN]) + bytes(payload)


def encode_readonly_packet(command_id: int) -> bytes:
    return encode_commander_packet(command_id)


def _split_frame(packet: bytes) -> tuple[int, bytes]:
    if len(packet) < 6:
        raise ProtocolError("Packet too short.")
    if not packet.startswith(START_BYTES):
        raise ProtocolError("Packet does not start with ff ff ff.")
    length = packet[3]
    expected_total = 4 + length
    if len(packet) != expected_total:
        raise ProtocolError(f"Packet length mismatch: length byte says {length}, total should be {expected_total}.")
    payload = packet[4:]
    if not payload.endswith(END_BYTES):
        raise ProtocolError("Packet does not end with 01 02.")
    return length, payload


def decode_host_packet(packet: bytes) -> dict[str, Any]:
    length, payload = _split_frame(packet)
    if length != HOST_TO_FIRMWARE_LEN:
        raise ProtocolError(f"Expected host-to-firmware length {HOST_TO_FIRMWARE_LEN}, got {length}.")
    return {
        "direction": "host_to_firmware",
        "positions": [_decode_signed(payload[i : i + 3]) for i in range(0, 18, 3)],
        "speeds": [_decode_signed(payload[i : i + 3]) for i in range(18, 36, 3)],
        "command": payload[36],
        "affected_joint": payload[37],
        "inout": payload[38],
        "inout_bits": _bits_msb_first(payload[38]),
        "timeout": payload[39],
        "gripper_position": _decode_signed(payload[40:42]),
        "gripper_speed": _decode_signed(payload[42:44]),
        "gripper_current": _decode_signed(payload[44:46]),
        "gripper_command": payload[46],
        "gripper_mode": payload[47],
        "gripper_id": payload[48],
        "crc_or_convention_byte": payload[49],
    }


def decode_response_packet(packet: bytes) -> dict[str, Any]:
    length, payload = _split_frame(packet)
    if length != FIRMWARE_TO_HOST_LEN:
        raise ProtocolError(f"Expected firmware-to-host length {FIRMWARE_TO_HOST_LEN}, got {length}.")
    return {
        "direction": "firmware_to_host",
        "positions": [_decode_signed(payload[i : i + 3]) for i in range(0, 18, 3)],
        "speeds": [_decode_signed(payload[i : i + 3]) for i in range(18, 36, 3)],
        "homed_bits": _bits_msb_first(payload[36]),
        "io_bits": _bits_msb_first(payload[37]),
        "temperature_error_bits": _bits_msb_first(payload[38]),
        "position_error_bits": _bits_msb_first(payload[39]),
        "timing_data": int.from_bytes(payload[40:42], byteorder="big", signed=False),
        "timeout_error": payload[42],
        "xtr2_command_echo": payload[43],
        "gripper_id": payload[44],
        "gripper_position": _decode_signed(payload[45:47]),
        "gripper_speed": _decode_signed(payload[47:49]),
        "gripper_current": _decode_signed(payload[49:51]),
        "gripper_status": payload[51],
        "object_detection": payload[52],
        "crc_or_convention_byte": payload[53],
    }


def decode_response_compat(packet: bytes) -> dict[str, Any]:
    """Decode a firmware response as far as the confirmed map allows.

    The full firmware-to-host packet shape is confirmed from Pack_data(), but
    host-side captures may be partial or noisy. This helper keeps raw bytes
    visible and marks unknown fields instead of pretending malformed captures
    are valid response facts.
    """
    result: dict[str, Any] = {
        "raw_hex": bytes_to_hex(packet),
        "raw_length": len(packet),
        "start_bytes_expected": bytes_to_hex(START_BYTES),
        "end_bytes_expected": bytes_to_hex(END_BYTES),
        "start_bytes_present": packet.startswith(START_BYTES),
        "end_bytes_present": packet.endswith(END_BYTES),
        "format_status": "partial_or_unknown",
        "message": "Response format partially confirmed; unknown fields are shown as raw bytes.",
        "unknown_raw_bytes": bytes_to_hex(packet),
    }
    if len(packet) >= 4:
        result["length_byte"] = packet[3]
        result["expected_total_length"] = 4 + packet[3]
    else:
        result["length_byte"] = "unknown"
        result["expected_total_length"] = "unknown"

    try:
        decoded = decode_response_packet(packet)
    except ProtocolError as exc:
        result["decode_error"] = str(exc)
        return result

    result.update(decoded)
    result["format_status"] = "confirmed_firmware_to_host"
    result["message"] = "Response format fully decoded for confirmed Pack_data() fields."
    result.pop("unknown_raw_bytes", None)
    return result


def decode_packet(packet: bytes) -> dict[str, Any]:
    length, _payload = _split_frame(packet)
    if length == HOST_TO_FIRMWARE_LEN:
        return decode_host_packet(packet)
    if length == FIRMWARE_TO_HOST_LEN:
        return decode_response_packet(packet)
    raise ProtocolError(f"Unsupported packet length byte {length}. Confirmed lengths are 52 and 56.")


def safest_readonly_command_id(path: str | Path | None = None) -> int:
    commands = load_command_map(path)
    for command_id in sorted(commands):
        info = commands[command_id]
        if info.risk == CommandRisk.READ_ONLY and info.allowed_in_safe_host_probe:
            return command_id
    raise SafetyError("No confirmed READ_ONLY Commander command is available.")
