# Limit / Opto Diagnostic CLI

`limit_opto_cli.py` is a Python 3 host-side helper for the firmware env `octopus_parol6_limit_opto_diag_0000`.

It sends simple USB Serial commands to read Octopus Stop0..Stop5 / optocoupler diagnostic state. It does not flash firmware and it does not move motors.

## Install pyserial

```bash
python3 -m pip install pyserial
```

If `pyserial` is missing, the CLI prints:

```text
Install with: python3 -m pip install pyserial
```

## List Ports

On macOS, the board usually appears as `/dev/cu.usbmodem...`.

```bash
python3 host_tools/limit_opto_cli.py --list-ports
```

You can pass a port manually:

```bash
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 status
```

Default baudrate is `115200`.

## Dry Run Examples

Dry run prints what would be sent without opening serial:

```bash
python3 host_tools/limit_opto_cli.py --dry-run status
python3 host_tools/limit_opto_cli.py --dry-run raw
python3 host_tools/limit_opto_cli.py --dry-run stream on
python3 host_tools/limit_opto_cli.py --dry-run invert on
python3 host_tools/limit_opto_cli.py --dry-run pullup off
python3 host_tools/limit_opto_cli.py --dry-run reset_counts
```

For `stream on`, dry run also shows that the CLI would later send `stream off`.

## Real Board Examples

```bash
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 safe
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 status
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 raw
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 counts
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 reset_counts
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 invert on
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 pullup off
python3 host_tools/limit_opto_cli.py --port /dev/cu.usbmodem1234 stream on --stream-seconds 5
```

Append a timestamped log:

```bash
python3 host_tools/limit_opto_cli.py --log host_tools/limit_opto_cli.log status
```

## Safety

- This CLI is input-only.
- It does not move motors.
- It does not enable motors.
- Never connect 24V directly to Octopus MCU inputs.
- Verify optocoupler output voltage before connecting to Stop inputs.
