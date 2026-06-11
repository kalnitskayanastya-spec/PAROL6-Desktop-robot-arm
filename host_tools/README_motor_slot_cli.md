# Motor Slot Diagnostic CLI

`motor_slot_cli.py` is a host-side Python 3 tool for talking to the safe diagnostic firmware `octopus_parol6_motor_slot_test_0000` over USB Serial.

It sends one firmware command, waits for the reply, and prints the response. It does not flash firmware.

## Install pyserial

```bash
python3 -m pip install pyserial
```

If `pyserial` is missing, the CLI prints:

```text
Install with: python3 -m pip install pyserial
```

## List Ports

On macOS the board usually appears as `/dev/cu.usbmodem...`.

```bash
python3 host_tools/motor_slot_cli.py --list-ports
```

You can also pass a port manually:

```bash
python3 host_tools/motor_slot_cli.py --port /dev/cu.usbmodem1234 status
```

Default baudrate is `115200`. Override it only if the firmware was built with a different serial speed:

```bash
python3 host_tools/motor_slot_cli.py --baud 115200 status
```

## Safe Commands

These commands do not require confirmation:

```bash
python3 host_tools/motor_slot_cli.py help
python3 host_tools/motor_slot_cli.py slots
python3 host_tools/motor_slot_cli.py where
python3 host_tools/motor_slot_cli.py safe
python3 host_tools/motor_slot_cli.py status
python3 host_tools/motor_slot_cli.py all_status
python3 host_tools/motor_slot_cli.py select 3
python3 host_tools/motor_slot_cli.py disable
python3 host_tools/motor_slot_cli.py emergency-disable
```

`emergency-disable` sends firmware command `disable` immediately.

## Commands That Require Confirmation

These commands can energize a driver, change direction, or move a motor:

```bash
python3 host_tools/motor_slot_cli.py enable
python3 host_tools/motor_slot_cli.py dir 1
python3 host_tools/motor_slot_cli.py step 10
python3 host_tools/motor_slot_cli.py jog -10
```

The CLI asks you to type `YES`. For scripted use, pass `--yes`:

```bash
python3 host_tools/motor_slot_cli.py enable --yes
python3 host_tools/motor_slot_cli.py jog 10 --yes
```

The CLI validates `select 0..5`, `dir 0|1`, and `abs(step/jog N) <= 200` before sending anything.

## Suggested Test Flow

With the board connected by USB and the diagnostic firmware already running:

```bash
python3 host_tools/motor_slot_cli.py slots
python3 host_tools/motor_slot_cli.py select 3
python3 host_tools/motor_slot_cli.py where
python3 host_tools/motor_slot_cli.py status
python3 host_tools/motor_slot_cli.py enable --yes
python3 host_tools/motor_slot_cli.py jog 10 --yes
python3 host_tools/motor_slot_cli.py disable
```

Before moving any motor connector, turn PSU output off. Never plug or unplug motors under power.

## Dry Run And Logging

Dry run prints what would be sent without opening serial:

```bash
python3 host_tools/motor_slot_cli.py --dry-run jog -10 --yes
```

Append a timestamped command/response log:

```bash
python3 host_tools/motor_slot_cli.py --log host_tools/motor_slot_cli.log status
```
