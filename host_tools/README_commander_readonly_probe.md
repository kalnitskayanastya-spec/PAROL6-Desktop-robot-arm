# PAROL6 Commander Read-Only Probe

Safe host-side helpers for exploring the original PAROL6 Commander/GUI serial protocol against `octopus_parol6_main_safe_0000`.

This tool is intentionally not a motion implementation. By default it only encodes/sends commands classified as `READ_ONLY`; `LOW` commands require `--allow-low`, and `HIGH`/`DANGEROUS`/unknown commands are always refused for real sends.

Examples:

```sh
python3 host_tools/commander_readonly_probe.py --explain-command 255
python3 host_tools/commander_readonly_probe.py --explain-command 100
python3 host_tools/commander_readonly_probe.py --dry-run --encode-readonly 255
python3 host_tools/commander_readonly_probe.py --decode-hex "00 01 02"
python3 host_tools/commander_readonly_probe.py --list-ports
```

The probe never sends homing, jog, go-to-position, repeatability/test motion, or motor-enable commands.
