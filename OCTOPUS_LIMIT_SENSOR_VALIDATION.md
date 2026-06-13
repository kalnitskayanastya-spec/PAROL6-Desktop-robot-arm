# PAROL6 Octopus Physical Limit Sensor Validation

## Hardware Used

- BIGTREETECH Octopus Pro F446
- 8-channel PC817 optocoupler isolation board
- Six physical limit/homing sensors, S1-S6
- Lab PSU for sensor/opto input side
- Octopus connected by USB
- 24V motor power disconnected

## Firmware Used

- `octopus_parol6_limit_opto_diag_0002`

## Test Path

Each sensor was tested individually through the already validated electrical path:

`sensor -> optocoupler IN1/G -> optocoupler V1/G -> Octopus DIAG0 / PG6`

This test reuses one known-good optocoupler input/output channel and one known-good Octopus input:

- Sensor side: optocoupler `IN1/G`
- Isolated output side: optocoupler `V1/G`
- Octopus input: `DIAG0 / PG6`
- DIAG 5V was not connected for this isolated output test.

## Sensor Validation Table

| Sensor ID | Sensor type | Test path | Expected inactive | Expected triggered | Result |
|---|---|---|---|---|---|
| S1 | 4mm NPN NO inductive sensor | IN1/G -> V1/G -> DIAG0 | OPEN / raw=111111 | TRIGGERED / raw=011111 | OK |
| S2 | GX-F8A inductive sensor | IN1/G -> V1/G -> DIAG0 | OPEN / raw=111111 | TRIGGERED / raw=011111 | OK |
| S3 | M5 NPN NO inductive sensor | IN1/G -> V1/G -> DIAG0 | OPEN / raw=111111 | TRIGGERED / raw=011111 | OK |
| S4 | Mechanical limit switch #1 | IN1/G -> V1/G -> DIAG0 | OPEN / raw=111111 | TRIGGERED / raw=011111 | OK |
| S5 | Mechanical limit switch #2 | IN1/G -> V1/G -> DIAG0 | OPEN / raw=111111 | TRIGGERED / raw=011111 | OK |
| S6 | Mechanical limit switch #3 | IN1/G -> V1/G -> DIAG0 | OPEN / raw=111111 | TRIGGERED / raw=011111 | OK |

## Observed Logic

- Inactive state: `OPEN`
- Triggered state: `TRIGGERED`
- Raw inactive state is normally `111111`.
- When a tested sensor is triggered through DIAG0, raw state is `011111`.
- Active LOW behavior is confirmed.
- Firmware default should remain `active_low on` for this diagnostic path.

## Validation Scope

This validates the six physical sensors electrically through the optocoupler and Octopus DIAG input path. It does not yet validate final mechanical mounting, final channel assignment, joint assignment, home direction, travel limits, collision clearance, or homing safety.

## Next Steps

- Assign S1-S6 to Joint1-Joint6.
- Assign final optocoupler channels CH1-CH6.
- Verify final wiring:
  - `S1 -> CH1 -> DIAG0`
  - `S2 -> CH2 -> DIAG1`
  - `S3 -> CH3 -> DIAG2`
  - `S4 -> CH4 -> DIAG3`
  - `S5 -> CH5 -> DIAG4`
  - `S6 -> CH6 -> DIAG5`
- Only after final wiring and mechanical placement, create one-joint homing diagnostic firmware.
