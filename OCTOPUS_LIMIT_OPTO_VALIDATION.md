# PAROL6 Octopus Limit / Optocoupler Validation

## Hardware Used

- BIGTREETECH Octopus Pro F446
- 8-channel PC817 optocoupler isolation board
- LJ12A3 NPN NO 3-wire inductive proximity sensor
- Lab PSU set to 12V for sensor/opto input side
- Octopus connected by USB only
- 24V motor power disconnected

## Sensor Wiring

- Brown sensor wire = +12V from lab PSU
- Blue sensor wire = lab PSU GND
- Black sensor wire = NPN signal output

For the tested input channel:

- Lab PSU +12V -> optocoupler IN1 and brown sensor wire
- Sensor black wire -> G next to IN1
- Sensor blue wire -> lab PSU GND

## Optocoupler Output Wiring To Octopus

- Use output pair V1/G from optocoupler board for validation.
- V1 -> selected Octopus DIAG signal pin
- G -> selected Octopus DIAG GND
- Do not connect DIAG 5V.
- Lab PSU GND and Octopus GND are not directly tied for this isolated output test.

## Validated DIAG / Stop Table

| Stop  | Octopus DIAG | STM32 pin | Optocoupler output used in test | Inactive raw | Triggered raw | Result |
| ----- | ------------ | --------- | ------------------------------- | ------------ | ------------- | ------ |
| Stop0 | DIAG0        | PG6       | V1/G moved to DIAG0             | 111111       | 011111        | OK     |
| Stop1 | DIAG1        | PG9       | V1/G moved to DIAG1             | 111111       | 101111        | OK     |
| Stop2 | DIAG2        | PG10      | V1/G moved to DIAG2             | 111111       | 110111        | OK     |
| Stop3 | DIAG3        | PG11      | V1/G moved to DIAG3             | 111111       | 111011        | OK     |
| Stop4 | DIAG4        | PG12      | V1/G moved to DIAG4             | 111111       | 111101        | OK     |
| Stop5 | DIAG5        | PG13      | V1/G moved to DIAG5             | 111111       | 111110        | OK     |

## Logic

- Inactive state is normally raw `1`.
- Triggered state is raw `0`.
- Therefore the limit inputs are active LOW.
- Firmware default should be `active_low on`.
- Pullups should be enabled for this diagnostic setup.

## Firmware Used

- `octopus_parol6_limit_opto_diag_0002`

Commands used:

- `pullup on`
- `stream on --stream-seconds 10`
- `status`
- `counts`
- `reset_counts`

## Safety Notes

- No motors connected/used for this test.
- No CAN.
- No TMC.
- No homing.
- No step pulses.
- DIAG 5V was not connected.
- 24V motor power was not connected.
- Lab PSU was set to 12V and used only for sensor/opto input side.

## Homing Readiness Conclusion

Stop0-Stop5 input reading through optocoupler is validated. This does not yet mean homing is safe. Before homing, each real mounted sensor must be assigned to a physical joint, checked for correct placement, checked for active LOW behavior, and verified mechanically.

## Next Steps

- Assign real sensors to joints.
- Decide final optocoupler channels CH1-CH6.
- Build final wiring table:
  `Sensor -> Opto channel -> Octopus DIAG -> Joint -> Home direction`.
- Add debounce/count diagnostics to pre-homing checklist.
- Only then create one-joint-at-a-time homing diagnostic firmware.
