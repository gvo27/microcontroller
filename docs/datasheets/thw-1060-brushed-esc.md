---
doc_type: user_manual_extract
product: THW-1060-BRUSHED
product_type: Water-proof brushed electronic speed controller (ESC), RTR version
source_pages_included: [protection_functions, intro_features_specs_wiring]
source_pages_excluded: [transmitter_setup_throttle_calibration_beeps_jumpers]
---

# THW-1060-BRUSHED ESC

## Safety notice (from manual)

- The power system for an RC model can be very dangerous; read the manual carefully.
- The manufacturer has no control over correct use, installation, application or maintenance.
- The manufacturer assumes no liability for damages, losses or costs resulting from use of the product.

## Features

1. Water-proof and dust-proof, for all-weather races.
2. Small size with built-in capacitor module.
3. Automatic throttle range calibration.
4. Multiple protections: low voltage cut-off (LiPo or NiMH), over-heat, throttle signal loss.
5. Easily programmed with jumpers.

## Specifications

| key | value |
| --- | --- |
| model | THW-1060-BRUSHED |
| continuous_current_forward_A | 60 |
| burst_current_forward_A | 360 |
| continuous_current_backward_A | 30 |
| burst_current_backward_A | 180 |
| input_battery | 2-3S LiPo, 5-9 cells NiMH |
| cars_applicable | 1:10 on-road, off-road, crawler |
| motor_limit_2S_LiPo_or_5-6_cells_NiMH | 540 or 550 size motor, >= 8T, or RPM < 45000 @ 7.2V |
| motor_limit_3S_LiPo_or_7-9_cells_NiMH | 540 or 550 size motor, >= 13T, or RPM < 30000 @ 7.2V |
| built_in_BEC_type | Switch mode |
| built_in_BEC_output | 3A / 6V |
| dimensions_mm | 36 x 30 x 18 |
| weight_g | 40 |

## Wiring

Connect the ESC, motor, receiver, battery and servo per the manual's diagram.

- The "+" and "-" wires of the ESC connect to the battery pack.
- WARNING: incorrect battery polarity will damage the ESC immediately.

| connection | wire / label | goes to |
| --- | --- | --- |
| Battery - | Black | Battery negative |
| Battery + | Red | Battery positive |
| Motor + | Yellow | Brushed motor |
| Motor - | Blue | Brushed motor |
| Control cable | Throttle signal lead | Receiver, "Throttle" channel |
| Servo | Plugs into receiver (not into ESC) | Receiver steering channel |

Other labeled parts in the diagram: Switch (OFF/ON), LED, Receiver, Servo, Brushed Motor.

## Protection functions

### 1. Low voltage cut-off (LVC)

- Trigger: battery pack voltage is below the threshold for 2 seconds; the ESC enters protection mode.
- Indication: when the car stops, the red LED blinks to indicate LVC has been activated.

| battery_type | output_reduces_50_percent_at_V | output_cuts_off_at_V | recoverable |
| --- | --- | --- | --- |
| 2S LiPo | 6.5 | 6.0 | no |
| 3S LiPo | 9.75 | 9.0 | no |
| 5-9 cells NiMH | 4.5 | 4.0 | no |

### 2. Over-heat protection

- Trigger: ESC internal temperature higher than 100 C (212 F) for 5 seconds.
- Action: the ESC reduces and then cuts off output power.
- Indication: when the car stops, the red LED blinks to indicate over-heat protection has been activated.
- Recovery: if the ESC cools down to 80 C (176 F), output power returns to normal.

### 3. Throttle signal loss protection

- Trigger: the throttle signal has been lost for 0.1 second.
- Action: the ESC cuts off output power.
- Note: the "Fail Save" function of the radio system is strongly recommended to be activated.
