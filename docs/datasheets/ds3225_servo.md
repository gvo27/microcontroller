# DS3225 / DS3225MG Digital Servo (DSSERVO)

Sources: manufacturer datasheet (DSSERVO, 2 pages) + retailer listing. Datasheet values take precedence where they conflict; conflicts are listed under `conflicts`.

## identity
- manufacturer: Dongguan City Dsservo Technology Co., Ltd
- model: DS3225 (retail: DS3225MG)
- type: digital, high-torque, waterproof RC servo
- rated_class: 6V / 25 kg
- vendor_site: www.dsservo.com (datasheets and 3D models)

## mechanical
- size_mm: 40 x 20 x 40.5 (L x W x H, body)
- weight_g: 60 (datasheet) | 67 (listing)
- gear_ratio: 275
- gear_material: metal (hard-anodized, per listing)
- case: CNC aluminum middle shell (per listing)
- bearing: double bearing
- motor: 3-pole (datasheet) | "DC motor" (listing)
- waterproof: IP66
- lead_length_mm: 300 +/- 5
- drawing_dims_mm: mounting-tab span 54.5; hole spacing 49.5; horn arm length 24; tab width 10; body height to tab 27.7; total height incl. horn 40.4
- mount_holes: 4, in end tabs (see datasheet drawing page 1 for exact positions; for SolidWorks, download the 3D model from dsservo.com)

## environment
- operating_voltage_v: 4.8 to 6.8
- operating_temp_c: -15 to 70
- storage_temp_c: -30 to 80

## electrical
| parameter | 5.0 V | 6.8 V |
| --- | --- | --- |
| idle_current_mA | 4 | 5 |
| no_load_speed_s_per_60deg | 0.15 | 0.13 |
| stall_torque_kg_cm | 21 | 24.5 (datasheet) / 25 (listing) |
| stall_current_A | 1.9 | 2.3 |

## control
- signal: PWM, pulse-width encoded position
- signal_level_v: 3.3 to 5 (3.3 V logic OK, e.g. ESP32 GPIO directly)
- pulse_width_us: 500 to 2500
- neutral_us: 1500
- deadband_us: 3
- frame_frequency_hz: 50 to 330 (listing says 333)
- direction: counterclockwise as pulse width increases (500 -> 2500 us)
- rotation_range_deg: 180 (this unit; confirmed by owner)
- angle_map: 500 us = 0deg, 1500 us = 90deg (neutral), 2500 us = 180deg
- angle_formula: pulse_us = 500 + (angle_deg / 180) * 2000
- resolution_note: deadband of 3 us corresponds to about 0.27 deg
- note: the datasheet also describes a 270 deg variant (2500 us = 270deg); that variant does not apply to this unit

## wiring
- connector: standard 3-wire servo lead (signal / power / ground)
- wire_colors: not specified in the datasheet; verify with a multimeter or the vendor listing before powering
- power_budget: size the supply for stall current, not idle. Allow at least 2.5 A at 6.8 V (2.3 A stall + margin) on a dedicated BEC. Do not power from the Pi 5 rail.
- grounds: tie servo supply ground to controller ground (common ground), or the signal has no reference.

## conflicts
- weight: 60 g (datasheet) vs 67 g (listing)
- stall torque at 6.8 V: 24.5 kg-cm (datasheet) vs 25 kg-cm (listing; likely rounded)
- max frequency: 330 Hz (datasheet) vs 333 Hz (listing)
- rotation range: datasheet lists 180 or 270 deg; listing says 180 only. Resolved: owner's unit is 180 deg.
- motor: 3-pole (datasheet) vs "DC motor" (listing); not contradictory, 3-pole is a type of DC motor
- listing typos ignored: pulse widths given as "/sec" should read microseconds (us); "Pulse width modification" should read modulation

## project notes (self-driving RC car)
- role: steering servo, driven by ESP32 PWM at 50 Hz
- recommended supply: 6.0 to 6.8 V from a dedicated BEC, separate from the Pi 5 regulator
- use 1500 us as the straight-ahead center and set mechanical end stops in firmware, so the steering linkage never drives the servo into a stall
