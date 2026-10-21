project: autonomous_rc_car_platform
scope: infrastructure + comms layer (actuation, safety, host link, logging); ML models/sims live in separate repos deployed to the rpi5
chassis: tamiya_tt02_1:10
goals: [sensor_fusion, object_detection, semantic_slam, racing_trajectory_optimization]
compute: {high_level: raspberry_pi_5 (ros2, future), real_time: esp32-s3-n16r8}
phase: actuator_bringup  # host -usb-> esp32 -pwm-> servo+esc
next: ros2_on_rpi5 + micro_ros_on_esp32
stack:
  mcu: {build: platformio, platform: espressif32@6.9.0, framework: arduino-esp32-2.0.17, lang: c++gnu11}
  host: {lang: python>=3.11, deps: tools/requirements.txt, venv: .venv}
hw: {steer: ds3225mg@gpio4, throttle: thw1060_esc@gpio5, battery: 2s_lipo}
usb_ports: {left_USB: native, planned regulated power input; right_COM: ch343 uart0, flashing + host link}
layout:
  firmware/: esp32 firmware; rules in .claude/rules/firmware.md
  tools/rccar.py: host cli + Link/SessionLog/tuning api
  tools/esp32_session.py: interactive tuning/sensor scaffold
  tuning.toml: runtime tuning source of truth; [dev] knobs, [dev.calibration] measured, [warmup.dev] routines
  logs/: session jsonl, gitignored
  docs/: proposal, setup.md
  docs/datasheets/: archived datasheets + components_list.xlsx (ground truth inventory)
cmd: make help
invariants:
  - esp32 role: actuator pwm + host comms only; powers nothing
  - sensors on separate power rail, common ground, <=3.3v signals
  - pins: left header P1 only; pins.h == README table
  - protocol change => main.cpp + rccar.py + docs
  - tuning key change => tuning.toml + rccar.FIRMWARE_KEYS + main.cpp setKey
  - no unconfirmed parameters in tuning.toml
