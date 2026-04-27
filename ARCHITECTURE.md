# Software Architecture

Identity uses a small, auditable control stack instead of learned locomotion or high-degree-of-freedom trajectory planning. The rover has six powered degrees of freedom, one per C-leg, and uses gait timing, phase offsets, speed commands, and terrain overlays to adapt behavior.

## Runtime Overview

`final_full_gait_test.py` is the main runtime program. It combines the Python gait engine, autonomous navigation state machine, terrain classification, safety governors, and terminal telemetry output.

The runtime is split into two cooperating process roles:

- Brain: sensor interpretation, navigation decisions, terrain classification, obstacle/cliff logic, and high-level gait selection.
- Heart: timing-critical 30 Hz gait loop, Buehler-clock phase progression, servo command generation, smoothing, and safety governors.

This split keeps slower navigation work from blocking the motor-control loop.

## Sensor Pipeline

`final_sensors.ino` runs on the Arduino Nano and handles ultrasonic and IMU timing. It streams a 20-column CSV frame to the Raspberry Pi at about 10 Hz.

The Python side parses this stream, classifies obstacle sectors, checks cliff/drop-off sensors, reads orientation state, and updates the navigation finite state machine.

Relevant files:

- `final_sensors.ino`: primary Arduino/C++ sensor firmware.
- `Detection_SensorHub_FINAL.ino`: alternate final sensor-hub firmware variant.
- `fusion.py`, `fusion2.py`: sensor interpretation and classification support.
- `input_thread.py`, `input_thread2.py`: serial input handling.

## Gait Engine

The gait engine follows a RHex-style Buehler clock. Each gait is defined by duty cycle, phase offsets, impact window, and speed setpoint.

Implemented gait families:

- Tripod: faster locomotion on flat and moderate terrain.
- Quadruped: stability/speed balance.
- Wave: higher-contact gait for inclines, rough terrain, and recovery-style traversal.

The active final runtime is `final_full_gait_test.py`. Earlier or specialized variants are retained for comparison and development history.

## Navigation FSM

The autonomous navigation layer uses an eight-state finite state machine for forward traversal, slow approach, arc turns, backing up, pivot turns, recovery wiggle, and safe stop behavior.

Inputs include:

- Ultrasonic obstacle sectors.
- Downward-facing cliff/drop-off readings.
- IMU pitch/roll orientation.
- Servo load telemetry.
- Watchdog and sensor-health state.

## Terrain Overlay

Terrain classification adjusts gait choice, speed, duty cycle, and impact window. The final overlay uses IMU pitch for slope handling, angular-rate and ultrasonic stability for rough terrain, sustained servo load for deep sand, and downward ultrasonic distance changes for cliff detection.

The phase-error governor reduces speed when commanded and measured servo phase diverge beyond the configured threshold.

## Safety Governors

The runtime includes guardrails for:

- Servo speed caps.
- Feedforward budget limits.
- Overload prevention timing.
- Stall threshold handling.
- Phase-error speed scaling.
- Brownout-aware speed limiting.
- Dry-run navigation checks.

Hardware operation still requires calibrated servos, confirmed sensor input, and a physical safety check before powering the rover.

## Simulation Coverage

The simulation suite is intentionally separate from the hardware loop so it can run in CI and on development machines.

Key entry points:

- `sim_verify.py`: gait timing, kinematic invariants, governor behavior, and recovery checks.
- `sim_terrain.py`: terrain-overlay and load/stall robustness scenarios.
- `sim_nav.py`: navigation FSM, parser, cliff detection, terrain overlay, and end-to-end scenario checks.

At the symposium-paper checkpoint, these scripts represented 40 passing checks.

## Calibration And Analysis Tools

Supporting scripts document setup, tuning, and review workflows:

- `auto_calibrate.py`, `calibrate_homes.py`, `calibrate_legs.py`: servo and leg calibration helpers.
- `validate_config.py`: configuration validation before running.
- `parse_telemetry.py`, `analyze_run_log.py`: telemetry and run-log analysis.
- `param_sweep.py`, `sweep_hz_governor.py`, `sweep_stall_threshold.py`: parameter tuning and sensitivity checks.
- `rover_statics.py`, `gait_viz.py`, `fsm_audit.py`, `load_monitor.py`: engineering analysis and audit helpers.
