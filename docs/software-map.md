# Software Map

[Back to main README](../README.md)

Software reference separating active runtime files from calibration, simulation, analysis, and regression support. See [`../RUNNING.md`](../RUNNING.md) for setup and command examples.

## Runtime And Gait Control

| File | Purpose |
| --- | --- |
| [`final_full_gait_test.py`](../final_full_gait_test.py) | Main Python autonomous gait engine and navigation FSM |
| [`offset_full_gait_test_v2.py`](../offset_full_gait_test_v2.py) | Earlier v2 gait-engine implementation retained for comparison and development history |
| [`final_full_gait_test_tripod_default.py`](../final_full_gait_test_tripod_default.py) | Tripod-default final gait-engine variant for comparison and fallback testing |
| [`home_tripod_wave_test.py`](../home_tripod_wave_test.py) | Legs-home sequence followed by forward tripod and forward wave gait checks |

## Main Program Modes

`final_full_gait_test.py` supports hardware runs, dry runs, subsystem diagnostics, and gait-specific checks:

| Mode | Purpose |
| --- | --- |
| `sudo python3 final_full_gait_test.py` | Full maneuver demo across the implemented gaits and stance behaviors |
| `sudo python3 final_full_gait_test.py --competition` | Autonomous navigation with the eight-state FSM, Arduino sensor stream, terrain adaptation, and live terminal state output |
| `sudo python3 final_full_gait_test.py --competition-dry-run` | Autonomous navigation logic with sensors active and servo speed held at zero |
| `sudo python3 final_full_gait_test.py --test-competition` | Timed fallback sequence with no sensor dependency |
| `sudo python3 final_full_gait_test.py --test-tripod`, `--test-quad`, or `--test-wave` | Gait-specific forward, turning, pivot, and reverse checks |
| `sudo python3 final_full_gait_test.py --test-recovery` | Recovery wiggle and self-right behavior checks |
| `sudo python3 final_full_gait_test.py --test-arduino`, `--test-sensors`, or `--test-nav` | Arduino serial, sensor-processing, and full navigation-pipeline diagnostics |
| `sudo python3 final_full_gait_test.py --drift-test <gait>` | Heading-drift measurement for gait tuning |
| `--no-verbose-telemetry` | Optional logging flag for reducing extended servo, sensor, and navigation output |

## Sensor Firmware And Input

| File | Purpose |
| --- | --- |
| [`final_sensors.ino`](../final_sensors.ino) | Arduino/C++ Nano sensor firmware for ultrasonic and IMU data collection |
| [`Detection_SensorHub_FINAL.ino`](../Detection_SensorHub_FINAL.ino) | Alternate final Arduino/C++ sensor-hub firmware variant |
| [`fusion.py`](../fusion.py), [`fusion2.py`](../fusion2.py) | Sensor interpretation and classification support |
| [`input_thread.py`](../input_thread.py), [`input_thread2.py`](../input_thread2.py) | Serial input handling |

## Calibration And Configuration

| File | Purpose |
| --- | --- |
| [`auto_calibrate.py`](../auto_calibrate.py) | Automated setup and tuning calibration helper |
| [`calibrate_homes.py`](../calibrate_homes.py) | Servo home-position calibration utility |
| [`calibrate_legs.py`](../calibrate_legs.py) | Leg calibration utility for physical alignment |
| [`validate_config.py`](../validate_config.py) | Configuration validation before running the rover |

## Simulation, Analysis, And Tests

| File | Purpose |
| --- | --- |
| [`sim_verify.py`](../sim_verify.py) | Kinematic and gait-engine checks |
| [`sim_terrain.py`](../sim_terrain.py) | Terrain and governor stress scenarios |
| [`sim_nav.py`](../sim_nav.py) | Navigation FSM tests with synthetic sensor frames |
| [`monte_carlo_terrain.py`](../monte_carlo_terrain.py) | Monte Carlo terrain simulation and robustness exploration |
| [`rover_statics.py`](../rover_statics.py) | Static geometry/load analysis support |
| [`gait_viz.py`](../gait_viz.py) | Gait visualization helper |
| [`fsm_audit.py`](../fsm_audit.py) | Navigation FSM audit/support script |
| [`load_monitor.py`](../load_monitor.py) | Servo load monitoring support |
| [`param_sweep.py`](../param_sweep.py), [`sweep_hz_governor.py`](../sweep_hz_governor.py), [`sweep_stall_threshold.py`](../sweep_stall_threshold.py) | Parameter and governor sweep tooling |
| [`parse_telemetry.py`](../parse_telemetry.py), [`analyze_run_log.py`](../analyze_run_log.py) | Telemetry and run-log analysis |
| [`test_governor_ff_budget.py`](../test_governor_ff_budget.py), [`test_nav_logic.py`](../test_nav_logic.py), [`test_nav_serial.py`](../test_nav_serial.py), [`test_sensor_classification.py`](../test_sensor_classification.py) | Pytest regression tests |
