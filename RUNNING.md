# Running The Rover Code

This document is for reviewers or future team members who want to run the simulation checks, inspect the hardware launch modes, or understand what must be verified before powering the rover.

## Environment

Use Python 3.11 or newer for the simulation and analysis scripts.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Hardware runs were developed for a Raspberry Pi connected to the Feetech servo bus and Arduino Nano sensor hub. Simulation scripts can run without connected rover hardware.

## Simulation Checks

Run the same simulation suite used by the GitHub Actions workflow:

```bash
python3 sim_verify.py
python3 sim_terrain.py
python3 sim_nav.py
```

The expected checkpoint result is 40/40 passing checks across gait timing, terrain-overlay behavior, governor safety logic, and navigation FSM scenarios.

Run the pytest regression tests:

```bash
python3 -m pytest -q
```

## Hardware Modes

The main hardware entry point is `final_full_gait_test.py`.

```bash
sudo python3 final_full_gait_test.py --competition
sudo python3 final_full_gait_test.py --competition-dry-run
sudo python3 final_full_gait_test.py --test-tripod
sudo python3 final_full_gait_test.py --test-quad
sudo python3 final_full_gait_test.py --test-wave
sudo python3 final_full_gait_test.py --test-nav
```

Use `--competition-dry-run` before a powered autonomous run when checking sensor and navigation behavior. Dry-run mode keeps the navigation logic active while holding servo speed at zero.

## Arduino Sensor Firmware

The primary Arduino Nano firmware is `final_sensors.ino`. It reads the HC-SR04 ultrasonic sensors and BNO085 IMU, then streams a 20-column CSV frame to the Raspberry Pi at about 10 Hz.

`Detection_SensorHub_FINAL.ino` is retained as an alternate final sensor-hub firmware variant.

## Safety Checklist

Before powering the rover on the ground:

- Confirm servo home positions and leg clearance.
- Confirm the Feetech servo bus is connected through the FE-URT-1 interface.
- Confirm the Arduino sensor stream is active and parseable.
- Run the rover raised off the ground before a loaded run.
- Check battery voltage and wiring strain relief.
- Keep the workspace clear of hands, cables, and loose objects.
- Start with a dry run or gait-specific test before autonomous navigation.

## Notes For Reviewers

The simulation suite validates control invariants and navigation behavior, but it does not model all physical effects. Mechanical compliance, backlash, deformable terrain, and abrupt terrain transitions still require hardware judgment.
