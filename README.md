# Identity: Team Euler Autonomous Hexapod Rover

Identity is a six-legged autonomous rover built by Team Euler for the COSGC robotics challenge. The platform uses Python control software on a Raspberry Pi 3B+, Arduino/C++ firmware on an Arduino Nano sensor hub, six Feetech STS3215 servos, eight HC-SR04 ultrasonic sensors, and a BNO085 IMU to test terrain-adaptive locomotion with a simple single-actuator-per-leg design.

The core research question was whether rough-terrain adaptation could be handled with a small set of gait parameters: Buehler-clock duty cycle, impact window, phase offsets, and a global speed setpoint, without per-joint trajectory planning or teleoperation.

## Highlights

- Dual-process Brain/Heart software architecture on a Raspberry Pi 3B+
- 30 Hz Heart loop for gait timing, servo commands, safety governors, and fault handling
- Approximately 10 Hz Brain loop for sensor acquisition, terrain classification, and navigation
- Arduino/C++ sensor firmware for ultrasonic timing and IMU polling
- Terrain-adaptive gait overlays for flat ground, rough terrain, deep sand, and inclines
- Tripod, wave, and quadruped gait patterns using a Buehler-clock CPG
- Simulation-backed validation with kinematic, terrain, and navigation test suites
- Formal field validation across seven terrain categories

## Results

Formal validation produced 31 successful autonomous traversals out of 32 trials, an observed 96.9% traversal success rate across tile, carpet, packed earth, loose sand, gravel, stone fields, and 20-degree inclines.

The symposium paper treats this as a pilot validation phase rather than a fully powered statistical proof: the 95% confidence interval lower bound falls below 90%, and the exact per-trial software hashes were not preserved. Within that scope, the results support high observed traversal reliability and show that the fixed hardware setup could adapt across seven terrain categories by changing gait choice, impact window, duty cycle, and speed.

## Hardware

| Component | Role |
| --- | --- |
| Raspberry Pi 3B+ | Main controller running Python navigation and gait logic |
| Arduino Nano | C++ sensor hub firmware for ultrasonic ranging and IMU polling |
| Feetech STS3215 servos x6 | Single-actuator leg drive using serial bus control |
| HC-SR04 ultrasonic sensors x8 | Obstacle and cliff-distance sensing |
| BNO085 IMU x1 | Fused orientation for terrain and safety logic |
| 3S LiPo, 3000 mAh | 11.1 V nominal power source |
| PETG chassis and C-shaped legs | 3D-printed compliant hexapod body and legs |
| TPU feet | Improved ground contact and traction |

Measured platform dimensions reported in the symposium paper:

- 511 mm body length
- 280 mm total width
- 74 mm ground clearance
- 2 to 3 kg total mass

## Software Architecture

The software uses a Brain/Heart split so navigation logic cannot block motor timing. Python handles the rover control stack on the Raspberry Pi, while Arduino/C++ firmware handles timing-sensitive sensor acquisition on the Nano.

- `final_full_gait_test.py`: main Python autonomous gait engine and state machine
- `final_sensors.ino`: Arduino/C++ Nano sensor firmware
- `Detection_SensorHub_FINAL.ino`: final Arduino/C++ sensor-hub firmware variant
- `fusion.py`, `fusion2.py`: sensor interpretation and classification support
- `input_thread.py`, `input_thread2.py`: serial input handling
- `sim_verify.py`: kinematic and gait-engine checks
- `sim_terrain.py`: terrain and governor stress scenarios
- `sim_nav.py`: navigation FSM tests with synthetic sensor frames
- `parse_telemetry.py`, `analyze_run_log.py`: telemetry and log analysis tools

## Gait And Navigation

The gait engine follows a RHex-style Buehler clock. Duty cycle defines stance fraction, phase offsets define inter-leg timing, and a global speed setpoint controls phase progression.

Implemented gait patterns:

- Tripod: fast locomotion with two groups of three legs
- Wave: slower, high-contact gait for rough terrain and inclines
- Quadruped: diagonal-pair gait balancing speed and stability

The autonomous navigation layer uses an eight-state finite state machine for forward motion, slow approach, arc turns, backing up, pivot turns, recovery wiggle, and safe stop behavior. Terrain classification uses IMU orientation, servo telemetry, and ultrasonic stability to select overlays for flat ground, rough terrain, deep sand, and incline traversal.

## Simulation

The simulation framework contains 40 automated checks:

- 14 kinematic and gait checks in `sim_verify.py`
- 14 terrain and governor scenarios in `sim_terrain.py`
- 12 navigation FSM categories in `sim_nav.py`

At the time of the symposium paper, the full suite passed 40/40 checks. The simulation validates timing, state transitions, terrain overlays, and control invariants, but it does not model all physical effects such as compliance, backlash, or terrain deformation.

Run the simulation checks:

```bash
python3 sim_verify.py
python3 sim_terrain.py
python3 sim_nav.py
```

## Releases

- [Competition Build](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/Competition): release snapshot used for the competition build.
- [V0.5 Full Program](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/pre-release): earlier semi-working autonomous rover program release.

## Demo Videos

- [Home Tripod Wave Test](https://youtube.com/shorts/x6369QqBHbY?feature=share)
- [Full Gait Test 3/5/26](https://youtu.be/S6lwZjQxPko)

## Resume Summary

Built and validated a six-legged autonomous rover with Python-based Raspberry Pi control, Arduino/C++ sensor firmware, terrain-adaptive gait overlays, Brain/Heart process architecture, and simulation-backed navigation tests. Formal field validation showed 31/32 successful traversals across seven terrain categories.

## Notes

This repository is a cleaned public portfolio version of the Team Euler rover codebase. Hardware operation requires calibrated servos, a connected sensor hub, and appropriate safety checks before powering the robot.
