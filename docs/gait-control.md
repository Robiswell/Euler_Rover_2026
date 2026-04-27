# Gait Control Reference

This page collects the gait-control details summarized in the main README: Buehler-clock timing, implemented gait modes, smart-servo feedback, gait visuals, and terrain overlays.

## Buehler-Clock Gait Model

The gait engine follows a RHex-style Buehler clock. Duty cycle defines stance fraction, phase offsets define inter-leg timing, and a global speed setpoint controls phase progression.

| Gait | Duty Cycle | Primary Use |
| --- | --- | --- |
| Tripod | 0.5 | Faster locomotion on flat and moderate terrain |
| Wave | 0.75 | Higher-contact gait for rough terrain and inclines |
| Quadruped | 0.7 | Balance of stability and speed |

## Smart-Servo Feedback For Gait Changes

The Feetech STS3215 smart servos were used as feedback sensors, not just as motors. The Heart process reads present position, load, and speed during the live gait loop, then rotates lower-rate telemetry reads for voltage, current, temperature, and hardware error flags.

| Servo Feedback | How It Changed The Gait System |
| --- | --- |
| Present position | Converted into actual leg phase and compared against the Buehler-clock target phase |
| Phase error | Triggered the phase-error governor to slow the gait before lag caused foot drag or lost ground clearance |
| Present load | Drove stall detection, self-right load monitoring, and terrain signals such as sustained heavy load in sand |
| Present speed | Showed when commanded motion was not translating into actual leg speed under load |
| Voltage, current, temperature, and error flags | Fed brownout, current-budget, thermal, and overload-protection limits |
| Post-run telemetry | Informed feedforward caps, walking speed limits, impact windows, duty cycles, and final gait transitions |

## Gait Pattern Visuals

The diagrams below show the servo grouping and top-view leg order used by the implemented gait modes.

| Tripod | Quadruped | Wave |
| --- | --- | --- |
| ![Tripod gait locomotion diagram](assets/gait-tripod.jpg) | ![Quadruped gait locomotion diagram](assets/gait-quadruped.jpg) | ![Wave gait locomotion diagram](assets/gait-wave.jpg) |

## Terrain Overlays

Terrain overlays adjust gait choice, impact window, duty cycle, and speed for flat ground, rough terrain, deep sand, and incline traversal. A phase-error governor reduces speed when commanded and measured servo phase diverge beyond the threshold used in the validation work.
