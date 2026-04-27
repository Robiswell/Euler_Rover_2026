# Identity: Team Euler Autonomous Hexapod Rover

![Identity rover banner](docs/assets/identity-project-banner.png)

[![Simulation Checks](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml/badge.svg)](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml)
[![Raspberry Pi/Python: Gait Control Program](https://img.shields.io/badge/Raspberry%20Pi%2FPython-Gait%20Control%20Program-2ea44f)](final_full_gait_test.py)
[![Arduino/C++: Sensor Firmware](https://img.shields.io/badge/Arduino%2FC%2B%2B-Sensor%20Firmware-00878F)](final_sensors.ino)
[![License: PolyForm Noncommercial 1.0.0](https://img.shields.io/badge/License-PolyForm%20Noncommercial%201.0.0-orange.svg)](LICENSE)

Identity is a six-legged autonomous rover built by Team Euler for the COSGC robotics challenge. It combines Python control software on a Raspberry Pi 3B+, Arduino/C++ sensor firmware on an Arduino Nano, six Feetech STS3215 smart servos with runtime telemetry, ultrasonic sensing, and IMU feedback to test low-cost rough-terrain locomotion.

The project explored whether a six-servo hexapod could adapt to rough outdoor terrain using a small set of interpretable gait parameters instead of complex multi-joint planning or learned locomotion. The key control variables were Buehler-clock duty cycle, impact window, phase offsets, and a global speed setpoint.

The final build combined field-tested hardware, simulation-backed control logic, and documented validation results showing 31/32 successful traversals across seven terrain categories.

## Quick Reviewer Path

| Reader | Start Here | What It Shows |
| --- | --- | --- |
| Recruiters and portfolio reviewers | [Results At A Glance](#results-at-a-glance) and [Recognition And Publications](#recognition-and-publications) | Project outcome, validation result, awards, and public deliverables |
| Robotics and controls reviewers | [`final_full_gait_test.py`](final_full_gait_test.py), [Gait Control](#gait-control), and [Simulation And Testing](#simulation-and-testing) | Brain/Heart control split, smart-servo feedback, terrain adaptation, and validation strategy |
| Hardware reviewers | [Hardware Stack](#hardware-stack) and [Bill Of Materials](#bill-of-materials) | Actuation, sensing, power, printed structure, tread design, and purchasing traceability |
| Media reviewers | [Media Gallery](docs/media.md) | Full field demo videos, course success runs, symposium materials, and award media |
| Reproducing or running code | [Software Map](#software-map) and [`RUNNING.md`](RUNNING.md) | Main runtime entry points, diagnostics, simulations, and launch commands |

## Table Of Contents

- [Quick Reviewer Path](#quick-reviewer-path)
- [Results At A Glance](#results-at-a-glance)
- [Repository Status](#repository-status)
- [My Role](#my-role)
- [System Architecture](#system-architecture)
- [Key Engineering Decisions](#key-engineering-decisions)
- [Hardware Stack](#hardware-stack)
  - [Bill Of Materials](#bill-of-materials)
  - [Measured Platform Geometry](#measured-platform-geometry)
  - [CAD Models](#cad-models)
- [Software Map](#software-map)
- [Gait Control](#gait-control)
- [Navigation And Terrain Adaptation](#navigation-and-terrain-adaptation)
- [Media Gallery](#media-gallery)
- [Recognition And Publications](#recognition-and-publications)
- [Simulation And Testing](#simulation-and-testing)
- [Releases](#releases)
- [Known Limits](#known-limits)

## Results At A Glance

| Metric | Result |
| --- | --- |
| Formal validation trials | 31/32 successful |
| Observed traversal success | 96.9% |
| Terrain categories | 7 |
| Simulation tests | 40/40 passing |
| Awards | 3 COSGC recognitions: Outstanding Demonstration of Creative Locomotion, Best Robotics Poster, and People's Choice Video |
| Heart loop rate | 30 Hz |
| Sensor update rate | ~10 Hz |
| Steady-state servo load margin | At least 42% in formal trials |

The symposium paper treats the 32-trial validation set as a pilot study because the 95% confidence interval lower bound falls below 90%. Within that scope, the data support high observed traversal reliability across tile, carpet, packed earth, loose sand, gravel, stone fields, and 20-degree inclines.

## Repository Status

This repository contains the final public code, validation previews, CAD references, release links, and documentation for the Team Euler rover build. Longer reference pages are collected in the [docs index](docs/README.md), and full-resolution videos and symposium PDFs are hosted in the [Portfolio Media Assets](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/media-assets) release to keep the repository lightweight.

| Area | Status |
| --- | --- |
| Main rover program | Final post-competition build is published, with cliff detection restored after the competition troubleshooting snapshot |
| Validated release history | Release links preserve the final post-competition, competition, and earlier autonomous rover milestones |
| Simulation coverage | 40/40 checks passing at the symposium-paper checkpoint; [GitHub Actions](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml) runs the simulation suite and pytest regressions on code/firmware/dependency changes, pull request, and manual dispatch |
| Setup documentation | `RUNNING.md`, `ARCHITECTURE.md`, and `requirements.txt` document setup, runtime modes, and software structure |
| Hardware operation | Requires calibrated servos, connected Arduino sensor firmware, and pre-run safety checks before powering the rover |
| Documentation included | README includes system architecture, software map, CAD links, field demos, course success videos, paper/poster links, and award documentation |
| License | PolyForm Noncommercial 1.0.0; commercial use requires separate permission |
| Known limits | Simulation does not fully model compliance, backlash, or deformable terrain; abrupt terrain transitions remained the clearest unresolved risk |

## My Role

- Developed and tuned the Python gait/navigation stack, terrain overlays, telemetry analysis, simulation validation, and competition-readiness fixes.
- Programmed the Arduino/C++ sensor firmware for ultrasonic and IMU data collection.
- Integrated the sensor firmware with the Raspberry Pi control stack.
- Designed the CAD models, managed the 3D printing workflow, and selected the ordered hardware components.
- Contributed to electrical assembly, including soldering and wiring.

## System Architecture

![Identity rover system architecture diagram](docs/assets/symposium-system-architecture.jpg)

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full software architecture map, including the Brain/Heart process split, sensor pipeline, terrain overlay, safety governors, and simulation coverage.

### Brain/Heart Process Split

![Brain/Heart process split diagram](docs/assets/brain-heart-process-split.jpg)

The Brain process handles sensor interpretation, terrain classification, obstacle/cliff logic, and navigation state transitions. The Heart process runs the timing-critical gait loop, computes servo commands, reads STS3215 position/load/speed telemetry, applies safety governors, and keeps motor control isolated from slower navigation work.

## Key Engineering Decisions

- Kept each leg to one powered degree of freedom to reduce mechanical complexity and make the control system easier to audit.
- Used RHex-style C-shaped legs for rolling ground contact, passive compliance, and simple stance/swing timing.
- Split the software into Brain and Heart processes so navigation logic cannot block the 30 Hz gait loop.
- Offloaded ultrasonic timing and IMU polling to an Arduino Nano because Linux on the Raspberry Pi is not real time.
- Used Buehler-clock gait parameters rather than per-joint trajectory planning, keeping terrain adaptation interpretable.
- Treated the STS3215 smart servos as both actuators and sensors, using position, load, speed, voltage, current, temperature, and error telemetry to tune gait behavior and safety limits.
- Built simulation tests for gait timing, terrain overlays, governors, and navigation FSM regressions before hardware deployment.

## Hardware Stack

| Subsystem | Components | Purpose |
| --- | --- | --- |
| Main compute | Raspberry Pi 3B+ | Runs the Python Brain/Heart control stack, navigation FSM, gait engine, telemetry handling, and simulation-derived safety logic |
| Sensor hub | Arduino Nano | Runs Arduino/C++ firmware for deterministic ultrasonic timing and IMU polling, then streams a 20-column CSV frame to the Raspberry Pi at ~10 Hz |
| Actuation and servo telemetry | 6x Feetech STS3215 serial bus smart servos | One actuator per leg, commanded with synchronized bus writes and read back for position, load, speed, voltage, current, temperature, and error telemetry |
| Servo bus interface | FE-URT-1 debug board | Provides the serial interface used for Feetech servo configuration, calibration, and bus-level debugging |
| Obstacle and cliff sensing | 8x HC-SR04 ultrasonic sensors | Provides 360-degree obstacle coverage plus downward-facing cliff/drop-off detection |
| Orientation sensing | BNO085 IMU | Provides fused orientation for slope detection, rough-terrain classification, tip/fall safety logic, and navigation state decisions |
| Power | 3S 3000 mAh LiPo battery, 11.1 V nominal | Powers the rover with software brownout protection and speed limiting under voltage sag |
| Chassis | PETG 3D-printed octagonal body | Supports six servo modules with front/rear leg pairs splayed at 35 degrees and middle legs mounted perpendicular |
| Legs | C-shaped PETG arc legs with TPU tread interface layer, 125 mm effective radius, 195-degree arc span | RHex-style rolling contact geometry with a flexible layer between the printed legs and bumper-pad tread |
| Ground contact | Adhesive rubber bumper pads with hot-glue V tread | Adds compliant grip and a NASA rover-inspired chevron contact pattern for rough terrain |

### Bill Of Materials

The README keeps the BOM at summary level. The detailed categorized table lives in [`docs/BOM.md`](docs/BOM.md), with the exported source sheet preserved as [`docs/purchasing-bom.csv`](docs/purchasing-bom.csv). Quantities in both files are ordered package counts, with integration notes explaining installed usage.

<table>
  <tr>
    <th>Purchase Lines</th>
    <th>Locomotion Core</th>
    <th>Sensor Coverage</th>
    <th>Fabrication Stack</th>
  </tr>
  <tr>
    <td align="center"><strong>31</strong><br>tracked source items</td>
    <td align="center"><strong>6</strong><br>STS3215 C-leg smart servos</td>
    <td align="center"><strong>8</strong><br>ultrasonic sensors plus BNO085 IMU</td>
    <td align="center"><strong>PETG + TPU</strong><br>with bumper-pad V tread</td>
  </tr>
</table>

| Detailed BOM View | Contents |
| --- | --- |
| [Categorized Bill of Materials](docs/BOM.md) | Control electronics, smart servos, sensors, power, printed structure, tread materials, wiring, adhesives, fasteners, and assembly supplies |
| [Purchasing Source CSV](docs/purchasing-bom.csv) | Exported source rows from `Robotics_Purchasing_2025-2026.xlsx` |

### Measured Platform Geometry

- Body length: 511 mm
- Total width: 280 mm
- Static ground clearance: 74 mm
- Effective leg radius: 125 mm from servo shaft to outer contact surface
- Leg arc span: 195 degrees
- Estimated mass: 2 to 3 kg

### CAD Models

| Part | Onshape Link |
| --- | --- |
| Split lid | [Open model](https://cad.onshape.com/documents/4f3de965fea211f4280a3c9d/w/cdf7ccd18bab44fcea532f37/e/039c12e9b8634e9de48b237f) |
| Chassis | [Open model](https://cad.onshape.com/documents/b0cc0fb5d7d22bf167ea7f76/w/008ad764f5d31a5d18c51176/e/51bfb1e7ed0292f5d8418b8c) |
| C-leg | [Open model](https://cad.onshape.com/documents/da3a47429f3542b58cbfb9b8/w/d8b269a68f869584fe54d88e/e/9680572918098f7f67734634) |
| Leg adapters | [Open model](https://cad.onshape.com/documents/60e3d6ac247373a6c9d27099/w/3073c20af8fd58d7202c08cb/e/fac01f63525d9f291b8ab335) |

This hardware layout intentionally trades fine-grained foot placement for mechanical simplicity, passive compliance, and an auditable control model. The Arduino Nano isolates microsecond-sensitive sensor timing from the Raspberry Pi, while the Pi handles higher-level gait and navigation logic.

## Software Map

The README lists only the main entry points. The full file-by-file map, diagnostics, launch modes, calibration tools, simulation helpers, telemetry analysis, and regression tests live in [`docs/software-map.md`](docs/software-map.md).

| Area | Primary Files | Why Reviewers Should Care |
| --- | --- | --- |
| Runtime gait and autonomy | [`final_full_gait_test.py`](final_full_gait_test.py), [`final_sensors.ino`](final_sensors.ino) | Main Raspberry Pi control loop plus Arduino sensor firmware used by the final rover |
| Running and diagnostics | [`RUNNING.md`](RUNNING.md), [`docs/software-map.md`](docs/software-map.md) | Hardware launch commands, dry-run modes, subsystem tests, and gait-specific checks |
| Simulation and regression | [`sim_verify.py`](sim_verify.py), [`sim_terrain.py`](sim_terrain.py), [`sim_nav.py`](sim_nav.py), pytest files | Kinematic, terrain, governor, navigation, and regression coverage |
| Telemetry and tuning | [`parse_telemetry.py`](parse_telemetry.py), [`analyze_run_log.py`](analyze_run_log.py), [`param_sweep.py`](param_sweep.py) | Post-run evidence used to tune gait constants, load limits, and safety governors |

## Gait Control

The gait engine follows a RHex-style Buehler clock. Duty cycle defines stance fraction, phase offsets define inter-leg timing, and a global speed setpoint controls phase progression.

| Gait | Duty Cycle | Primary Use |
| --- | --- | --- |
| Tripod | 0.5 | Faster locomotion on flat and moderate terrain |
| Wave | 0.75 | Higher-contact gait for rough terrain and inclines |
| Quadruped | 0.7 | Balance of stability and speed |

### Smart-Servo Feedback For Gait Changes

The Feetech STS3215 smart servos were used as feedback sensors, not just as motors. The Heart process reads present position, load, and speed during the live gait loop, then rotates lower-rate telemetry reads for voltage, current, temperature, and hardware error flags.

| Servo Feedback | How It Changed The Gait System |
| --- | --- |
| Present position | Converted into actual leg phase and compared against the Buehler-clock target phase |
| Phase error | Triggered the phase-error governor to slow the gait before lag caused foot drag or lost ground clearance |
| Present load | Drove stall detection, self-right load monitoring, and terrain signals such as sustained heavy load in sand |
| Present speed | Showed when commanded motion was not translating into actual leg speed under load |
| Voltage, current, temperature, and error flags | Fed brownout, current-budget, thermal, and overload-protection limits |
| Post-run telemetry | Informed feedforward caps, walking speed limits, impact windows, duty cycles, and final gait transitions |

### Gait Pattern Visuals

The diagrams below show the servo grouping and top-view leg order used by the implemented gait modes.

| Tripod | Quadruped | Wave |
| --- | --- | --- |
| ![Tripod gait locomotion diagram](docs/assets/gait-tripod.jpg) | ![Quadruped gait locomotion diagram](docs/assets/gait-quadruped.jpg) | ![Wave gait locomotion diagram](docs/assets/gait-wave.jpg) |

Terrain overlays adjust gait choice, impact window, duty cycle, and speed for flat ground, rough terrain, deep sand, and incline traversal. A phase-error governor reduces speed when commanded and measured servo phase diverge beyond the threshold used in the validation work.

## Navigation And Terrain Adaptation

The autonomous navigation layer uses an eight-state finite state machine for forward traversal, slow approach, arc turns, backing up, pivot turns, recovery wiggle, and safe stop behavior. Sensor input comes from ultrasonic range data, IMU orientation, servo telemetry, and watchdog state.

![Navigation finite-state machine flowchart](docs/assets/navigation-state-flowchart.jpg)

### Terrain Classification

| Signal | Used For |
| --- | --- |
| IMU pitch | Incline and descent detection |
| Angular-rate and ultrasonic stability | Rough-terrain classification |
| Sustained servo load | Deep-sand detection |
| Downward ultrasonic distance changes | Cliff/drop-off detection |

![Terrain validation classes and overlay parameters](docs/assets/terrain-validation-overlays.jpg)

### Servo Load Margin

Across the validation terrain set, measured servo loads stayed below the configured stall threshold.

![Servo load margin by terrain type with stall threshold](docs/assets/stall-threshold-load-margin.jpg)

## Media Gallery

The README highlights the video evidence without embedding every preview table. The full gallery is in [`docs/media.md`](docs/media.md), including field demos, course success runs, symposium materials, and award video links.

| Gallery | What The Link Opens |
| --- | --- |
| [Field Demo Videos](docs/media.md#field-demo-videos) | Clickable GIF previews linked to full traversal videos: sand hill, cliff detection, indoor navigation, and park obstacle navigation |
| [Course Success Runs](docs/media.md#course-success-runs) | Course 1-5 and challenge-course success videos with preview GIFs |
| [Research Symposium Materials](docs/media.md#research-symposium-materials) | Paper, presentation slides, poster, and COSGC award video |

## Recognition And Publications

<table>
  <tr>
    <td width="58%"><img src="docs/assets/cosgc-robotics-challenge-creative-locomotion-certificate.jpg" alt="COSGC 2026 Robotics Challenge certificate for Outstanding Demonstration of Creative Locomotion" width="520"></td>
    <td>
      <strong>Outstanding Demonstration of Creative Locomotion</strong><br><br>
      Awarded at the 2026 Colorado Space Grant Consortium Robotics Challenge for the rover's six C-leg locomotion system.
    </td>
  </tr>
</table>
<table>
  <tr>
    <td align="center"><strong>2026 Best Robotics Poster</strong></td>
    <td align="center"><strong>2026 People's Choice Video</strong></td>
  </tr>
  <tr>
    <td><img src="docs/assets/cosgc-best-robotics-poster-certificate.jpg" alt="COSGC 2026 Best Robotics Poster certificate" width="360"></td>
    <td><img src="docs/assets/cosgc-peoples-choice-video-certificate.jpg" alt="COSGC 2026 People's Choice Video certificate" width="360"></td>
  </tr>
</table>

| Research Output | Link |
| --- | --- |
| Paper | [Development of a Six-Legged Autonomous Robot for Rough Terrain Navigation Paper](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc.pdf) |
| Presentation Slides | [Development of a Six-Legged Autonomous Robot for Rough Terrain Navigation Paper Presentation Slides](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc-presentation-slides.pdf) |
| Poster | [Development of a Six-Legged Autonomous Robot for Rough Terrain Navigation Poster](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc-poster.pdf) |
| Award Video | [Identity COSGC Video](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/identity-cosgc-2026-award-video.mp4) |
| Full Media Gallery | [Field demos, course runs, symposium media, and award previews](docs/media.md) |

<table>
  <tr>
    <td width="58%"><img src="docs/assets/cosgc-poster-presentation-setup.jpg" alt="COSGC symposium poster presentation setup with rover prototype and printed parts" width="520"></td>
    <td>
      <strong>2026 NASA Colorado Space Grant Consortium Research Symposium Display</strong><br><br>
      Presented the rover prototype, printed CAD components, and research poster together so reviewers could connect the software, mechanical design, and validation results.
    </td>
  </tr>
</table>


## Simulation And Testing

The simulation framework contains 40 automated checks:

- 14 kinematic and gait checks in `sim_verify.py`
- 14 terrain and governor scenarios in `sim_terrain.py`
- 12 navigation FSM categories in `sim_nav.py`

At the time of the symposium paper, the full suite passed 40/40 checks. The simulation validates timing, state transitions, terrain overlays, and control invariants, but it does not model all physical effects such as compliance, backlash, or deformable terrain.

[GitHub Actions](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml) runs these simulation checks and the pytest regression suite on code, firmware, dependency, workflow, pull request, and manual-dispatch events.

Run the simulation checks:

```bash
python3 sim_verify.py
python3 sim_terrain.py
python3 sim_nav.py
```

## Releases

- [Final Post-Competition Build](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/final-post-competition-build): final public build after the COSGC competition, with cliff detection turned back on.
- [Competition Build](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/Competition): competition snapshot used during the event.
- [V0.5 Full Program](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/pre-release): earlier autonomous rover milestone.

## Known Limits

| Category | Limit |
| --- | --- |
| Validation scope | The 32-trial validation set supports pilot-scale reliability claims, not a fully powered statistical proof. |
| Reproducibility | Late-stage validation did not preserve exact per-trial software hashes. |
| Simulation fidelity | The simulation framework validates timing, state transitions, terrain overlays, and control invariants, but does not model all physical compliance, backlash, or deformable-terrain effects. |
| Hardware behavior | The clearest remaining failure mode was abrupt terrain transition within a single stride. |
| Application claims | Search-and-rescue and planetary robotics are future application targets, not demonstrated deployment domains. |
