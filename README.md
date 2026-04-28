# Identity: Team Euler Autonomous Hexapod Rover

![Identity rover banner](docs/assets/identity-project-banner.png)

[![Simulation Checks](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml/badge.svg)](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml)
[![Raspberry Pi/Python: Gait Control Program](https://img.shields.io/badge/Raspberry%20Pi%2FPython-Gait%20Control%20Program-2ea44f)](final_full_gait_test.py)
[![Arduino/C++: Sensor Firmware](https://img.shields.io/badge/Arduino%2FC%2B%2B-Sensor%20Firmware-00878F)](final_sensors.ino)
[![License: PolyForm Noncommercial 1.0.0](https://img.shields.io/badge/License-PolyForm%20Noncommercial%201.0.0-orange.svg)](LICENSE)

Identity is a six-legged autonomous rover built by Team Euler for the COSGC robotics challenge. It combines Python control software on a Raspberry Pi 3B+, Arduino/C++ sensor firmware on an Arduino Nano, six Feetech STS3215 smart servos with runtime telemetry, ultrasonic sensing, and IMU feedback to test cost-conscious rough-terrain locomotion.

The project explored whether a six-servo hexapod could adapt to rough outdoor terrain using a small set of interpretable gait parameters instead of complex multi-joint planning or learned locomotion. The key control variables were Buehler-clock duty cycle, impact window, phase offsets, and a global speed setpoint.

The final build combined field-tested hardware, simulation-backed control logic, and documented validation results showing 31/32 successful traversals across seven terrain categories.

## Project Snapshot

| Category | Summary |
| --- | --- |
| Project | Autonomous six-legged rover using RHex-style C-leg locomotion |
| Stack | Python, C++, Raspberry Pi 3B+, Arduino Nano, Feetech STS3215 serial smart servos, BNO085 IMU, HC-SR04 ultrasonic sensors |
| My focus | Software, controls, validation, CAD/printing workflow, hardware selection, and build documentation |
| Outcome | 31/32 formal traversals, 40/40 simulation checks, and 3 COSGC recognitions |

The main README is intentionally concise; detailed build, validation, gait, media, release, and purchasing references are kept in [`docs/`](docs/README.md) for reviewers who want deeper evidence.

## Quick Reviewer Path

| Reader | Start Here | What It Shows |
| --- | --- | --- |
| Recruiters and portfolio reviewers | [Project Snapshot](#project-snapshot), [My Role](#my-role), and [Recognition And Publications](#recognition-and-publications) | Project outcome, validation result, awards, and public deliverables |
| Robotics and controls reviewers | [`final_full_gait_test.py`](final_full_gait_test.py), [Gait Control Reference](docs/gait-control.md), and [Validation Reference](docs/validation.md) | Brain/Heart control split, smart-servo feedback, terrain adaptation, and validation strategy |
| Hardware reviewers | [Hardware Reference](docs/hardware.md) and [Categorized Bill of Materials](docs/BOM.md) | Actuation, sensing, power, printed structure, tread design, and purchasing traceability |
| Reproducing or running code | [Software Map](docs/software-map.md) and [`RUNNING.md`](RUNNING.md) | Main runtime entry points, diagnostics, simulations, and launch commands |

For control-software review, start with [`final_full_gait_test.py`](final_full_gait_test.py), [`ARCHITECTURE.md`](ARCHITECTURE.md), and [`docs/gait-control.md`](docs/gait-control.md).

## Results At A Glance

| Metric | Result |
| --- | --- |
| Formal validation trials | 31/32 successful |
| Observed traversal success | 96.9% |
| Terrain categories | 7 |
| Simulation tests | 40/40 passing |
| Awards | 3 COSGC recognitions |
| Heart loop rate | 30 Hz |
| Sensor update rate | ~10 Hz |
| Steady-state servo load margin | At least 42% in formal trials |

Validation is presented as pilot-scale testing; statistical framing and known limits are documented in [`docs/validation.md`](docs/validation.md).

## Engineering Highlights

| Highlight | Why It Matters |
| --- | --- |
| Multiprocess Brain/Heart architecture | Keeps navigation logic from blocking the 30 Hz gait loop |
| STS3215 smart-servo telemetry | Treats actuator feedback as sensor data for phase error, load, speed, voltage, current, temperature, and fault monitoring |
| Arduino Nano sensor hub | Moves ultrasonic timing and IMU polling off the Raspberry Pi so sensor collection stays deterministic |
| Buehler-clock gait control | Keeps rough-terrain adaptation interpretable through duty cycle, impact window, phase offsets, and speed setpoint changes |
| Simulation-backed validation | Tests gait timing, terrain overlays, governors, and navigation FSM behavior before hardware runs |
| Field-tested C-leg locomotion | Validated a cost-conscious PETG/TPU C-leg platform across sand, gravel, stone, carpet, packed earth, tile, and inclines |

## Repository Status

This repository contains the final public code, validation previews, CAD references, media links, and supporting documentation for the Team Euler rover build. Longer reference pages are collected in the [docs index](docs/README.md), and full-resolution media is hosted in the [Portfolio Media Assets](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/media-assets) release.

| Area | Status |
| --- | --- |
| Final rover build | [Final Post-Competition Build](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/final-post-competition-build), with cliff detection enabled in the public configuration |
| Primary runtime | [`final_full_gait_test.py`](final_full_gait_test.py) on Raspberry Pi plus [`final_sensors.ino`](final_sensors.ino) on Arduino Nano |
| Validation | 31/32 formal traversals successful, with 40/40 simulation checks passing at the symposium-paper checkpoint |
| Setup and references | [`RUNNING.md`](RUNNING.md), [`ARCHITECTURE.md`](ARCHITECTURE.md), [`docs/README.md`](docs/README.md), and [`requirements.txt`](requirements.txt) |
| Release history | Earlier public milestones are preserved in [`docs/releases.md`](docs/releases.md) |

## My Role

This was a team robotics project. My individual work focused on software, controls, validation, and documenting the parts of the build I led or integrated.

- Co-developed and tuned the Python gait/navigation stack, including terrain overlays, telemetry-driven gait tuning, simulation validation, and competition-readiness fixes.
- Integrated Arduino/C++ sensor firmware with the Raspberry Pi control stack and contributed firmware-facing fixes for ultrasonic/IMU data handling.
- Built the simulation and telemetry-analysis tooling used to validate gait timing, navigation behavior, terrain overlays, safety governors, and post-run logs.
- Used STS3215 smart-servo telemetry and field logs to tune gait behavior, load limits, phase-error handling, and safety governors.
- Designed CAD models, managed the 3D printing workflow, and selected and ordered hardware components recorded in the public BOM.
- Contributed to electrical assembly, including soldering and wiring.

## System Architecture

<p align="center">
  <img src="docs/assets/symposium-system-architecture.jpg" alt="Identity rover system architecture diagram" width="960">
</p>

The Brain process handles sensor interpretation, terrain classification, obstacle/cliff logic, and navigation state transitions. The Heart process runs the timing-critical gait loop, computes servo commands, reads STS3215 telemetry, applies safety governors, and keeps motor control isolated from slower navigation work.

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full software architecture map, or [`docs/architecture.md`](docs/architecture.md) for a diagram-focused overview.

## Hardware Stack

Identity uses a cost-conscious, field-serviceable hardware stack built around one smart servo per C-leg, a Raspberry Pi control computer, and an Arduino sensor hub. The full hardware reference, measured geometry, CAD links, tread construction notes, and BOM routing live in [`docs/hardware.md`](docs/hardware.md).

| Subsystem | Components | Purpose |
| --- | --- | --- |
| Compute and sensing | Raspberry Pi 3B+, Arduino Nano, BNO085 IMU, 8x HC-SR04 ultrasonic sensors | Brain/Heart control, deterministic sensor timing, orientation feedback, obstacle detection, and cliff/drop-off sensing |
| Locomotion | 6x Feetech STS3215 smart servos with C-shaped PETG/TPU legs | Single-actuator C-leg motion with servo telemetry used for phase, load, speed, voltage, current, temperature, and error feedback |
| Structure and traction | PETG octagonal chassis, TPU tread interface, adhesive bumper pads, hot-glue V tread | Mechanically simple rough-terrain platform with a NASA rover-inspired chevron contact pattern |
| Power and integration | 3S 3000 mAh LiPo, FE-URT-1 servo interface, wiring harnesses, fasteners, adhesives | Portable power, servo bus configuration, and field-serviceable assembly |

### Bill Of Materials

| BOM Snapshot | Key Details | Link |
| --- | --- | --- |
| 31 tracked purchase lines | Control electronics, sensors, power, printed structure, wiring, adhesives, fasteners, and assembly supplies | [Categorized BOM](docs/BOM.md) |
| Locomotion core | 6 STS3215 smart servos driving PETG/TPU C-legs | [BOM locomotion entries](docs/BOM.md#control-actuation-and-sensing) |
| Sensor coverage | 8 ultrasonic sensors plus BNO085 IMU | [BOM sensing entries](docs/BOM.md#control-actuation-and-sensing) |
| Source export | CSV generated from `Robotics_Purchasing_2025-2026.xlsx` | [Purchasing source CSV](docs/purchasing-bom.csv) |

## Software Map

The main runtime is split between the Raspberry Pi gait/navigation program and Arduino sensor firmware. The full file-by-file map, diagnostics, launch modes, calibration tools, simulation helpers, telemetry analysis, and regression tests live in [`docs/software-map.md`](docs/software-map.md).

| Area | Primary Files | Reviewer Signal |
| --- | --- | --- |
| Runtime gait and autonomy | [`final_full_gait_test.py`](final_full_gait_test.py), [`final_sensors.ino`](final_sensors.ino) | Main Raspberry Pi control loop plus Arduino sensor firmware used by the final rover |
| Running and diagnostics | [`RUNNING.md`](RUNNING.md), [`docs/software-map.md`](docs/software-map.md) | Hardware launch commands, dry-run modes, subsystem tests, and gait-specific checks |

## Gait Control

The gait engine follows a RHex-style Buehler clock. Duty cycle defines stance fraction, phase offsets define inter-leg timing, and a global speed setpoint controls phase progression. Tripod, wave, and quadruped modes were tuned for different terrain demands.

| Control Feature | Implementation |
| --- | --- |
| Interpretable gait parameters | Duty cycle, impact window, phase offsets, and global speed setpoint |
| STS3215 feedback loop | Position, phase error, load, speed, voltage, current, temperature, and error flags |
| Safety governors | Phase-error slowdown, stall detection, brownout/current limits, thermal monitoring, and overload prevention |
| Detailed reference | [`docs/gait-control.md`](docs/gait-control.md) |

STS3215 telemetry was central to the gait changes: the rover compared commanded and measured leg phase, slowed before phase lag became foot drag, used sustained load as a terrain signal, and applied voltage/current/temperature limits during field testing.

## Navigation And Terrain Adaptation

The autonomous navigation layer uses an eight-state finite state machine for forward traversal, slow approach, arc turns, backing up, pivot turns, recovery wiggle, and safe stop behavior. Sensor input comes from ultrasonic range data, IMU orientation, servo telemetry, and watchdog state.

Terrain classification combines IMU pitch/roll, angular-rate stability, ultrasonic range changes, and servo-load trends. The navigation FSM diagram, terrain overlay chart, and servo load margin figure are collected in [`docs/validation.md`](docs/validation.md).

## Simulation And Testing

At the symposium-paper checkpoint, the project passed 40/40 automated simulation checks covering gait timing, terrain overlays, safety governors, and navigation FSM behavior. GitHub Actions runs the simulation and pytest regression suite on code, firmware, dependency, workflow, pull request, and manual-dispatch events.

| Validation Resource | Link |
| --- | --- |
| GitHub Actions simulation workflow | [Simulation Checks](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml) |
| Field results and known limits | [Validation Reference](docs/validation.md) |
| Local simulation commands | [`docs/validation.md`](docs/validation.md#simulation-coverage) |

## Recognition And Publications

<table>
  <tr>
    <td width="58%"><img src="docs/assets/cosgc-robotics-challenge-creative-locomotion-certificate.jpg" alt="COSGC 2026 Robotics Challenge certificate for Outstanding Demonstration of Creative Locomotion" width="520"></td>
    <td>
      <strong>Outstanding Demonstration of Creative Locomotion</strong><br><br>
      Awarded at the 2026 Colorado Space Grant Consortium Robotics Challenge for the rover's six C-leg locomotion system.
    </td>
  </tr>
  <tr>
    <td width="58%"><img src="docs/assets/cosgc-best-robotics-poster-certificate.jpg" alt="COSGC 2026 Best Robotics Poster certificate" width="520"></td>
    <td>
      <strong>Best Robotics Poster</strong><br><br>
      Awarded at the 2026 Colorado Space Grant Consortium Undergraduate Space Research Symposium for the rover research poster and technical presentation.
    </td>
  </tr>
  <tr>
    <td width="58%"><img src="docs/assets/cosgc-peoples-choice-video-certificate.jpg" alt="COSGC 2026 People's Choice Video certificate" width="520"></td>
    <td>
      <strong>People's Choice Video</strong><br><br>
      Awarded at the 2026 Colorado Space Grant Consortium Undergraduate Space Research Symposium for the Identity rover project video.
    </td>
  </tr>
</table>
<table>
  <tr>
    <td width="58%"><img src="docs/assets/cosgc-poster-presentation-setup.jpg" alt="COSGC symposium poster presentation setup with rover prototype and printed parts" width="520"></td>
    <td>
      <strong>2026 NASA Colorado Space Grant Consortium Research Symposium Display</strong><br><br>
      Presented the rover prototype, printed CAD components, and research poster together so reviewers could connect the software, mechanical design, and validation results.
    </td>
  </tr>
</table>

| Research Output | View | Download |
| --- | --- | --- |
| Paper | <div align="center">[View Paper](https://docs.google.com/gview?embedded=1&url=https%3A%2F%2Fgithub.com%2FRobiswell%2FEuler_Rover_2026%2Freleases%2Fdownload%2Fmedia-assets%2Fdevelopment-of-six-legged-autonomous-robot-frcc.pdf)</div> | <div align="center">[Download Paper PDF](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc.pdf)</div> |
| Presentation Slides | <div align="center">[View Slides](docs/paper-presentation-slides.md)</div> | <div align="center">[Download Slides PDF](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc-presentation-slides.pdf)</div> |
| Paper Presentation Video | <div align="center">[Watch on YouTube](https://youtu.be/zEp2kFS_MPs)</div> | <div align="center">[Download Presentation MP4](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/COSGC.Symposium.Paper.Presentation.Euler.2026.mp4)</div> |
| Poster | <div align="center">[View Poster](https://docs.google.com/gview?embedded=1&url=https%3A%2F%2Fgithub.com%2FRobiswell%2FEuler_Rover_2026%2Freleases%2Fdownload%2Fmedia-assets%2Fdevelopment-of-six-legged-autonomous-robot-frcc-poster.pdf)</div> | <div align="center">[Download Poster PDF](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc-poster.pdf)</div> |
| People's Choice Video | <div align="center">[Watch on YouTube](https://youtu.be/kA2DoByfL78)</div> | <div align="center">[Download Video MP4](https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/identity-cosgc-2026-award-video.mp4)</div> |
| Field Demos & Course Success Videos | <div align="center">[Open Gallery](docs/media.md)</div> | <div align="center">N/A</div> |
