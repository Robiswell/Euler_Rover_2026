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

<table width="100%">
  <tr>
    <th width="24%">Category</th>
    <th width="76%">Summary</th>
  </tr>
  <tr>
    <td>Project</td>
    <td>Autonomous six-legged rover using RHex-style C-leg locomotion</td>
  </tr>
  <tr>
    <td>Stack</td>
    <td>Python, C++, Raspberry Pi 3B+, Arduino Nano, Feetech STS3215 serial smart servos, BNO085 IMU, HC-SR04 ultrasonic sensors</td>
  </tr>
  <tr>
    <td>My focus</td>
    <td>Software, controls, validation, CAD/printing workflow, hardware selection, and build documentation</td>
  </tr>
  <tr>
    <td>Outcome</td>
    <td>31/32 formal traversals, 40/40 simulation checks, and 3 COSGC recognitions</td>
  </tr>
</table>

The main README is intentionally concise; detailed build, validation, gait, media, release, and purchasing references are kept in [`docs/`](docs/README.md) for reviewers who want deeper evidence.

## Quick Reviewer Path

<table width="100%">
  <tr>
    <th width="24%">Reader</th>
    <th width="36%">Start Here</th>
    <th width="40%">What It Shows</th>
  </tr>
  <tr>
    <td>Recruiters and portfolio reviewers</td>
    <td><a href="#project-snapshot">Project Snapshot</a>, <a href="#my-role">My Role</a>, and <a href="#recognition-and-publications">Recognition And Publications</a></td>
    <td>Project outcome, validation result, awards, and public deliverables</td>
  </tr>
  <tr>
    <td>Robotics and controls reviewers</td>
    <td><a href="final_full_gait_test.py"><code>final_full_gait_test.py</code></a>, <a href="docs/gait-control.md">Gait Control Reference</a>, and <a href="docs/validation.md">Validation Reference</a></td>
    <td>Brain/Heart control split, smart-servo feedback, terrain adaptation, and validation strategy</td>
  </tr>
  <tr>
    <td>Hardware reviewers</td>
    <td><a href="docs/hardware.md">Hardware Reference</a> and <a href="docs/BOM.md">Categorized Bill of Materials</a></td>
    <td>Actuation, sensing, power, printed structure, tread design, and purchasing traceability</td>
  </tr>
  <tr>
    <td>Reproducing or running code</td>
    <td><a href="docs/software-map.md">Software Map</a> and <a href="RUNNING.md"><code>RUNNING.md</code></a></td>
    <td>Main runtime entry points, diagnostics, simulations, and launch commands</td>
  </tr>
</table>

For control-software review, start with [`final_full_gait_test.py`](final_full_gait_test.py), [`ARCHITECTURE.md`](ARCHITECTURE.md), and [`docs/gait-control.md`](docs/gait-control.md).

## Results At A Glance

<table width="100%">
  <tr>
    <th width="25%">Metric<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="245" height="1"></th>
    <th width="25%">Result<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="245" height="1"></th>
    <th width="25%">Metric<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="245" height="1"></th>
    <th width="25%">Result<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="245" height="1"></th>
  </tr>
  <tr>
    <td>Formal validation trials</td>
    <td>31/32 successful</td>
    <td>Observed traversal success</td>
    <td>96.9%</td>
  </tr>
  <tr>
    <td>Terrain categories</td>
    <td>7</td>
    <td>Simulation tests</td>
    <td>40/40 passing</td>
  </tr>
  <tr>
    <td>Awards</td>
    <td>3 COSGC recognitions</td>
    <td>Heart loop rate</td>
    <td>30 Hz</td>
  </tr>
  <tr>
    <td>Sensor update rate</td>
    <td>~10 Hz</td>
    <td>Steady-state servo load margin</td>
    <td>At least 42% in formal trials</td>
  </tr>
</table>

Validation is presented as pilot-scale testing; statistical framing and known limits are documented in [`docs/validation.md`](docs/validation.md).

## Engineering Highlights

<table width="100%">
  <tr>
    <th width="35%">Highlight</th>
    <th width="65%">Why It Matters</th>
  </tr>
  <tr><td>Multiprocess Brain/Heart architecture</td><td>Keeps navigation logic from blocking the 30 Hz gait loop</td></tr>
  <tr><td>STS3215 smart-servo telemetry</td><td>Treats actuator feedback as sensor data for phase error, load, speed, voltage, current, temperature, and fault monitoring</td></tr>
  <tr><td>Arduino Nano sensor hub</td><td>Moves ultrasonic timing and IMU polling off the Raspberry Pi so sensor collection stays deterministic</td></tr>
  <tr><td>Buehler-clock gait control</td><td>Keeps rough-terrain adaptation interpretable through duty cycle, impact window, phase offsets, and speed setpoint changes</td></tr>
  <tr><td>Simulation-backed validation</td><td>Tests gait timing, terrain overlays, governors, and navigation FSM behavior before hardware runs</td></tr>
  <tr><td>Field-tested C-leg locomotion</td><td>Validated a cost-conscious PETG/TPU C-leg platform across sand, gravel, stone, carpet, packed earth, tile, and inclines</td></tr>
</table>

## Repository Status

This repository contains the final public code, validation previews, CAD references, media links, and supporting documentation for the Team Euler rover build. Longer reference pages are collected in the [docs index](docs/README.md), and full-resolution media is hosted in the [Portfolio Media Assets](https://github.com/Robiswell/Euler_Rover_2026/releases/tag/media-assets) release.

<table width="100%">
  <tr>
    <th width="30%">Area</th>
    <th width="70%">Status</th>
  </tr>
  <tr>
    <td>Final rover build</td>
    <td><a href="https://github.com/Robiswell/Euler_Rover_2026/releases/tag/final-post-competition-build">Final Post-Competition Build</a>, with cliff detection enabled in the public configuration</td>
  </tr>
  <tr>
    <td>Primary runtime</td>
    <td><a href="final_full_gait_test.py"><code>final_full_gait_test.py</code></a> on Raspberry Pi plus <a href="final_sensors.ino"><code>final_sensors.ino</code></a> on Arduino Nano</td>
  </tr>
  <tr>
    <td>Validation</td>
    <td>31/32 formal traversals successful, with 40/40 simulation checks passing at the symposium-paper checkpoint</td>
  </tr>
  <tr>
    <td>Setup and references</td>
    <td><a href="RUNNING.md"><code>RUNNING.md</code></a>, <a href="ARCHITECTURE.md"><code>ARCHITECTURE.md</code></a>, <a href="docs/README.md"><code>docs/README.md</code></a>, and <a href="requirements.txt"><code>requirements.txt</code></a></td>
  </tr>
  <tr>
    <td>Release history</td>
    <td>Earlier public milestones are preserved in <a href="docs/releases.md"><code>docs/releases.md</code></a></td>
  </tr>
</table>

## My Role

This was a team robotics project. My individual work focused on software, controls, validation, and the mechanical design and 3D-printed fabrication work I personally owned, plus documentation for the parts of the build I led or integrated.

- Co-developed and tuned the Python gait/navigation stack, including terrain overlays, telemetry-driven gait tuning, simulation validation, and competition-readiness fixes.
- Integrated Arduino/C++ sensor firmware with the Raspberry Pi control stack and contributed firmware-facing fixes for ultrasonic/IMU data handling.
- Built the simulation and telemetry-analysis tooling used to validate gait timing, navigation behavior, terrain overlays, safety governors, and post-run logs.
- Used STS3215 smart-servo telemetry and field logs to tune gait behavior, load limits, phase-error handling, and safety governors.
- Owned the CAD modeling and 3D printing workflow for the rover's custom mechanical parts, and selected and ordered hardware components recorded in the public BOM.
- Contributed to electrical assembly, including soldering and wiring.

## System Architecture

<p align="center">
  <img src="docs/assets/symposium-system-architecture.jpg" alt="Identity rover system architecture diagram" width="960">
</p>

The Brain process handles sensor interpretation, terrain classification, obstacle/cliff logic, and navigation state transitions. The Heart process runs the timing-critical gait loop, computes servo commands, reads STS3215 telemetry, applies safety governors, and keeps motor control isolated from slower navigation work.

See [`ARCHITECTURE.md`](ARCHITECTURE.md) for the full software architecture map, or [`docs/architecture.md`](docs/architecture.md) for a diagram-focused overview.

## Hardware Stack

Identity uses a cost-conscious, field-serviceable hardware stack built around one smart servo per C-leg, a Raspberry Pi control computer, and an Arduino sensor hub. The full hardware reference, measured geometry, CAD links, tread construction notes, and BOM routing live in [`docs/hardware.md`](docs/hardware.md).

<table width="100%">
  <tr>
    <th width="24%">Subsystem</th>
    <th width="34%">Components</th>
    <th width="42%">Purpose</th>
  </tr>
  <tr>
    <td>Compute and sensing</td>
    <td>Raspberry Pi 3B+, Arduino Nano, BNO085 IMU, 8x HC-SR04 ultrasonic sensors</td>
    <td>Brain/Heart control, deterministic sensor timing, orientation feedback, obstacle detection, and cliff/drop-off sensing</td>
  </tr>
  <tr>
    <td>Locomotion</td>
    <td>6x Feetech STS3215 smart servos with C-shaped PETG/TPU legs</td>
    <td>Single-actuator C-leg motion with servo telemetry used for phase, load, speed, voltage, current, temperature, and error feedback</td>
  </tr>
  <tr>
    <td>Structure and traction</td>
    <td>PETG octagonal chassis, TPU tread interface, adhesive bumper pads, hot-glue V tread</td>
    <td>Mechanically simple rough-terrain platform with a NASA rover-inspired chevron contact pattern</td>
  </tr>
  <tr>
    <td>Power and integration</td>
    <td>3S 3000 mAh LiPo, FE-URT-1 servo interface, wiring harnesses, fasteners, adhesives</td>
    <td>Portable power, servo bus configuration, and field-serviceable assembly</td>
  </tr>
</table>

### Bill Of Materials

<table width="100%">
  <tr>
    <th width="26%">BOM Snapshot</th>
    <th width="44%">Key Details</th>
    <th width="30%">Link</th>
  </tr>
  <tr>
    <td>31 tracked purchase lines</td>
    <td>Control electronics, sensors, power, printed structure, wiring, adhesives, fasteners, and assembly supplies</td>
    <td><a href="docs/BOM.md">Categorized BOM</a></td>
  </tr>
  <tr>
    <td>Locomotion core</td>
    <td>6 STS3215 smart servos driving PETG/TPU C-legs</td>
    <td><a href="docs/BOM.md#control-actuation-and-sensing">BOM locomotion entries</a></td>
  </tr>
  <tr>
    <td>Sensor coverage</td>
    <td>8 ultrasonic sensors plus BNO085 IMU</td>
    <td><a href="docs/BOM.md#control-actuation-and-sensing">BOM sensing entries</a></td>
  </tr>
  <tr>
    <td>Source export</td>
    <td>CSV generated from <code>Robotics_Purchasing_2025-2026.xlsx</code></td>
    <td><a href="docs/purchasing-bom.csv">Purchasing source CSV</a></td>
  </tr>
</table>

## Software Map

The main runtime is split between the Raspberry Pi gait/navigation program and Arduino sensor firmware. The full file-by-file map, diagnostics, launch modes, calibration tools, simulation helpers, telemetry analysis, and regression tests live in [`docs/software-map.md`](docs/software-map.md).

<table width="100%">
  <tr>
    <th width="24%">Area</th>
    <th width="38%">Primary Files</th>
    <th width="38%">Reviewer Signal</th>
  </tr>
  <tr>
    <td>Runtime gait and autonomy</td>
    <td><a href="final_full_gait_test.py"><code>final_full_gait_test.py</code></a>, <a href="final_sensors.ino"><code>final_sensors.ino</code></a></td>
    <td>Main Raspberry Pi control loop plus Arduino sensor firmware used by the final rover</td>
  </tr>
  <tr>
    <td>Running and diagnostics</td>
    <td><a href="RUNNING.md"><code>RUNNING.md</code></a>, <a href="docs/software-map.md"><code>docs/software-map.md</code></a></td>
    <td>Hardware launch commands, dry-run modes, subsystem tests, and gait-specific checks</td>
  </tr>
</table>

## Gait Control

The gait engine follows a RHex-style Buehler clock. Duty cycle defines stance fraction, phase offsets define inter-leg timing, and a global speed setpoint controls phase progression. Tripod, wave, and quadruped modes were tuned for different terrain demands.

<table width="100%">
  <tr>
    <th width="32%">Control Feature</th>
    <th width="68%">Implementation</th>
  </tr>
  <tr><td>Interpretable gait parameters</td><td>Duty cycle, impact window, phase offsets, and global speed setpoint</td></tr>
  <tr><td>STS3215 feedback loop</td><td>Position, phase error, load, speed, voltage, current, temperature, and error flags</td></tr>
  <tr><td>Safety governors</td><td>Phase-error slowdown, stall detection, brownout/current limits, thermal monitoring, and overload prevention</td></tr>
  <tr><td>Detailed reference</td><td><a href="docs/gait-control.md"><code>docs/gait-control.md</code></a></td></tr>
</table>

STS3215 telemetry was central to the gait changes: the rover compared commanded and measured leg phase, slowed before phase lag became foot drag, used sustained load as a terrain signal, and applied voltage/current/temperature limits during field testing.

## Navigation And Terrain Adaptation

The autonomous navigation layer uses an eight-state finite state machine for forward traversal, slow approach, arc turns, backing up, pivot turns, recovery wiggle, and safe stop behavior. Sensor input comes from ultrasonic range data, IMU orientation, servo telemetry, and watchdog state.

Terrain classification combines IMU pitch/roll, angular-rate stability, ultrasonic range changes, and servo-load trends. The navigation FSM diagram, terrain overlay chart, and servo load margin figure are collected in [`docs/validation.md`](docs/validation.md).

## Simulation And Testing

At the symposium-paper checkpoint, the project passed 40/40 automated simulation checks covering gait timing, terrain overlays, safety governors, and navigation FSM behavior. GitHub Actions runs the simulation and pytest regression suite on code, firmware, dependency, workflow, pull request, and manual-dispatch events.

<table width="100%">
  <tr>
    <th width="30%">Validation Resource<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="295" height="1"></th>
    <th width="45%">What It Verifies<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="440" height="1"></th>
    <th width="25%">Link<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="245" height="1"></th>
  </tr>
  <tr>
    <td>GitHub Actions simulation workflow</td>
    <td>Automated simulation and pytest regression checks across code, firmware, dependency, workflow, pull request, and manual-dispatch events</td>
    <td align="center"><a href="https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml">Simulation Checks</a></td>
  </tr>
  <tr>
    <td>Field results and known limits</td>
    <td>Formal traversal outcomes, terrain categories, servo load margin, pilot-test scope, and remaining validation limits</td>
    <td align="center"><a href="docs/validation.md">Validation Reference</a></td>
  </tr>
  <tr>
    <td>Local simulation commands</td>
    <td>Command references for <code>sim_verify.py</code>, <code>sim_terrain.py</code>, and <code>sim_nav.py</code>, including what the simulations do and do not model</td>
    <td align="center"><a href="docs/validation.md#simulation-coverage"><code>docs/validation.md</code></a></td>
  </tr>
</table>

## Recognition And Publications

<table width="100%">
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
<table width="100%">
  <tr>
    <td width="58%"><img src="docs/assets/cosgc-poster-presentation-setup.jpg" alt="COSGC symposium poster presentation setup with rover prototype and printed parts" width="520"></td>
    <td>
      <strong>2026 NASA Colorado Space Grant Consortium Research Symposium Display</strong><br><br>
      Presented the rover prototype, printed CAD components, and research poster together so reviewers could connect the software, mechanical design, and validation results.
    </td>
  </tr>
</table>

<table width="100%">
  <tr>
    <th width="24%">Research Output<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="235" height="1"></th>
    <th width="28%">Context<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="275" height="1"></th>
    <th width="24%">View<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="235" height="1"></th>
    <th width="24%">Download<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="235" height="1"></th>
  </tr>
  <tr>
    <td>Paper</td>
    <td>Full technical writeup for the rover design, gait control approach, validation results, and project limitations</td>
    <td align="center"><a href="https://docs.google.com/gview?embedded=1&url=https%3A%2F%2Fgithub.com%2FRobiswell%2FEuler_Rover_2026%2Freleases%2Fdownload%2Fmedia-assets%2Fdevelopment-of-six-legged-autonomous-robot-frcc.pdf">View Paper</a></td>
    <td align="center"><a href="https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc.pdf">Download Paper PDF</a></td>
  </tr>
  <tr>
    <td>Presentation Slides</td>
    <td>Symposium presentation deck summarizing the engineering problem, rover architecture, gait strategy, and validation outcomes</td>
    <td align="center"><a href="docs/paper-presentation-slides.md">View Slides</a></td>
    <td align="center"><a href="https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc-presentation-slides.pdf">Download Slides PDF</a></td>
  </tr>
  <tr>
    <td>Paper Presentation Video</td>
    <td>Recorded version of the symposium paper presentation with audio hosted externally for browser playback</td>
    <td align="center"><a href="https://youtu.be/zEp2kFS_MPs">Watch on YouTube</a></td>
    <td align="center"><a href="https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/COSGC.Symposium.Paper.Presentation.Euler.2026.mp4">Download Presentation MP4</a></td>
  </tr>
  <tr>
    <td>Poster</td>
    <td>Research-poster version of the project for quick review of motivation, design, validation, and outcomes</td>
    <td align="center"><a href="https://docs.google.com/gview?embedded=1&url=https%3A%2F%2Fgithub.com%2FRobiswell%2FEuler_Rover_2026%2Freleases%2Fdownload%2Fmedia-assets%2Fdevelopment-of-six-legged-autonomous-robot-frcc-poster.pdf">View Poster</a></td>
    <td align="center"><a href="https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/development-of-six-legged-autonomous-robot-frcc-poster.pdf">Download Poster PDF</a></td>
  </tr>
  <tr>
    <td>People's Choice Video</td>
    <td>Public-facing award video showing the rover and project story in a shorter presentation format</td>
    <td align="center"><a href="https://youtu.be/kA2DoByfL78">Watch on YouTube</a></td>
    <td align="center"><a href="https://github.com/Robiswell/Euler_Rover_2026/releases/download/media-assets/identity-cosgc-2026-award-video.mp4">Download Video MP4</a></td>
  </tr>
  <tr>
    <td>Field Demos & Course Success Videos</td>
    <td>Gallery of full-resolution field demos and course-success videos with GIF previews</td>
    <td align="center"><a href="docs/media.md">Open Gallery</a></td>
    <td align="center">N/A</td>
  </tr>
</table>

## Appendix: Project Reference Index

This optional index is organized by what someone might need to find later, rather than by file name. It points to the deeper references behind the portfolio summary above.

<table width="100%">
  <tr>
    <th width="28%">Looking For<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="275" height="1"></th>
    <th width="32%">Best Starting Point<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="315" height="1"></th>
    <th width="40%">What It Contains<br><img src="docs/assets/readme-table-spacer.svg" alt="" width="390" height="1"></th>
  </tr>
  <tr>
    <td>Fast reviewer orientation</td>
    <td><a href="#quick-reviewer-path">Quick Reviewer Path</a>, <a href="#my-role">My Role</a></td>
    <td>Shortest route to the code, architecture, validation evidence, hardware summary, and individual contribution scope.</td>
  </tr>
  <tr>
    <td>Programming contribution evidence</td>
    <td><a href="final_full_gait_test.py"><code>final_full_gait_test.py</code></a>, <a href="final_sensors.ino"><code>final_sensors.ino</code></a>, <a href="docs/software-map.md"><code>docs/software-map.md</code></a>, <a href="ARCHITECTURE.md"><code>ARCHITECTURE.md</code></a></td>
    <td>Main Python gait/navigation stack, Arduino sensor firmware, runtime modes, Brain/Heart process split, safety governors, and simulation hooks.</td>
  </tr>
  <tr>
    <td>Gait tuning and STS3215 telemetry</td>
    <td><a href="docs/gait-control.md#smart-servo-feedback-for-gait-changes">Smart-Servo Feedback</a>, <a href="docs/validation.md#servo-load-margin">Servo Load Margin</a></td>
    <td>How actuator telemetry was used as sensor input for phase error, load trends, speed feedback, voltage/current limits, temperature monitoring, and gait changes.</td>
  </tr>
  <tr>
    <td>Validation claims and limits</td>
    <td><a href="docs/validation.md"><code>docs/validation.md</code></a>, <a href="https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml">Simulation Checks</a></td>
    <td>Formal traversal outcomes, terrain categories, pilot-scale framing, known limits, and the automated simulation/pytest checkpoint.</td>
  </tr>
  <tr>
    <td>Mechanical design and fabrication</td>
    <td><a href="docs/hardware.md#cad-models">CAD Models</a>, <a href="docs/hardware.md#printed-body-assembly">Printed Body Assembly</a>, <a href="docs/hardware.md#tread-construction">Tread Construction</a></td>
    <td>CAD references, printed chassis/lid assembly, C-leg geometry, TPU layer, adhesive bumper tread construction, hot-glue V-tread pattern, and fabrication notes.</td>
  </tr>
  <tr>
    <td>BOM and purchasing traceability</td>
    <td><a href="docs/BOM.md"><code>docs/BOM.md</code></a>, <a href="docs/purchasing-bom.csv"><code>docs/purchasing-bom.csv</code></a></td>
    <td>Categorized purchasing table, source links, quantities, pack counts, subsystem notes, and integration notes for the final public build.</td>
  </tr>
  <tr>
    <td>Running, diagnostics, and reviewer commands</td>
    <td><a href="RUNNING.md"><code>RUNNING.md</code></a>, <a href="docs/software-map.md#main-program-modes">Main Program Modes</a></td>
    <td>Simulation commands, dry-run modes, hardware launch commands, Arduino firmware notes, safety checklist, and review-friendly execution paths.</td>
  </tr>
  <tr>
    <td>Sensor stack and data flow</td>
    <td><a href="docs/software-map.md#sensor-firmware-and-input">Sensor Firmware And Input</a>, <a href="ARCHITECTURE.md#sensor-pipeline">Sensor Pipeline</a></td>
    <td>Arduino Nano hub, ultrasonic sensors, BNO085 IMU input, Raspberry Pi handoff, watchdog behavior, and sensor integration points.</td>
  </tr>
  <tr>
    <td>Field demos and course videos</td>
    <td><a href="docs/media.md"><code>docs/media.md</code></a></td>
    <td>Full-resolution traversal videos, course-success runs, and GIF previews for sand, obstacle, indoor, and park-style navigation demos.</td>
  </tr>
  <tr>
    <td>Research outputs and symposium materials</td>
    <td><a href="#recognition-and-publications">Recognition And Publications</a>, <a href="docs/paper-presentation-slides.md"><code>docs/paper-presentation-slides.md</code></a></td>
    <td>Paper, presentation slides, poster, paper-presentation video, People's Choice video, award certificates, and symposium display context.</td>
  </tr>
  <tr>
    <td>Release history and public milestones</td>
    <td><a href="docs/releases.md"><code>docs/releases.md</code></a>, <a href="https://github.com/Robiswell/Euler_Rover_2026/releases/tag/media-assets">Portfolio Media Assets</a></td>
    <td>Final public release trail, earlier milestone summaries, and the GitHub release that hosts full-resolution PDFs and MP4s.</td>
  </tr>
  <tr>
    <td>Complete documentation map</td>
    <td><a href="docs/README.md"><code>docs/README.md</code></a></td>
    <td>One-page index of the supporting docs for hardware, BOM, software, validation, media, release history, and reviewer references.</td>
  </tr>
</table>
