# Validation Reference

This page collects the validation details summarized in the main README: field results, terrain classification signals, servo load margin, simulation coverage, and known limits.

## Field Results

| Metric | Result |
| --- | --- |
| Formal validation trials | 31/32 successful |
| Observed traversal success | 96.9% |
| Terrain categories | 7 |
| Simulation tests | 40/40 passing |
| Steady-state servo load margin | At least 42% in formal trials |

The symposium paper treats the 32-trial validation set as a pilot study because the 95% confidence interval lower bound falls below 90%. Within that scope, the data support high observed traversal reliability across tile, carpet, packed earth, loose sand, gravel, stone fields, and 20-degree inclines.

## Terrain Classification Signals

| Signal | Used For |
| --- | --- |
| IMU pitch and roll | Incline, tilt, and recovery logic |
| Angular-rate and ultrasonic stability | Rough-terrain classification |
| Sustained servo load | Deep-sand detection |
| Downward ultrasonic distance changes | Cliff/drop-off detection |

## Servo Load Margin

Across the validation terrain set, measured servo loads stayed below the configured stall threshold.

![Servo load margin by terrain type with stall threshold](assets/stall-threshold-load-margin.jpg)

## Simulation Coverage

The simulation framework contains 40 automated checks:

- 14 kinematic and gait checks in `sim_verify.py`
- 14 terrain and governor scenarios in `sim_terrain.py`
- 12 navigation FSM categories in `sim_nav.py`

At the time of the symposium paper, the full suite passed 40/40 checks. The simulation validates timing, state transitions, terrain overlays, and control invariants, but it does not model all physical effects such as compliance, backlash, or deformable terrain.

[GitHub Actions](https://github.com/Robiswell/Euler_Rover_2026/actions/workflows/simulation.yml) runs these simulation checks and the pytest regression suite on code, firmware, dependency, workflow, pull request, and manual-dispatch events.

## Known Limits

| Category | Limit |
| --- | --- |
| Validation scope | The 32-trial validation set supports pilot-scale reliability claims, not a fully powered statistical proof. |
| Reproducibility | Late-stage validation did not preserve exact per-trial software hashes. |
| Simulation fidelity | The simulation framework validates timing, state transitions, terrain overlays, and control invariants, but does not model all physical compliance, backlash, or deformable-terrain effects. |
| Hardware behavior | The clearest remaining failure mode was abrupt terrain transition within a single stride. |
| Application claims | Search-and-rescue and planetary robotics are future application targets, not demonstrated deployment domains. |
