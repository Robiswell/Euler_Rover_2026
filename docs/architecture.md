# Architecture Overview

This page routes reviewers to the main architecture references for the Identity rover software stack.

## Primary Architecture Document

See [`../ARCHITECTURE.md`](../ARCHITECTURE.md) for the full software architecture map, including the Brain/Heart process split, sensor pipeline, terrain overlay, safety governors, and simulation coverage.

## Runtime Split

![Brain/Heart process split diagram](assets/brain-heart-process-split.jpg)

The Brain process handles sensor interpretation, terrain classification, obstacle/cliff logic, and navigation state transitions. The Heart process runs the timing-critical gait loop, computes servo commands, reads STS3215 position/load/speed telemetry, applies safety governors, and keeps motor control isolated from slower navigation work.

## System Diagram

![Identity rover system architecture diagram](assets/symposium-system-architecture.jpg)
