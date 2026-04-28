# Hardware Reference

This page collects the hardware details summarized in the main README: subsystem roles, measured platform geometry, CAD links, tread construction, and purchasing traceability.

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

## Measured Platform Geometry

- Body length: 511 mm
- Total width: 280 mm
- Static ground clearance: 74 mm
- Effective leg radius: 125 mm from servo shaft to outer contact surface
- Leg arc span: 195 degrees
- Estimated mass: 2 to 3 kg

## CAD Models

| Part | Onshape Link |
| --- | --- |
| Split lid | [Open model](https://cad.onshape.com/documents/4f3de965fea211f4280a3c9d/w/cdf7ccd18bab44fcea532f37/e/039c12e9b8634e9de48b237f) |
| Chassis | [Open model](https://cad.onshape.com/documents/b0cc0fb5d7d22bf167ea7f76/w/008ad764f5d31a5d18c51176/e/51bfb1e7ed0292f5d8418b8c) |
| C-leg | [Open model](https://cad.onshape.com/documents/da3a47429f3542b58cbfb9b8/w/d8b269a68f869584fe54d88e/e/9680572918098f7f67734634) |
| Leg adapters | [Open model](https://cad.onshape.com/documents/60e3d6ac247373a6c9d27099/w/3073c20af8fd58d7202c08cb/e/fac01f63525d9f291b8ab335) |

## Printed Body Assembly

The printed chassis and lid were each built from two PETG pieces. Before bonding, all glued surfaces were sanded to improve adhesion. J-B Weld Plastic Bonder was used to bond the two chassis halves and the two split-lid halves; the chassis joint also used cylindrical inserts through the alignment holes so the two printed halves had a more solid mechanical connection across the seam.

## Tread Construction

<p align="center">
  <img src="assets/identity-v-tread-legs.jpg" alt="Identity rover C-legs with adhesive bumper pads and hot-glue V tread" width="720">
</p>

The C-legs use a TPU layer between the rigid PETG arcs and the adhesive rubber bumper pads, with hot glue built up between and over the pads to form a repeated V-shaped tread. The pattern was inspired by the chevron-style grousers used on NASA rover wheels: each V gives the leg an angled edge to bite into loose sand, gravel, carpet, and packed soil while the TPU layer preserves compliance for the rolling C-leg motion.

## Purchasing Traceability

- [Categorized Bill of Materials](BOM.md)
- [Purchasing source CSV](purchasing-bom.csv)
