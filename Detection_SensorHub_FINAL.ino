/*
  rhex_logger.ino
  RHex Rover — 8 Ultrasonic Sensors + BNO085 IMU → CSV over Serial

  Sensor ordering (matches Python distances array):
    Index 0  FrontDiagonalLeft   (FDL)
    Index 1  FrontCenterForward  (FCF)
    Index 2  FrontCenterDown     (FCD)  ← cliff / ground sensor
    Index 3  FrontDiagonalRight  (FDR)
    Index 4  RearDiagonalLeft    (RDL)
    Index 5  RearCenterForward   (RCF)
    Index 6  RearCenterDown      (RCD)
    Index 7  RearDiagonalRight   (RDR)

  Per-sensor classification (output values):
    -1.0   VERY NEAR  — echo too short / in blind zone / no pulse
    300.0  VERY FAR   — echo timeout or distance > 300 cm
    2–300  valid distance in cm (clamped)

  CSV columns (20 total):
    timestamp_ms,
    FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR,   (8 distances, cm)
    QuatW, QuatX, QuatY, QuatZ,
    AccelX, AccelY, AccelZ,                     (m/s²)
    GyroX,  GyroY,  GyroZ,                      (rad/s)
    UpsideDown                                   (0=normal, 1=inverted)

  UpsideDown detection (IMU mounted upside-down in chassis):
    az > 7.0 → rover is right-side up   → UpsideDown = 0
    az ≤ 7.0 → rover is physically inverted → UpsideDown = 1

  Wiring notes:
    - HC-SR04: share GND with Arduino; if MCU is 3.3 V add a voltage divider
      (1 kΩ + 2 kΩ) on each ECHO pin before connecting.
    - BNO085: connect SDA→A4, SCL→A5; power from 3.3 V rail.
    - Keep sensor wires short and grounds common to reduce crosstalk.

  Dependencies:
    SparkFun BNO080 Arduino Library  (install via Library Manager)
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// pulseInLong is not defined on all cores — fall back to pulseIn
#ifndef pulseInLong
#define pulseInLong pulseIn
#endif

// -----------------------------------------------------------------------
// IMU
// -----------------------------------------------------------------------
BNO080 imu;

// -----------------------------------------------------------------------
// Sensor names used in the CSV header
// -----------------------------------------------------------------------
const char* SENSOR_NAMES[8] = {
  "FrontDiagonalLeft",
  "FrontCenterForward",
  "FrontCenterDown",
  "FrontDiagonalRight",
  "RearDiagonalLeft",
  "RearCenterForward",
  "RearCenterDown",
  "RearDiagonalRight"
};

// -----------------------------------------------------------------------
// Ultrasonic pin assignments (TRIG / ECHO pairs)
// -----------------------------------------------------------------------
const int TRIG_PINS[8] = { 2,  4,  6,  8, 10, 12, A0, A2 };
const int ECHO_PINS[8] = { 3,  5,  7,  9, 11, 13, A1, A3 };

// -----------------------------------------------------------------------
// Measurement buffers
// -----------------------------------------------------------------------
long  durations_us[8];    // raw measured pulse width (µs)
float distances_cm[8];    // classified output: -1, 2..300, or 300

// -----------------------------------------------------------------------
// Loop scheduling
// -----------------------------------------------------------------------
unsigned long prev_millis    = 0;
const unsigned long INTERVAL = 100;          // ms between CSV rows
const unsigned int  SENSOR_GAP_MS = 6;       // inter-sensor gap (crosstalk guard)

// -----------------------------------------------------------------------
// Ultrasonic physics + classification thresholds
// -----------------------------------------------------------------------
const float SOUND_CM_PER_US = 0.0343f;       // speed of sound ~20 °C

const float BLIND_ZONE_CM   = 3.0f;          // < 3 cm → VERY NEAR
const float LIMIT_CM        = 300.0f;        // clamped upper bound

// Round-trip time equivalents
const unsigned long NEAR_RT_US  =
    (unsigned long)((2.0f * BLIND_ZONE_CM) / SOUND_CM_PER_US);  // ~175 µs
const unsigned long LIMIT_RT_US =
    (unsigned long)((2.0f * LIMIT_CM)      / SOUND_CM_PER_US);  // ~17489 µs

// Early-window check: HIGH within this window after trigger → VERY NEAR
const unsigned long EARLY_WINDOW_US = 300UL;

// Total measurement timeout (60 ms → stable "300" classification on open air)
const unsigned long TIMEOUT_TOTAL = 60000UL;

// -----------------------------------------------------------------------
// IMU state
// -----------------------------------------------------------------------
int upside_down_flag = 0;   // 0 = normal, 1 = rover is physically inverted

// -----------------------------------------------------------------------
// Utility: drain ECHO pin to LOW before triggering
// -----------------------------------------------------------------------
static inline bool drainEchoLow(int echoPin, unsigned long maxWait_us) {
  unsigned long t0 = micros();
  while (digitalRead(echoPin) == HIGH && (micros() - t0) < maxWait_us) {
    /* wait */
  }
  return digitalRead(echoPin) == LOW;
}

// -----------------------------------------------------------------------
// Single-sensor classified measurement
// Returns: -1.0 (VERY NEAR), 300.0 (VERY FAR), or 2.0..300.0 (cm)
// -----------------------------------------------------------------------
float measureClassified(uint8_t idx) {
  const int trig = TRIG_PINS[idx];
  const int echo = ECHO_PINS[idx];

  // A) Drain residual echo; if stuck HIGH → treat as saturated / very near
  if (!drainEchoLow(echo, 2000UL)) {
    durations_us[idx] = 0;
    return -1.0f;
  }

  // B) Send 10 µs trigger pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // C) Early-window check: any echo within EARLY_WINDOW_US → VERY NEAR
  unsigned long early = pulseInLong(echo, HIGH, EARLY_WINDOW_US);
  if (early > 0) {
    durations_us[idx] = (long)early;
    return -1.0f;
  }

  // D) Full measurement with long timeout
  unsigned long dur = pulseInLong(echo, HIGH, TIMEOUT_TOTAL);
  durations_us[idx] = (long)dur;

  if (dur == 0) {
    // No echo within timeout → treat as VERY NEAR (something absorbing the pulse)
    return -1.0f;
  }

  // E) Pulse too short → blind zone
  if (dur < NEAR_RT_US) {
    return -1.0f;
  }

  // F) Convert to distance
  float d = (float)dur * SOUND_CM_PER_US * 0.5f;

  // Extra blind-zone guard after conversion
  if (d < BLIND_ZONE_CM) {
    return -1.0f;
  }

  // G) Echo too late or distance over range limit → VERY FAR
  if (dur > LIMIT_RT_US || d > LIMIT_CM) {
    return 300.0f;
  }

  // H) Valid range: clamp to [2, 300]
  if (d < 2.0f)   d = 2.0f;
  if (d > 300.0f) d = 300.0f;
  return d;
}

// -----------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // IMU initialisation (BNO085 at address 0x4A)
  if (!imu.begin(0x4A, Wire)) {
    Serial.println("# ERROR: BNO085 not detected — check wiring!");
    while (1) { delay(1000); }
  }
  imu.enableRotationVector(50);   // 50 Hz quaternion output
  imu.enableAccelerometer(50);
  imu.enableGyro(50);

  // Ultrasonic pin setup
  for (int i = 0; i < 8; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    digitalWrite(TRIG_PINS[i], LOW);
    durations_us[i]  = 0;
    distances_cm[i]  = 300.0f;   // default: very far (safe assumption)
  }

  // CSV header — must match the 20-column format expected by input_thread.py
  Serial.print("timestamp_ms");
  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    Serial.print(SENSOR_NAMES[i]);
  }
  Serial.println(",QuatW,QuatX,QuatY,QuatZ"
                 ",AccelX,AccelY,AccelZ"
                 ",GyroX,GyroY,GyroZ"
                 ",UpsideDown");
}

// -----------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------
void loop() {
  unsigned long now = millis();

  if (now - prev_millis < INTERVAL) {
    return;   // not yet time for the next row
  }
  prev_millis = now;

  // --- Read 8 ultrasonic sensors sequentially (avoids crosstalk) ---
  for (int i = 0; i < 8; i++) {
    distances_cm[i] = measureClassified((uint8_t)i);
    delay(SENSOR_GAP_MS);
  }

  // --- Read IMU (only print CSV row when fresh data is available) ---
  if (!imu.dataAvailable()) {
    return;   // no new IMU sample this cycle — skip row to keep data fresh
  }

  float qx, qy, qz, qw, radAcc;
  uint8_t acc;
  imu.getQuat(qx, qy, qz, qw, radAcc, acc);

  float ax = imu.getAccelX();
  float ay = imu.getAccelY();
  float az = imu.getAccelZ();

  float gx = imu.getGyroX();
  float gy = imu.getGyroY();
  float gz = imu.getGyroZ();

  // UpsideDown detection for upside-down-mounted IMU:
  //   When the rover is right-side up the IMU (mounted flipped) reports az > +7.
  //   When the rover is physically inverted az drops below the threshold.
  upside_down_flag = (az > 7.0f) ? 0 : 1;

  // --- Emit CSV row ---
  Serial.print(now);
  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    Serial.print(distances_cm[i], 2);
  }
  Serial.print(",");
  Serial.print(qw, 6); Serial.print(",");
  Serial.print(qx, 6); Serial.print(",");
  Serial.print(qy, 6); Serial.print(",");
  Serial.print(qz, 6); Serial.print(",");
  Serial.print(ax, 4); Serial.print(",");
  Serial.print(ay, 4); Serial.print(",");
  Serial.print(az, 4); Serial.print(",");
  Serial.print(gx, 4); Serial.print(",");
  Serial.print(gy, 4); Serial.print(",");
  Serial.print(gz, 4); Serial.print(",");
  Serial.println(upside_down_flag);
}
