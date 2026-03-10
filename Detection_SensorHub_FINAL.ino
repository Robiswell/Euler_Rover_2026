/*
  RHex Rover Logger with 8 Ultrasonics + BNO085 (with "very-near" priority logic)

  - 8 ultrasonic sensors (FrontDiagonalLeft → RearDiagonalRight)
  - Per-sensor robust classification (based on user's single-sensor logic):
      -1    => VERY NEAR (blind zone / no pulse / too-short pulse / early activity)
      300   => VERY FAR (distance > 300 cm or late echo)
      2..300=> valid distance in cm (clamped to [2..300])
  - BNO085 IMU readings (quaternion, accel, gyro)
  - UpsideDown detection adjusted for upside-down-mounted IMU
  - CSV output for logging to Raspberry Pi

  Notes:
  - Ultrasonics are read sequentially to avoid crosstalk; a small inter-sensor delay is used.
  - Keep wiring short and grounds common. If MCU is 3.3V, use a divider on HC-SR04 ECHO.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#ifndef pulseInLong
#define pulseInLong pulseIn
#endif

// ------------------------------- IMU ---------------------------------------
BNO080 imu;

// Sensor names (CSV columns)
const char* sensorNames[8] = {
  "FrontDiagonalLeft","FrontCenterForward","FrontCenterDown","FrontDiagonalRight",
  "RearDiagonalLeft","RearCenterForward","RearCenterDown","RearDiagonalRight"
};


// Ultrasonic pin assignments 
const int trigPins[8] = {2, 4, 6, 8, 10, 12, A0, A2};
const int echoPins[8] = {3, 5, 7, 9, 11, 13, A1, A3};

// Measurement buffers
long  durations_us[8];  // raw high pulse width (us), for reference if needed
float distances_cm[8];  // classified outputs: -1, 2..300, or 300

// Loop scheduling
unsigned long previousMillis = 0;
const unsigned long interval = 100;  // 100 ms between CSV rows
const unsigned int  perSensorGapMs = 6; // small gap to mitigate ultrasonic crosstalk

// ----------------------- Ultrasonic physics and thresholds -----------------
/*
  The logic below mirrors the user's single-sensor code:

    A) Ensure ECHO is LOW before triggering (drain residual echoes up to ~2 ms)
    B) Trigger pulse (10 us)
    C) Early window: if ECHO goes HIGH within EARLY_WINDOW_US => VERY NEAR (-1)
    D) Normal pulse measurement with a long timeout (TIMEOUT_TOTAL)
       - dur == 0        => -1 (no pulse)
       - dur < NEAR_RT_US=> -1 (too short: blind zone)
    E) Convert to cm, extra blind-zone check
    F) dur > LIMIT_RT_US or d > LIMIT_CM => 300
    G) Otherwise clamp to [2..300]
*/

// Speed of sound (~20°C)
const float SOUND_CM_PER_US = 0.0343f; // cm/us

// Blind zone and limits
const float ZONA_CIEGA_CM   = 3.0f;    // <3 cm => VERY NEAR
const float LIMITE_CM       = 300.0f;  // clamp upper bound => 300

// Round-trip thresholds (microseconds)
const unsigned long NEAR_RT_US   = (unsigned long)((2.0f * ZONA_CIEGA_CM) / SOUND_CM_PER_US);  // ~175 us (3 cm)
const unsigned long LIMITE_RT_US = (unsigned long)((2.0f * LIMITE_CM)   / SOUND_CM_PER_US);    // ~17489 us (300 cm)

// Early window: prioritize VERY NEAR over late echoes
const unsigned long EARLY_WINDOW_US = 300UL;     // use 300 us as per original convention

// Timeouts / pacing
const unsigned long TIMEOUT_TOTAL = 60000UL;     // 60 ms helps stabilize the "300" far classification

// ------------------------------- IMU config --------------------------------
// IMU (BNO085) at 0x4A (per base). Upside-down logic using accel-Z (m/s^2).
// IMU mounted upside-down -> upsideDown if az > 7.0 (as in base).
int upsideDown = 0; // 1 if upside down, else 0

// -------------------------------- Utilities --------------------------------
static inline bool drainEchoToLow(int echoPin, unsigned long max_wait_us) {
  unsigned long t0 = micros();
  while (digitalRead(echoPin) == HIGH && (micros() - t0) < max_wait_us) {
    // wait for ECHO to go LOW (drain residual)
  }
  return (digitalRead(echoPin) == LOW);
}

// Single-sensor classified measurement (returns: -1, 300, or 2..300)
float measureClassified(uint8_t idx) {
  const int trig = trigPins[idx];
  const int echo = echoPins[idx];

  // A) Ensure ECHO is LOW before triggering; if it won't go LOW, treat as VERY NEAR
  if (!drainEchoToLow(echo, 2000UL)) {
    // echo stuck HIGH pre-trigger => consider near (absorbing/saturated)
    durations_us[idx] = 0;
    return -1.0f;
  }

  // B) Trigger 10 us
  digitalWrite(trig, LOW); delayMicroseconds(4);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // C) Early window: any early HIGH => VERY NEAR (-1)
  unsigned long early = pulseInLong(echo, HIGH, EARLY_WINDOW_US);
  if (early > 0) {
    durations_us[idx] = early;
    return -1.0f;
  }

  // D) Normal pulse (may be late). If none => VERY NEAR
  unsigned long dur = pulseInLong(echo, HIGH, TIMEOUT_TOTAL);
  durations_us[idx] = dur;

  if (dur == 0) {
    // No pulse detected within timeout => near by convention
    return -1.0f;
  }

  // E) Extremely short pulse => VERY NEAR
  if (dur < NEAR_RT_US) {
    return -1.0f;
  }

  // Convert to cm (round-trip / 2)
  float d = dur * SOUND_CM_PER_US * 0.5f;

  // Additional blind-zone guard
  if (d < ZONA_CIEGA_CM) {
    return -1.0f;
  }

  // F) Late or > 300 cm => 300
  if (dur > LIMITE_RT_US || d > LIMITE_CM) {
    return 300.0f;
  }

  // G) Valid range: clamp [2..300]
  if (d < 2.0f)   d = 2.0f;
  if (d > 300.0f) d = 300.0f;

  return d;
}

// ----------------------------------- SETUP ---------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // IMU init (0x4A as in base)
  if (!imu.begin(0x4A, Wire)) {
    Serial.println("BNO085 not detected!");
    while (1);
  }

  imu.enableRotationVector(50); // 50 Hz
  imu.enableAccelerometer(50);
  imu.enableGyro(50);

  // Ultrasonic pins
  for (int i = 0; i < 8; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
    digitalWrite(trigPins[i], LOW);
    durations_us[i] = 0;
    distances_cm[i] = 300.0f; // default far
  }

  // CSV header (kept identical to base)
  Serial.print("timestamp_ms");
  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    Serial.print(sensorNames[i]);
  }
  Serial.println(",QuatW,QuatX,QuatY,QuatZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,UpsideDown");
}

// ------------------------------------ LOOP ---------------------------------
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // --- Read 8 ultrasonics sequentially to avoid crosstalk ---
    for (int i = 0; i < 8; i++) {
      distances_cm[i] = measureClassified((uint8_t)i);
      delay(perSensorGapMs); // small gap between sensors
    }

    // --- Read IMU (if new data available) ---
    if (imu.dataAvailable()) {
      float qx, qy, qz, qw, radAccuracy;
      uint8_t accuracy;
      imu.getQuat(qx, qy, qz, qw, radAccuracy, accuracy);

      float ax = imu.getAccelX(); // m/s^2
      float ay = imu.getAccelY();
      float az = imu.getAccelZ();

      float gx = imu.getGyroX();  // rad/s
      float gy = imu.getGyroY();
      float gz = imu.getGyroZ();

      // Upside-down detection for an upside-down-mounted IMU (same criterion as base)
      upsideDown = (az > 7.0f) ? 0 : 1;

      // --- Print CSV row (same structure as base) ---
      Serial.print(currentMillis);
      for (int i = 0; i < 8; i++) {
        Serial.print(",");
        Serial.print(distances_cm[i], 2); // prints -1.00, 300.00, or valid distance
      }
      Serial.print(",");
      Serial.print(qw); Serial.print(","); Serial.print(qx); Serial.print(","); Serial.print(qy); Serial.print(","); Serial.print(qz);
      Serial.print(",");
      Serial.print(ax); Serial.print(","); Serial.print(ay); Serial.print(","); Serial.print(az);
      Serial.print(",");
      Serial.print(gx); Serial.print(","); Serial.print(gy); Serial.print(","); Serial.print(gz);
      Serial.print(",");
      Serial.println(upsideDown);
    }
  }
}
