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
    -1.0   VERY NEAR  — echo too short / in blind zone / sensor saturated
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
    gravity_z > 5.0 → rover is right-side up   → UpsideDown = 0
    gravity_z ≤ 5.0 → rover is physically inverted → UpsideDown = 1

  Wiring notes:
    - HC-SR04: share GND with Arduino; if MCU is 3.3 V add a voltage divider
      (1 kΩ + 2 kΩ) on each ECHO pin before connecting.
    - BNO085: connect SDA→A4, SCL→A5; power from 3.3 V rail.
    - Keep sensor wires short and grounds common to reduce crosstalk.

  Dependencies:
    SparkFun BNO080 Arduino Library  (install via Library Manager)

  Fixes applied (2026-03-13):
    FIX 1:  Rewrite measureClassified — single-pass rising/falling edge (no partial echo)
    FIX 2:  Timeout returns 300.0 (VERY FAR), not -1.0 (VERY NEAR)
    FIX 3:  TIMEOUT_TOTAL 60000→20000, SENSOR_GAP_MS 6→2, INTERVAL 100→0 (~5 Hz)
    FIX 4:  Always emit CSV row; use last-known IMU values on I2C miss
    FIX 5:  Wire.setWireTimeout + IMU reinit after 50 consecutive misses
    FIX 6:  UpsideDown uses gravity report instead of raw accelerometer
    FIX 7:  Pin 13 ECHO uses INPUT_PULLUP to counteract onboard LED load
    FIX 8:  drainEchoLow timeout 2000→40000 us (covers max HC-SR04 echo)
    FIX 9:  Timestamp captured after measurements, not before
    FIX 10: Sensor names moved to PROGMEM (~130 bytes SRAM saved)
    FIX 11: Switch from Rotation Vector to Game Rotation Vector (boot-relative,
            no magnetometer) — avoids 180° quaternion flip from upside-down IMU
            mounting that causes P1 NAV_STOP_SAFE to fire immediately.
            getQuat() replaced with getQuatI/J/K/Real() individual getters
            (shared rawQuat fields serve whichever RV type is enabled).
    FIX V0.5.01
    FIX 12: Blind-zone false-VERY-FAR fix — when step-C times out (no rising
            edge) the sensor cannot distinguish "nothing nearby" from "object
            < 2 cm (blind zone / transducer still ringing)".  Two heuristics
            are added to correctly classify these as VERY NEAR (-1.0):
              FIX 12a  Double-ping: fires a second quick trigger and polls echo
                       for NEAR_RECHECK_US (2000 µs ≈ 34 cm round-trip).  If
                       an echo pulse is detected on the retry the result is
                       measured and returned as -1.0 (short pulse) or the real
                       distance.  Catches sensors that need a second trigger to
                       respond after a blind-zone event.
              FIX 12b  Hysteresis: if distances_cm[idx] (the previous reading
                       for this sensor) is already < HYST_NEAR_CM (10 cm) a
                       timeout most likely means the object slid into the blind
                       zone, not that it vanished → return -1.0 immediately
                       without wasting time on a double-ping.
*/

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// -----------------------------------------------------------------------
// IMU
// -----------------------------------------------------------------------
BNO080 imu;

// -----------------------------------------------------------------------
// Sensor names in PROGMEM (FIX 10 — saves ~130 bytes SRAM)
// -----------------------------------------------------------------------
const char NAME_0[] PROGMEM = "FrontDiagonalLeft";
const char NAME_1[] PROGMEM = "FrontCenterForward";
const char NAME_2[] PROGMEM = "FrontCenterDown";
const char NAME_3[] PROGMEM = "FrontDiagonalRight";
const char NAME_4[] PROGMEM = "RearDiagonalLeft";
const char NAME_5[] PROGMEM = "RearCenterForward";
const char NAME_6[] PROGMEM = "RearCenterDown";
const char NAME_7[] PROGMEM = "RearDiagonalRight";

const char* const SENSOR_NAMES[8] PROGMEM = {
  NAME_0, NAME_1, NAME_2, NAME_3,
  NAME_4, NAME_5, NAME_6, NAME_7
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
// Loop scheduling (FIX 3 — self-throttled by measurement time, ~5 Hz)
// -----------------------------------------------------------------------
unsigned long prev_millis    = 0;
const unsigned long INTERVAL = 0;             // ms between CSV rows (0 = as fast as sensors allow)
const unsigned int  SENSOR_GAP_MS = 2;        // inter-sensor gap (FIX 3: reduced from 6)

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

// Total measurement timeout (FIX 3: 20 ms covers 340 cm, well beyond 300 cm limit)
const unsigned long TIMEOUT_TOTAL = 20000UL;
// FIX 0.5.01
// FIX 12a: Double-ping retry window after a step-C timeout.
//   Poll echo for at most NEAR_RECHECK_US µs on the second trigger.
//   2000 µs ≈ 34 cm round-trip — enough to catch blind-zone echoes.
const unsigned long NEAR_RECHECK_US = 2000UL;
//FIX V0.5.01
// FIX 12b: Hysteresis threshold.
//   If distances_cm[idx] (previous reading) was already within HYST_NEAR_CM,
//   a step-C timeout means the object entered the blind zone, not that it
//   disappeared — classify as VERY NEAR instead of VERY FAR.
const float HYST_NEAR_CM = 10.0f;


// -----------------------------------------------------------------------
// IMU state — last-known values (FIX 4 + FIX 6)
// -----------------------------------------------------------------------
int upside_down_flag = 0;   // 0 = normal, 1 = rover is physically inverted
float last_qw = 1.0f, last_qx = 0.0f, last_qy = 0.0f, last_qz = 0.0f;
float last_ax = 0.0f, last_ay = 0.0f, last_az = 9.81f;
float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
float last_gravity_z = 9.81f;   // gravity Z for upside-down detection (FIX 6)
unsigned long last_imu_update_ms = 0;
unsigned int imu_fail_count = 0;
const unsigned int IMU_FAIL_RESET = 50;  // reinit IMU after 50 consecutive misses

// -----------------------------------------------------------------------
// Utility: drain ECHO pin to LOW before triggering
// (FIX 8: timeout increased to 40 ms to cover max HC-SR04 echo)
// -----------------------------------------------------------------------
static inline bool drainEchoLow(int echoPin, unsigned long maxWait_us) {
  unsigned long t0 = micros();
  while (digitalRead(echoPin) == HIGH && (micros() - t0) < maxWait_us) {
    /* wait */
  }
  return digitalRead(echoPin) == LOW;
}

// -----------------------------------------------------------------------
// Single-sensor classified measurement (FIX 1 + FIX 2 rewrite)
// Single-pass: one rising-edge wait, one falling-edge wait.
// Returns: -1.0 (VERY NEAR), 300.0 (VERY FAR), or 2.0..300.0 (cm)
// -----------------------------------------------------------------------
float measureClassified(uint8_t idx) {
  const int trig = TRIG_PINS[idx];
  const int echo = ECHO_PINS[idx];

  // A) Drain residual echo; if stuck HIGH → saturated / very near
  if (!drainEchoLow(echo, 40000UL)) {   // FIX 8: 40 ms drain
    durations_us[idx] = 0;
    return -1.0f;
  }

  // B) Send 10 µs trigger pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  // C) Wait for ECHO to go HIGH (rising edge)
  //    HC-SR04 raises ECHO within ~200-450 µs of trigger (burst transmission).
  //    The distance is encoded in the HIGH duration, not the rising-edge delay.
  unsigned long t0 = micros();
  while (digitalRead(echo) == LOW) {
    if (micros() - t0 > TIMEOUT_TOTAL) {
      // No rising edge at all → sensor disconnected or fault → VERY FAR (FIX 2)
      durations_us[idx] = 0;
      return 300.0f;
    }
  }
  unsigned long rise_time = micros();

  // D) Measure full HIGH duration (falling edge = echo return or timeout)
  while (digitalRead(echo) == HIGH) {
    if (micros() - rise_time > TIMEOUT_TOTAL) {
      durations_us[idx] = (long)(micros() - rise_time);
      //return 300.0f;  // timeout → no echo → VERY FAR (FIX 2)
      //FIX 0.5.01
      // FIX 12b: Hysteresis — previous reading was already close, so the
      //   object most likely entered the HC-SR04 blind zone (< 2 cm) rather
      //   than disappearing.  Return VERY NEAR immediately.
      //if (distances_cm[idx] >= 0.0f && distances_cm[idx] < HYST_NEAR_CM) {
      if (distances_cm[idx] < HYST_NEAR_CM) {
        return -1.0f;
      }
      // FIX 12a: Double-ping — fire one more trigger and poll echo for a
      //   short window (NEAR_RECHECK_US).  Some HC-SR04 variants need a
      //   second trigger to respond after a blind-zone event, and some produce
      //   a brief echo that was missed on the first attempt.
      drainEchoLow(echo, 40000UL);
      digitalWrite(trig, LOW);
      delayMicroseconds(4);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      unsigned long t1 = micros();
      while (digitalRead(echo) == LOW) {
        if (micros() - t1 > NEAR_RECHECK_US) {
          return 300.0f;   // FIX 2: still no echo → VERY FAR
        }
      }
      // Echo detected on second ping — measure its duration.
      unsigned long rise2 = micros();
      while (digitalRead(echo) == HIGH) {
        if (micros() - rise2 > NEAR_RECHECK_US) break;
      }
      unsigned long dur2 = micros() - rise2;
      durations_us[idx] = (long)dur2;
      if (dur2 < NEAR_RT_US) return -1.0f;
      float d2 = (float)dur2 * SOUND_CM_PER_US * 0.5f;
      if (d2 < BLIND_ZONE_CM) return -1.0f;
      if (d2 > LIMIT_CM)      return 300.0f;
      if (d2 < 2.0f)  d2 = 2.0f;
      if (d2 > 300.0f) d2 = 300.0f;
      return d2;
    }
  }
  unsigned long dur = micros() - rise_time;
  durations_us[idx] = (long)dur;

  // F) Pulse too short → blind zone
  if (dur < NEAR_RT_US) {
    return -1.0f;
  }

  // G) Convert to distance
  float d = (float)dur * SOUND_CM_PER_US * 0.5f;

  // H) Extra blind-zone guard after conversion
  if (d < BLIND_ZONE_CM) {
    return -1.0f;
  }

  // I) Echo too late or distance over range limit → VERY FAR
  if (dur > LIMIT_RT_US || d > LIMIT_CM) {
    return 300.0f;
  }

  // J) Valid range: clamp to [2, 300]
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

  // FIX 0.5.01 - Changed from 3000 to 25000 
  // Use an I2C timeout to prevent the sketch from hanging forever if the IMU/bus stalls.
  //  I2C transfer can legitimately take more than 3 ms. If Wire resets the bus that early,
  //   imu.begin() can fail and you'll see "BNO085 not detected" even though the wiring is
  Wire.setWireTimeout(25000, true);  // FIX 5: 3 ms I2C timeout, reset bus on timeout

  // IMU initialisation (BNO085 at address 0x4A)
  if (!imu.begin(0x4A, Wire)) {
    Serial.println("# ERROR: BNO085 not detected — check wiring!");
    //  FIX 0.5.01 
    while (1) { delay(25000); } 
  }
  
  imu.enableGameRotationVector(50);  // 50 Hz quaternion — Game RV (no mag, boot-relative)
  imu.enableAccelerometer(50);
  imu.enableGyro(50);
  imu.enableGravity(50);          // FIX 6: gravity vector for UpsideDown detection

  // Ultrasonic pin setup (FIX 7: pin 13 uses INPUT_PULLUP)
  for (int i = 0; i < 8; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    if (ECHO_PINS[i] == 13) {
      pinMode(ECHO_PINS[i], INPUT_PULLUP);  // FIX 7: counteract LED load
    } else {
      pinMode(ECHO_PINS[i], INPUT);
    }
    digitalWrite(TRIG_PINS[i], LOW);
    durations_us[i]  = 0;
    distances_cm[i]  = 300.0f;   // default: very far (safe assumption)
  }

  // CSV header — must match the 20-column format expected by input_thread.py
  char buf[24];
  Serial.print("timestamp_ms");
  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    strcpy_P(buf, (char*)pgm_read_ptr(&SENSOR_NAMES[i]));  // FIX 10: read from PROGMEM
    Serial.print(buf);
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
  unsigned long loop_start = millis();   // FIX 9: separate from CSV timestamp

  if (loop_start - prev_millis < INTERVAL) {
    return;   // not yet time for the next row
  }
  prev_millis = loop_start;

  // --- Read 8 ultrasonic sensors sequentially (avoids crosstalk) ---
  for (int i = 0; i < 8; i++) {
    distances_cm[i] = measureClassified((uint8_t)i);
    delay(SENSOR_GAP_MS);
  }

  // --- Read IMU: update last-known values if fresh data available (FIX 4) ---
  // Drain all pending IMU reports so getters return the most recent values
  bool got_imu = false;
  while (imu.dataAvailable()) {
    got_imu = true;
  }
  if (got_imu) {
    last_qx = imu.getQuatI();
    last_qy = imu.getQuatJ();
    last_qz = imu.getQuatK();
    last_qw = imu.getQuatReal();
    last_ax = imu.getAccelX();
    last_ay = imu.getAccelY();
    last_az = imu.getAccelZ();
    last_gx = imu.getGyroX();
    last_gy = imu.getGyroY();
    last_gz = imu.getGyroZ();
    last_gravity_z = imu.getGravityZ();   // FIX 6
    last_imu_update_ms = loop_start;
    imu_fail_count = 0;
  } else {
    imu_fail_count++;
    if (imu_fail_count >= IMU_FAIL_RESET) {
      // FIX 5: IMU likely locked up — attempt reinit
      if (imu.begin(0x4A, Wire)) {
        imu.enableGameRotationVector(50);
        imu.enableAccelerometer(50);
        imu.enableGyro(50);
        imu.enableGravity(50);
      }
      imu_fail_count = 0;  // reset regardless to limit retry rate
    }
  }

  // UpsideDown detection using gravity report (FIX 6)
  // Gravity Z for upside-down-mounted IMU:
  //   Right-side up: gravity_z > +5 (gravity points "up" in IMU frame)
  //   Inverted:      gravity_z ≤ +5
  // NOTE: verify polarity on hardware — if IMU Z-axis points down in chassis,
  //       flip comparison to (last_gravity_z < -5.0f)
  upside_down_flag = (last_gravity_z > 5.0f) ? 0 : 1;

  // --- Emit CSV row with accurate timestamp (FIX 9) ---
  unsigned long now = millis();   // timestamp reflects when data is ready
  Serial.print(now);
  for (int i = 0; i < 8; i++) {
    Serial.print(",");
    Serial.print(distances_cm[i], 2);
  }
  Serial.print(",");
  Serial.print(last_qw, 6); Serial.print(",");
  Serial.print(last_qx, 6); Serial.print(",");
  Serial.print(last_qy, 6); Serial.print(",");
  Serial.print(last_qz, 6); Serial.print(",");
  Serial.print(last_ax, 4); Serial.print(",");
  Serial.print(last_ay, 4); Serial.print(",");
  Serial.print(last_az, 4); Serial.print(",");
  Serial.print(last_gx, 4); Serial.print(",");
  Serial.print(last_gy, 4); Serial.print(",");
  Serial.print(last_gz, 4); Serial.print(",");
  Serial.println(upside_down_flag);
}
