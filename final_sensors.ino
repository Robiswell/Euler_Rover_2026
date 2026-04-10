  /*
    rhex_logger.ino
    RHex Rover — 8 Ultrasonic Sensors + BNO085 IMU → CSV over Serial

    Sensor ordering (matches Python distances array):
      Index 0  FrontDiagonalLeft   (FDL)
      Index 1  FrontCenterForward  (FCF)
      Index 2  FrontCenterDown     (FCD)  <- cliff / ground sensor
      Index 3  FrontDiagonalRight  (FDR)
      Index 4  RearDiagonalLeft    (RDL)
      Index 5  RearCenterForward   (RCF)
      Index 6  RearCenterDown      (RCD)
      Index 7  RearDiagonalRight   (RDR)

    Per-sensor classification (output values):
      -2.0   FAULT      -- no rising edge / sensor disconnected or wiring fault
      -1.0   VERY NEAR  -- echo too short / in blind zone / sensor saturated
      300.0  VERY FAR   -- echo timeout or distance > 300 cm
      2-300  valid distance in cm (clamped)

    CSV columns (20 total):
      timestamp_ms,
      FDL, FCF, FCD, FDR, RDL, RCF, RCD, RDR,   (8 distances, cm)
      QuatW, QuatX, QuatY, QuatZ,
      AccelX, AccelY, AccelZ,                     (m/s^2)
      GyroX,  GyroY,  GyroZ,                      (rad/s)
      UpsideDown                                   (0=normal, 1=inverted)

    UpsideDown detection (IMU mounted face-up in chassis):
      gravity_z > 5.0 -> rover is right-side up   -> UpsideDown = 0
      gravity_z <= 0   -> rover is physically inverted -> UpsideDown = 1

    Wiring notes:
      - HC-SR04: share GND with Arduino; if MCU is 3.3 V add a voltage divider
        (1k + 2k) on each ECHO pin before connecting.
      - BNO085: connect SDA->A4, SCL->A5; power from 3.3 V rail.
      - Keep sensor wires short and grounds common to reduce crosstalk.

    Dependencies:
      SparkFun BNO080 Arduino Library  (install via Library Manager)

    Fixes applied (2026-03-13):
      FIX 1:  Rewrite measureClassified -- single-pass rising/falling edge (no partial echo)
      FIX 2:  Timeout returns 300.0 (VERY FAR), not -1.0 (VERY NEAR)
      FIX 3:  TIMEOUT_TOTAL 60000->20000, SENSOR_GAP_MS 6->2, INTERVAL 100->0 (~5 Hz)
      FIX 4:  Always emit CSV row; use last-known IMU values on I2C miss
      FIX 5:  Wire.setWireTimeout + IMU reinit after 50 consecutive misses
      FIX 6:  UpsideDown uses gravity report instead of raw accelerometer
      FIX 7:  Pin 13 ECHO uses INPUT_PULLUP to counteract onboard LED load
      FIX 8:  drainEchoLow timeout 2000->40000 us (covers max HC-SR04 echo)
      FIX 9:  Timestamp captured after measurements, not before
      FIX 10: Sensor names moved to PROGMEM (~130 bytes SRAM saved)
      FIX 11: Switch from Rotation Vector to Game Rotation Vector (boot-relative,
              no magnetometer) -- avoids absolute orientation encoding that
              caused P1 NAV_STOP_SAFE to fire immediately on some boot poses.
              getQuat() replaced with getQuatI/J/K/Real() individual getters
              (shared rawQuat fields serve whichever RV type is enabled).
      FIX 12: Blind-zone false-VERY-FAR fix -- when step-D times out (echo HIGH
              too long) the sensor cannot distinguish "nothing nearby" from "object
              < 2 cm (blind zone / transducer still ringing)".  Two heuristics:
                FIX 12a  Double-ping: fires a second quick trigger and polls echo
                        for NEAR_RECHECK_US (2000 us = 34 cm round-trip).  If
                        an echo pulse is detected on the retry the result is
                        measured and returned as -1.0 (short pulse) or the real
                        distance.
                FIX 12b  Hysteresis: if distances_cm[idx] (the previous reading
                        for this sensor) was a valid positive value < HYST_NEAR_CM
                        (10 cm), a timeout most likely means the object slid into
                        the blind zone -> return -1.0 immediately.
                        NOTE: the >= 0.0 guard is critical -- without it, a
                        previous -1.0 reading (which is < 10) would lock the
                        sensor at -1 permanently via feedback loop.
      FIX 13: Step-C timeout (no rising edge) returns -2.0 (FAULT) instead of
              300.0, distinguishing sensor disconnect from out-of-range.
      FIX 14: UpsideDown uses 2-consecutive-reading hysteresis to avoid flicker.
      FIX 15: IMU drain capped at 10 iterations to prevent stall.
      FIX 16: Wire.clearWireTimeoutFlag() in setup and IMU reinit.
      FIX 17: FIX 12a drain uses NEAR_RECHECK_US (not 40 ms) + pre-check for
              echo still HIGH before retrigger.
      FIX 18: Double-ping echo timeout (break) fell through to distance calc,
              producing false 34 cm readings.  Changed break -> return 300.0f.
      FIX 19: SENSOR_DISABLED bitmask skips FAULT sensors (idx 1 + 5).
              Reports -2.0 without blocking on dead echoes.  Saves ~40ms/frame.
      FIX 20: All ECHO pins set to INPUT_PULLUP to reject EMI false edges
              from 1 Mbaud RS-485 servo bus on floating pins.
      FIX 21: Real-time 4-layer sensor filter -- ring buffer (5 readings per
              sensor) + median of valid readings + rate limit + consecutive
              FAULT counter with auto-recovery.  Supersedes FIX 19 hard-disable:
              all 8 sensors stay active; bad readings are filtered dynamically;
              a sensor that starts reading well again recovers on its own
              without a reflash.  The old SENSOR_DISABLED bitmask is retained
              as sensor_disabled_mask with default 0 -- emergency manual
              override only, for physically destroyed sensors.
  */

  #include <Arduino.h>
  #include <Wire.h>
  #include <SparkFun_BNO080_Arduino_Library.h>

  // -----------------------------------------------------------------------
  // IMU
  // -----------------------------------------------------------------------
  BNO080 imu;

  // -----------------------------------------------------------------------
  // Sensor names in PROGMEM (FIX 10 -- saves ~130 bytes SRAM)
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

  // FIX 21: Real-time 4-layer sensor filter state (supersedes FIX 19 hard-disable).
  // Ring buffer of 5 raw readings per sensor + median + rate limit + FAULT counter.
  static float   sensor_history[8][5];        // 160 B -- ring buffer of raw readings
  static uint8_t ring_pos[8];                  //   8 B -- next write index into history
  static uint8_t fault_count[8];               //   8 B -- consecutive FAULT readings (L4)
  static uint8_t rate_reject_count[8];         //   8 B -- consecutive rate-limit rejects (L3)
  static float   last_output[8];               //  32 B -- last filtered value emitted
  static const float   HISTORY_EMPTY   = -1000.0f;  // sentinel: slot never written
  static const uint8_t FAULT_THRESHOLD = 10;        // ~2 s at 5 Hz -> declare dead
  static const uint8_t RATE_REJECT_MAX = 3;         // accept after 3 rejections (escape hatch)
  static const float   RATE_LIMIT_CM   = 100.0f;    // max plausible per-frame change (cm)

  // FIX 19 (retained as emergency override): bitmask of sensors to skip entirely.
  // Default = 0: all 8 sensors active, filter handles intermittent FAULT automatically.
  // Set bit N to 1 ONLY if sensor N is physically destroyed and cannot be repaired.
  uint8_t sensor_disabled_mask = 0;

  // -----------------------------------------------------------------------
  // Measurement buffers
  // -----------------------------------------------------------------------
  long  durations_us[8];    // raw measured pulse width (us)
  float distances_cm[8];    // classified output: -2, -1, 2..300, or 300

  // -----------------------------------------------------------------------
  // Loop scheduling (FIX 3 -- self-throttled by measurement time, ~5 Hz)
  // -----------------------------------------------------------------------
  unsigned long prev_millis    = 0;
  const unsigned long INTERVAL = 50;            // ms between CSV rows (50ms = 20Hz cap, prevents Serial flood on sensor disconnect)
  const unsigned int  SENSOR_GAP_MS = 2;        // inter-sensor gap (FIX 3: reduced from 6)

  // -----------------------------------------------------------------------
  // Ultrasonic physics + classification thresholds
  // -----------------------------------------------------------------------
  const float SOUND_CM_PER_US = 0.0343f;       // speed of sound ~20 C

  const float BLIND_ZONE_CM   = 3.0f;          // < 3 cm -> VERY NEAR
  const float LIMIT_CM        = 300.0f;        // clamped upper bound

  // Round-trip time equivalents
  const unsigned long NEAR_RT_US  =
      (unsigned long)((2.0f * BLIND_ZONE_CM) / SOUND_CM_PER_US);  // ~175 us
  const unsigned long LIMIT_RT_US =
      (unsigned long)((2.0f * LIMIT_CM)      / SOUND_CM_PER_US);  // ~17489 us

  // Total measurement timeout (FIX 3: 20 ms covers 340 cm, well beyond 300 cm limit)
  const unsigned long TIMEOUT_TOTAL = 20000UL;

  // FIX 12a: Double-ping retry window after a step-D timeout.
  //   Poll echo for at most NEAR_RECHECK_US us on the second trigger.
  //   2000 us = 34 cm round-trip -- enough to catch blind-zone echoes.
  const unsigned long NEAR_RECHECK_US = 2000UL;

  // FIX 12b: Hysteresis threshold.
  //   If distances_cm[idx] (previous reading) was a valid positive value
  //   within HYST_NEAR_CM, a step-D timeout means the object entered the
  //   blind zone, not that it disappeared -> classify as VERY NEAR.
  const float HYST_NEAR_CM = 10.0f;


  // -----------------------------------------------------------------------
  // IMU state -- last-known values (FIX 4 + FIX 6)
  // -----------------------------------------------------------------------
  int upside_down_flag = 0;   // 0 = normal, 1 = rover is physically inverted
  float last_qw = 1.0f, last_qx = 0.0f, last_qy = 0.0f, last_qz = 0.0f;
  float last_ax = 0.0f, last_ay = 0.0f, last_az = 9.81f;
  float last_gx = 0.0f, last_gy = 0.0f, last_gz = 0.0f;
  float last_gravity_z = 9.81f;   // gravity Z for upside-down detection (FIX 6)
  unsigned long last_imu_update_ms = 0;
  unsigned int imu_fail_count = 0;
  const unsigned int IMU_FAIL_RESET = 50;  // reinit IMU after 50 consecutive misses
  static uint8_t upside_down_consec = 0;   // FIX 14: hysteresis counter

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
  // Returns: -2.0 (FAULT), -1.0 (VERY NEAR), 300.0 (VERY FAR), or 2.0..300.0 (cm)
  // -----------------------------------------------------------------------
  float measureClassified(uint8_t idx) {
    const int trig = TRIG_PINS[idx];
    const int echo = ECHO_PINS[idx];

    // A) Drain residual echo; if stuck HIGH -> saturated / very near
    if (!drainEchoLow(echo, 40000UL)) {   // FIX 8: 40 ms drain
      durations_us[idx] = 0;
      return -1.0f;
    }

    // B) Send 10 us trigger pulse
    digitalWrite(trig, LOW);
    delayMicroseconds(4);
    digitalWrite(trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig, LOW);

    // C) Wait for ECHO to go HIGH (rising edge)
    //    HC-SR04 raises ECHO within ~200-450 us of trigger (burst transmission).
    //    The distance is encoded in the HIGH duration, not the rising-edge delay.
    unsigned long t0 = micros();
    while (digitalRead(echo) == LOW) {
      if (micros() - t0 > TIMEOUT_TOTAL) {
        // FIX 13: No rising edge -> sensor disconnected or wiring fault
        durations_us[idx] = 0;
        return -2.0f;
      }
    }
    unsigned long rise_time = micros();

    // D) Measure full HIGH duration (falling edge = echo return or timeout)
    while (digitalRead(echo) == HIGH) {
      if (micros() - rise_time > TIMEOUT_TOTAL) {
        durations_us[idx] = (long)(micros() - rise_time);

        // FIX 12b: Hysteresis -- previous reading was a valid close distance,
        //   so the object most likely entered the HC-SR04 blind zone (< 2 cm)
        //   rather than disappearing.  Return VERY NEAR immediately.
        //   The >= 0.0f guard prevents -1.0 or -2.0 from triggering hysteresis,
        //   which would lock the sensor at -1 permanently (feedback loop).
        if (distances_cm[idx] >= 0.0f && distances_cm[idx] < HYST_NEAR_CM) {
          return -1.0f;
        }

        // FIX 12a + FIX 17: Double-ping -- fire one more trigger and poll echo
        //   for a short window.  Must drain for 40 ms (HC-SR04 internal timeout
        //   is ~38 ms) -- a 2 ms drain is too short and leaves echo HIGH for
        //   far-away objects, incorrectly returning -1.
        drainEchoLow(echo, 40000UL);
        if (digitalRead(echo) == HIGH) {
          return 300.0f;  // HC-SR04 still holding echo -- object is far, not near
        }
        digitalWrite(trig, LOW);
        delayMicroseconds(4);
        digitalWrite(trig, HIGH);
        delayMicroseconds(10);
        digitalWrite(trig, LOW);
        unsigned long t1 = micros();
        while (digitalRead(echo) == LOW) {
          if (micros() - t1 > NEAR_RECHECK_US) {
            return 300.0f;   // Still no echo -> VERY FAR
          }
        }
        // Echo detected on second ping -- measure its duration.
        unsigned long rise2 = micros();
        while (digitalRead(echo) == HIGH) {
          if (micros() - rise2 > NEAR_RECHECK_US) return 300.0f;  // FIX 18: echo still HIGH after recheck -> far, not 34 cm
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

    // F) Pulse too short -> blind zone
    if (dur < NEAR_RT_US) {
      return -1.0f;
    }

    // G) Convert to distance
    float d = (float)dur * SOUND_CM_PER_US * 0.5f;

    // H) Extra blind-zone guard after conversion
    if (d < BLIND_ZONE_CM) {
      return -1.0f;
    }

    // I) Echo too late or distance over range limit -> VERY FAR
    if (dur > LIMIT_RT_US || d > LIMIT_CM) {
      return 300.0f;
    }

    // J) Valid range: clamp to [2, 300]
    if (d < 2.0f)   d = 2.0f;
    if (d > 300.0f) d = 300.0f;
    return d;
  }

  // -----------------------------------------------------------------------
  // FIX 21: 4-layer real-time sensor filter helpers
  // -----------------------------------------------------------------------

  // Insertion sort for tiny arrays (N <= 5). No stdlib dependency.
  static void insertionSort(float *arr, uint8_t n) {
    for (uint8_t i = 1; i < n; i++) {
      float key = arr[i];
      int8_t j = (int8_t)i - 1;
      while (j >= 0 && arr[j] > key) {
        arr[j + 1] = arr[j];
        j--;
      }
      arr[j + 1] = key;
    }
  }

  // Returns median of valid readings in sensor_history[idx][].
  // "Valid" = numeric range [2.0, 300.0). Excludes -2 / -1 / 300 / HISTORY_EMPTY.
  static float medianOfValid(uint8_t idx) {
    float valid[5];
    uint8_t n = 0;
    for (uint8_t k = 0; k < 5; k++) {
      float v = sensor_history[idx][k];
      if (v >= 2.0f && v < 300.0f) valid[n++] = v;
    }
    if (n == 0) return HISTORY_EMPTY;
    insertionSort(valid, n);
    return valid[n / 2];
  }

  // Most recent valid reading, scanning backwards from ring_pos.
  static float mostRecentValid(uint8_t idx) {
    for (uint8_t k = 0; k < 5; k++) {
      int8_t slot = (int8_t)ring_pos[idx] - 1 - (int8_t)k;
      if (slot < 0) slot += 5;
      float v = sensor_history[idx][slot];
      if (v >= 2.0f && v < 300.0f) return v;
    }
    return HISTORY_EMPTY;
  }

  // Propagate worst classification seen in history (-2 > -1 > 300).
  static float worstClassificationInHistory(uint8_t idx) {
    bool has_fault = false, has_near = false, has_far = false;
    for (uint8_t k = 0; k < 5; k++) {
      float v = sensor_history[idx][k];
      if (v == -2.0f)       has_fault = true;
      else if (v == -1.0f)  has_near  = true;
      else if (v >= 300.0f) has_far   = true;
    }
    if (has_fault) return -2.0f;
    if (has_near)  return -1.0f;
    return 300.0f;
  }

  // FIX 21: 4-layer real-time filter.
  //   L1 ring buffer -> L2 median of valid -> L3 rate limit -> L4 FAULT counter.
  static float filterReading(uint8_t idx, float raw) {
    // Layer 1: store raw into ring buffer
    sensor_history[idx][ring_pos[idx]] = raw;
    ring_pos[idx] = (ring_pos[idx] + 1) % 5;

    // Layer 4 pre-update: consecutive FAULT tracking with auto-recovery
    if (raw == -2.0f) {
      if (fault_count[idx] < 255) fault_count[idx]++;
    } else {
      fault_count[idx] = 0;  // any non-FAULT read resets counter (auto-recovery)
    }

    // Layer 4 early exit: sensor declared dead until it reads well again
    if (fault_count[idx] >= FAULT_THRESHOLD) {
      last_output[idx]       = -2.0f;
      rate_reject_count[idx] = 0;  // clear stale L3 state across dead/recover cycle
      return -2.0f;
    }

    // Layer 2: count valid readings in history window
    uint8_t n_valid = 0;
    for (uint8_t k = 0; k < 5; k++) {
      float v = sensor_history[idx][k];
      if (v >= 2.0f && v < 300.0f) n_valid++;
    }

    float candidate;
    if (n_valid >= 3) {
      candidate = medianOfValid(idx);
    } else if (n_valid >= 1) {
      candidate = mostRecentValid(idx);
    } else {
      // No valid numeric readings in window -- pass sentinel through.
      if (raw == -2.0f || raw == -1.0f || raw >= 300.0f) {
        candidate = raw;
      } else {
        candidate = worstClassificationInHistory(idx);
      }
      last_output[idx] = candidate;
      rate_reject_count[idx] = 0;
      return candidate;
    }

    // Layer 3: rate limit between valid numeric values only
    float prev = last_output[idx];
    if (prev >= 2.0f && prev < 300.0f &&
        candidate >= 2.0f && candidate < 300.0f) {
      float delta = candidate - prev;
      if (delta < 0) delta = -delta;
      if (delta > RATE_LIMIT_CM) {
        if (rate_reject_count[idx] < RATE_REJECT_MAX) {
          rate_reject_count[idx]++;
          return prev;  // hold last output
        }
        rate_reject_count[idx] = 0;  // escape hatch: accept real scene change
      } else {
        rate_reject_count[idx] = 0;
      }
    } else {
      rate_reject_count[idx] = 0;
    }

    last_output[idx] = candidate;
    return candidate;
  }

  // -----------------------------------------------------------------------
  // Setup
  // -----------------------------------------------------------------------
  void setup() {
    Serial.begin(115200);
    Wire.begin();

    // FIX 16: Clear any stale timeout flag before IMU init
    Wire.setWireTimeout(25000, true);  // FIX 5: 25 ms I2C timeout, reset bus on timeout
    Wire.clearWireTimeoutFlag();

    // IMU initialisation (BNO085 at address 0x4A)
    if (!imu.begin(0x4A, Wire)) {
      Serial.println("# ERROR: BNO085 not detected -- check wiring!");
      Serial.println("# HALTED: BNO085 init failed");
      while (1) {}
    }

    imu.enableGameRotationVector(50);  // 50 Hz quaternion -- Game RV (no mag, boot-relative)
    imu.enableAccelerometer(50);
    imu.enableGyro(50);
    imu.enableGravity(50);          // FIX 6: gravity vector for UpsideDown detection

    // Ultrasonic pin setup (FIX 7 + FIX 20: INPUT_PULLUP on ALL echo pins)
    // Prevents floating pins from picking up servo bus EMI as false rising edges.
    // Pin 13 already had INPUT_PULLUP (FIX 7); now all echo pins match.
    for (int i = 0; i < 8; i++) {
      pinMode(TRIG_PINS[i], OUTPUT);
      pinMode(ECHO_PINS[i], INPUT_PULLUP);  // FIX 20: all ECHO pins pulled up
      digitalWrite(TRIG_PINS[i], LOW);
      durations_us[i]  = 0;
      distances_cm[i]  = 300.0f;   // default: very far (safe assumption)
      // FIX 21: init 4-layer filter state
      for (uint8_t k = 0; k < 5; k++) sensor_history[i][k] = HISTORY_EMPTY;
      ring_pos[i]          = 0;
      fault_count[i]       = 0;
      rate_reject_count[i] = 0;
      last_output[i]       = 300.0f;
    }

    // CSV header -- must match the 20-column format expected by input_thread.py
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

    // --- Read 8 ultrasonic sensors in interleaved order (reduces acoustic crosstalk) ---
    // Alternates front/rear so physically adjacent sensors never fire back-to-back.
    static const uint8_t FIRE_ORDER[8] = {0, 5, 2, 7, 4, 1, 6, 3};
    for (int i = 0; i < 8; i++) {
      uint8_t idx = FIRE_ORDER[i];
      if (sensor_disabled_mask & (1 << idx)) {
        distances_cm[idx] = -2.0f;  // FIX 19 emergency override: physically destroyed sensor
        continue;
      }
      // FIX 21: raw reading piped through 4-layer real-time filter
      distances_cm[idx] = filterReading(idx, measureClassified(idx));
      delay(SENSOR_GAP_MS);
    }

    // --- Read IMU: update last-known values if fresh data available (FIX 4) ---
    // FIX 15: Cap drain at 10 iterations to prevent stall
    bool got_imu = false;
    int drain_count = 0;
    while (imu.dataAvailable() && drain_count < 10) {
      got_imu = true;
      drain_count++;
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
        // FIX 5: IMU likely locked up -- attempt reinit
        Wire.clearWireTimeoutFlag();   // FIX 16
        if (imu.begin(0x4A, Wire)) {
          imu.enableGameRotationVector(50);
          imu.enableAccelerometer(50);
          imu.enableGyro(50);
          imu.enableGravity(50);
        }
        imu_fail_count = 0;  // reset regardless to limit retry rate
      }
    }

    // FIX 14: UpsideDown detection with 2-consecutive-reading hysteresis
    // Gravity Z for detecting physical inversion:
    //   Right-side up: gravity_z > +5 (gravity points "up" in IMU frame)
    //   Inverted:      gravity_z <= 0 (gravity pointing wrong way)
    if (last_gravity_z <= 0.0f) {
      if (upside_down_consec < 2) upside_down_consec++;
    } else {
      upside_down_consec = 0;  // any non-inverted reading resets counter
    }
    upside_down_flag = (upside_down_consec >= 2) ? 1 : 0;

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
