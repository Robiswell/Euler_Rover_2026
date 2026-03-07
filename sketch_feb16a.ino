/*
  RHex Rover Logger with 8 Ultrasonics + BNO085

  - 8 ultrasonic sensors (FrontDiagonalLeft → RearDiagonalRight)
  - BNO085 IMU readings (quaternion, accel, gyro)
  - UpsideDown detection based on pitch (>150°)
  - CSV output for logging to Raspberry Pi
*/

#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 imu;

// Sensor positions
const char* sensorNames[8] = {
  "FrontDiagonalLeft","FrontCenterLeft","FrontCenterRight","FrontDiagonalRight",
  "RearDiagonalLeft","RearCenterLeft","RearCenterRight","RearDiagonalRight"
};

// Ultrasonic pin assignments
const int trigPins[8] = {2,4,6,8,10,12,A0,A2};
const int echoPins[8] = {3,5,7,9,11,13,A1,A3};

long durations[8];
float distances[8];

unsigned long previousMillis = 0;
const unsigned long interval = 100; // 100 ms

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!imu.begin()) {
    Serial.println("BNO085 not detected!");
    while(1);
  }

  imu.enableRotationVector(50); // 50 Hz
  imu.enableAccelerometer(50);
  imu.enableGyro(50);

  for (int i=0;i<8;i++){
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // CSV header
  Serial.print("timestamp_ms");
  for (int i=0;i<8;i++) Serial.print("," + String(sensorNames[i]));
  Serial.println(",QuatW,QuatX,QuatY,QuatZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,UpsideDown");
}

// Convert quaternion to pitch (degrees)
float quaternionToPitch(float w, float x, float y, float z){
  return asin(2*(w*y - z*x)) * 180.0/3.14159265;
}

void loop() {
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;

    // --- Read ultrasonic sensors sequentially to avoid crosstalk ---
    for(int i=0;i<8;i++){
      digitalWrite(trigPins[i], LOW); delayMicroseconds(2);
      digitalWrite(trigPins[i], HIGH); delayMicroseconds(10);
      digitalWrite(trigPins[i], LOW);
      durations[i] = pulseIn(echoPins[i], HIGH, 30000); // 30 ms timeout
      distances[i] = durations[i]*0.0343/2.0; // cm
      delay(5); // small delay to reduce interference
    }

    // --- Read IMU ---
    if(imu.dataAvailable()){
      float qx,qy,qz,qw,radAccuracy;
      uint8_t accuracy;

      imu.getQuat(qx,qy,qz,qw,radAccuracy,accuracy); // BNO085 style

      float ax = imu.getAccelX();
      float ay = imu.getAccelY();
      float az = imu.getAccelZ();

      float gx = imu.getGyroX();
      float gy = imu.getGyroY();
      float gz = imu.getGyroZ();

      // UpsideDown detection based on pitch
      float pitch = quaternionToPitch(qw,qx,qy,qz);
      int upsideDown = (abs(pitch) > 150) ? 1 : 0;

      // --- Print CSV row ---
      Serial.print(currentMillis);
      for(int i=0;i<8;i++) Serial.print("," + String(distances[i]));
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

