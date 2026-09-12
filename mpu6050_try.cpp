// mpu6050: Orientation & Angular Velocity
#include <Arduino.h>
#include <Wire.h>

// --- ESP32 Wire Pin Definitions ---
#define SDA_PIN 11
#define SCL_PIN 10

// --- Loop Timing ---
uint32_t LoopTimer;
const float dt = 0.004; // 250 Hz Loop (4ms)

// --- IMU Raw & Calibrated Variables ---
float RateRoll, RatePitch, RateYaw;
float RateCalRoll = 0, RateCalPitch = 0, RateCalYaw = 0;
float AccX, AccY, AccZ;

// Output 1: Angles (Degrees)
float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0;

// Output 2: Angular Rates (Deg/s) - Rates after calibration bias subtraction
float RateRollDegS, RatePitchDegS, RateYawDegS;

// --- 1D Kalman Filter Variables (Roll & Pitch) ---
float KalmanAngleRoll = 0, UncertaintyRoll = 4.0;
float KalmanAnglePitch = 0, UncertaintyPitch = 4.0;

// --- Function Prototypes ---
void initIMU();
void readIMU();
void calibrateIMU();
void update1DKalman(float &state, float &uncertainty, float gyroRate, float accelAngle);

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with explicit SDA and SCL pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C Fast Mode

  initIMU();
  calibrateIMU();

  LoopTimer = micros();
}

// ================================================================
// MAIN LOOP
// ================================================================
void loop() {
  readIMU();

  // 1. Calibrated Angular Velocities (deg/s)
  RateRollDegS  = RateRoll  - RateCalRoll;
  RatePitchDegS = RatePitch - RateCalPitch;
  RateYawDegS   = RateYaw   - RateCalYaw;

  // Compute Accelerometer Pitch/Roll Angles
  float AccAngleRoll  =  atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / 3.14159265);
  float AccAnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / 3.14159265);

  // 2. Accurate Angles using 1D Kalman Filter (Degrees)
  update1DKalman(KalmanAngleRoll, UncertaintyRoll, RateRollDegS, AccAngleRoll);
  update1DKalman(KalmanAnglePitch, UncertaintyPitch, RatePitchDegS, AccAnglePitch);
  
  if (abs(RateYawDegS) < 3) RateYawDegS = 0;
  AngleYaw += RateYawDegS * dt; // Yaw integration (heading relative to startup)

  AngleRoll  = KalmanAngleRoll;
  AnglePitch = KalmanAnglePitch;

  // --- Print Telemetry Outputs ---
  Serial.print("Angles(deg) R:"); Serial.print(AngleRoll, 3);
  Serial.print(" P:"); Serial.print(AnglePitch, 3);
  Serial.print(" Y:"); Serial.print(AngleYaw, 3);

  Serial.print(" | Rates(deg/s) R:"); Serial.print(RateRollDegS, 4);
  Serial.print(" P:"); Serial.print(RatePitchDegS, 4);
  Serial.print(" Y:"); Serial.print(RateYawDegS, 4);
  
  Serial.print(" | Acc(m/s2) X:"); Serial.print(AccX, 4);
  Serial.print(" Y:"); Serial.print(AccY, 4);
  Serial.print(" Z:"); Serial.println(AccZ, 4);

  // Maintain strict 250Hz loop speed
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}

// ================================================================
// KALMAN FILTER HELPER FUNCTIONS
// ================================================================

// 1D Kalman Filter (Orientation Angles)
void update1DKalman(float &state, float &uncertainty, float gyroRate, float accelAngle) {
  state += dt * gyroRate;
  uncertainty += dt * dt * 16.0; // Process noise
  float gain = uncertainty / (uncertainty + 9.0); // Measurement noise
  state += gain * (accelAngle - state);
  uncertainty *= (1.0 - gain);
}

// ================================================================
// IMU HARDWARE INTERFACE (MPU6050)
// ================================================================
void initIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power Management 1
  Wire.write(0x00); // Wake up
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // Accelerometer Configuration
  Wire.write(0x10); // +/- 8g full scale range
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Gyroscope Configuration
  Wire.write(0x08); // +/- 500 deg/s full scale range
  Wire.endTransmission();
}

void readIMU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for Accel data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  int16_t axLSB = Wire.read() << 8 | Wire.read();
  int16_t ayLSB = Wire.read() << 8 | Wire.read();
  int16_t azLSB = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip temperature bytes
  int16_t gxLSB = Wire.read() << 8 | Wire.read();
  int16_t gyLSB = Wire.read() << 8 | Wire.read();
  int16_t gzLSB = Wire.read() << 8 | Wire.read();

  // Convert to physical units (FS_SEL=1 for Gyro: 65.5 LSB/deg/s, AFS_SEL=2 for Accel: 4096 LSB/g)
  RateRoll  = (float)gxLSB / 65.5;
  RatePitch = (float)gyLSB / 65.5;
  RateYaw   = (float)gzLSB / 65.5;

  AccX = (float)axLSB / 4096.0;
  AccY = (float)ayLSB / 4096.0;
  AccZ = (float)azLSB / 4096.0;
}

void calibrateIMU() {
  for (int i = 0; i < 4000; i++) {
    readIMU();
    RateCalRoll  += RateRoll;
    RateCalPitch += RatePitch;
    RateCalYaw   += RateYaw;
    delay(1);
  }
  RateCalRoll  /= 4000.0;
  RateCalPitch /= 4000.0;
  RateCalYaw   /= 4000.0;
}
