#include <Wire.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

#define SDA_PIN 11
#define SCL_PIN 10

uint32_t LoopTimer;
const float dt = 0.004; // 250 Hz Loop (4ms)
float RateRoll, RatePitch, RateYaw;
float RateCalRoll = 0, RateCalPitch = 0, RateCalYaw = 0;
float AccX, AccY, AccZ;
float AccCalX = 0, AccCalY = 0, AccCalZ = 0;

// callable start
float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0;
float RateRollDegS, RatePitchDegS, RateYawDegS;
float AccXEarth = 0, AccYEarth = 0, AccZEarth = 0;
// callable end

float KalmanAngleRoll = 0, UncertaintyRoll = 4.0;
float KalmanAnglePitch = 0, UncertaintyPitch = 4.0;

// 1D Kalman Filter (Orientation Angles)
void update1DKalman(float &state, float &uncertainty, float gyroRate, float accelAngle) {
  state += dt * gyroRate;
  uncertainty += dt * dt * 16.0; // Process noise
  float gain = uncertainty / (uncertainty + 9.0); // Measurement noise
  state += gain * (accelAngle - state);
  uncertainty *= (1.0 - gain);
}

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
    AccCalX      += AccX;
    AccCalY      += AccY;
    AccCalZ      += (AccZ - 1.0); // Account for 1g gravity when stationary upright
    delay(1);
  }
  RateCalRoll  /= 4000.0;
  RateCalPitch /= 4000.0;
  RateCalYaw   /= 4000.0;
  AccCalX      /= 4000.0;
  AccCalY      /= 4000.0;
  AccCalZ      /= 4000.0;
}

void processIMUData() {
    readIMU();

  RateRollDegS  = RateRoll  - RateCalRoll;
  RatePitchDegS = RatePitch - RateCalPitch;
  RateYawDegS   = RateYaw   - RateCalYaw;

  // Compute Accelerometer Pitch/Roll Angles
  float AccAngleRoll  =  atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / 3.14159265);
  float AccAnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / 3.14159265);

  // 2. Accurate Angles using 1D Kalman Filter (Degrees)
  update1DKalman(KalmanAngleRoll, UncertaintyRoll, RateRollDegS, AccAngleRoll);
  update1DKalman(KalmanAnglePitch, UncertaintyPitch, RatePitchDegS, AccAnglePitch);
  AngleYaw += RateYawDegS * dt; // Yaw integration (heading relative to startup)

  //3. Logic to remove yaw drift
  if (abs(RateYawDegS) < 3) RateYawDegS = 0;
  bool isStationary = (AccXEarth == 0.0) && (AccYEarth == 0.0) && (AccZEarth == 0.0);
  if (!isStationary) {
    AngleYaw += RateYawDegS * dt;
  }

  AngleRoll  = KalmanAngleRoll;
  AnglePitch = KalmanAnglePitch;

  // 4. Earth-Frame Linear Accelerations (m/s^2)
  // Convert angles to Radians
  float rRoll  = AngleRoll  * (3.14159265 / 180.0);
  float rPitch = AnglePitch * (3.14159265 / 180.0);

  // Remove Accelerometer Calibration Bias & Convert g to m/s^2
  float ax = (AccX - AccCalX) * 9.81;
  float ay = (AccY - AccCalY) * 9.81;
  float az = (AccZ - AccCalZ) * 9.81;

  // Rotate body acceleration to Earth navigation frame
  AccXEarth = 0.34 + cos(rPitch) * ax + sin(rRoll) * sin(rPitch) * ay + cos(rRoll) * sin(rPitch) * az;
  AccYEarth = cos(rRoll) * ay - sin(rRoll) * az;
  AccZEarth = -sin(rPitch) * ax + sin(rRoll) * cos(rPitch) * ay + cos(rRoll) * cos(rPitch) * az - 9.81; // Subtract Gravity
  

  // 5. Motion Detection & Zero Velocity Update (ZUPT)
  float motionThreshold = 0.15; // m/s^2 threshold for noise filtering
  if (abs(AccXEarth) < motionThreshold) AccXEarth = 0;
  if (abs(AccYEarth) < motionThreshold) AccYEarth = 0;
  if (abs(AccZEarth) < motionThreshold) AccZEarth = 0;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C Fast Mode
  initIMU();
  calibrateIMU();
  LoopTimer = micros();
}

void loop() {
  // Maintain strict 250Hz loop speed
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
