// mpu6050: Orientation & Angular Velocity, Inertial Acceleration, Movement & Distance, Zero Velocity Update

#include <Wire.h>
#include <BasicLinearAlgebra.h>

using namespace BLA;

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
float AccCalX = 0, AccCalY = 0, AccCalZ = 0;

// Output 1: Angles (Degrees)
float AngleRoll = 0, AnglePitch = 0, AngleYaw = 0;

// Output 2: Angular Rates (Deg/s) - Rates after calibration bias subtraction
float RateRollDegS, RatePitchDegS, RateYawDegS;

// Output 3: Linear Acceleration in Earth Frame (m/s^2)
float AccXEarth = 0, AccYEarth = 0, AccZEarth = 0;

// Output 4: Position and Distance (Meters)
float PosX = 0, PosY = 0, PosZ = 0;
float TotalDistanceMoved = 0;
float PrevPosX = 0, PrevPosY = 0, PrevPosZ = 0;

// --- 1D Kalman Filter Variables (Roll & Pitch) ---
float KalmanAngleRoll = 0, UncertaintyRoll = 4.0;
float KalmanAnglePitch = 0, UncertaintyPitch = 4.0;

// --- 3D Kinematic Kalman Filters for Distance Estimation ---
struct Kalman3D {
  BLA::Matrix<2,1> State; // [Position, Velocity]
  BLA::Matrix<2,2> P;     // Error Covariance
  BLA::Matrix<2,2> F;     // Transition Matrix
  BLA::Matrix<2,1> G;     // Control Matrix
  BLA::Matrix<2,2> Q;     // Process Noise
  BLA::Matrix<1,1> R;     // Measurement Noise
  BLA::Matrix<1,2> H;     // Measurement Matrix
  BLA::Matrix<2,2> I;     // Identity Matrix
};

Kalman3D kfX, kfY, kfZ;

// --- Function Prototypes ---
void initIMU();
void readIMU();
void calibrateIMU();
void update1DKalman(float &state, float &uncertainty, float gyroRate, float accelAngle);
void init3DKalman(Kalman3D &kf);
void update3DKalman(Kalman3D &kf, float accelInput, float &outPos, float &outVel);

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  
  // Initialize I2C with explicit SDA (21) and SCL (22) pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C Fast Mode

  initIMU();
  calibrateIMU();

  init3DKalman(kfX);
  init3DKalman(kfY);
  init3DKalman(kfZ);

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
  AngleYaw += RateYawDegS * dt; // Yaw integration (heading relative to startup)

  if (abs(RateYawDegS) < 3) RateYawDegS = 0;

  bool isStationary = (AccXEarth == 0.0) && (AccYEarth == 0.0) && (AccZEarth == 0.0);
  if (!isStationary) {
    AngleYaw += RateYawDegS * dt;
  }

  AngleRoll  = KalmanAngleRoll;
  AnglePitch = KalmanAnglePitch;

  // 3. Earth-Frame Linear Accelerations (m/s^2)
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
  

  // 4. Motion Detection & Zero Velocity Update (ZUPT)
  float motionThreshold = 0.15; // m/s^2 threshold for noise filtering
  if (abs(AccXEarth) < motionThreshold) AccXEarth = 0;
  if (abs(AccYEarth) < motionThreshold) AccYEarth = 0;
  if (abs(AccZEarth) < motionThreshold) AccZEarth = 0;

  // Run 3D Kalman Filter for Position
  float VelX, VelY, VelZ;
  update3DKalman(kfX, AccXEarth, PosX, VelX);
  update3DKalman(kfY, AccYEarth, PosY, VelY);
  update3DKalman(kfZ, AccZEarth, PosZ, VelZ);

  // ZUPT: Reset velocity if no active acceleration is detected
  if (AccXEarth == 0) kfX.State(1,0) = 0;
  if (AccYEarth == 0) kfY.State(1,0) = 0;
  if (AccZEarth == 0) kfZ.State(1,0) = 0;

  // Compute Total Cumulative Distance Moved (Meters)
  float dX = PosX - PrevPosX;
  float dY = PosY - PrevPosY;
  float dZ = PosZ - PrevPosZ;
  TotalDistanceMoved += sqrt(dX * dX + dY * dY + dZ * dZ);

  PrevPosX = PosX;
  PrevPosY = PosY;
  PrevPosZ = PosZ;

  // --- Print Telemetry Outputs ---
  Serial.print("Angles(deg) R:"); Serial.print(AngleRoll, 3);
  Serial.print(" P:"); Serial.print(AnglePitch, 3);
  Serial.print(" Y:"); Serial.print(AngleYaw, 3);

  Serial.print(" | Rates(deg/s) R:"); Serial.print(RateRollDegS, 4);
  Serial.print(" P:"); Serial.print(RatePitchDegS, 4);
  Serial.print(" Y:"); Serial.print(RateYawDegS, 4);

  Serial.print(" | Acc(m/s2) X:"); Serial.print(AccXEarth, 4);
  Serial.print(" Y:"); Serial.print(AccYEarth, 4);
  Serial.print(" Z:"); Serial.println(AccZEarth, 4);

  //Serial.print(" | Distance(m):"); Serial.println(TotalDistanceMoved, 3);

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

// 3D Kinematic Kalman Filter Initialization (Position/Velocity)
void init3DKalman(Kalman3D &kf) {
  kf.State = {0, 0};
  kf.F = {1, dt,
          0, 1};
  kf.G = {0.5f * dt * dt,
          dt};
  kf.H = {1, 0};
  kf.I = {1, 0,
          0, 1};
  kf.Q = kf.G * ~kf.G * 0.5f * 0.5f; // Process noise covariance
  kf.R = {0.05f};                    // Measurement noise covariance
  kf.P = {0, 0,
          0, 0};
}

// 3D Kinematic Kalman Filter Step
void update3DKalman(Kalman3D &kf, float accelInput, float &outPos, float &outVel) {
  BLA::Matrix<1,1> AccelMat = {accelInput};
  kf.State = kf.F * kf.State + kf.G * AccelMat;
  kf.P = kf.F * kf.P * ~kf.F + kf.Q;

  outPos = kf.State(0,0);
  outVel = kf.State(1,0);
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
