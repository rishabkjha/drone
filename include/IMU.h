// ============================================================================
//  IMU.h  -  MPU6050, RX_RP_Tune's 1D Kalman + the vibration gate.
//  Drone frame after the read: X forward, Y left, Z up.
//  roll + = right side down, pitch + = nose down, yaw + = clockwise from above
//  Use the global object `imu`: imu.Angle[0] roll, imu.Angle[1] pitch (deg),
//  imu.Rate[0..2] roll/pitch/yaw rate (deg/s).
// ============================================================================
#pragma once
#include <Wire.h>
#include "Config.h"

class Imu {
public:
  static constexpr float GYRO_LSB  = 65.5f;           // +-500 deg/s
  static constexpr float ACC_LSB   = 2048.0f;         // +-16 g (shaking past +-8 g clipped one side)
  static constexpr float KALMAN_GYRO_STD  = 4.0f;     // deg/s   (RX_RP_Tune)
  static constexpr float KALMAN_ACCEL_STD = 3.0f;     // deg     (RX_RP_Tune)
  static constexpr float ACC_HEIGHT_HZ    = 20.0f;    // smoothed accel for the height filter

  // results
  float Rate[3] = {0, 0, 0};               // roll, pitch, yaw deg/s, bias removed
  float Angle[2] = {0, 0};                 // roll, pitch deg (Kalman, level trim applied)
  float Uncert[2] = {4, 4};                // Kalman variance, deg^2
  float AccFX = 0, AccFY = 0, AccFZ = 1;   // ACC_HEIGHT_HZ smoothed accel, g
  float AccVib = 0;                        // smoothed | |accel| - AccRestG |, g
  float AccRestG = 1.0f;                   // |accel| at rest as this chip reads it (yours ~0.88 g)
  bool  AccTrusted = true;
  float TrimR = 0, TrimP = 0;              // level trim in use, deg

  void setup() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);
    delay(250);
    mpuInit();
    delay(100);

    float zSum = 0;                        // which way up is the chip?
    for (int i = 0; i < 50; i++) { read(0.002f); zSum += RawAccZ; delay(2); }
    UpSign = zSum >= 0 ? 1 : -1;
    Serial.printf("IMU: front = %s%c, chip %s\n", FORWARD_SIGN > 0 ? "+" : "-", FORWARD_AXIS,
                  UpSign > 0 ? "Z up" : "upside down (Z down)");

    // calibration: keep the drone still and level (~2 s)
    const int N = 2000;
    float sr = 0, sp = 0, sy = 0, sax = 0, say = 0, saz = 0;
    for (int i = 0; i < N; i++) {
      read(0.001f);
      sr += RateR; sp += RateP; sy += RateY;
      sax += AccX; say += AccY; saz += AccZ;
      delay(1);
    }
    GyroBiasR = sr / N; GyroBiasP = sp / N; GyroBiasY = sy / N;
    float ax = sax / N, ay = say / N, az = saz / N;
    AccRestG = sqrtf(ax * ax + ay * ay + az * az);
    if (AccRestG < 0.5f || AccRestG > 1.5f) AccRestG = 1.0f;   // moved during boot

    float r = accRollRaw(ax, ay, az), p = accPitchRaw(ax, ay, az);
    if (AUTO_LEVEL) { TrimR = r; TrimP = p; }
    else            { TrimR = LEVEL_TRIM_ROLL; TrimP = LEVEL_TRIM_PITCH; }
    Serial.printf("IMU level: sensor reads roll %.2f pitch %.2f deg at power-up, trim used %.2f / %.2f%s\n",
                  r, p, TrimR, TrimP, AUTO_LEVEL ? " (AUTO_LEVEL)" : "");
    Serial.printf("IMU: accel at rest %.2f g (vibration gate %.2f g)\n", AccRestG, ACC_VIB_MAX);

    // RX_RP_Tune: start the Kalman at the resting accel angle instead of 0
    Angle[0] = r - TrimR; Uncert[0] = 4;
    Angle[1] = p - TrimP; Uncert[1] = 4;
    AccVib = 0;
  }

  void update(float dt) {
    read(dt);
    Rate[0] = RateR - GyroBiasR;
    Rate[1] = RateP - GyroBiasP;
    Rate[2] = RateY - GyroBiasY;

    // vibration gate with hysteresis: out fast, back in after a quiet spell
    uint32_t now = millis();
    if (AccVib > ACC_VIB_MAX) { AccTrusted = false; AccOkSinceMs = 0; }
    else if (!AccTrusted) {
      if (AccVib >= ACC_VIB_OK) AccOkSinceMs = 0;
      else if (AccOkSinceMs == 0) AccOkSinceMs = now ? now : 1;
      else if (now - AccOkSinceMs >= ACC_SETTLE_MS) AccTrusted = true;
    }

    kalman(0, Rate[0], accRollRaw(AccX, AccY, AccZ)  - TrimR, dt);
    kalman(1, Rate[1], accPitchRaw(AccX, AccY, AccZ) - TrimP, dt);
  }

private:
  int   UpSign = 1;                        // +1 chip Z up, -1 upside down (found at boot)
  float GyroBiasR = 0, GyroBiasP = 0, GyroBiasY = 0;
  // raw, in the drone frame
  float RateR = 0, RateP = 0, RateY = 0;   // deg/s (before bias removal)
  float AccX = 0, AccY = 0, AccZ = 0;      // g
  float RawAccZ = 0;                       // chip Z, g (upside-down check)
  uint32_t AccOkSinceMs = 0;
  bool  Seeded = false;

  void writeReg(uint8_t reg, uint8_t v) {
    Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(v); Wire.endTransmission();
  }

  void mpuInit() {
    writeReg(0x6B, 0x00);                  // wake up
    writeReg(0x1A, MPU_DLPF_CFG);          // low-pass, gyro 1 kHz sample rate
    writeReg(0x1C, 0x18);                  // accel +-16 g
    writeReg(0x1B, 0x08);                  // gyro +-500 deg/s
  }

  // chip axes -> drone axes (forward, left, up)
  void toBody(float cx, float cy, float cz, float &f, float &l, float &u) {
    if (FORWARD_AXIS == 'Y') { f = FORWARD_SIGN * cy; l = -UpSign * FORWARD_SIGN * cx; }
    else                     { f = FORWARD_SIGN * cx; l =  UpSign * FORWARD_SIGN * cy; }
    u = UpSign * cz;
  }

  void read(float dt) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0x68, 14);
    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();              // temperature
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();
    float gu;
    toBody(gx / GYRO_LSB, gy / GYRO_LSB, gz / GYRO_LSB, RateR, RateP, gu);
    RateY = -gu;                           // + = clockwise from above
    RawAccZ = az / ACC_LSB;
    toBody(ax / ACC_LSB, ay / ACC_LSB, RawAccZ, AccX, AccY, AccZ);

    float dev = fabsf(sqrtf(AccX * AccX + AccY * AccY + AccZ * AccZ) - AccRestG);
    if (!Seeded) {
      AccFX = AccX; AccFY = AccY; AccFZ = AccZ; AccVib = dev; Seeded = true;
    } else {
      const float aH = dt / (dt + 1.0f / (2.0f * PI * ACC_HEIGHT_HZ));
      const float aV = dt / (dt + 1.0f / (2.0f * PI * ACC_VIB_HZ));
      AccFX += aH * (AccX - AccFX); AccFY += aH * (AccY - AccFY); AccFZ += aH * (AccZ - AccFZ);
      AccVib += aV * (dev - AccVib);
    }
  }

  // Accelerometer tilt angles (deg), untrimmed (same formulas as RX_RP_Tune)
  static float accRollRaw(float x, float y, float z)  { return  atan2f(y, sqrtf(x * x + z * z)) * RAD_TO_DEG; }
  static float accPitchRaw(float x, float y, float z) { return -atan2f(x, sqrtf(y * y + z * z)) * RAD_TO_DEG; }

  // RX_RP_Tune Kalman1D::update, plus the vibration gate
  void kalman(int i, float rate, float accAngle, float dt) {
    Angle[i]  += dt * rate;
    Uncert[i] += dt * dt * KALMAN_GYRO_STD * KALMAN_GYRO_STD;
    if (!AccTrusted) return;                                                // shaken: gyro only
    if (fabsf(accAngle - Angle[i]) > ACC_AGREE_DEG && AccVib >= ACC_QUIET) return;   // implausible
    float gain = Uncert[i] / (Uncert[i] + KALMAN_ACCEL_STD * KALMAN_ACCEL_STD);
    Angle[i]  += gain * (accAngle - Angle[i]);
    Uncert[i]  = (1.0f - gain) * Uncert[i];
  }
};

Imu imu;
