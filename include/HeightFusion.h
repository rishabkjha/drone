// ============================================================================
//  HeightFusion.h  -  height and climb rate from the VL53L0X ToF, the BMP280
//  barometer and the accelerometer.
//
//  One complementary filter (height, climb rate, accel bias) runs all the
//  time; only its measurement changes source, so the climb rate never jumps:
//    ToF   height = distance x cos(roll) x cos(pitch) - reading on the ground
//    baro  height = baro change since boot / scale + offset
//  Which one (the app's Options > Enable Baro switch = baroSwitch):
//    switch OFF  ToF. The baro only takes over while the ToF has no reading
//                (out of range, blocked, tilted past 35 deg).
//    switch ON   ToF below BARO_HANDOFF_CM, baro above.
//  While the ToF is in use it trains the baro: the offset follows the ToF all
//  the time (it also removes baro drift on the ground), and over a climb the
//  scale is fitted (baro cm per real cm). The scale is saved when you disarm
//  and used from the next boot, blended with each new fit.
//  Handover: ToF -> baro sets the baro offset so the height carries on from
//  where it was; baro -> ToF glides the difference out over BARO_GLIDE_TAU_S.
//  Global object `height`: height.cm(), height.velocity(), height.ok(),
//  height.maxCm() (the height limit for the switch position).
// ============================================================================
#pragma once
#include <Adafruit_VL53L0X.h>
#include <Preferences.h>
#include "Config.h"
#include "IMU.h"
#include "HoverItRx.h"

class HeightEstimator {
public:
  float H = 0, V = 0, Bias = 0;            // cm, cm/s, cm/s^2
  float Meas = 0;                          // cm, measurement the filter follows now

  void setup() {
    tofInit();
    baroInit();
    float as = 0, ts = 0; int tn = 0;
    double bs = 0; int bn = 0;
    const int N = 200;                       // ~2 s, drone still on the ground
    for (int n = 0; n < N; n++) {
      imu.update(0.01f); as += verticalAcc();
      float d;
      if (tofRead(d)) { ts += d; tn++; }
      if (BaroPresent && n % 5 == 4 && baroRead()) { bs += BaroAbs; bn++; }
      delay(10);
    }
    AccZBias = as / N;
    if (tn > 0) TofZero = ts / tn;
    Serial.printf("[Height] ToF on the ground %.1f cm (%d readings) = height 0\n", TofZero, tn);
    if (BaroPresent && bn > 0) {
      BaroZero = bs / bn;
      loadScale();
    } else if (BaroPresent) {
      BaroPresent = false;
      Serial.println("[Height] BMP280 gave no readings: barometer off");
    }
    H = V = Bias = Meas = 0;
    LastTofMs = LastPollMs = LastBaroMs = millis();
  }

  // every loop. baroSwitch = the app's Enable Baro switch,
  // flying = motors running (last loop), for the scale training.
  void update(float dt, bool baroSwitch, bool flying) {
    BaroSwitch = baroSwitch;
    uint32_t now = millis();

    // ---- ToF
    float d;
    bool tofBack = false;                    // first ToF sample after a gap
    if (now - LastPollMs >= TOF_POLL_MS) {
      LastPollMs = now;
      if (tofRead(d) && fabsf(imu.Angle[0]) <= TOF_MAX_TILT && fabsf(imu.Angle[1]) <= TOF_MAX_TILT) {
        float t = d * cosf(imu.Angle[0] * DEG_TO_RAD) * cosf(imu.Angle[1] * DEG_TO_RAD) - TofZero;
        if (t < 0) t = 0;
        if (!HaveTof) tofBack = true;
        LastTof = t; LastTofMs = now; HaveTof = true;
      }
    }
    if (HaveTof && now - LastTofMs > TOF_LOST_MS) HaveTof = false;

    // ---- barometer (a new reading every ~45 ms)
    bool newBaro = false;
    if (BaroPresent && now - LastBaroMs >= BARO_PERIOD_MS) {
      LastBaroMs = now;
      if (baroRead()) { BaroRel = (float)(BaroAbs - BaroZero); newBaro = true; }
    }

    // ---- scale training: one set of samples per flight
    if (flying && !WasFlying) trainReset();
    if (!flying && WasFlying) fitScale();
    WasFlying = flying;
    if (ScalePending && !hoverItArmed()) saveScale();   // flash write: never while armed

    // ---- which source
    bool wantBaro = false;
    if (BaroPresent) {
      if (!HaveTof) wantBaro = true;                                   // ToF lost: baro
      else if (LastTof < TOF_LAND_CM) wantBaro = false;                // near the ground: always the ToF
      else if (baroSwitch) wantBaro = H > BARO_HANDOFF_CM - (UsingBaro ? BARO_HANDOFF_HYST_CM : 0.0f);
    }
    if (wantBaro && !UsingBaro) {
      fitScale();                            // this flight's climb, if long enough
      Offset += H - baroHeight();            // carry on from the current height
      Glide = 0;
      UsingBaro = true;
      Serial.printf("[Height] barometer in use at %.0f cm (%s)\n", H, HaveTof ? "above handover" : "no ToF reading");
    } else if (!wantBaro && UsingBaro) {
      UsingBaro = false;
      Glide = H - LastTof;                   // fade the baro/ToF difference out
      Serial.printf("[Height] ToF in use at %.1f cm%s\n", LastTof, LastTof < TOF_LAND_CM ? " (near the ground)" : "");
    }

    // ---- train the baro while the ToF is the source
    if (newBaro && !UsingBaro && HaveTof) {
      const float k = (BARO_PERIOD_MS / 1000.0f) / BARO_OFFSET_TAU_S;
      Offset += k * ((LastTof - BaroRel / Scale) - Offset);
      if (flying) trainAdd(LastTof, BaroRel);
    }

    // ---- measurement
    if (UsingBaro) {
      if (newBaro) Meas = baroHeight();
    } else if (HaveTof) {
      if (tofBack && !BaroPresent) { H = LastTof; V = 0; }   // no baro to bridge the gap: start from it
      Meas = LastTof + Glide;
      Glide -= Glide * min(1.0f, dt / BARO_GLIDE_TAU_S);
    }

    bool accOk = imu.AccTrusted && fabsf(imu.Angle[0]) < MAX_TILT_ACC && fabsf(imu.Angle[1]) < MAX_TILT_ACC;
    float a = accOk ? verticalAcc() - AccZBias + Bias : 0;
    if (!UsingBaro && !HaveTof) {            // no measurement at all: coast, bleed the climb rate off
      V += a * dt;
      V -= V * 2.0f * dt;
      H += V * dt;
      return;
    }
    // complementary filter on the (held) measurement
    const float tau = UsingBaro ? (accOk ? HEIGHT_TAU_BARO_S : HEIGHT_TAU_BARO_GATED_S)
                                : (accOk ? HEIGHT_TAU_S : HEIGHT_TAU_TOF_S);
    const float k1 = 3.0f / tau, k2 = 3.0f / (tau * tau), k3 = 1.0f / (tau * tau * tau);
    float e = Meas - H;
    if (accOk) Bias += k3 * e * dt;
    V += (a + k2 * e) * dt;
    H += (V + k1 * e) * dt;
  }

  float cm() const { return H; }
  // climb rate with a small deadband (blends back in from 1x to 2x)
  float velocity() const {
    float a = fabsf(V), db = VEL_DEADBAND_CMS;
    if (db <= 0 || a >= 2 * db) return V;
    if (a <= db) return 0;
    return (V > 0 ? 1.0f : -1.0f) * 2.0f * (a - db);
  }
  bool  ok() const { return HaveTof || BaroPresent; }          // some height source
  float tofCm() const { return HaveTof ? LastTof : -1.0f; }   // -1 = no reading
  float baroCm() const { return BaroPresent ? baroHeight() : -1.0f; }
  bool  usingBaro() const { return UsingBaro; }
  float baroScale() const { return Scale; }
  // height limit: MAX_HEIGHT_CM with the baro switch off, MAX_HEIGHT_BARO_CM with it on
  float maxCm() const { return BaroPresent && BaroSwitch ? MAX_HEIGHT_BARO_CM : MAX_HEIGHT_CM; }

private:
  static constexpr float TOF_MAX_TILT = 35.0f;        // deg
  static constexpr float TOF_MAX_CM   = 200.0f;       // longer readings are ignored
  static constexpr float MAX_TILT_ACC = 45.0f;
  static const uint32_t  TOF_POLL_MS  = 5;
  static const uint32_t  BARO_PERIOD_MS = 45;         // BMP280 output rate at these settings
  static const int       TRAIN_MIN_SAMPLES = 30;

  Adafruit_VL53L0X tof;
  bool  TofPresent = false;
  float TofZero = 0;                       // cm, reading on the ground at boot
  float AccZBias = 0;                      // cm/s^2 at rest
  float LastTof = -1;                      // cm, last ToF height
  uint32_t LastTofMs = 0, LastPollMs = 0, LastBaroMs = 0;
  bool  HaveTof = false;                   // a ToF sample within TOF_LOST_MS

  // barometer
  uint8_t BaroAddr = 0x76;
  bool   BaroPresent = false;
  bool   BaroSwitch = false;
  bool   UsingBaro = false;
  double BaroAbs = 0, BaroZero = 0;        // cm (absolute, and at boot)
  float  BaroRel = 0;                      // cm since boot, raw
  float  Offset = 0;                       // cm, baro height = BaroRel / Scale + Offset
  float  Scale = 1.0f, SavedScale = 1.0f;  // baro cm per real cm
  bool   HaveSaved = false, ScalePending = false;
  float  Glide = 0;                        // cm, baro -> ToF blend
  uint16_t dig_T1 = 0, dig_P1 = 0;
  int16_t  dig_T2 = 0, dig_T3 = 0, dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0;
  int16_t  dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;
  // scale training: least squares of baroRel = scale * tof + c
  bool   WasFlying = false;
  double TrX = 0, TrY = 0, TrXX = 0, TrXY = 0;
  float  TrMin = 0, TrMax = 0;
  int    TrN = 0;

  float baroHeight() const { return BaroRel / Scale + Offset; }

  void tofSoftReset() {
    auto wr = [](uint8_t reg, uint8_t v) {
      Wire.beginTransmission(0x29); Wire.write(reg); Wire.write(v); return Wire.endTransmission() == 0;
    };
    auto rd = [](uint8_t reg) -> int {
      Wire.beginTransmission(0x29); Wire.write(reg);
      if (Wire.endTransmission(false) != 0) return -1;
      if (Wire.requestFrom(0x29, 1) != 1) return -1;
      return Wire.read();
    };
    if (!wr(0xBF, 0x00)) { Serial.println("[Height] VL53L0X reset: no answer at 0x29"); return; }
    for (int i = 0; i < 50 && rd(0xC0) != 0; i++) delay(1);      // in reset: model id reads 0
    wr(0xBF, 0x01);
    int id = -1;
    for (int i = 0; i < 50; i++) { id = rd(0xC0); if (id > 0) break; delay(1); }   // booted again
    delay(2);
    Serial.printf("[Height] VL53L0X reset, model id 0x%02X (should be 0xEE)\n", id < 0 ? 0 : id);
  }

  void tofInit() {
    // which chips answer on the bus: 0x29 VL53L0X, 0x68 MPU6050, 0x76/0x77 BMP280
    Serial.print("[Height] I2C devices on SDA 9 / SCL 8:");
    int found = 0;
    for (uint8_t a = 1; a < 127; a++) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", a); found++; }
    }
    Serial.println(found ? "" : " none");
    // The VL53L0X keeps its power when the ESP32 resets (upload, reset button),
    // so it can still be ranging from the last boot; the library's init then
    // times out. Soft-reset it first (ST API VL53L0X_ResetDevice, register 0xBF).
    tofSoftReset();
    // up to 3 tries; tries 2 and 3 at 100 kHz, try 3 prints the library's debug
    // lines so we can see which init step fails
    for (int attempt = 1; attempt <= 3 && !TofPresent; attempt++) {
      if (attempt > 1) { delay(100); Wire.setClock(100000); tofSoftReset(); }
      TofPresent = tof.begin(0x29, attempt == 3, &Wire);
      Wire.begin(SDA_PIN, SCL_PIN);         // the library re-begins Wire: back to 8 / 9, 400 kHz
      Wire.setClock(400000);
      if (!TofPresent) Serial.printf("[Height] VL53L0X init try %d failed\n", attempt);
      else if (attempt > 1) Serial.printf("[Height] VL53L0X init OK on try %d (100 kHz)\n", attempt);
    }
    Serial.println(TofPresent ? "[Height] VL53L0X found" : "[Height] VL53L0X NOT found: height from the barometer only");
    if (!TofPresent) return;
    tof.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
    tof.startRangeContinuous(33);           // ~30 Hz
  }

  bool tofRead(float &cm) {
    if (!TofPresent || !tof.isRangeComplete()) return false;
    uint16_t mm = tof.readRangeResult();
    if (mm == 0 || mm >= 8190) return false;
    cm = mm / 10.0f;
    return cm < TOF_MAX_CM;
  }

  // ---- BMP280 (0x76 or 0x77)
  uint8_t baroReg(uint8_t reg) {
    Wire.beginTransmission(BaroAddr); Wire.write(reg); Wire.endTransmission();
    Wire.requestFrom((int)BaroAddr, 1);
    return Wire.available() ? Wire.read() : 0;
  }
  void baroInit() {
    if (!USE_BARO) { Serial.println("[Height] barometer OFF (USE_BARO = false): ToF only"); return; }
    const uint8_t addrs[2] = { 0x76, 0x77 };
    for (int i = 0; i < 2 && !BaroPresent; i++) {
      BaroAddr = addrs[i];
      uint8_t id = baroReg(0xD0);
      BaroPresent = id == 0x58 || id == 0x60;      // BMP280 / BME280
    }
    if (!BaroPresent) { Serial.println("[Height] BMP280 not found: ToF only"); return; }
    // lowest-noise setup: pressure x16, temperature x2, normal mode, IIR x16
    // -> a new reading every ~44 ms
    Wire.beginTransmission(BaroAddr); Wire.write(0xF4); Wire.write(0x57); Wire.endTransmission();
    Wire.beginTransmission(BaroAddr); Wire.write(0xF5); Wire.write(0x14); Wire.endTransmission();
    uint8_t c[24] = {0};
    Wire.beginTransmission(BaroAddr); Wire.write(0x88); Wire.endTransmission();
    Wire.requestFrom((int)BaroAddr, 24);
    for (int i = 0; i < 24 && Wire.available(); i++) c[i] = Wire.read();
    dig_T1 = (c[1] << 8) | c[0];   dig_T2 = (c[3] << 8) | c[2];   dig_T3 = (c[5] << 8) | c[4];
    dig_P1 = (c[7] << 8) | c[6];   dig_P2 = (c[9] << 8) | c[8];   dig_P3 = (c[11] << 8) | c[10];
    dig_P4 = (c[13] << 8) | c[12]; dig_P5 = (c[15] << 8) | c[14]; dig_P6 = (c[17] << 8) | c[16];
    dig_P7 = (c[19] << 8) | c[18]; dig_P8 = (c[21] << 8) | c[20]; dig_P9 = (c[23] << 8) | c[22];
    Serial.printf("[Height] BMP280 found at 0x%02X\n", BaroAddr);
  }
  // BaroAbs = altitude in cm from the pressure (Bosch 64-bit compensation)
  bool baroRead() {
    Wire.beginTransmission(BaroAddr); Wire.write(0xF7); Wire.endTransmission();
    if (Wire.requestFrom((int)BaroAddr, 6) < 6) return false;
    int32_t b[6];
    for (int i = 0; i < 6; i++) b[i] = Wire.read();
    int32_t adc_P = (b[0] << 12) | (b[1] << 4) | (b[2] >> 4);
    int32_t adc_T = (b[3] << 12) | (b[4] << 4) | (b[5] >> 4);
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    int64_t v1 = ((int64_t)t_fine) - 128000;
    int64_t v2 = v1 * v1 * (int64_t)dig_P6;
    v2 = v2 + ((v1 * (int64_t)dig_P5) << 17);
    v2 = v2 + (((int64_t)dig_P4) << 35);
    v1 = ((v1 * v1 * (int64_t)dig_P3) >> 8) + ((v1 * (int64_t)dig_P2) << 12);
    v1 = ((((int64_t)1) << 47) + v1) * ((int64_t)dig_P1) >> 33;
    if (v1 == 0) return false;
    int64_t p = 1048576 - (int64_t)adc_P;
    p = (((p << 31) - v2) * 3125) / v1;
    v1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    v2 = (((int64_t)dig_P8) * p) >> 19;
    p = ((p + v1 + v2) >> 8) + (((int64_t)dig_P7) << 4);   // Pa * 256
    double hPa = (double)p / 25600.0;
    if (hPa < 300 || hPa > 1100) return false;               // nonsense reading
    BaroAbs = 44330.0 * (1.0 - pow(hPa / 1013.25, 1.0 / 5.255)) * 100.0;
    return true;
  }

  // ---- scale training
  void trainReset() { TrX = TrY = TrXX = TrXY = 0; TrN = 0; TrMin = 1e9f; TrMax = -1e9f; }
  void trainAdd(float tofCm, float baroRel) {
    TrX += tofCm; TrY += baroRel; TrXX += (double)tofCm * tofCm; TrXY += (double)tofCm * baroRel; TrN++;
    if (tofCm < TrMin) TrMin = tofCm;
    if (tofCm > TrMax) TrMax = tofCm;
  }
  // new scale from this flight (blended with the saved one); false = not enough climb
  bool fitScale() {
    if (TrN < TRAIN_MIN_SAMPLES || TrMax - TrMin < BARO_TRAIN_SPAN_CM) return false;
    double mx = TrX / TrN, my = TrY / TrN;
    double varx = TrXX / TrN - mx * mx, cov = TrXY / TrN - mx * my;
    if (varx <= 1.0) return false;
    float fit = (float)(cov / varx);
    if (fit < BARO_SCALE_MIN || fit > BARO_SCALE_MAX) {
      Serial.printf("[Height] baro scale fit %.3f looks wrong (%d samples, %.0f-%.0f cm): kept %.3f\n",
                    fit, TrN, TrMin, TrMax, Scale);
      return false;
    }
    Scale = HaveSaved ? 0.5f * (SavedScale + fit) : fit;
    ScalePending = true;
    Serial.printf("[Height] baro trained: fit %.3f over %.0f-%.0f cm (%d samples) -> scale %.3f\n",
                  fit, TrMin, TrMax, TrN, Scale);
    return true;
  }
  void loadScale() {
    Preferences pr;
    pr.begin("simpletune", false);
    if (CLEAR_BARO_SCALE) pr.remove("bscale");
    float s = pr.getFloat("bscale", 0);
    pr.end();
    HaveSaved = s >= BARO_SCALE_MIN && s <= BARO_SCALE_MAX;
    SavedScale = Scale = HaveSaved ? s : 1.0f;
    Serial.printf("[Height] baro scale %.3f (%s)\n", Scale, HaveSaved ? "trained, from flash" : "not trained yet");
  }
  void saveScale() {
    Preferences pr;
    pr.begin("simpletune", false);
    pr.putFloat("bscale", Scale);
    pr.end();
    SavedScale = Scale; HaveSaved = true; ScalePending = false;
    Serial.printf("[Height] baro scale %.3f saved to flash\n", Scale);
  }

  // earth-frame vertical acceleration, cm/s^2, gravity removed (before bias)
  float verticalAcc() {
    float r = imu.Angle[0] * DEG_TO_RAD, p = imu.Angle[1] * DEG_TO_RAD;
    float sp = sinf(p), cp = cosf(p);
    float a = -sp * imu.AccFX + cp * sinf(r) * imu.AccFY + cp * cosf(r) * imu.AccFZ;
    return (a - imu.AccRestG) * 981.0f;
  }
};

HeightEstimator height;
