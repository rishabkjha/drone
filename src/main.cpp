// ============================================================================
//  SuperMini_Simple.ino  -  ONE-FILE flight controller for the ESP32-C3
//  SuperMini 3 inch quad, flown and tuned from the HoverIt phone app.
//
//  The control loops and filters are the ones from RX_RP_Tune.ino (the RFM69
//  flight controller whose gains were tuned in a few minutes), unchanged:
//    - 1D Kalman per axis (gyro predicts, accel corrects, gyro std 4 deg/s,
//      accel std 3 deg), run with the REAL measured loop time
//    - cascaded PID:  angle P (2.0)  ->  desired rate  ->  rate PID on the gyro
//      (Kp 0.6, Ki 3.5, Kd 0.03), yaw rate PID (Kp 2, Ki 12)
//    - D on the gyro (no stick kick), 30 Hz D-term low-pass, trapezoidal I
//    - throttle capped at 1800 us, motors never below 1100 us while flying,
//      every I term cleared while the throttle is low
//  What is new, because this drone is different hardware:
//    - MPU6050 on I2C (Y arrow to the front, X arrow to the left), with the
//      vibration gate from IMU.h: while the motors shake the accelerometer
//      (your log: up to 4 g) the angle runs on the gyro alone
//    - HoverIt app instead of the RFM69 radio (protocol v3, the app needs no
//      change): sticks, arm, flight mode, and the Tuning sliders set the gains
//    - 4 ESCs at 400 Hz, quad X mixer for this frame's props
//    - VL53L0X ToF + BMP280 barometer height for LOITER (altitude hold) and LAND
//    - 2S battery on GPIO 0 -> battery level in the app
//
//  Based on the Carbon Aeronautics quadcopter flight controller (v1.0, 2022),
//  through working_drone.ino and RX_RP_Tune.ino: its Kalman filter,
//  pid_equation, cascaded angle -> rate loops and gains. That work is licensed
//  CC BY-NC-SA 4.0, so this file is too: credit Carbon Aeronautics,
//  non-commercial use only, share changes under the same licence. Carbon
//  Aeronautics does not endorse this version.
//
//  Libraries: ESP32Servo, VL53L0X by Pololu (WiFi, Wire, Preferences come with
//  the ESP32 core). No other files: this .ino is the whole program.
//
//  Wiring
//    MPU6050 0x68, VL53L0X 0x29, BMP280 0x76 on I2C SDA 8 / SCL 9
//    ESC front left  GPIO 5   (prop ACW)       ESC front right GPIO 6  (prop CW)
//    ESC rear left   GPIO 10  (prop CW)        ESC rear right  GPIO 3  (prop ACW)
//    Battery 18K / 10K divider -> GPIO 0
//
//  Flight modes (picked in the app's Options):
//    STABILIZE   throttle stick 0-255 = 1000-2000 us, sticks set the angle
//    LOITER      stick at 50 % = hold height, above climbs, below sinks
//    LAND        25 cm/s down to the takeoff height, then disarms itself
//    AUTONOMOUS  arm = take off, 30 x 30 cm square at 50 cm, land, disarm (section 9)
//  Options > Enable Baro switch (section 7):
//    OFF  ToF height, max height 50 cm in every mode. The barometer takes
//         over only while the ToF has no reading.
//    ON   ToF below 50 cm (it trains the barometer there), barometer above,
//         max height 3 m. The trained baro scale is saved for the next flight.
//  The app shows "baro" next to the height while the barometer is in use.
//
//  Tuning from the app (Tuning screen, ENABLE at the top right):
//    ROLL+PITCH  rate loop for roll and pitch (one set of gains)
//    YAW RATE    yaw rate loop
//    VERT VEL    climb-rate loop for LOITER / LAND
//  The sliders jump to the drone's gains when you pick a loop; moving one
//  changes the gain live. Closing Tuning while DISARMED saves them to flash.
//  The first boot uses the RX_RP_Tune gains below (DEFAULTS). Your old
//  FlightController gains are NOT loaded: this sketch has its own flash slot.
//
//  !! SAFETY !!  Props OFF for these checks, Serial Monitor at 115200:
//   1. Always power up sitting level and still (AUTO_LEVEL takes that as level).
//   2. Motor test at boot: each motor spins alone for 1 s and the Serial
//      Monitor names the corner. A motor spinning somewhere else = ESC wire on
//      the wrong pin. Fix that first: it reverses roll or pitch.
//   3. Tilt by hand: right side down -> "ANG R" goes +, nose down -> "P" goes +,
//      turn clockwise (from above) -> RATE "Y" goes +.
//   4. Arm, throttle just above the bottom (PID starts at 1060 us). Press the
//      right side down: the RIGHT motors speed up. Press the nose down: the
//      FRONT motors speed up. Yaw stick right: front left + rear right speed up.
//   5. Then props on, low hover over grass, tethered if you can.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>
#include <ESP32Servo.h>
#include <VL53L0X.h>               // "VL53L0X" by Pololu

// ============================================================================
//  1. SETTINGS  (everything you are likely to change is here)
// ============================================================================

// ---- Wi-Fi: your phone's hotspot (or uncomment SOFTAP for the drone's own network)
#define WIFI_SSID "Rishab's Phone"     // !! your phone hotspot name !!
#define WIFI_PASS "12345678"           // !! and its password !!
// #define HOVERIT_SOFTAP 1            // drone makes its own network "HoverIt" / hoverit123

// ---- Pins
const int SDA_PIN = 8, SCL_PIN = 9;
const int ESC_FL_PIN = 5;              // front left,  prop anticlockwise
const int ESC_FR_PIN = 6;              // front right, prop clockwise
const int ESC_RL_PIN = 10;             // rear left,   prop clockwise
const int ESC_RR_PIN = 3;              // rear right,  prop anticlockwise
const int BATT_PIN   = 0;

// ---- Loop
// RX_RP_Tune ran at 1 kHz on SPI. The MPU6050 is on I2C here and the C3 has a
// single core shared with Wi-Fi, so this runs at 500 Hz. The PIDs and the
// Kalman use the real measured dt, so the gains mean the same thing.
const uint32_t LOOP_US = 2000;         // 500 Hz

// ---- ESCs
const int ESC_FREQ_HZ     = 400;       // your ESCs are set to 400 Hz
const int ESC_MIN_US      = 1000;
const int ESC_MAX_US      = 2000;
const int THROTTLE_MAX_US = 1800;      // RX_RP_Tune: headroom for the PID
const int ESC_IDLE_US     = 1100;      // RX_RP_Tune: flying, no motor below this
                                       // (raise to ~1150-1180 if a motor stutters)
const int PID_ON_US       = 1060;      // RX_RP_Tune MIN_THROTTLE_FOR_PID (60 of 1000)
const int READY_SPIN_US   = 1060;      // boot motor test (your motors start at 1036)

// ---- Sticks
const float MAX_ANGLE_DEG = 30.0f;      // RX_RP_Tune: full stick = 30 deg
const float MAX_YAW_RATE  = 75.0f;     // deg/s at full yaw stick
const float ROLL_SIGN  = 1.0f;         // -1 if stick right rolls LEFT
const float PITCH_SIGN = 1.0f;         // -1 if stick forward tilts the nose UP
const float YAW_SIGN   = 1.0f;         // -1 if stick right yaws LEFT
const float CRASH_TILT = 60.0f;        // deg: motors cut until you disarm

// ---- Controller gains (RX_RP_Tune.ino)
const float ANGLE_FF_LPF_HZ = 15.0f;   // smooths the app's 50 Hz stick steps (angle FF)
const float ANGLE_KP        = 4.0f;    // angle error (deg) -> desired rate (deg/s)
                                       // (RX_RP_Tune 2.0; 4.0 = snappier sticks)
const float ANGLE_FF        = 0.5f;    // stick SPEED passed straight to the rate loop, so the
                                       // drone starts tilting the moment the stick moves
// Flight trim: if it still drifts in STABILIZE with the sticks centred, nudge
// these (deg, 0.5-1 at a time). Drifts forward -> make PITCH more negative.
// Drifts right -> make ROLL more negative.
const float TRIM_ROLL_DEG  = 1.2f;
const float TRIM_PITCH_DEG = 1.0f;
const float DTERM_LPF_HZ = 30.0f;      // rate-loop D-term filter
// Rate-loop and climb gains at first boot. After that the app's sliders set
// them and they are saved in flash (set LOAD_SAVED_GAINS false to ignore flash).
struct Gains { float kp, ki, kd, ff; };
const Gains DEFAULT_RP   = { 0.60f,  3.5f, 0.030f, 0.10f };  // RX_RP_Tune rate loop + FF
const Gains DEFAULT_YAW  = { 2.00f, 12.0f, 0.000f, 0.40f };  // RX_RP_Tune yaw loop + FF
// FF (feedforward) = motor push straight from the stick, before any error
// builds up. If you already saved gains from the app, these defaults are not
// used: set FF with the app's FF slider (ROLL+PITCH ~0.10, YAW RATE ~0.40).
const Gains DEFAULT_VERT = { 1.00f,  0.5f, 0.000f, 0.65f };  // climb loop (gentle: see below)
// VERT_VEL is kept soft so it also holds while vibration shuts the
// accelerometer out (climb rate from the ToF alone). Raise it from the app.
const bool  LOAD_SAVED_GAINS = true;

// ---- IMU
// MPU6050 digital low-pass (register 0x1A), the same for gyro and accel on
// this chip: 0x03 = ~44 Hz (RX_RP_Tune used ~41 Hz on the gyro).
const uint8_t MPU_DLPF_CFG = 0x03;
// The MPU6050's Y arrow points to the FRONT and X to the left (chip upside
// down: found from gravity at boot, keep the drone level when powering on).
const char FORWARD_AXIS = 'Y';         // chip arrow pointing to the front: 'X' or 'Y'
const int  FORWARD_SIGN = 1;           // -1 if that arrow points to the BACK
// Level: every MPU6050 reads a few degrees off when level (yours read -4.4
// pitch once). Uncorrected, the drone holds that "level" and drifts (the
// forward bank). AUTO_LEVEL = true: whatever it reads at power-up counts as
// level, so ALWAYS power up sitting on a level surface.
// AUTO_LEVEL = false: the two LEVEL_TRIM values below are used instead.
const bool  AUTO_LEVEL       = true;
const float LEVEL_TRIM_ROLL  = 0.0f;   // deg (AUTO_LEVEL false only)
const float LEVEL_TRIM_PITCH = 0.0f;   // deg
// Vibration gate (IMU.h): accel ignored while it is shaken
const float ACC_VIB_HZ    = 2.0f;      // smoothing of the vibration level
const float ACC_VIB_MAX   = 0.25f;     // g, above this: angles on the gyro alone
const float ACC_VIB_OK    = 0.12f;     // g, below this for ACC_SETTLE_MS: accel back
const uint32_t ACC_SETTLE_MS = 500;
const float ACC_AGREE_DEG = 15.0f;     // accel angle this far from the estimate is ignored
const float ACC_QUIET     = 0.05f;     // ... unless the drone is quiet (sitting still)

// ---- Height (LOITER, LAND): VL53L0X ToF + BMP280 barometer (section 7)
const int   HOVER_US      = 1335;      // hovers here: your screenshot, throttle stick at 85 / 255
const int   HOLD_MIN_US   = 1150;
const int   HOLD_MAX_US   = 1800;
const float HEIGHT_KP     = 1.0f;      // cm off -> cm/s of climb
const float HEIGHT_FF     = 0.8f;      // climb stick passed through
const float MAX_CLIMB_CMS = 60.0f;
const float LOITER_CLIMB  = 40.0f;     // cm/s at full stick up / down
const float LOITER_DEADBAND = 0.10f;   // stick band around 50 % = hold
const float LAND_CMS      = 25.0f;
const float LEASH_CM      = 40.0f;     // held height stays within this of the real one
const float MAX_HEIGHT_CM = 50.0f;     // Enable Baro OFF: the drone never goes above this
                                       // (LOITER holds below it, STABILIZE caps the throttle)
// STABILIZE height limit: the climb rate allowed shrinks as the drone nears
// the limit (so it can brake in time), and the throttle is capped to keep to it
const float CEIL_DECEL    = 100.0f;    // cm/s^2 braking it plans for
const float CEIL_KP       = 1.0f;      // above the limit: cm/s down per cm over
const float CEIL_KV       = 1.0f;      // us of throttle per cm/s of climb too fast
const float CEIL_MAX_DROP = 150.0f;    // us: the cap never goes below HOVER_US minus this
const float VEL_DEADBAND_CMS = 3.0f;   // climb rate reads 0 below this
const float HEIGHT_TAU_S  = 0.4f;      // ToF/accel blend: higher = smoother, slower
const float HEIGHT_TAU_TOF_S = 0.25f;  // same while vibration shuts the accel out (ToF only)
const uint32_t TOF_LOST_MS = 300;      // no ToF reading this long = height unknown

// ---- Autonomous (app mode AUTONOMOUS): arm = take off, fly a square, land
// There is no position sensor, so each side is flown by time: tilt
// AUTO_TILT_DEG toward the side for half of it, tilt back the other way to
// stop, then hover still. Trims (TRIM_*_DEG), HOVER_US and the PID gains from
// the app are all used as in LOITER. If the real square comes out too small
// raise AUTO_SIDE_GAIN (1.2 = 20 % longer pushes), too big lower it.
const float AUTO_HEIGHT_CM  = 50.0f;   // square flown at this height
const float AUTO_SIDE_CM    = 30.0f;   // side of the square
const float AUTO_TILT_DEG   = 5.0f;    // lean while moving (on top of the trim)
const float AUTO_TILT_RATE  = 40.0f;   // deg/s the lean changes at (smooth starts and stops)
const float AUTO_SIDE_GAIN  = 1.0f;    // scales the push time per side (see above)
const float AUTO_BRAKE_GAIN = 1.0f;    // scales the stopping lean: lower it if the drone
                                       // drifts BACK after each side, raise it if it keeps sliding on
const float AUTO_CLIMB_CMS  = 25.0f;   // take-off climb rate
const float AUTO_SPOOL_S    = 1.0f;    // motors at idle this long after arming, before lift-off
const float AUTO_SETTLE_S   = 2.0f;    // hover at height before the first side
const float AUTO_CORNER_S   = 2.0f;    // hover at each corner
// Order of the sides, as stick directions: forward, right, back, left
const int8_t AUTO_PATH[4][2] = { {0, 1}, {1, 0}, {0, -1}, {-1, 0} };   // {roll, pitch}

// ---- Barometer (BMP280): the app's Options > Enable Baro switch
// OFF: ToF height as before. The barometer only takes over if the ToF has no
//      reading (out of range, blocked), then hands back when the ToF returns.
// ON:  ToF below BARO_HANDOFF_CM, barometer above. While the ToF is in use it
//      trains the barometer (offset all the time, scale over the climb), so
//      the handover is seamless. The scale is saved and reused next flight.
const float BARO_HANDOFF_CM      = 50.0f;   // switch ON: baro above this height
const float BARO_HANDOFF_HYST_CM = 5.0f;    // back to the ToF below 45 cm
const float MAX_HEIGHT_BARO_CM   = 300.0f;  // LOITER ceiling while the baro is usable
const float HEIGHT_TAU_BARO_S    = 1.0f;    // baro/accel blend: smoother (the baro is noisier)
const float HEIGHT_TAU_BARO_GATED_S = 0.35f; // while vibration shuts the accel out (faster, or the hold bobs)
const float BARO_OFFSET_TAU_S    = 2.0f;    // how fast the ToF corrects the baro offset
const float BARO_GLIDE_TAU_S     = 1.0f;    // baro -> ToF hand-back blends over this
const float TOF_LAND_CM          = 15.0f;   // ToF reads below this: the ToF is used, whatever the switch
                                            // (landing on the barometer, it takes over near the ground)
const float BARO_TRAIN_SPAN_CM   = 25.0f;   // climb needed for a new scale fit
const float BARO_SCALE_MIN = 0.6f, BARO_SCALE_MAX = 1.6f;   // fits outside are thrown away
const bool  CLEAR_BARO_SCALE     = false;   // true for one boot: forget the saved scale
// HOT FIX (2026-10-23): false = barometer never used, height from the ToF only.
// With no ToF reading there is then no height: LOITER refuses to take off.
// Set back to true to use the barometer again.
const bool  USE_BARO             = false;

// ---- Battery (2S, 18K top / 10K bottom)
const float BATT_DIVIDER = 2.8f;
const float BATT_CAL     = 1.000f;     // multimeter volts / printed volts
const float BATT_EMPTY_V = 6.8f;       // app shows 0
const float BATT_FULL_V  = 8.4f;       // app shows 255

// ---- Flight mode types (numbers and status list match the app's DroneLink.kt).
// Declared here, before any function: the Arduino IDE puts its automatic
// function prototypes above the first function, and they need these types.
enum Mode : uint8_t { MODE_STABILIZE = 0, MODE_LOITER = 1, MODE_LAND = 2, MODE_AUTO = 3 };
enum Status : uint8_t { ST_IDLE = 0, ST_FLYING, ST_HOLD, ST_CLIMB, ST_DESCEND, ST_LANDING, ST_LANDED,
                        ST_SPOOL, ST_AUTO_CLIMB, ST_SQUARE1 /* .. ST_SQUARE1 + 3 */ };
const char* const MODE_NAMES[4] = { "STABILIZE", "LOITER", "LAND", "AUTO" };

struct Cmd {
  bool  motors = false;      // false = motors off
  float throttle = 1000;     // us
  bool  pid = false;         // attitude PIDs running (RX_RP_Tune pidActive)
  bool  disarm = false;      // Land finished
  uint8_t status = ST_IDLE;
  float autoRoll = 0, autoPitch = 0;   // AUTONOMOUS: lean added to the sticks, deg (stick directions)
};

// ============================================================================
//  2. HOVERIT APP LINK  (HoverItRx.h, protocol v3, unchanged apart from the
//     hotspot name and password coming from WIFI_SSID / WIFI_PASS above)
// ============================================================================

#ifndef HOVERIT_SOFTAP
#define HOVERIT_SOFTAP 0                       // 0 = join phone hotspot, 1 = own network
#endif

namespace HoverIt {

// ---------------- Settings ---------------------------------------------------
const char* const HOTSPOT_SSID = WIFI_SSID;
const char* const HOTSPOT_PASS = WIFI_PASS;
const char* const AP_SSID      = "HoverIt";          // used when HOVERIT_SOFTAP = 1
const char* const AP_PASS      = "hoverit123";       // at least 8 characters

const uint16_t DRONE_PORT       = 4210;    // drone listens here (must match the app)
const uint16_t PHONE_PORT       = 4211;    // app listens here   (must match the app)
const uint32_t FAILSAFE_MS      = 500;     // no packet this long: disarm + centre
const uint32_t TELEMETRY_MS     = 100;     // battery to the phone at 10 Hz
const uint8_t  ARM_MAX_THROTTLE = 15;      // throttle must be at/below this to arm

// ESP32-C3 SuperMini boards have a poorly matched antenna: at full power
// (19.5 dBm) Wi-Fi drops packets in bursts. 8.5 dBm is the usual fix and still
// reaches 10-20 m. Try WIFI_POWER_11dBm or WIFI_POWER_5dBm if it is still patchy.
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_8_5dBm;

// ---------------- Packets (keep in sync with DroneLink.kt) ------------------
const uint8_t PROTOCOL_VERSION = 3;          // 3: flight mode bits, height + status in telemetry

struct __attribute__((packed)) CmdPacket {   // phone -> drone, 14 bytes
  uint8_t magic;                             // 'H'
  uint8_t seq;
  uint8_t throttle, yaw, pitch, roll;
  uint8_t kp, ki, kd, ff;
  uint8_t loop;                              // PID loop the sliders are for
  uint8_t flags;                             // bit0 armed, bit1 baro training, bit2 tuning,
                                             // bits4-5 flight mode (0-3)
  uint8_t version;
  uint8_t check;                             // XOR of all bytes before it
};

struct __attribute__((packed)) TelemPacket { // drone -> phone, 12 bytes
  uint8_t magic;                             // 'D'
  uint8_t seq;
  uint8_t battery;                           // 0-255
  uint8_t flags;                             // bit0 armed, bit1 baro training running,
                                             // bits4-5 flight mode the drone is flying
  uint8_t loop, kp, ki, kd, ff;              // the drone's gains for that loop
  uint8_t height;                            // cm / 2 (0-510 cm)
  uint8_t status;                            // what the mode is doing
  uint8_t check;
};

// ---------------- State ------------------------------------------------------
struct Values {
  uint8_t throttle = 0, yaw = 128, pitch = 128, roll = 128;
  uint8_t kp = 0, ki = 0, kd = 0, ff = 0;
  uint8_t loop = 0;
  uint8_t mode = 0;                          // 0 Stabilize, 1 Loiter, 2 Land, 3 Autonomous
  bool armed = false, training = false, tuning = false;
};
Values Rx;                    // what the flight code reads

WiFiUDP Udp;
IPAddress PhoneIp;
bool HavePhone = false;
bool Linked = false;
bool LastArmBit = false;      // for the 0 -> 1 arm edge
uint8_t Battery = 0;
uint8_t ModeNow = 0, StatusNow = 0, HeightNow = 0;
bool TrainingNow = false;
uint8_t GainLoop = 0, GainKp = 0, GainKi = 0, GainKd = 0, GainFf = 0;
uint8_t TxSeq = 0;
uint32_t LastPacketMs = 0, LastTelemMs = 0, LastWifiLogMs = 0;
uint32_t Packets = 0, BadPackets = 0;
uint32_t LastBadLogMs = 0, LoggedBad = 0;
uint32_t RateStartMs = 0, RatePackets = 0, MaxGapMs = 0, PrevPacketMs = 0;
int LastBadSize = 0, LastBadVersion = -1;      // what the last rejected packet looked like

inline uint8_t xorBytes(const uint8_t* p, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

inline void failsafe() {
  Rx.throttle = 0;
  Rx.yaw = Rx.pitch = Rx.roll = 128;
  Rx.armed = false;
  Rx.training = false;
  // kp..ff and loop keep their last values so a dropout doesn't zero your gains
}

inline IPAddress broadcastIp() {
#if HOVERIT_SOFTAP
  return IPAddress(192, 168, 4, 255);
#else
  if (WiFi.status() != WL_CONNECTED) return IPAddress(255, 255, 255, 255);
  IPAddress ip = WiFi.localIP(), mask = WiFi.subnetMask();
  return IPAddress(ip[0] | ~mask[0], ip[1] | ~mask[1], ip[2] | ~mask[2], ip[3] | ~mask[3]);
#endif
}

inline void handlePacket(const CmdPacket& c, IPAddress from) {
  if (!HavePhone || from != PhoneIp) {
    PhoneIp = from;
    HavePhone = true;
    Serial.print("[HoverIt] phone at ");
    Serial.println(PhoneIp);
  }
  if (!Linked) Serial.println("[HoverIt] link up");
  Linked = true;
  uint32_t t = millis();
  if (PrevPacketMs && t - PrevPacketMs > MaxGapMs) MaxGapMs = t - PrevPacketMs;
  PrevPacketMs = t;
  LastPacketMs = t;
  Packets++;

  Rx.throttle = c.throttle;
  Rx.yaw = c.yaw;
  Rx.pitch = c.pitch;
  Rx.roll = c.roll;
  Rx.kp = c.kp;
  Rx.ki = c.ki;
  Rx.kd = c.kd;
  Rx.ff = c.ff;
  Rx.loop = c.loop;
  Rx.training = c.flags & 0x02;
  Rx.mode = (c.flags >> 4) & 0x03;
  Rx.tuning = c.flags & 0x04;

  bool armBit = c.flags & 0x01;
  if (!armBit) {
    Rx.armed = false;
  } else if (!LastArmBit && !Rx.armed) {            // 0 -> 1: arm request
    if (c.throttle <= ARM_MAX_THROTTLE) Rx.armed = true;
    else Serial.println("[HoverIt] arm refused: throttle not at the bottom");
  }
  LastArmBit = armBit;
}

inline void sendTelemetry() {
  TelemPacket t;
  t.magic = 'D';
  t.seq = TxSeq++;
  t.battery = Battery;
  t.flags = (Rx.armed ? 0x01 : 0x00) | (TrainingNow ? 0x02 : 0x00) | ((ModeNow & 0x03) << 4);
  t.loop = GainLoop;
  t.kp = GainKp; t.ki = GainKi; t.kd = GainKd; t.ff = GainFf;
  t.height = HeightNow;
  t.status = StatusNow;
  t.check = xorBytes((const uint8_t*)&t, sizeof(t) - 1);
  // linked: straight to the phone. Not linked: broadcast so the app can find
  // us, and also to the last phone we knew, because some phones drop
  // broadcasts on the drone's own network (HOVERIT_SOFTAP).
  if (!(HavePhone && Linked)) {
    Udp.beginPacket(broadcastIp(), PHONE_PORT);
    Udp.write((const uint8_t*)&t, sizeof(t));
    Udp.endPacket();
  }
  if (HavePhone) {
    Udp.beginPacket(PhoneIp, PHONE_PORT);
    Udp.write((const uint8_t*)&t, sizeof(t));
    Udp.endPacket();
  }
}

} // namespace HoverIt

inline void hoverItSetup() {
  using namespace HoverIt;
  failsafe();
  WiFi.persistent(false);
#if HOVERIT_SOFTAP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[HoverIt] own network \"");
  Serial.print(AP_SSID);
  Serial.print("\" at ");
  Serial.println(WiFi.softAPIP());
#else
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(HOTSPOT_SSID, HOTSPOT_PASS);
  Serial.print("[HoverIt] joining hotspot \"");
  Serial.print(HOTSPOT_SSID);
  Serial.println("\" ...");
#endif
  WiFi.setSleep(false);                  // modem sleep adds 100+ ms of lag
  WiFi.setTxPower(WIFI_TX_POWER);        // SuperMini antenna fix (see settings)
  Udp.begin(DRONE_PORT);
}

inline void hoverItUpdate() {
  using namespace HoverIt;
  uint32_t now = millis();

  // read everything that arrived since the last loop, keep the newest
  int size;
  while ((size = Udp.parsePacket()) > 0) {
    CmdPacket c;
    if (size != sizeof(CmdPacket)) {   // next parsePacket() drops the rest
      BadPackets++;
      LastBadSize = size;
      LastBadVersion = -1;
      continue;
    }
    Udp.read((uint8_t*)&c, sizeof(c));
    if (c.magic != 'H' || c.version != PROTOCOL_VERSION ||
        c.check != xorBytes((const uint8_t*)&c, sizeof(c) - 1)) {
      BadPackets++;
      LastBadSize = size;
      LastBadVersion = c.version;
      continue;
    }
    handlePacket(c, Udp.remoteIP());
  }
  now = millis();   // AFTER reading: handlePacket() stamps LastPacketMs later than the old 'now'

  // rejected packets: say why, every 2 s (13 bytes or protocol 1-2 = old app)
  if (BadPackets != LoggedBad && now - LastBadLogMs > 2000) {
    LastBadLogMs = now;
    Serial.printf("[HoverIt] %u packets rejected: last was %d bytes", (unsigned)(BadPackets - LoggedBad), LastBadSize);
    if (LastBadVersion >= 0) Serial.printf(", protocol %d", LastBadVersion);
    Serial.printf(" (this drone wants %u bytes, protocol %u)%s\n", (unsigned)sizeof(CmdPacket), PROTOCOL_VERSION,
                  (LastBadSize == 13 || (LastBadVersion >= 0 && LastBadVersion < PROTOCOL_VERSION))
                      ? " -> the app is the OLD version, reinstall it" : "");
    LoggedBad = BadPackets;
  }

  if (Linked && (int32_t)(now - LastPacketMs) > (int32_t)FAILSAFE_MS) {
    Linked = false;
    LastArmBit = false;
    failsafe();
    Serial.println("[HoverIt] link lost -> failsafe (disarmed)");
  }

  // link quality every 5 s while packets arrive: the app sends 50 per second
  if (now - RateStartMs >= 5000) {
    uint32_t got = Packets - RatePackets;
    if (got > 0)
      Serial.printf("[HoverIt] %u packets/s from the app (50 expected), longest gap %u ms, signal %d dBm\n",
                    (unsigned)(got * 1000 / (now - RateStartMs)), (unsigned)MaxGapMs, (int)WiFi.RSSI());
    RateStartMs = now;
    RatePackets = Packets;
    MaxGapMs = 0;
  }

  // no network yet (hotspot not joined): sending would only fail (error 118)
  bool netUp = HOVERIT_SOFTAP || WiFi.status() == WL_CONNECTED;
  if (netUp && now - LastTelemMs >= TELEMETRY_MS) {
    LastTelemMs = now;
    sendTelemetry();
  }

#if !HOVERIT_SOFTAP
  if (now - LastWifiLogMs > 3000) {
    LastWifiLogMs = now;
    static bool wasUp = false;
    bool up = WiFi.status() == WL_CONNECTED;
    if (up && !wasUp) { Serial.print("[HoverIt] on hotspot, drone IP "); Serial.println(WiFi.localIP()); }
    if (!up) {
      // status 1 = hotspot name not found (wrong name, or 5 GHz only),
      // 4 = wrong password / WPA3-only, 6 = still trying
      Serial.print("[HoverIt] not on hotspot \"");
      Serial.print(HOTSPOT_SSID);
      Serial.print("\" yet, status ");
      Serial.println((int)WiFi.status());
    }
    wasUp = up;
  }
#endif
}

inline bool hoverItConnected() { return HoverIt::Linked; }
inline bool hoverItArmed()     { return HoverIt::Rx.armed; }

// The drone ends a flight by itself (Land touched down). The arm bit from the
// app stays 1, so this does not re-arm until the app sends a new 0 -> 1 press.
inline void hoverItDisarm() {
  if (HoverIt::Rx.armed) Serial.println("[HoverIt] landed -> disarmed by the drone");
  HoverIt::Rx.armed = false;
}

inline void hoverItSetStatus(uint8_t mode, uint8_t status, float heightCm, bool training) {
  using namespace HoverIt;
  ModeNow = mode; StatusNow = status; TrainingNow = training;
  float h = heightCm / 2.0f;
  HeightNow = h <= 0 ? 0 : h >= 255 ? 255 : (uint8_t)(h + 0.5f);
}

inline void hoverItSetBattery(uint8_t level) { HoverIt::Battery = level; }

inline void hoverItSetGains(uint8_t loop, uint8_t kp, uint8_t ki, uint8_t kd, uint8_t ff) {
  using namespace HoverIt;
  if (loop != GainLoop) LastTelemMs = millis() - TELEMETRY_MS;   // new loop: tell the app now
  GainLoop = loop; GainKp = kp; GainKi = ki; GainKd = kd; GainFf = ff;
}

inline int   hoverItStickUs(uint8_t b)  { return 1000 + (int)((b * 1000L + 127) / 255); }
inline float hoverItCentered(uint8_t b) { return ((int)b - 128) / (b >= 128 ? 127.0f : 128.0f); }

// ============================================================================
//  3. IMU  -  MPU6050, RX_RP_Tune's 1D Kalman + IMU.h's vibration gate
//  Drone frame after the read: X forward, Y left, Z up.
//  roll + = right side down, pitch + = nose down, yaw + = clockwise from above
// ============================================================================

namespace Imu {

const float GYRO_LSB  = 65.5f;           // +-500 deg/s
const float ACC_LSB   = 2048.0f;         // +-16 g (shaking past +-8 g clipped one side)
const float KALMAN_GYRO_STD  = 4.0f;     // deg/s   (RX_RP_Tune)
const float KALMAN_ACCEL_STD = 3.0f;     // deg     (RX_RP_Tune)
const float ACC_HEIGHT_HZ    = 20.0f;    // smoothed accel for the height filter

int   UpSign = 1;                        // +1 chip Z up, -1 upside down (found at boot)
float GyroBiasR = 0, GyroBiasP = 0, GyroBiasY = 0;
float AccRestG = 1.0f;                   // |accel| at rest as this chip reads it (yours ~0.88 g)
float TrimR = 0, TrimP = 0;              // level trim in use, deg

// raw, in the drone frame
float RateR, RateP, RateY;               // deg/s (before bias removal)
float AccX, AccY, AccZ;                  // g
float RawAccZ;                           // chip Z, g (upside-down check)
// results
float Rate[3] = {0, 0, 0};               // roll, pitch, yaw deg/s, bias removed
float Angle[2] = {0, 0};                 // roll, pitch deg (Kalman, level trim applied)
float Uncert[2] = {4, 4};                // Kalman variance, deg^2
float AccFX = 0, AccFY = 0, AccFZ = 1;   // ACC_HEIGHT_HZ smoothed accel, g
float AccVib = 0;                        // smoothed | |accel| - AccRestG |, g
bool  AccTrusted = true;
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
inline void toBody(float cx, float cy, float cz, float &f, float &l, float &u) {
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
inline float accRollRaw(float x, float y, float z)  { return  atan2f(y, sqrtf(x * x + z * z)) * RAD_TO_DEG; }
inline float accPitchRaw(float x, float y, float z) { return -atan2f(x, sqrtf(y * y + z * z)) * RAD_TO_DEG; }

// RX_RP_Tune Kalman1D::update, plus the IMU.h gate
void kalman(int i, float rate, float accAngle, float dt) {
  Angle[i]  += dt * rate;
  Uncert[i] += dt * dt * KALMAN_GYRO_STD * KALMAN_GYRO_STD;
  if (!AccTrusted) return;                                                // shaken: gyro only
  if (fabsf(accAngle - Angle[i]) > ACC_AGREE_DEG && AccVib >= ACC_QUIET) return;   // implausible
  float gain = Uncert[i] / (Uncert[i] + KALMAN_ACCEL_STD * KALMAN_ACCEL_STD);
  Angle[i]  += gain * (accAngle - Angle[i]);
  Uncert[i]  = (1.0f - gain) * Uncert[i];
}

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

} // namespace Imu

// ============================================================================
//  4. PID  -  RX_RP_Tune's PID: P on error, trapezoidal I clamped, D on the
//  measurement (filtered), output clamped. Plus FF on the setpoint (0 unless
//  you raise the FF slider) and an I preload for a bumpless altitude hold.
// ============================================================================

struct PID {
  float kp, ki, kd, ff;
  float outLimit;      // |output| clamp
  float iLimit;        // |I-term| clamp
  float dLpfHz;        // D-term low-pass, 0 = none
  float iterm, prevError, prevMeas, dFilt, out;
  bool  first;

  PID(float p, float i, float d, float f, float outLim, float iLim, float dHz)
    : kp(p), ki(i), kd(d), ff(f), outLimit(outLim), iLimit(iLim), dLpfHz(dHz),
      iterm(0), prevError(0), prevMeas(0), dFilt(0), out(0), first(true) {}

  float update(float setpoint, float meas, float dt) {
    float error = setpoint - meas;
    if (first) { prevError = error; prevMeas = meas; }
    float pterm = kp * error;

    iterm += ki * (error + prevError) * dt * 0.5f;
    iterm = constrain(iterm, -iLimit, iLimit);

    float dRaw = first ? 0.0f : -(meas - prevMeas) / dt;
    float w = 2.0f * PI * dLpfHz * dt;
    float dAlpha = dLpfHz > 0 ? w / (1.0f + w) : 1.0f;
    dFilt += dAlpha * (dRaw - dFilt);
    float dterm = kd * dFilt;

    prevError = error;
    prevMeas  = meas;
    first     = false;
    out = constrain(pterm + iterm + dterm + ff * setpoint, -outLimit, outLimit);
    return out;
  }
  void set(const Gains& g) { kp = g.kp; ki = g.ki; kd = g.kd; ff = g.ff; }
  void preloadI(float v)   { iterm = constrain(v, -iLimit, iLimit); }
  void reset() { iterm = 0; prevError = 0; prevMeas = 0; dFilt = 0; out = 0; first = true; }
};

//                  (all gains: Tune::setup)
PID pidAngleRoll  ( ANGLE_KP, 0, 0, 0, 400, 400, DTERM_LPF_HZ );    // deg -> deg/s
PID pidAnglePitch ( ANGLE_KP, 0, 0, 0, 400, 400, DTERM_LPF_HZ );
PID pidRateRoll   ( 0, 0, 0, 0, 400, 400, DTERM_LPF_HZ );            // deg/s -> us
PID pidRatePitch  ( 0, 0, 0, 0, 400, 400, DTERM_LPF_HZ );
PID pidRateYaw    ( 0, 0, 0, 0, 200, 200, DTERM_LPF_HZ );
PID pidClimb      ( 0, 0, 0, 0, 300, 250, 10.0f );                   // cm/s -> us

void resetAttitudePid() {
  pidAngleRoll.reset(); pidAnglePitch.reset();
  pidRateRoll.reset();  pidRatePitch.reset(); pidRateYaw.reset();
}

// ============================================================================
//  5. MOTORS  (ESP32Servo, 400 Hz)
// ============================================================================

namespace Motors {
enum { FL, FR, RL, RR };
const int PINS[4] = { ESC_FL_PIN, ESC_FR_PIN, ESC_RL_PIN, ESC_RR_PIN };
const char* const NAMES[4] = { "FRONT LEFT (ACW)", "FRONT RIGHT (CW)", "REAR LEFT (CW)", "REAR RIGHT (ACW)" };
Servo esc[4];
int Us[4] = { 1000, 1000, 1000, 1000 };

void write(int i, int us) {
  Us[i] = constrain(us, ESC_MIN_US, ESC_MAX_US);
  esc[i].writeMicroseconds(Us[i]);
}
void all(int us) { for (int i = 0; i < 4; i++) write(i, us); }

void setup() {
  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  for (int i = 0; i < 4; i++) {
    esc[i].setPeriodHertz(ESC_FREQ_HZ);
    esc[i].attach(PINS[i], ESC_MIN_US, ESC_MAX_US);
    esc[i].writeMicroseconds(ESC_MIN_US);   // stop signal right away
  }
}

// Boot check: each motor alone, named on the Serial Monitor. If the one that
// spins is not where the line says, the ESC wires are on the wrong pins.
void readySpin() {
  for (int i = 0; i < 4; i++) {
    all(ESC_MIN_US);
    Serial.printf("Motor test: GPIO %d should be %s\n", PINS[i], NAMES[i]);
    write(i, READY_SPIN_US);
    delay(1000);
    all(ESC_MIN_US);
    delay(400);
  }
}
} // namespace Motors

// ============================================================================
//  6. BATTERY  (2S on GPIO 0, Battery.h)
// ============================================================================

namespace Batt {
float Volts = 0;
uint32_t LastMs = 0;

float readVolts() {
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(BATT_PIN);
  return (mv / 8.0f) / 1000.0f * BATT_DIVIDER * BATT_CAL;
}
void setup() {
  analogReadResolution(12);
  analogSetPinAttenuation(BATT_PIN, ADC_11db);
  pinMode(BATT_PIN, INPUT);
  Volts = readVolts();
  LastMs = millis();
}
void update() {
  uint32_t now = millis();
  if (now - LastMs < 20) return;          // 50 Hz
  LastMs = now;
  Volts += 0.05f * (readVolts() - Volts);
}
uint8_t level() {
  float f = (Volts - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V);
  f = constrain(f, 0.0f, 1.0f);
  return (uint8_t)(f * 255.0f + 0.5f);
}
} // namespace Batt

// ============================================================================
//  7. HEIGHT  -  height and climb rate from the VL53L0X ToF, the BMP280
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
//  Object `height`: height.cm(), height.velocity(), height.ok(),
//  height.maxCm() (the height limit for the switch position).
// ============================================================================

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
      Imu::update(0.01f); as += verticalAcc();
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
      if (tofRead(d) && fabsf(Imu::Angle[0]) <= TOF_MAX_TILT && fabsf(Imu::Angle[1]) <= TOF_MAX_TILT) {
        float t = d * cosf(Imu::Angle[0] * DEG_TO_RAD) * cosf(Imu::Angle[1] * DEG_TO_RAD) - TofZero;
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

    bool accOk = Imu::AccTrusted && fabsf(Imu::Angle[0]) < MAX_TILT_ACC && fabsf(Imu::Angle[1]) < MAX_TILT_ACC;
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

  VL53L0X tof;
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
    Serial.print("[Height] I2C devices on SDA 8 / SCL 9:");
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
    // Pololu library: unlike Adafruit_VL53L0X it accepts every chip revision
    // (this sensor reports rev 1.15, Adafruit only takes 1.1)
    tof.setBus(&Wire);
    tof.setTimeout(50);                     // ms, only used during init
    for (int attempt = 1; attempt <= 3 && !TofPresent; attempt++) {
      if (attempt > 1) { delay(100); tofSoftReset(); }
      TofPresent = tof.init();
      if (!TofPresent) Serial.printf("[Height] VL53L0X init try %d failed\n", attempt);
    }
    Serial.println(TofPresent ? "[Height] VL53L0X found" : "[Height] VL53L0X NOT found: height from the barometer only");
    if (!TofPresent) return;
    // long range: lower signal limit, longer laser pulses (Pololu's LONG_RANGE example)
    tof.setSignalRateLimit(0.1f);
    tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tof.setMeasurementTimingBudget(33000);  // us
    tof.startContinuous(33);                // ~30 Hz
  }

  bool tofRead(float &cm) {
    // non-blocking: readRangeContinuousMillimeters() would wait for the sample
    if (!TofPresent || (tof.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return false;
    uint16_t mm = tof.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    tof.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
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
    float r = Imu::Angle[0] * DEG_TO_RAD, p = Imu::Angle[1] * DEG_TO_RAD;
    float sp = sinf(p), cp = cosf(p);
    float a = -sp * Imu::AccFX + cp * sinf(r) * Imu::AccFY + cp * cosf(r) * Imu::AccFZ;
    return (a - Imu::AccRestG) * 981.0f;
  }
};

HeightEstimator height;

// ============================================================================
//  8. TUNING FROM THE APP  (3 loops, same order as PID_LOOPS in the app's
//  DroneLink.kt: gain = byte / 255 * MAX)
// ============================================================================

namespace Tune {
enum { RP_RATE, YAW_RATE, VERT_VEL, LOOPS };
const char* const NAMES[LOOPS] = { "RP_RATE", "YAW_RATE", "VERT_VEL" };
const char* const DEFAULT_NAMES[LOOPS] = { "DEFAULT_RP  ", "DEFAULT_YAW ", "DEFAULT_VERT" };
const Gains MAX[LOOPS] = {
  //  kp     ki     kd     ff
  { 2.0f,  8.0f, 0.08f, 1.0f },   // RP_RATE   us per deg/s
  { 5.0f, 25.0f, 0.10f, 2.0f },   // YAW_RATE  us per deg/s
  { 5.0f,  5.0f, 0.50f, 3.0f },   // VERT_VEL  us per cm/s
};
const int ANGLE_BUILD_LOOPS = 5;  // a build that also tuned the angle loops saved 5 rows
uint8_t Bytes[LOOPS][4];
uint8_t Selected = RP_RATE;
bool Synced = false, WasTuning = false, Dirty = false;
Preferences Prefs;

uint8_t toByte(float v, float max) {
  float b = max > 0 ? v / max * 255.0f + 0.5f : 0;
  return b < 0 ? 0 : b > 255 ? 255 : (uint8_t)b;
}
float toGain(uint8_t b, float max) { return b * max / 255.0f; }
Gains gains(int l) {
  return { toGain(Bytes[l][0], MAX[l].kp), toGain(Bytes[l][1], MAX[l].ki),
           toGain(Bytes[l][2], MAX[l].kd), toGain(Bytes[l][3], MAX[l].ff) };
}
void fromGains(int l, const Gains& g) {
  Bytes[l][0] = toByte(g.kp, MAX[l].kp); Bytes[l][1] = toByte(g.ki, MAX[l].ki);
  Bytes[l][2] = toByte(g.kd, MAX[l].kd); Bytes[l][3] = toByte(g.ff, MAX[l].ff);
}
void apply(int l) {
  Gains g = gains(l);
  if (l == RP_RATE)  { pidRateRoll.set(g); pidRatePitch.set(g); }
  if (l == YAW_RATE) pidRateYaw.set(g);
  if (l == VERT_VEL) pidClimb.set(g);
}
void print() {
  for (int l = 0; l < LOOPS; l++) {
    Gains g = gains(l);
    Serial.printf("  %-9s kp %.4f  ki %.4f  kd %.4f  ff %.4f\n", NAMES[l], g.kp, g.ki, g.kd, g.ff);
  }
  Serial.printf("  fixed     angle kp %.2f ff %.2f | height kp %.2f ff %.2f | hover %d us\n",
                ANGLE_KP, ANGLE_FF, HEIGHT_KP, HEIGHT_FF, HOVER_US);
}
void save() {
  Prefs.begin("simpletune", false);
  Prefs.putBytes("g", Bytes, sizeof(Bytes));
  Prefs.end();
  Dirty = false;
  Serial.println("[Tune] gains saved to flash. To make them the defaults, copy into the sketch:");
  for (int l = 0; l < LOOPS; l++) {
    Gains g = gains(l);
    Serial.printf("  %s = { %.4ff, %.4ff, %.4ff, %.4ff }\n", DEFAULT_NAMES[l], g.kp, g.ki, g.kd, g.ff);
  }
}
void setup() {
  fromGains(RP_RATE, DEFAULT_RP);
  fromGains(YAW_RATE, DEFAULT_YAW);
  fromGains(VERT_VEL, DEFAULT_VERT);
  if (LOAD_SAVED_GAINS) {
    Prefs.begin("simpletune", false);     // read-write: creates the slot on first boot
    size_t len = Prefs.getBytesLength("g");
    if (len == sizeof(Bytes)) {
      Prefs.getBytes("g", Bytes, sizeof(Bytes));
      Serial.println("[Tune] gains loaded from flash");
    } else if (len == ANGLE_BUILD_LOOPS * 4) {  // keep its first 3 rows, angle loops are fixed again
      uint8_t all[ANGLE_BUILD_LOOPS][4];
      Prefs.getBytes("g", all, sizeof(all));
      memcpy(Bytes, all, sizeof(Bytes));
      Serial.println("[Tune] rate + climb gains loaded from flash");
    } else {
      Serial.println("[Tune] first boot: RX_RP_Tune gains");
    }
    Prefs.end();
  }
  for (int l = 0; l < LOOPS; l++) apply(l);
  print();
}
// Drone is the master copy: nothing changes until the app's sliders match the
// drone's bytes once (the app loads them from telemetry), then every move applies.
void update() {
  const HoverIt::Values& rx = HoverIt::Rx;
  if (hoverItConnected() && rx.loop < LOOPS && rx.loop != Selected) { Selected = rx.loop; Synced = false; }
  bool tuning = hoverItConnected() && rx.tuning;
  if (!tuning) {
    Synced = false;
    if (WasTuning) Serial.println("[Tune] tuning closed");
    WasTuning = false;
    if (Dirty && !hoverItArmed()) save();  // flash writes stall the loop: never while armed
  } else {
    if (!WasTuning) Serial.printf("[Tune] tuning %s\n", NAMES[Selected]);
    WasTuning = true;
    uint8_t in[4] = { rx.kp, rx.ki, rx.kd, rx.ff };
    bool same = memcmp(in, Bytes[Selected], 4) == 0;
    if (!Synced) { if (same) Synced = true; }
    else if (!same) { memcpy(Bytes[Selected], in, 4); apply(Selected); Dirty = true; }
  }
  hoverItSetGains(Selected, Bytes[Selected][0], Bytes[Selected][1], Bytes[Selected][2], Bytes[Selected][3]);
}
} // namespace Tune

// ============================================================================
//  9. FLIGHT MODES  (numbers and status list match the app's DroneLink.kt)
// ============================================================================

// (Mode, Status and Cmd are declared at the end of section 1)

uint8_t ModeFlown = 0xFF;    // 0xFF = enter the app's mode fresh
bool    InAir = false;       // Loiter / Land: motors running under the height loops
float   Setpoint = 0;        // cm, held height
uint32_t LowSinceMs = 0, StuckSinceMs = 0;

float tiltFactor() {
  float r = min(fabsf(Imu::Angle[0]), 35.0f), p = min(fabsf(Imu::Angle[1]), 35.0f);
  return cosf(r * DEG_TO_RAD) * cosf(p * DEG_TO_RAD);
}

// climb rate (cm/s) -> throttle (us); hold = true also runs the height loop
float heightLoop(float climb, bool hold, float dt) {
  float h = height.cm(), v = height.velocity(), desired;
  if (hold) {
    float top = height.maxCm();
    if ((climb > 0 && Setpoint >= top) || (climb < 0 && Setpoint <= 0)) climb = 0;   // at the limit
    Setpoint += constrain(climb, -MAX_CLIMB_CMS, MAX_CLIMB_CMS) * dt;
    Setpoint = constrain(Setpoint, 0.0f, top);
    Setpoint = constrain(Setpoint, h - LEASH_CM, h + LEASH_CM);
    desired = HEIGHT_KP * (Setpoint - h) + HEIGHT_FF * climb;
  } else {
    Setpoint = h;
    desired = climb;
  }
  desired = constrain(desired, -MAX_CLIMB_CMS, MAX_CLIMB_CMS);
  float thr = HOVER_US + pidClimb.update(desired, v, dt);
  thr = 1000.0f + (thr - 1000.0f) / tiltFactor();
  return constrain(thr, (float)HOLD_MIN_US, (float)HOLD_MAX_US);
}

// STABILIZE height limit: the most throttle allowed so it stops at height.maxCm()
// (the climb rate allowed shrinks near the limit so it can brake in time)
float ceilingThrottle() {
  float room = height.maxCm() - height.cm();                 // cm left below the limit
  float vMax = room > 0 ? sqrtf(2.0f * CEIL_DECEL * room) : CEIL_KP * room;   // cm/s
  float thr = HOVER_US + CEIL_KV * (vMax - height.velocity());
  thr = max(thr, HOVER_US - CEIL_MAX_DROP);
  return 1000.0f + (thr - 1000.0f) / tiltFactor();
}

// start holding from the throttle the motors are at (no jump)
void heightStart(float throttleUs) {
  pidClimb.reset();
  Setpoint = height.cm();
  float level = 1000.0f + (throttleUs - 1000.0f) * tiltFactor();
  pidClimb.preloadI(level - HOVER_US);
  LowSinceMs = StuckSinceMs = 0;
}

// on the ground: low and still for 0.6 s, or pushing down without moving for 2 s
bool touchedDown(float thr) {
  uint32_t now = millis();
  float h = height.cm(), v = fabsf(height.velocity()), t = height.tofCm();
  bool low = (h < 10.0f || (t >= 0 && t < 6.0f)) && v < 10.0f;
  bool stuck = v < 5.0f && thr < HOVER_US - 150;
  if (!low) LowSinceMs = 0; else if (!LowSinceMs) LowSinceMs = now;
  if (!stuck) StuckSinceMs = 0; else if (!StuckSinceMs) StuckSinceMs = now;
  return (LowSinceMs && now - LowSinceMs >= 600) || (StuckSinceMs && now - StuckSinceMs >= 2000);
}

// ---- AUTONOMOUS: arm in the app and the drone does it all:
//   spool up -> climb to AUTO_HEIGHT_CM -> hover -> fly a square of
//   AUTO_SIDE_CM (forward, right, back, left) -> land -> disarm.
// Height is held exactly like LOITER (same VERT VEL gains, HOVER_US, limit).
// There is no position sensor, so each side is flown by time: lean
// AUTO_TILT_DEG toward the side, lean back the same amount to stop, then
// hover at the corner. The lean is added on top of the trim, so a well
// trimmed drone draws a square; a badly trimmed one draws a drifting one.
// To stop it: pick LOITER (holds height where it is) or LAND, or disarm.
namespace Auto {
enum Stage_t : uint8_t { SPOOL, CLIMB, SETTLE, PUSH, BRAKE, CORNER, LAND, DONE };
Stage_t Stage = SPOOL;
float T = 0, Lean = 0, TPush = 0.5f, TRamp = 0.1f;
int Side = 0;
uint32_t WarnMs = 0;

float target() { return min(AUTO_HEIGHT_CM, height.maxCm()); }
void next(Stage_t s, const char* msg) { Stage = s; T = 0; if (msg) Serial.println(msg); }

// called when AUTONOMOUS is picked (inAir: already flying)
void begin(bool inAir) {
  Stage = inAir ? CLIMB : SPOOL;
  T = 0; Side = 0; Lean = 0;
  // push time per side: lean a = g*tan(tilt) for TPush, then the other way
  // for TPush + ramp, so it stops. Distance ~ a * TPush * (TPush + ramp).
  float a = 981.0f * tanf(AUTO_TILT_DEG * DEG_TO_RAD);          // cm/s^2
  float ramp = AUTO_TILT_DEG / AUTO_TILT_RATE;                    // s
  TRamp = ramp;
  TPush = AUTO_SIDE_GAIN * 0.5f * (-ramp + sqrtf(ramp * ramp + 4.0f * AUTO_SIDE_CM / a));
  Serial.printf("AUTO: %.0f cm square at %.0f cm, lean %.1f deg for %.2f s per side\n",
                AUTO_SIDE_CM, target(), AUTO_TILT_DEG, TPush);
}

Cmd update(float dt) {                          // the app locks the sticks in this mode
  Cmd c;
  T += dt;
  float leanTo = 0;

  switch (Stage) {
    case SPOOL:                                 // motors at idle, PIDs off, still on the ground
      if (!height.ok()) {
        if (millis() - WarnMs > 2000) { WarnMs = millis(); Serial.println("AUTO: no ToF or barometer reading, not taking off"); }
        c.status = ST_IDLE; T = 0; return c;
      }
      c.motors = true; c.throttle = ESC_IDLE_US; c.status = ST_SPOOL;
      if (T >= AUTO_SPOOL_S) {
        heightStart(HOVER_US - 50);              // just under hover: lifts off gently
        InAir = true;
        next(CLIMB, "AUTO: takeoff");
      }
      return c;

    case CLIMB: {                               // to the square's height (or down to it)
      float tgt = target(), err = tgt - height.cm();
      float climb = fabsf(err) > 5.0f ? (err > 0 ? AUTO_CLIMB_CMS : -AUTO_CLIMB_CMS) : 0;
      c.throttle = heightLoop(climb, true, dt);
      // the held height runs ahead of the drone but never past the target
      if ((err > 0 && Setpoint > tgt) || (err < 0 && Setpoint < tgt)) Setpoint = tgt;
      c.status = ST_AUTO_CLIMB;
      if (climb == 0) { Setpoint = tgt; next(SETTLE, "AUTO: at height"); }
      break;
    }

    case SETTLE:                                // hover still before the first side
      c.throttle = heightLoop(0, true, dt);
      c.status = ST_HOLD;
      if (T >= AUTO_SETTLE_S) next(PUSH, nullptr);
      break;

    case PUSH:                                  // lean toward the side
      leanTo = AUTO_TILT_DEG;
      c.throttle = heightLoop(0, true, dt);
      c.status = ST_SQUARE1 + Side;
      if (T >= TPush) next(BRAKE, nullptr);
      break;

    case BRAKE:                                 // lean the other way to stop
      leanTo = -AUTO_TILT_DEG;
      c.throttle = heightLoop(0, true, dt);
      c.status = ST_SQUARE1 + Side;
      if (T >= AUTO_BRAKE_GAIN * (TPush + TRamp)) next(CORNER, nullptr);
      break;

    case CORNER:                                // level, hover at the corner
      c.throttle = heightLoop(0, true, dt);
      c.status = ST_SQUARE1 + Side;
      if (T >= AUTO_CORNER_S) {
        Serial.printf("AUTO: side %d done\n", Side + 1);
        if (++Side >= 4) next(LAND, "AUTO: square done, landing");
        else next(PUSH, nullptr);
      }
      break;

    case LAND:
      c.throttle = heightLoop(-LAND_CMS, false, dt);
      c.status = ST_LANDING;
      if (touchedDown(c.throttle)) {
        InAir = false;
        Serial.printf("AUTO: touched down at %.0f cm, motors off, disarming\n", height.cm());
        c = Cmd(); c.status = ST_LANDED; c.disarm = true;
        Stage = DONE;
        return c;
      }
      break;

    case DONE:
      c.status = ST_LANDED;
      return c;
  }

  // smooth lean toward the side being flown
  Lean += constrain(leanTo - Lean, -AUTO_TILT_RATE * dt, AUTO_TILT_RATE * dt);
  int s = Side < 4 ? Side : 3;
  c.autoRoll  = AUTO_PATH[s][0] * Lean;
  c.autoPitch = AUTO_PATH[s][1] * Lean;
  c.motors = c.pid = true;
  return c;
}
} // namespace Auto

Cmd flightMode(const HoverIt::Values& rx, bool flying, float lastThrottle, float dt) {
  uint8_t m = rx.mode & 3;
  if (m != ModeFlown) {
    Serial.printf("MODE: %s\n", MODE_NAMES[m]);
    InAir = flying && m != MODE_STABILIZE;
    if (InAir) heightStart(lastThrottle);
    if (m == MODE_LAND && InAir) Serial.printf("LAND: descending from %.0f cm\n", height.cm());
    if (m == MODE_AUTO) Auto::begin(InAir);
    ModeFlown = m;
  }

  if (m == MODE_AUTO) return Auto::update(dt);

  Cmd c;
  if (m == MODE_STABILIZE) {
    c.throttle = min(hoverItStickUs(rx.throttle), THROTTLE_MAX_US);
    c.motors = c.throttle > ESC_MIN_US;
    c.pid = c.throttle >= PID_ON_US;
    if (c.pid && height.ok()) c.throttle = min(c.throttle, ceilingThrottle());   // height limit
    c.status = c.motors ? ST_FLYING : ST_IDLE;
    return c;
  }

  if (m == MODE_LOITER) {
    float s = hoverItCentered(rx.throttle), climb = 0;
    if (s >  LOITER_DEADBAND) climb = (s - LOITER_DEADBAND) / (1 - LOITER_DEADBAND) * LOITER_CLIMB;
    if (s < -LOITER_DEADBAND) climb = (s + LOITER_DEADBAND) / (1 - LOITER_DEADBAND) * LOITER_CLIMB;
    if (!InAir) {
      if (climb <= 0) { c.status = ST_IDLE; return c; }       // on the ground until stick > middle
      if (!height.ok()) {                                      // no height reading: nothing to hold
        static uint32_t warnMs = 0;
        if (millis() - warnMs > 2000) { warnMs = millis(); Serial.println("LOITER: no ToF or barometer reading, not taking off"); }
        c.status = ST_IDLE; return c;
      }
      heightStart(HOVER_US - 150);
      InAir = true;
      Serial.println("LOITER: takeoff");
    }
    c.motors = c.pid = true;
    c.throttle = heightLoop(climb, true, dt);
    c.status = climb > 0 ? ST_CLIMB : climb < 0 ? ST_DESCEND : ST_HOLD;
    if (climb < 0 && touchedDown(c.throttle)) {
      InAir = false;
      Serial.println("LOITER: touched down, motors off");
      c = Cmd(); c.status = ST_LANDED;
    } else if (climb >= 0) {
      LowSinceMs = StuckSinceMs = 0;
    }
    return c;
  }

  // MODE_LAND: throttle stick ignored
  if (!InAir) { c.status = ST_LANDED; return c; }
  c.motors = c.pid = true;
  c.throttle = heightLoop(-LAND_CMS, false, dt);
  c.status = ST_LANDING;
  if (touchedDown(c.throttle)) {
    InAir = false;
    Serial.printf("LAND: touched down at %.0f cm, motors off, disarming\n", height.cm());
    c = Cmd(); c.status = ST_LANDED; c.disarm = true;
  }
  return c;
}

// ============================================================================
//  10. SETUP + LOOP
// ============================================================================

uint32_t LoopTimer = 0;
bool  CrashCut = false;
bool  Flying = false;
float Throttle = 1000;
uint8_t StatusNow = ST_IDLE;
float SpRoll = 0, SpPitch = 0, DesRateR = 0, DesRateP = 0, DesRateY = 0;
float PrevSpRoll = 0, PrevSpPitch = 0, StickRateR = 0, StickRateP = 0;   // angle FF
float OutR = 0, OutP = 0, OutY = 0;
uint32_t MaxWorkUs = 0, SumWorkUs = 0, NLoops = 0, LateLoops = 0;



void setup() {
  Motors::setup();                        // first: ESCs see "stop" at once
  Serial.begin(115200);
  delay(500);
  Serial.println("\nSuperMini_Simple: keep the drone still and level ...");
  Imu::setup();
  height.setup();
  Batt::setup();
  Tune::setup();
  hoverItSetup();
  Motors::readySpin();
  Serial.println("SuperMini_Simple: ready (500 Hz, RX_RP_Tune loops)");
  LoopTimer = micros();
}

void loop() {
  // ---- strict 500 Hz, real dt (RX_RP_Tune) --------------------------------
  // The C3 has one core: sleep through most of the wait so Wi-Fi gets time.
  while ((uint32_t)(micros() - LoopTimer) < LOOP_US) {
    if ((uint32_t)(micros() - LoopTimer) < LOOP_US - 1100) delay(1);
  }
  uint32_t now = micros();
  float dt = (now - LoopTimer) / 1000000.0f;
  LoopTimer = now;
  dt = constrain(dt, 0.0005f, 0.008f);    // a stall must not blow up I and D

  // ---- sensors + link -------------------------------------------------------
  Imu::update(dt);
  height.update(dt, HoverIt::Rx.training, Flying);   // Rx.training = the app's Enable Baro switch
  hoverItUpdate();
  Tune::update();
  Batt::update();
  hoverItSetBattery(Batt::level());
  const HoverIt::Values& rx = HoverIt::Rx;

  // ---- crash cut ------------------------------------------------------------
  if (!hoverItArmed()) CrashCut = false;
  if (hoverItArmed() && !CrashCut && (fabsf(Imu::Angle[0]) > CRASH_TILT || fabsf(Imu::Angle[1]) > CRASH_TILT)) {
    CrashCut = true;
    Serial.println("CRASH CUT: tilt too high, motors off until you disarm");
  }

  Cmd c;
  if (hoverItArmed() && !CrashCut) {
    c = flightMode(rx, Flying, Throttle, dt);
    if (c.disarm) hoverItDisarm();
  } else {
    ModeFlown = 0xFF;                     // next arm enters the picked mode fresh
    InAir = false;
  }
  StatusNow = c.status;

  // ---- RX_RP_Tune cascaded PID ---------------------------------------------
  // c.autoRoll / autoPitch: AUTONOMOUS lean (deg), added as if the sticks were pushed
  float stickR = ROLL_SIGN  * (hoverItCentered(rx.roll)  * MAX_ANGLE_DEG + c.autoRoll);
  float stickP = PITCH_SIGN * (hoverItCentered(rx.pitch) * MAX_ANGLE_DEG + c.autoPitch);
  SpRoll  = stickR + TRIM_ROLL_DEG;
  SpPitch = stickP + TRIM_PITCH_DEG;
  float yawSp = YAW_SIGN * hoverItCentered(rx.yaw) * MAX_YAW_RATE;
  // stick speed (deg/s), smoothed: the angle feedforward
  {
    float w = 2.0f * PI * ANGLE_FF_LPF_HZ * dt, a = w / (1.0f + w);
    StickRateR += a * ((SpRoll  - PrevSpRoll)  / dt - StickRateR);
    StickRateP += a * ((SpPitch - PrevSpPitch) / dt - StickRateP);
    PrevSpRoll = SpRoll; PrevSpPitch = SpPitch;
  }
  if (c.motors && c.pid) {
    // angle -> rate: P on the angle error + the stick's speed
    DesRateR = constrain(pidAngleRoll.update(SpRoll,   Imu::Angle[0], dt) + ANGLE_FF * StickRateR, -400.0f, 400.0f);
    DesRateP = constrain(pidAnglePitch.update(SpPitch, Imu::Angle[1], dt) + ANGLE_FF * StickRateP, -400.0f, 400.0f);
    DesRateY = yawSp;
    OutR = pidRateRoll.update(DesRateR,  Imu::Rate[0], dt);         // rate -> us
    OutP = pidRatePitch.update(DesRateP, Imu::Rate[1], dt);
    OutY = pidRateYaw.update(DesRateY,   Imu::Rate[2], dt);
  } else {
    resetAttitudePid();                   // on the ground / low throttle: clear every I
    DesRateR = DesRateP = DesRateY = OutR = OutP = OutY = 0;
  }

  // ---- quad X mixer ----------------------------------------------------------
  // R + = more roll-right wanted: left motors up, right down.
  // P + = more nose-down wanted: rear motors up, front down.
  // Y + = more clockwise yaw wanted: the anticlockwise props (FL, RR) up.
  if (c.motors) {
    Flying = true;
    Throttle = c.throttle;
    int minUs = c.pid ? ESC_IDLE_US : ESC_MIN_US;
    float T = Throttle;
    Motors::write(Motors::FL, constrain((int)(T + OutR - OutP + OutY), minUs, ESC_MAX_US));
    Motors::write(Motors::FR, constrain((int)(T - OutR - OutP - OutY), minUs, ESC_MAX_US));
    Motors::write(Motors::RL, constrain((int)(T + OutR + OutP - OutY), minUs, ESC_MAX_US));
    Motors::write(Motors::RR, constrain((int)(T - OutR + OutP + OutY), minUs, ESC_MAX_US));
  } else {
    Flying = false;
    Throttle = 1000;
    Motors::all(ESC_MIN_US);
  }

  hoverItSetStatus(ModeFlown == 0xFF ? (rx.mode & 3) : ModeFlown,
                   StatusNow, height.cm(), height.usingBaro());   // app shows "baro"

  // ---- loop time ------------------------------------------------------------
  uint32_t work = micros() - now;
  if (work > MaxWorkUs) MaxWorkUs = work;
  SumWorkUs += work; NLoops++;
  if (work > LOOP_US) LateLoops++;

  // ---- Serial Monitor, 5 Hz -------------------------------------------------
  //  ANG  angle / setpoint (deg)     RATE gyro / desired (deg/s)    OUT PID (us)
  //  M    motors FL FR RL RR (us)    vib accel shaking (g), ACC = accel used, GYRO = ignored
  //  H    height / held (cm), climb rate, raw ToF height (-1 = no reading),
  //       baro height (-1 = no baro), TOF / BARO = source in use, limit (cm)
  //  K    roll+pitch rate gains      loop avg/max work time (us, must stay under 2000)
  static uint32_t printMs = 0;
  if (millis() - printMs >= 200) {
    printMs = millis();
    Serial.printf("%s%s %s%s | ANG R %5.1f/%5.1f P %5.1f/%5.1f | RATE R %6.1f/%6.1f P %6.1f/%6.1f Y %6.1f/%6.1f | "
                  "OUT %4.0f %4.0f %4.0f | M %4d %4d %4d %4d | vib %4.2f %s | H %5.1f/%5.1f v %5.1f tof %5.1f baro %5.1f %s max %.0f | "
                  "K %.3f %.3f %.4f | bat %.2fV | loop %u/%u us late %u\n",
                  hoverItConnected() ? "LINK" : "----", hoverItArmed() ? " ARM" : " ---",
                  ModeFlown == 0xFF ? "---" : MODE_NAMES[ModeFlown], CrashCut ? " CUT" : "",
                  Imu::Angle[0], SpRoll, Imu::Angle[1], SpPitch,
                  Imu::Rate[0], DesRateR, Imu::Rate[1], DesRateP, Imu::Rate[2], DesRateY,
                  OutR, OutP, OutY,
                  Motors::Us[0], Motors::Us[1], Motors::Us[2], Motors::Us[3],
                  Imu::AccVib, Imu::AccTrusted ? "ACC" : "GYRO",
                  height.cm(), Setpoint, height.velocity(), height.tofCm(),
                  height.baroCm(), height.usingBaro() ? "BARO" : "TOF", height.maxCm(),
                  pidRateRoll.kp, pidRateRoll.ki, pidRateRoll.kd,
                  Batt::Volts, (unsigned)(NLoops ? SumWorkUs / NLoops : 0), (unsigned)MaxWorkUs, (unsigned)LateLoops);
    MaxWorkUs = SumWorkUs = NLoops = LateLoops = 0;
  }
}
