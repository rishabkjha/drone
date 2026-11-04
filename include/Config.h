// ============================================================================
//  Config.h  -  every setting you are likely to change, in one place.
//  (FlightController v2: the SuperMini_Simple behaviour, split into classes)
// ============================================================================
#pragma once
#include <Arduino.h>

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

// ---- Height (LOITER, LAND): VL53L0X ToF only (no barometer)
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

