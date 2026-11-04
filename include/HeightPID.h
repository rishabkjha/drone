// ============================================================================
//  HeightPID.h  -  altitude hold for LOITER and LAND:
//    held height -> climb rate (HEIGHT_KP, HEIGHT_FF) -> VERT VEL PID -> throttle
//  Global object `heightHold`.
// ============================================================================
#pragma once
#include "Config.h"
#include "PIDFF.h"
#include "IMU.h"
#include "HeightFusion.h"

class HeightHold {
public:
  Pid   climb { 0, 0, 0, 0, 300, 250, 10.0f };     // cm/s -> us (gains: Tuner, VERT VEL)
  float Setpoint = 0;                               // cm, held height

  // climb rate (cm/s) -> throttle (us); hold = true also runs the height loop
  float update(float climbCms, bool hold, float dt) {
    float h = height.cm(), v = height.velocity(), desired;
    if (hold) {
      float top = height.maxCm();
      if ((climbCms > 0 && Setpoint >= top) || (climbCms < 0 && Setpoint <= 0)) climbCms = 0;   // at the limit
      Setpoint += constrain(climbCms, -MAX_CLIMB_CMS, MAX_CLIMB_CMS) * dt;
      Setpoint = constrain(Setpoint, 0.0f, top);
      Setpoint = constrain(Setpoint, h - LEASH_CM, h + LEASH_CM);
      desired = HEIGHT_KP * (Setpoint - h) + HEIGHT_FF * climbCms;
    } else {
      Setpoint = h;
      desired = climbCms;
    }
    desired = constrain(desired, -MAX_CLIMB_CMS, MAX_CLIMB_CMS);
    float thr = HOVER_US + climb.update(desired, v, dt);
    thr = 1000.0f + (thr - 1000.0f) / tiltFactor();
    return constrain(thr, (float)HOLD_MIN_US, (float)HOLD_MAX_US);
  }

  // start holding from the throttle the motors are at (no jump)
  void start(float throttleUs) {
    climb.reset();
    Setpoint = height.cm();
    float level = 1000.0f + (throttleUs - 1000.0f) * tiltFactor();
    climb.preloadI(level - HOVER_US);
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
  void clearTouchdown() { LowSinceMs = StuckSinceMs = 0; }

  // STABILIZE height limit: the most throttle allowed so it stops at height.maxCm()
  static float ceilingThrottle() {
    float room = height.maxCm() - height.cm();                 // cm left below the limit
    float vMax = room > 0 ? sqrtf(2.0f * CEIL_DECEL * room) : CEIL_KP * room;   // cm/s
    float thr = HOVER_US + CEIL_KV * (vMax - height.velocity());
    thr = max(thr, HOVER_US - CEIL_MAX_DROP);
    return 1000.0f + (thr - 1000.0f) / tiltFactor();
  }

private:
  uint32_t LowSinceMs = 0, StuckSinceMs = 0;
  static float tiltFactor() {
    float r = min(fabsf(imu.Angle[0]), 35.0f), p = min(fabsf(imu.Angle[1]), 35.0f);
    return cosf(r * DEG_TO_RAD) * cosf(p * DEG_TO_RAD);
  }
};

HeightHold heightHold;
