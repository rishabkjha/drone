// ============================================================================
//  AttitudePID.h  -  RX_RP_Tune's cascaded PID.
//    sticks -> angle setpoint (+ flight trim)
//    angle P (ANGLE_KP) + FF on the stick's speed (ANGLE_FF) -> desired rate
//    rate PID on the gyro (gains from the app: ROLL+PITCH, YAW RATE) -> us
//  Global object `attitude`: attitude.OutR / OutP / OutY go to the mixer.
// ============================================================================
#pragma once
#include "Config.h"
#include "PIDFF.h"
#include "IMU.h"
#include "HoverItRx.h"

class Attitude {
public:
  //            kp        ki kd ff out  iLim D filter     (rate gains: Tuner)
  Pid angleRoll  { ANGLE_KP, 0, 0, 0, 400, 400, DTERM_LPF_HZ };    // deg -> deg/s
  Pid anglePitch { ANGLE_KP, 0, 0, 0, 400, 400, DTERM_LPF_HZ };
  Pid rateRoll   { 0,        0, 0, 0, 400, 400, DTERM_LPF_HZ };    // deg/s -> us
  Pid ratePitch  { 0,        0, 0, 0, 400, 400, DTERM_LPF_HZ };
  Pid rateYaw    { 0,        0, 0, 0, 200, 200, DTERM_LPF_HZ };

  float SpRoll = 0, SpPitch = 0;                   // angle setpoints, deg
  float DesRateR = 0, DesRateP = 0, DesRateY = 0;  // desired rates, deg/s
  float OutR = 0, OutP = 0, OutY = 0;              // to the mixer, us

  // every loop. active = motors running with the PIDs on; otherwise every I clears.
  // autoR / autoP: AUTONOMOUS lean (deg), added as if the sticks were pushed.
  void update(const HoverIt::Values& rx, bool active, float dt, float autoR = 0, float autoP = 0) {
    float stickR = ROLL_SIGN  * (hoverItCentered(rx.roll)  * MAX_ANGLE_DEG + autoR);
    float stickP = PITCH_SIGN * (hoverItCentered(rx.pitch) * MAX_ANGLE_DEG + autoP);
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
    if (active) {
      // angle -> rate: P on the angle error + the stick's speed
      DesRateR = constrain(angleRoll.update(SpRoll,   imu.Angle[0], dt) + ANGLE_FF * StickRateR, -400.0f, 400.0f);
      DesRateP = constrain(anglePitch.update(SpPitch, imu.Angle[1], dt) + ANGLE_FF * StickRateP, -400.0f, 400.0f);
      DesRateY = yawSp;
      OutR = rateRoll.update(DesRateR,  imu.Rate[0], dt);         // rate -> us
      OutP = ratePitch.update(DesRateP, imu.Rate[1], dt);
      OutY = rateYaw.update(DesRateY,   imu.Rate[2], dt);
    } else {
      reset();                              // on the ground / low throttle: clear every I
      DesRateR = DesRateP = DesRateY = OutR = OutP = OutY = 0;
    }
  }

  void reset() {
    angleRoll.reset(); anglePitch.reset();
    rateRoll.reset();  ratePitch.reset(); rateYaw.reset();
  }

private:
  float PrevSpRoll = 0, PrevSpPitch = 0, StickRateR = 0, StickRateP = 0;   // angle FF
};

Attitude attitude;
