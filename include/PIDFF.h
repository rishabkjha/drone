// ============================================================================
//  PIDFF.h  -  RX_RP_Tune's PID: P on error, trapezoidal I clamped, D on the
//  measurement (filtered), output clamped. Plus FF on the setpoint (0 unless
//  you raise the FF slider) and an I preload for a bumpless altitude hold.
// ============================================================================
#pragma once
#include "Config.h"

class Pid {
public:
  float kp, ki, kd, ff;
  float outLimit;      // |output| clamp
  float iLimit;        // |I-term| clamp
  float dLpfHz;        // D-term low-pass, 0 = none
  float iterm, prevError, prevMeas, dFilt, out;
  bool  first;

  Pid(float p, float i, float d, float f, float outLim, float iLim, float dHz)
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
