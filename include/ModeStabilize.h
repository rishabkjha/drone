// ============================================================================
//  ModeStabilize.h  -  STABILIZE: throttle stick 0-255 = 1000-2000 us (capped
//  at THROTTLE_MAX_US), sticks set the angle. PIDs on from PID_ON_US.
//  Height limit: near height.maxCm() (50 cm with Baro Height off) the
//  throttle is capped so the drone stops there however high the stick is.
// ============================================================================
#pragma once
#include "FlightModeCommon.h"
#include "HeightPID.h"

class ModeStabilize : public FlightMode {
public:
  Cmd update(const HoverIt::Values& rx, bool& inAir, float dt) override {
    (void)inAir; (void)dt;
    Cmd c;
    c.throttle = min(hoverItStickUs(rx.throttle), THROTTLE_MAX_US);
    c.motors = c.throttle > ESC_MIN_US;
    c.pid = c.throttle >= PID_ON_US;
    if (c.pid && height.ok()) c.throttle = min(c.throttle, HeightHold::ceilingThrottle());
    c.status = c.motors ? ST_FLYING : ST_IDLE;
    return c;
  }
};
