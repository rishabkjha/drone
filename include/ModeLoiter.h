// ============================================================================
//  ModeLoiter.h  -  LOITER: throttle stick at 50 % = hold height (ToF, up to
//  50 cm with Baro Height off, 3 m with it on), above climbs, below sinks.
//  Takes off from the ground when the stick goes above the middle (only with
//  a height reading: ToF or barometer), switches the
//  motors off when it touches down with the stick below the middle.
// ============================================================================
#pragma once
#include "FlightModeCommon.h"
#include "HeightPID.h"

class ModeLoiter : public FlightMode {
public:
  Cmd update(const HoverIt::Values& rx, bool& inAir, float dt) override {
    Cmd c;
    float s = hoverItCentered(rx.throttle), climb = 0;
    if (s >  LOITER_DEADBAND) climb = (s - LOITER_DEADBAND) / (1 - LOITER_DEADBAND) * LOITER_CLIMB;
    if (s < -LOITER_DEADBAND) climb = (s + LOITER_DEADBAND) / (1 - LOITER_DEADBAND) * LOITER_CLIMB;
    if (!inAir) {
      if (climb <= 0) { c.status = ST_IDLE; return c; }       // on the ground until stick > middle
      if (!height.ok()) {                                       // no height reading: nothing to hold
        if (millis() - WarnMs > 2000) { WarnMs = millis(); Serial.println("LOITER: no ToF or barometer reading, not taking off"); }
        c.status = ST_IDLE; return c;
      }
      heightHold.start(HOVER_US - 150);
      inAir = true;
      Serial.println("LOITER: takeoff");
    }
    c.motors = c.pid = true;
    c.throttle = heightHold.update(climb, true, dt);
    c.status = climb > 0 ? ST_CLIMB : climb < 0 ? ST_DESCEND : ST_HOLD;
    if (climb < 0 && heightHold.touchedDown(c.throttle)) {
      inAir = false;
      Serial.println("LOITER: touched down, motors off");
      c = Cmd(); c.status = ST_LANDED;
    } else if (climb >= 0) {
      heightHold.clearTouchdown();
    }
    return c;
  }

private:
  uint32_t WarnMs = 0;
};
