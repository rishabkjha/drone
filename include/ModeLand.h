// ============================================================================
//  ModeLand.h  -  LAND: LAND_CMS down to the takeoff height, throttle stick
//  ignored, then the motors stop and the drone disarms itself.
// ============================================================================
#pragma once
#include "FlightModeCommon.h"
#include "HeightPID.h"

class ModeLand : public FlightMode {
public:
  Cmd update(const HoverIt::Values& rx, bool& inAir, float dt) override {
    (void)rx;
    Cmd c;
    if (!inAir) { c.status = ST_LANDED; return c; }
    c.motors = c.pid = true;
    c.throttle = heightHold.update(-LAND_CMS, false, dt);
    c.status = ST_LANDING;
    if (heightHold.touchedDown(c.throttle)) {
      inAir = false;
      Serial.printf("LAND: touched down at %.0f cm, motors off, disarming\n", height.cm());
      c = Cmd(); c.status = ST_LANDED; c.disarm = true;
    }
    return c;
  }
};
