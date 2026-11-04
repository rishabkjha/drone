// ============================================================================
//  FlightModes.h  -  runs the mode picked in the app's Options.
//  AUTONOMOUS (ModeAuto.h) flies a square and lands by itself.
//  Picking another mode in the air switches at once, from the current throttle.
//  Global object `flightModes`.
// ============================================================================
#pragma once
#include "FlightModeCommon.h"
#include "HeightPID.h"
#include "ModeStabilize.h"
#include "ModeLoiter.h"
#include "ModeLand.h"
#include "ModeAuto.h"

class FlightModes {
public:
  // armed: run the mode. flying / lastThrottle: what the motors did last loop.
  Cmd update(const HoverIt::Values& rx, bool flying, float lastThrottle, float dt) {
    uint8_t m = rx.mode & 3;
    if (m != ModeFlown) {
      Serial.printf("MODE: %s\n", MODE_NAMES[m]);
      InAir = flying && m != MODE_STABILIZE;
      if (InAir) heightHold.start(lastThrottle);
      if (m == MODE_LAND && InAir) Serial.printf("LAND: descending from %.0f cm\n", height.cm());
      if (m == MODE_AUTO) autonomous.begin(InAir);
      ModeFlown = m;
    }
    return modes[m]->update(rx, InAir, dt);
  }

  // disarmed / crash cut: the next arm enters the picked mode fresh
  void reset() { ModeFlown = 0xFF; InAir = false; }

  bool active() const { return ModeFlown != 0xFF; }
  // mode number for the app: the one flown, or the one picked while disarmed
  uint8_t shown(const HoverIt::Values& rx) const {
    return ModeFlown == 0xFF ? (rx.mode & 3) : ModeFlown;
  }
  const char* name() const { return ModeFlown == 0xFF ? "---" : MODE_NAMES[ModeFlown]; }

private:
  ModeStabilize stabilize;
  ModeLoiter    loiter;
  ModeLand      land;
  ModeAuto      autonomous;
  FlightMode* const modes[4] = { &stabilize, &loiter, &land, &autonomous };
  uint8_t ModeFlown = 0xFF;
  bool    InAir = false;
};

FlightModes flightModes;