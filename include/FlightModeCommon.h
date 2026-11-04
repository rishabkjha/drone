// ============================================================================
//  FlightModeCommon.h  -  flight mode numbers, status list and the command a
//  mode hands to the main loop. Numbers match the app's DroneLink.kt.
// ============================================================================
#pragma once
#include <Arduino.h>

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

#include "HoverItRx.h"

// Every flight mode is a class with this interface (ModeStabilize.h,
// ModeLoiter.h, ModeLand.h, ModeAuto.h). FlightModes.h picks one each loop.
// inAir: motors running under the height loops (LOITER / LAND), shared
// between modes so switching mode in the air carries on from where it was.
class FlightMode {
public:
  virtual Cmd update(const HoverIt::Values& rx, bool& inAir, float dt) = 0;
  virtual ~FlightMode() {}
};
