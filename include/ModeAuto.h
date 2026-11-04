// ============================================================================
//  ModeAuto.h  -  AUTONOMOUS: arm in the app and the drone does it all:
//    spool up -> climb to AUTO_HEIGHT_CM -> hover -> fly a square of
//    AUTO_SIDE_CM (forward, right, back, left) -> land -> disarm.
//  Height is held exactly like LOITER (same VERT VEL gains, HOVER_US, limit).
//  There is no position sensor, so each side is flown by time: lean
//  AUTO_TILT_DEG toward the side, lean back the same amount to stop, then
//  hover at the corner. The lean is added on top of the trim, so a well
//  trimmed drone draws a square; a badly trimmed one draws a drifting one.
//  To stop it: pick LOITER (holds height where it is) or LAND, or disarm.
// ============================================================================
#pragma once
#include "FlightModeCommon.h"
#include "HeightPID.h"

class ModeAuto : public FlightMode {
public:
  // FlightModes calls this when AUTONOMOUS is picked (inAir: already flying)
  void begin(bool inAir) {
    Stage = inAir ? CLIMB : SPOOL;
    T = 0; Side = 0; Lean = 0;
    // push time per side: lean a = g*tan(tilt) for tPush, then the other way
    // for tPush + ramp, so it stops. Distance ~ a * tPush * (tPush + ramp).
    float a = 981.0f * tanf(AUTO_TILT_DEG * DEG_TO_RAD);          // cm/s^2
    float ramp = AUTO_TILT_DEG / AUTO_TILT_RATE;                    // s
    TRamp = ramp;
    TPush = AUTO_SIDE_GAIN * 0.5f * (-ramp + sqrtf(ramp * ramp + 4.0f * AUTO_SIDE_CM / a));
    Serial.printf("AUTO: %.0f cm square at %.0f cm, lean %.1f deg for %.2f s per side\n",
                  AUTO_SIDE_CM, target(), AUTO_TILT_DEG, TPush);
  }

  Cmd update(const HoverIt::Values& rx, bool& inAir, float dt) override {
    (void)rx;                                     // the app locks the sticks in this mode
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
          heightHold.start(HOVER_US - 50);          // just under hover: lifts off gently
          inAir = true;
          next(CLIMB, "AUTO: takeoff");
        }
        return c;

      case CLIMB: {                               // to the square's height (or down to it)
        float tgt = target(), err = tgt - height.cm();
        float climb = fabsf(err) > 5.0f ? (err > 0 ? AUTO_CLIMB_CMS : -AUTO_CLIMB_CMS) : 0;
        c.throttle = heightHold.update(climb, true, dt);
        // the held height runs ahead of the drone but never past the target
        if ((err > 0 && heightHold.Setpoint > tgt) || (err < 0 && heightHold.Setpoint < tgt)) heightHold.Setpoint = tgt;
        c.status = ST_AUTO_CLIMB;
        if (climb == 0) { heightHold.Setpoint = tgt; next(SETTLE, "AUTO: at height"); }
        break;
      }

      case SETTLE:                                // hover still before the first side
        c.throttle = heightHold.update(0, true, dt);
        c.status = ST_HOLD;
        if (T >= AUTO_SETTLE_S) next(PUSH, nullptr);
        break;

      case PUSH:                                  // lean toward the side
        leanTo = AUTO_TILT_DEG;
        c.throttle = heightHold.update(0, true, dt);
        c.status = ST_SQUARE1 + Side;
        if (T >= TPush) next(BRAKE, nullptr);
        break;

      case BRAKE:                                 // lean the other way to stop
        leanTo = -AUTO_TILT_DEG;
        c.throttle = heightHold.update(0, true, dt);
        c.status = ST_SQUARE1 + Side;
        if (T >= AUTO_BRAKE_GAIN * (TPush + TRamp)) next(CORNER, nullptr);
        break;

      case CORNER:                                // level, hover at the corner
        c.throttle = heightHold.update(0, true, dt);
        c.status = ST_SQUARE1 + Side;
        if (T >= AUTO_CORNER_S) {
          Serial.printf("AUTO: side %d done\n", Side + 1);
          if (++Side >= 4) next(LAND, "AUTO: square done, landing");
          else next(PUSH, nullptr);
        }
        break;

      case LAND:
        c.throttle = heightHold.update(-LAND_CMS, false, dt);
        c.status = ST_LANDING;
        if (heightHold.touchedDown(c.throttle)) {
          inAir = false;
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

private:
  enum Stage_t : uint8_t { SPOOL, CLIMB, SETTLE, PUSH, BRAKE, CORNER, LAND, DONE };
  Stage_t Stage = SPOOL;
  float T = 0, Lean = 0, TPush = 0.5f, TRamp = 0.1f;
  int Side = 0;
  uint32_t WarnMs = 0;

  static float target() { return min(AUTO_HEIGHT_CM, height.maxCm()); }
  void next(Stage_t s, const char* msg) { Stage = s; T = 0; if (msg) Serial.println(msg); }
};