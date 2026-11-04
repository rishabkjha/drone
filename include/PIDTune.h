// ============================================================================
//  PIDTune.h  -  the app's Tuning sliders. 3 loops, same order as PID_LOOPS in
//  the app's DroneLink.kt: gain = byte / 255 * MAX.
//    ROLL+PITCH  rate loop for roll and pitch (one set of gains)
//    YAW RATE    yaw rate loop
//    VERT VEL    climb-rate loop for LOITER / LAND
//  The drone is the master copy: nothing changes until the app's sliders match
//  the drone's bytes once (the app loads them from telemetry), then every move
//  applies. Closing Tuning while DISARMED saves them to flash.
//  Global object `tuner`.
// ============================================================================
#pragma once
#include <Preferences.h>
#include "Config.h"
#include "HoverItRx.h"
#include "AttitudePID.h"
#include "HeightPID.h"

class Tuner {
public:
  enum { RP_RATE, YAW_RATE, VERT_VEL, LOOPS };
  uint8_t Bytes[LOOPS][4];

  void setup() {
    fromGains(RP_RATE, DEFAULT_RP);
    fromGains(YAW_RATE, DEFAULT_YAW);
    fromGains(VERT_VEL, DEFAULT_VERT);
    if (LOAD_SAVED_GAINS) {
      Prefs.begin("simpletune", false);     // read-write: creates the slot on first boot
      size_t len = Prefs.getBytesLength("g");
      if (len == sizeof(Bytes)) {
        Prefs.getBytes("g", Bytes, sizeof(Bytes));
        Serial.println("[Tune] gains loaded from flash");
      } else if (len == ANGLE_BUILD_LOOPS * 4) {  // keep its first 3 rows, angle loops are fixed
        uint8_t all[ANGLE_BUILD_LOOPS][4];
        Prefs.getBytes("g", all, sizeof(all));
        memcpy(Bytes, all, sizeof(Bytes));
        Serial.println("[Tune] rate + climb gains loaded from flash");
      } else {
        Serial.println("[Tune] first boot: RX_RP_Tune gains");
      }
      Prefs.end();
    }
    for (int l = 0; l < LOOPS; l++) apply(l);
    print();
  }

  void update() {
    const HoverIt::Values& rx = HoverIt::Rx;
    if (hoverItConnected() && rx.loop < LOOPS && rx.loop != Selected) { Selected = rx.loop; Synced = false; }
    bool tuning = hoverItConnected() && rx.tuning;
    if (!tuning) {
      Synced = false;
      if (WasTuning) Serial.println("[Tune] tuning closed");
      WasTuning = false;
      if (Dirty && !hoverItArmed()) save();  // flash writes stall the loop: never while armed
    } else {
      if (!WasTuning) Serial.printf("[Tune] tuning %s\n", NAMES[Selected]);
      WasTuning = true;
      uint8_t in[4] = { rx.kp, rx.ki, rx.kd, rx.ff };
      bool same = memcmp(in, Bytes[Selected], 4) == 0;
      if (!Synced) { if (same) Synced = true; }
      else if (!same) { memcpy(Bytes[Selected], in, 4); apply(Selected); Dirty = true; }
    }
    hoverItSetGains(Selected, Bytes[Selected][0], Bytes[Selected][1], Bytes[Selected][2], Bytes[Selected][3]);
  }

private:
  const char* const NAMES[LOOPS] = { "RP_RATE", "YAW_RATE", "VERT_VEL" };
  const char* const DEFAULT_NAMES[LOOPS] = { "DEFAULT_RP  ", "DEFAULT_YAW ", "DEFAULT_VERT" };
  const Gains MAX[LOOPS] = {
    //  kp     ki     kd     ff
    { 2.0f,  8.0f, 0.08f, 1.0f },   // RP_RATE   us per deg/s
    { 5.0f, 25.0f, 0.10f, 2.0f },   // YAW_RATE  us per deg/s
    { 5.0f,  5.0f, 0.50f, 3.0f },   // VERT_VEL  us per cm/s
  };
  static const int ANGLE_BUILD_LOOPS = 5;  // a build that also tuned the angle loops saved 5 rows
  uint8_t Selected = RP_RATE;
  bool Synced = false, WasTuning = false, Dirty = false;
  Preferences Prefs;

  static uint8_t toByte(float v, float max) {
    float b = max > 0 ? v / max * 255.0f + 0.5f : 0;
    return b < 0 ? 0 : b > 255 ? 255 : (uint8_t)b;
  }
  static float toGain(uint8_t b, float max) { return b * max / 255.0f; }
  Gains gains(int l) const {
    return { toGain(Bytes[l][0], MAX[l].kp), toGain(Bytes[l][1], MAX[l].ki),
             toGain(Bytes[l][2], MAX[l].kd), toGain(Bytes[l][3], MAX[l].ff) };
  }
  void fromGains(int l, const Gains& g) {
    Bytes[l][0] = toByte(g.kp, MAX[l].kp); Bytes[l][1] = toByte(g.ki, MAX[l].ki);
    Bytes[l][2] = toByte(g.kd, MAX[l].kd); Bytes[l][3] = toByte(g.ff, MAX[l].ff);
  }
  void apply(int l) {
    Gains g = gains(l);
    if (l == RP_RATE)  { attitude.rateRoll.set(g); attitude.ratePitch.set(g); }
    if (l == YAW_RATE) attitude.rateYaw.set(g);
    if (l == VERT_VEL) heightHold.climb.set(g);
  }
  void print() {
    for (int l = 0; l < LOOPS; l++) {
      Gains g = gains(l);
      Serial.printf("  %-9s kp %.4f  ki %.4f  kd %.4f  ff %.4f\n", NAMES[l], g.kp, g.ki, g.kd, g.ff);
    }
    Serial.printf("  fixed     angle kp %.2f ff %.2f | height kp %.2f ff %.2f | hover %d us\n",
                  ANGLE_KP, ANGLE_FF, HEIGHT_KP, HEIGHT_FF, HOVER_US);
  }
  void save() {
    Prefs.begin("simpletune", false);
    Prefs.putBytes("g", Bytes, sizeof(Bytes));
    Prefs.end();
    Dirty = false;
    Serial.println("[Tune] gains saved to flash. To make them the defaults, copy into Config.h:");
    for (int l = 0; l < LOOPS; l++) {
      Gains g = gains(l);
      Serial.printf("  %s = { %.4ff, %.4ff, %.4ff, %.4ff }\n", DEFAULT_NAMES[l], g.kp, g.ki, g.kd, g.ff);
    }
  }
};

Tuner tuner;
