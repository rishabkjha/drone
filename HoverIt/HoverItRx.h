// ============================================================================
//  HoverItRx.h  -  Wi-Fi receiver for the HoverIt phone app (ESP32 / SuperMini)
//  The drone joins your phone's hotspot and talks to the app over UDP.
//  Replaces the RFM69 radio: you get sticks, PID sliders, arm and
//  autonomous bits from the phone, and send the battery level back.
//
//  Wi-Fi modes (HOVERIT_SOFTAP below):
//   0 = drone joins the phone hotspot (default). Set HOTSPOT_SSID / _PASS.
//   1 = drone makes its own network "HoverIt" (pass hoverit123); connect the
//       phone to it. Use this if your phone can't run a hotspot.
//  Either way the app finds the drone by itself: the drone broadcasts its
//  telemetry until the phone answers, then both talk directly.
//
//  What the phone sends (50 Hz). All 8-bit values are 0-255:
//    throttle   left stick Y, 0 = bottom, holds where you leave it
//    yaw        left stick X, 128 = centre
//    pitch      right stick Y, 128 = centre, 255 = stick up (forward)
//    roll       right stick X, 128 = centre, 255 = right
//    kp ki kd rr  the four Tuning sliders, 0 at the bottom
//    armed      1 bit, arm button
//    autonomous 1 bit, Options > Autonomous Mode
//    tuning     1 bit, true while the Tuning screen is open
//  What the drone sends back (10 Hz): battery 0-255 (fills the Options button,
//  127 = half), and whether it is really armed.
//
//  !! SAFETY !!
//   - FAILSAFE: no packet for FAILSAFE_MS (500 ms) -> disarmed, sticks centred,
//     throttle 0. The app also disarms itself when it loses the drone (1 s).
//   - Arming is refused unless throttle <= ARM_MAX_THROTTLE. The arm bit has
//     to go 0 -> 1 with throttle low; staying 1 never re-arms after a failsafe.
//   - Phone Wi-Fi is not a race radio: expect 5-50 ms latency and the odd
//     dropout. First flights tethered, in the open, away from crowded Wi-Fi.
//
//  Public API:
//    hoverItSetup()              once in setup(), after Serial.begin()
//    hoverItUpdate()             every loop; never blocks
//    hoverItConnected()          true while packets arrive
//    hoverItArmed()              true when armed (failsafe and arm guard applied)
//    hoverItSetBattery(b)        0-255 battery level to show in the app
//    hoverItSetBatteryVolts(v, vEmpty, vFull)   same, from a voltage
//    hoverItStickUs(b)           0-255 -> 1000-2000 us
//    hoverItCentered(b)          0-255 -> -1.0 .. +1.0 (128 = 0)
//    hoverItScale(b, lo, hi)     0-255 -> lo .. hi (for the PID sliders)
//    HoverIt::Rx.throttle, .yaw, .pitch, .roll, .kp, .ki, .kd, .rr,
//    HoverIt::Rx.autonomous, .tuning      the latest values (failsafe applied)
//
//  Serial Monitor (115200): Wi-Fi status, the phone's IP when it is found,
//  link lost / regained, arm refused.
// ============================================================================
#pragma once

#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef HOVERIT_SOFTAP
#define HOVERIT_SOFTAP 0                       // 0 = join phone hotspot, 1 = own network
#endif

namespace HoverIt {

// ---------------- Settings ---------------------------------------------------
const char* const HOTSPOT_SSID = "MyPhoneHotspot";   // !! your phone hotspot name !!
const char* const HOTSPOT_PASS = "password123";      // !! and its password !!
const char* const AP_SSID      = "HoverIt";          // used when HOVERIT_SOFTAP = 1
const char* const AP_PASS      = "hoverit123";       // at least 8 characters

const uint16_t DRONE_PORT       = 4210;    // drone listens here (must match the app)
const uint16_t PHONE_PORT       = 4211;    // app listens here   (must match the app)
const uint32_t FAILSAFE_MS      = 500;     // no packet this long: disarm + centre
const uint32_t TELEMETRY_MS     = 100;     // battery to the phone at 10 Hz
const uint8_t  ARM_MAX_THROTTLE = 15;      // throttle must be at/below this to arm

// ---------------- Packets (keep in sync with DroneLink.kt) ------------------
const uint8_t PROTOCOL_VERSION = 1;

struct __attribute__((packed)) CmdPacket {   // phone -> drone, 13 bytes
  uint8_t magic;                             // 'H'
  uint8_t seq;
  uint8_t throttle, yaw, pitch, roll;
  uint8_t kp, ki, kd, rr;
  uint8_t flags;                             // bit0 armed, bit1 autonomous, bit2 tuning
  uint8_t version;
  uint8_t check;                             // XOR of all bytes before it
};

struct __attribute__((packed)) TelemPacket { // drone -> phone, 5 bytes
  uint8_t magic;                             // 'D'
  uint8_t seq;
  uint8_t battery;                           // 0-255
  uint8_t flags;                             // bit0 armed
  uint8_t check;
};

// ---------------- State ------------------------------------------------------
struct Values {
  uint8_t throttle = 0, yaw = 128, pitch = 128, roll = 128;
  uint8_t kp = 0, ki = 0, kd = 0, rr = 0;
  bool armed = false, autonomous = false, tuning = false;
};
Values Rx;                    // what your flight code reads

WiFiUDP Udp;
IPAddress PhoneIp;
bool HavePhone = false;
bool Linked = false;
bool LastArmBit = false;      // for the 0 -> 1 arm edge
uint8_t Battery = 0;
uint8_t TxSeq = 0;
uint32_t LastPacketMs = 0, LastTelemMs = 0, LastWifiLogMs = 0;
uint32_t Packets = 0, BadPackets = 0;

inline uint8_t xorBytes(const uint8_t* p, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

inline void failsafe() {
  Rx.throttle = 0;
  Rx.yaw = Rx.pitch = Rx.roll = 128;
  Rx.armed = false;
  Rx.autonomous = false;
  // kp..rr keep their last values so a dropout doesn't zero your gains
}

inline IPAddress broadcastIp() {
#if HOVERIT_SOFTAP
  return IPAddress(192, 168, 4, 255);
#else
  if (WiFi.status() != WL_CONNECTED) return IPAddress(255, 255, 255, 255);
  IPAddress ip = WiFi.localIP(), mask = WiFi.subnetMask();
  return IPAddress(ip[0] | ~mask[0], ip[1] | ~mask[1], ip[2] | ~mask[2], ip[3] | ~mask[3]);
#endif
}

inline void handlePacket(const CmdPacket& c, IPAddress from) {
  if (!HavePhone || from != PhoneIp) {
    PhoneIp = from;
    HavePhone = true;
    Serial.print("[HoverIt] phone at ");
    Serial.println(PhoneIp);
  }
  if (!Linked) Serial.println("[HoverIt] link up");
  Linked = true;
  LastPacketMs = millis();
  Packets++;

  Rx.throttle = c.throttle;
  Rx.yaw = c.yaw;
  Rx.pitch = c.pitch;
  Rx.roll = c.roll;
  Rx.kp = c.kp;
  Rx.ki = c.ki;
  Rx.kd = c.kd;
  Rx.rr = c.rr;
  Rx.autonomous = c.flags & 0x02;
  Rx.tuning = c.flags & 0x04;

  bool armBit = c.flags & 0x01;
  if (!armBit) {
    Rx.armed = false;
  } else if (!LastArmBit && !Rx.armed) {            // 0 -> 1: arm request
    if (c.throttle <= ARM_MAX_THROTTLE) Rx.armed = true;
    else Serial.println("[HoverIt] arm refused: throttle not at the bottom");
  }
  LastArmBit = armBit;
}

inline void sendTelemetry() {
  TelemPacket t;
  t.magic = 'D';
  t.seq = TxSeq++;
  t.battery = Battery;
  t.flags = Rx.armed ? 0x01 : 0x00;
  t.check = xorBytes((const uint8_t*)&t, sizeof(t) - 1);
  // before the phone is known (or after losing it) broadcast, so the app can find us
  Udp.beginPacket(HavePhone && Linked ? PhoneIp : broadcastIp(), PHONE_PORT);
  Udp.write((const uint8_t*)&t, sizeof(t));
  Udp.endPacket();
}

} // namespace HoverIt

// ---------------- Public API -------------------------------------------------
inline void hoverItSetup() {
  using namespace HoverIt;
  failsafe();
  WiFi.persistent(false);
#if HOVERIT_SOFTAP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[HoverIt] own network \"");
  Serial.print(AP_SSID);
  Serial.print("\" at ");
  Serial.println(WiFi.softAPIP());
#else
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(HOTSPOT_SSID, HOTSPOT_PASS);
  Serial.print("[HoverIt] joining hotspot \"");
  Serial.print(HOTSPOT_SSID);
  Serial.println("\" ...");
#endif
  WiFi.setSleep(false);                  // modem sleep adds 100+ ms of lag
  Udp.begin(DRONE_PORT);
}

inline void hoverItUpdate() {
  using namespace HoverIt;
  uint32_t now = millis();

  // read everything that arrived since the last loop, keep the newest
  int size;
  while ((size = Udp.parsePacket()) > 0) {
    CmdPacket c;
    if (size != sizeof(CmdPacket)) {   // next parsePacket() drops the rest
      BadPackets++;
      continue;
    }
    Udp.read((uint8_t*)&c, sizeof(c));
    if (c.magic != 'H' || c.version != PROTOCOL_VERSION ||
        c.check != xorBytes((const uint8_t*)&c, sizeof(c) - 1)) {
      BadPackets++;
      continue;
    }
    handlePacket(c, Udp.remoteIP());
  }

  if (Linked && now - LastPacketMs > FAILSAFE_MS) {
    Linked = false;
    LastArmBit = false;
    failsafe();
    Serial.println("[HoverIt] link lost -> failsafe (disarmed)");
  }

  if (now - LastTelemMs >= TELEMETRY_MS) {
    LastTelemMs = now;
    sendTelemetry();
  }

#if !HOVERIT_SOFTAP
  if (now - LastWifiLogMs > 3000) {
    LastWifiLogMs = now;
    static bool wasUp = false;
    bool up = WiFi.status() == WL_CONNECTED;
    if (up != wasUp) {
      wasUp = up;
      if (up) { Serial.print("[HoverIt] on hotspot, drone IP "); Serial.println(WiFi.localIP()); }
      else Serial.println("[HoverIt] hotspot not reachable, retrying ...");
    }
  }
#endif
}

inline bool hoverItConnected() { return HoverIt::Linked; }
inline bool hoverItArmed()     { return HoverIt::Rx.armed; }

inline void hoverItSetBattery(uint8_t level) { HoverIt::Battery = level; }

inline void hoverItSetBatteryVolts(float v, float vEmpty, float vFull) {
  float f = (v - vEmpty) / (vFull - vEmpty);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  HoverIt::Battery = (uint8_t)(f * 255.0f + 0.5f);
}

inline int   hoverItStickUs(uint8_t b)  { return 1000 + (int)((b * 1000L + 127) / 255); }
inline float hoverItCentered(uint8_t b) { return ((int)b - 128) / (b >= 128 ? 127.0f : 128.0f); }
inline float hoverItScale(uint8_t b, float lo, float hi) { return lo + (hi - lo) * (b / 255.0f); }

// ============================================================================
//  Example: HoverIt app as the radio in your flight controller sketch
// ============================================================================
//
//  // #define HOVERIT_SOFTAP 1     // uncomment to make the drone its own network
//  #include "HoverItRx.h"
//  #include "IMU.h"
//  #include "Motors.h"
//
//  const int BATT_PIN = 3;         // battery through a divider, e.g. 1S: 100k/100k
//  const float DIVIDER = 2.0f;     // volts at battery / volts at pin
//
//  uint32_t LoopTimer;
//
//  void setup() {
//    Serial.begin(115200);
//    motorsSetup();
//    imuSetup();
//    hoverItSetup();
//    analogReadResolution(12);
//    LoopTimer = micros();
//  }
//
//  void loop() {
//    imuUpdate();
//    hoverItUpdate();
//
//    // battery -> app (fills the Options button like water)
//    float vBatt = analogReadMilliVolts(BATT_PIN) / 1000.0f * DIVIDER;
//    hoverItSetBatteryVolts(vBatt, 3.3f, 4.2f);     // 1S LiPo empty / full
//
//    // PID gains from the Tuning sliders (same ranges as TX_RP_Tune.ino)
//    float Kp = hoverItScale(HoverIt::Rx.kp, 0.0f, 2.0f);
//    float Ki = hoverItScale(HoverIt::Rx.ki, 0.0f, 8.0f);
//    float Kd = hoverItScale(HoverIt::Rx.kd, 0.0f, 0.08f);
//    float rr = hoverItScale(HoverIt::Rx.rr, 0.0f, 1.0f);   // RR: use as you like
//
//    if (!hoverItArmed()) {
//      motorsOff();                // also covers link loss (failsafe)
//    } else if (HoverIt::Rx.autonomous) {
//      // your autonomous code, e.g. TrainFlight.h
//    } else {
//      int   throttle     = hoverItStickUs(HoverIt::Rx.throttle);       // 1000-2000 us
//      float desiredRoll  = hoverItCentered(HoverIt::Rx.roll)  * 30.0f; // deg
//      float desiredPitch = hoverItCentered(HoverIt::Rx.pitch) * 30.0f; // deg
//      float desiredYawRate = hoverItCentered(HoverIt::Rx.yaw) * 90.0f; // deg/s
//      // your attitude PIDs with Kp/Ki/Kd -> InputRoll, InputPitch, InputYaw
//      // then the quad X mixer:
//      // setMotors(throttle - InputRoll - InputPitch - InputYaw, ...);
//    }
//
//    static uint32_t printMs = 0;
//    if (millis() - printMs > 250) {
//      printMs = millis();
//      Serial.printf("link:%d arm:%d T:%3u Y:%3u P:%3u R:%3u | kp:%3u ki:%3u kd:%3u rr:%3u | auto:%d\n",
//                    hoverItConnected(), hoverItArmed(),
//                    HoverIt::Rx.throttle, HoverIt::Rx.yaw, HoverIt::Rx.pitch, HoverIt::Rx.roll,
//                    HoverIt::Rx.kp, HoverIt::Rx.ki, HoverIt::Rx.kd, HoverIt::Rx.rr,
//                    HoverIt::Rx.autonomous);
//    }
//
//    while (micros() - LoopTimer < 4000);        // 250 Hz
//    LoopTimer = micros();
//  }
