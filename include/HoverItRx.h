// ============================================================================
//  HoverItRx.h  -  Wi-Fi receiver for the HoverIt phone app (ESP32 / SuperMini)
//  The drone joins your phone's hotspot and talks to the app over UDP.
//  Replaces the RFM69 radio: you get sticks, PID sliders, arm, flight mode
//  and baro-training bits from the phone, and send battery, mode and height back.
//
//  Wi-Fi modes (HOVERIT_SOFTAP below):
//   0 = drone joins the phone hotspot (default). Set WIFI_SSID / WIFI_PASS in Config.h.
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
//    kp ki kd ff  the four Tuning sliders, 0 at the bottom
//    loop       which PID loop the sliders are for (0-2, see PIDTune.h)
//    armed      1 bit, arm button
//    training   1 bit, Options > Baro Training (TrainFlight.h)
//    tuning     1 bit, true while the Tuning sliders are enabled
//    mode       2 bits, flight mode picked in Options:
//               0 Stabilize, 1 Loiter, 2 Land, 3 Autonomous (FlightModes.h)
//  What the drone sends back (10 Hz): battery 0-255 (fills the Options button,
//  127 = half), whether it is really armed, the mode it is flying and what it
//  is doing in it (status, see FlightModeCommon.h), its height, and the
//  selected loop's gains (loop, kp, ki, kd, ff bytes) so the app's sliders
//  show what the drone uses.
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
//    hoverItDisarm()             the drone disarms itself (after an automatic
//                                landing); the app follows and needs a new Arm press
//    hoverItSetStatus(mode, status, heightCm, training)   shown in the app
//    hoverItSetBattery(b)        0-255 battery level to show in the app
//    hoverItSetBatteryVolts(v, vEmpty, vFull)   same, from a voltage
//    hoverItSetGains(loop, kp, ki, kd, ff)      gains to show on the app sliders
//                                (PIDTune.h calls this for you)
//    hoverItStickUs(b)           0-255 -> 1000-2000 us
//    hoverItCentered(b)          0-255 -> -1.0 .. +1.0 (128 = 0)
//    hoverItScale(b, lo, hi)     0-255 -> lo .. hi (for the PID sliders)
//    HoverIt::Rx.throttle, .yaw, .pitch, .roll, .kp, .ki, .kd, .ff, .loop,
//    HoverIt::Rx.mode, .training, .tuning  the latest values (failsafe applied)
//
//  Serial Monitor (115200): Wi-Fi status, the phone's IP when it is found,
//  link lost / regained, arm refused.
// ============================================================================
#pragma once
#include "Config.h"                // WIFI_SSID / WIFI_PASS

#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef HOVERIT_SOFTAP
#define HOVERIT_SOFTAP 0                       // 0 = join phone hotspot, 1 = own network
#endif

namespace HoverIt {

// ---------------- Settings ---------------------------------------------------
const char* const HOTSPOT_SSID = WIFI_SSID;          // set in Config.h
const char* const HOTSPOT_PASS = WIFI_PASS;
const char* const AP_SSID      = "HoverIt";          // used when HOVERIT_SOFTAP = 1
const char* const AP_PASS      = "hoverit123";       // at least 8 characters

const uint16_t DRONE_PORT       = 4210;    // drone listens here (must match the app)
const uint16_t PHONE_PORT       = 4211;    // app listens here   (must match the app)
const uint32_t FAILSAFE_MS      = 500;     // no packet this long: disarm + centre
const uint32_t TELEMETRY_MS     = 100;     // battery to the phone at 10 Hz
const uint8_t  ARM_MAX_THROTTLE = 15;      // throttle must be at/below this to arm

// ESP32-C3 SuperMini boards have a poorly matched antenna: at full power
// (19.5 dBm) Wi-Fi drops packets in bursts. 8.5 dBm is the usual fix and still
// reaches 10-20 m. Try WIFI_POWER_11dBm or WIFI_POWER_5dBm if it is still patchy.
const wifi_power_t WIFI_TX_POWER = WIFI_POWER_8_5dBm;

// ---------------- Packets (keep in sync with DroneLink.kt) ------------------
const uint8_t PROTOCOL_VERSION = 3;          // 3: flight mode bits, height + status in telemetry

struct __attribute__((packed)) CmdPacket {   // phone -> drone, 14 bytes
  uint8_t magic;                             // 'H'
  uint8_t seq;
  uint8_t throttle, yaw, pitch, roll;
  uint8_t kp, ki, kd, ff;
  uint8_t loop;                              // PID loop the sliders are for
  uint8_t flags;                             // bit0 armed, bit1 baro training, bit2 tuning,
                                             // bits4-5 flight mode (0-3)
  uint8_t version;
  uint8_t check;                             // XOR of all bytes before it
};

struct __attribute__((packed)) TelemPacket { // drone -> phone, 12 bytes
  uint8_t magic;                             // 'D'
  uint8_t seq;
  uint8_t battery;                           // 0-255
  uint8_t flags;                             // bit0 armed, bit1 baro training running,
                                             // bits4-5 flight mode the drone is flying
  uint8_t loop, kp, ki, kd, ff;              // the drone's gains for that loop
  uint8_t height;                            // cm / 2 (0-510 cm)
  uint8_t status;                            // what the mode is doing (FlightModeCommon.h)
  uint8_t check;
};

// ---------------- State ------------------------------------------------------
struct Values {
  uint8_t throttle = 0, yaw = 128, pitch = 128, roll = 128;
  uint8_t kp = 0, ki = 0, kd = 0, ff = 0;
  uint8_t loop = 0;
  uint8_t mode = 0;                          // 0 Stabilize, 1 Loiter, 2 Land, 3 Autonomous
  bool armed = false, training = false, tuning = false;
};
Values Rx;                    // what your flight code reads

WiFiUDP Udp;
IPAddress PhoneIp;
bool HavePhone = false;
bool Linked = false;
bool LastArmBit = false;      // for the 0 -> 1 arm edge
uint8_t Battery = 0;
uint8_t ModeNow = 0, StatusNow = 0, HeightNow = 0;
bool TrainingNow = false;
uint8_t GainLoop = 0, GainKp = 0, GainKi = 0, GainKd = 0, GainFf = 0;
uint8_t TxSeq = 0;
uint32_t LastPacketMs = 0, LastTelemMs = 0, LastWifiLogMs = 0;
uint32_t Packets = 0, BadPackets = 0;
uint32_t LastBadLogMs = 0, LoggedBad = 0;
uint32_t RateStartMs = 0, RatePackets = 0, MaxGapMs = 0, PrevPacketMs = 0;
int LastBadSize = 0, LastBadVersion = -1;      // what the last rejected packet looked like

inline uint8_t xorBytes(const uint8_t* p, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

inline void failsafe() {
  Rx.throttle = 0;
  Rx.yaw = Rx.pitch = Rx.roll = 128;
  Rx.armed = false;
  Rx.training = false;
  // kp..ff and loop keep their last values so a dropout doesn't zero your gains
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
  uint32_t t = millis();
  if (PrevPacketMs && t - PrevPacketMs > MaxGapMs) MaxGapMs = t - PrevPacketMs;
  PrevPacketMs = t;
  LastPacketMs = t;
  Packets++;

  Rx.throttle = c.throttle;
  Rx.yaw = c.yaw;
  Rx.pitch = c.pitch;
  Rx.roll = c.roll;
  Rx.kp = c.kp;
  Rx.ki = c.ki;
  Rx.kd = c.kd;
  Rx.ff = c.ff;
  Rx.loop = c.loop;
  Rx.training = c.flags & 0x02;
  Rx.mode = (c.flags >> 4) & 0x03;
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
  t.flags = (Rx.armed ? 0x01 : 0x00) | (TrainingNow ? 0x02 : 0x00) | ((ModeNow & 0x03) << 4);
  t.loop = GainLoop;
  t.kp = GainKp; t.ki = GainKi; t.kd = GainKd; t.ff = GainFf;
  t.height = HeightNow;
  t.status = StatusNow;
  t.check = xorBytes((const uint8_t*)&t, sizeof(t) - 1);
  // linked: straight to the phone. Not linked: broadcast so the app can find
  // us, and also to the last phone we knew, because some phones drop
  // broadcasts on the drone's own network (HOVERIT_SOFTAP).
  if (!(HavePhone && Linked)) {
    Udp.beginPacket(broadcastIp(), PHONE_PORT);
    Udp.write((const uint8_t*)&t, sizeof(t));
    Udp.endPacket();
  }
  if (HavePhone) {
    Udp.beginPacket(PhoneIp, PHONE_PORT);
    Udp.write((const uint8_t*)&t, sizeof(t));
    Udp.endPacket();
  }
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
  WiFi.setTxPower(WIFI_TX_POWER);        // SuperMini antenna fix (see settings)
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
      LastBadSize = size;
      LastBadVersion = -1;
      continue;
    }
    Udp.read((uint8_t*)&c, sizeof(c));
    if (c.magic != 'H' || c.version != PROTOCOL_VERSION ||
        c.check != xorBytes((const uint8_t*)&c, sizeof(c) - 1)) {
      BadPackets++;
      LastBadSize = size;
      LastBadVersion = c.version;
      continue;
    }
    handlePacket(c, Udp.remoteIP());
  }
  now = millis();   // AFTER reading: handlePacket() stamps LastPacketMs later than the old 'now'

  // rejected packets: say why, every 2 s (13 bytes or protocol 1-2 = old app)
  if (BadPackets != LoggedBad && now - LastBadLogMs > 2000) {
    LastBadLogMs = now;
    Serial.printf("[HoverIt] %u packets rejected: last was %d bytes", (unsigned)(BadPackets - LoggedBad), LastBadSize);
    if (LastBadVersion >= 0) Serial.printf(", protocol %d", LastBadVersion);
    Serial.printf(" (this drone wants %u bytes, protocol %u)%s\n", (unsigned)sizeof(CmdPacket), PROTOCOL_VERSION,
                  (LastBadSize == 13 || (LastBadVersion >= 0 && LastBadVersion < PROTOCOL_VERSION))
                      ? " -> the app is the OLD version, reinstall it" : "");
    LoggedBad = BadPackets;
  }

  if (Linked && (int32_t)(now - LastPacketMs) > (int32_t)FAILSAFE_MS) {
    Linked = false;
    LastArmBit = false;
    failsafe();
    Serial.println("[HoverIt] link lost -> failsafe (disarmed)");
  }

  // link quality every 5 s while packets arrive: the app sends 50 per second
  if (now - RateStartMs >= 5000) {
    uint32_t got = Packets - RatePackets;
    if (got > 0)
      Serial.printf("[HoverIt] %u packets/s from the app (50 expected), longest gap %u ms, signal %d dBm\n",
                    (unsigned)(got * 1000 / (now - RateStartMs)), (unsigned)MaxGapMs, (int)WiFi.RSSI());
    RateStartMs = now;
    RatePackets = Packets;
    MaxGapMs = 0;
  }

  // no network yet (hotspot not joined): sending would only fail (error 118)
  bool netUp = HOVERIT_SOFTAP || WiFi.status() == WL_CONNECTED;
  if (netUp && now - LastTelemMs >= TELEMETRY_MS) {
    LastTelemMs = now;
    sendTelemetry();
  }

#if !HOVERIT_SOFTAP
  if (now - LastWifiLogMs > 3000) {
    LastWifiLogMs = now;
    static bool wasUp = false;
    bool up = WiFi.status() == WL_CONNECTED;
    if (up && !wasUp) { Serial.print("[HoverIt] on hotspot, drone IP "); Serial.println(WiFi.localIP()); }
    if (!up) {
      // status 1 = hotspot name not found (wrong name, or 5 GHz only),
      // 4 = wrong password / WPA3-only, 6 = still trying
      Serial.print("[HoverIt] not on hotspot \"");
      Serial.print(HOTSPOT_SSID);
      Serial.print("\" yet, status ");
      Serial.println((int)WiFi.status());
    }
    wasUp = up;
  }
#endif
}

inline bool hoverItConnected() { return HoverIt::Linked; }
inline bool hoverItArmed()     { return HoverIt::Rx.armed; }

// The drone ends a flight by itself (Land / Autonomous touched down). The arm
// bit from the app stays 1, so this does not re-arm until the app sends a new
// 0 -> 1 press; the app sees "disarmed" in telemetry and follows.
inline void hoverItDisarm() {
  if (HoverIt::Rx.armed) Serial.println("[HoverIt] landed -> disarmed by the drone");
  HoverIt::Rx.armed = false;
}

// What the app shows on the flying screen. heightCm is clamped to 0-510.
inline void hoverItSetStatus(uint8_t mode, uint8_t status, float heightCm, bool training) {
  using namespace HoverIt;
  ModeNow = mode; StatusNow = status; TrainingNow = training;
  float h = heightCm / 2.0f;
  HeightNow = h <= 0 ? 0 : h >= 255 ? 255 : (uint8_t)(h + 0.5f);
}

inline void hoverItSetBattery(uint8_t level) { HoverIt::Battery = level; }

inline void hoverItSetBatteryVolts(float v, float vEmpty, float vFull) {
  float f = (v - vEmpty) / (vFull - vEmpty);
  if (f < 0) f = 0;
  if (f > 1) f = 1;
  HoverIt::Battery = (uint8_t)(f * 255.0f + 0.5f);
}

inline void hoverItSetGains(uint8_t loop, uint8_t kp, uint8_t ki, uint8_t kd, uint8_t ff) {
  using namespace HoverIt;
  if (loop != GainLoop) LastTelemMs = millis() - TELEMETRY_MS;   // new loop: tell the app now
  GainLoop = loop; GainKp = kp; GainKi = ki; GainKd = kd; GainFf = ff;
}

inline int   hoverItStickUs(uint8_t b)  { return 1000 + (int)((b * 1000L + 127) / 255); }
inline float hoverItCentered(uint8_t b) { return ((int)b - 128) / (b >= 128 ? 127.0f : 128.0f); }
inline float hoverItScale(uint8_t b, float lo, float hi) { return lo + (hi - lo) * (b / 255.0f); }
